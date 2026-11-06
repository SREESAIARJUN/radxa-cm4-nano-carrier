#!/usr/bin/env python3
"""
Radxa CM4 Nano Carrier - Rev 11: Micro-HDMI (Type D) TMDS breakout
====================================================================

HDMI 2.0 TMDS (HDMI0_TX0..2, HDMI0_CLK) from module connector J2
(DF40, pins 170-190, outer row) to J3, a XUNPU HDMI-101S Micro-HDMI
Type D receptacle (LCSC C720620) on the bottom side at the Y=40 edge.

Every one of the 8 TMDS nets is length-matched to within 0.15 mm of every
other (which also covers the P/N skew inside each pair). That matching is
computed here, written into the board and then *measured back* from the
saved .kicad_pcb, so the numbers in the report are what the fab gets.

Sub-commands
------------
  generate  Load the board, delete the previous Rev 11 group (idempotent),
            write the J3 footprint, place J3, route the 4 pairs (centreline
            -> chamfer -> miter offset -> pair trombones -> P/N phase
            bumps), add the GND return/stitching vias and the L2/L3 GND
            reference planes, fill zones, save. Also writes the HDMI_TMDS
            net class into .kicad_pro and the DRC rules into .kicad_dru.
  verify    Re-load the saved board with pcbnew and check lengths, skew,
            coupling geometry, vias and pinout. Then run kicad-cli DRC
            with mutation tests (a TMDS segment lengthened by 0.20 mm must
            raise a skew violation, and so on). Writes
            artifacts/rev11_hdmi_tmds_report.md. Exits 1 on failure.

Pipeline: tools/cm4_mechanical.py generate  ->  tools/hdmi_tmds.py generate
(cm4_mechanical rebuilds the board from scratch; this stage is additive).

Geometry (carrier frame, mm, KiCad Y-down, viewed from the top)
---------------------------------------------------------------
J2 outer row (Y=38.50, F.Cu): CLK_N 17.2  CLK_P 17.6 | GND 18.0 | TX0_N 18.4
TX0_P 18.8 | GND 19.2 | TX1_N 19.6 TX1_P 20.0 | GND 20.4 | TX2_N 20.8 TX2_P 21.2
| GND 21.6.   Every pair has a GND pin on each side.

J3 (B.Cu, flipped L/R) has staggered rows 0.2 mm apart. The rear row (odd
pins: D2+ 3, D2- 5, D0+ 9, D0- 11) escapes backward on B.Cu. The front row
(even pins: D1+ 6, D1- 8, CLK+ 12, CLK- 14) cannot pass between rear pads
(0.20 mm gaps), so it drops through a via pair just in front of the pads and
runs on F.Cu, entering the gap between J2's two pad rows from the right.
Result: every TMDS net has exactly ONE through via, so via barrel length
cancels in all comparisons, and F.Cu/B.Cu are identical microstrips (L1/L2
and L4/L3 GND), so physical length == electrical length.
"""
from __future__ import annotations

import argparse
import json
import math
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PCB = ROOT / "radxa-cm4-nano-carrier.kicad_pcb"
PRO = ROOT / "radxa-cm4-nano-carrier.kicad_pro"
DRU = ROOT / "radxa-cm4-nano-carrier.kicad_dru"
LIB = ROOT / "lib" / "radxa-cm4-nano-carrier.pretty"
REPORT = ROOT / "artifacts" / "rev11_hdmi_tmds_report.md"
GROUP = "REV11_HDMI0_TMDS"

# --------------------------------------------------------------------------
# 1. Electrical targets
# --------------------------------------------------------------------------
# W/S solved with the validated 2-D FD solver in tools/mdi_impedance.py
# (JLC04161H-7628, 0.2104 mm PP er 4.4, 35 um Cu, 10 um mask):
#   solve_w(0.125) -> W 0.165 -> Zdiff 100.03 ohm, eeff_odd 2.742
# Re-run with `hdmi_tmds.py solve` (about 100 s).
Z_DIFF = 100.0                 # HDMI 2.0 sec. 4.2.x: 100 ohm +/-15 % (board budget +/-10 %)
TMDS_W = 0.165
TMDS_S = 0.125                 # edge-to-edge gap
TMDS_PITCH = TMDS_W + TMDS_S   # 0.29 P/N centre-to-centre
EEFF_ODD = 2.742
TPD_PS_PER_MM = math.sqrt(EEFF_ODD) / 299_792_458.0 * 1e12 / 1e3   # 5.52 ps/mm
SKEW_MAX = 0.150               # mm, ALL 8 nets (intra-pair AND inter-pair)
INTERPAIR_EDGE = 0.40          # min edge gap pair-to-pair on parallel runs (~1.9 h)
OTHER_CLEARANCE = 0.20         # TMDS copper to any non-TMDS copper (DRU)
MAX_UNCOUPLED = 2.0            # fan-out + phase bumps per net (mm), DRU-enforced
TROMBONE_PITCH = 1.0           # trombone leg centreline spacing (edge gap 0.545 >= 3W)
CHAMFER = 0.35                 # 90-degree corners -> 2 x 45 deg
BUMP_H, BUMP_FLAT = 0.20, 0.30 # max phase-bump height, flat top
VIA_D, VIA_DRILL = 0.45, 0.20
VIA_PAIR_PITCH = 0.55          # diff via pitch (0.10 mm edge gap)
GND_VIA_D, GND_VIA_DRILL = 0.45, 0.20
GND_STUB_W = 0.15

# HDMI 2.0 timing context (for the report): TMDS character rate up to 600 MHz
# (5.94 Gbit/s/lane after 1/4 scrambling clock ratio), UI = 168 ps.
HDMI20_LANE_GBPS = 5.94
UI_PS = 1e3 / HDMI20_LANE_GBPS

PAIRS = {
    # name : (J2 P net, J2 N net, J3 P pin, J3 N pin, layer of main run)
    "TX2": ("HDMI0_TX2_P", "HDMI0_TX2_N", "3", "5", "B"),
    "TX1": ("HDMI0_TX1_P", "HDMI0_TX1_N", "6", "8", "F"),
    "TX0": ("HDMI0_TX0_P", "HDMI0_TX0_N", "9", "11", "B"),
    "CLK": ("HDMI0_CLK_P", "HDMI0_CLK_N", "12", "14", "F"),
}
TMDS_NETS = [n for p in PAIRS.values() for n in p[:2]]

# J3 pin -> net (HDMI Type D pin assignment, HDMI 1.4b Table 4-3 / XUNPU symbol)
J3_NETS = {
    "1": "HDMI0_HOTPLUG", "2": "", "3": "HDMI0_TX2_P", "4": "GND", "5": "HDMI0_TX2_N",
    "6": "HDMI0_TX1_P", "7": "GND", "8": "HDMI0_TX1_N", "9": "HDMI0_TX0_P", "10": "GND",
    "11": "HDMI0_TX0_N", "12": "HDMI0_CLK_P", "13": "GND", "14": "HDMI0_CLK_N",
    "15": "HDMI0_CEC", "16": "GND", "17": "HDMI0_SCL", "18": "HDMI0_SDA", "19": "HDMI0_5V",
    "20": "GND",
}

# --------------------------------------------------------------------------
# 2. J3 footprint: XUNPU HDMI-101S (LCSC C720620), land pattern taken from
#    the EasyEDA/LCSC footprint "HDMI-SMD_HDMI-101S" (pad centres, sizes,
#    shell slots verbatim). Body/mouth outline is approximate: confirm
#    against the XUNPU drawing before release.
# --------------------------------------------------------------------------
J3_FP = "HDMI_Micro-D_XUNPU_HDMI-101S"
J3_REF = "J3"
J3_AT = (40.6, 36.0)           # origin, carrier frame; footprint on B.Cu, mouth -> +Y edge
J3_MOUTH_Y = 3.75              # local Y of the receptacle mouth (front of shell)
J3_BODY_X = 3.35               # half width of the metal shell (approx.)
J3_BODY_REAR = -2.95


def j3_footprint() -> str:
    L = [f'(footprint "{J3_FP}"', '  (version 20240108)', '  (generator "hdmi_tmds.py")', '  (layer "F.Cu")',
         '  (descr "Micro HDMI Type D receptacle, right angle, SMD + 4 shell slots. XUNPU HDMI-101S, LCSC C720620. '
         'Land pattern = LCSC/EasyEDA HDMI-SMD_HDMI-101S. Mouth at +Y.")',
         '  (tags "HDMI micro type D connector 19P 0.4mm")', '  (attr smd)',
         '  (property "Reference" "REF**" (at 0 -4.0 0) (layer "F.SilkS") (effects (font (size 0.8 0.8) (thickness 0.12))))',
         f'  (property "Value" "HDMI-101S" (at 0 1.6 0) (layer "F.Fab") (effects (font (size 0.6 0.6) (thickness 0.1))))',
         '  (property "LCSC" "C720620" (at 0 0 0) (layer "F.Fab") (hide yes) (effects (font (size 1 1))))',
         '  (property "MPN" "HDMI-101S" (at 0 0 0) (layer "F.Fab") (hide yes) (effects (font (size 1 1))))']
    bx, br, bm = J3_BODY_X, J3_BODY_REAR, J3_MOUTH_Y
    for a, b in [((-bx, br), (bx, br)), ((bx, br), (bx, bm)), ((bx, bm), (-bx, bm)), ((-bx, bm), (-bx, br))]:
        L.append(f'  (fp_line (start {a[0]} {a[1]}) (end {b[0]} {b[1]}) (stroke (width 0.1) (type solid)) (layer "F.Fab"))')
    L.append(f'  (fp_text user "PCB EDGE / MOUTH" (at 0 {bm - 0.5}) (layer "F.Fab") (effects (font (size 0.4 0.4) (thickness 0.06))))')
    cx0, cy0, cx1, cy1 = -4.1, -3.05, 4.1, bm + 0.2
    for a, b in [((cx0, cy0), (cx1, cy0)), ((cx1, cy0), (cx1, cy1)), ((cx1, cy1), (cx0, cy1)), ((cx0, cy1), (cx0, cy0))]:
        L.append(f'  (fp_line (start {a[0]} {a[1]}) (end {b[0]} {b[1]}) (stroke (width 0.05) (type solid)) (layer "F.CrtYd"))')
    L.append('  (fp_poly (pts (xy 1.8 -3.25) (xy 1.6 -3.55) (xy 2.0 -3.55)) (stroke (width 0.1) (type solid)) (fill solid) (layer "F.SilkS"))')
    for pin in range(1, 20):
        x = round(1.8 - 0.2 * (pin - 1), 3)
        if pin % 2:
            y, sz = -2.385, (0.2, 0.84)
        else:
            y, sz = -1.165, (0.2, 1.0)
        L.append(f'  (pad "{pin}" smd rect (at {x} {y}) (size {sz[0]} {sz[1]}) (layers "F.Cu" "F.Paste" "F.Mask"))')
    for (x, y) in [(-3.1, 2.385), (-3.1, -0.975), (3.1, 2.385), (3.1, -0.975)]:
        L.append(f'  (pad "20" thru_hole oval (at {x} {y}) (size 1.5 2.55) (drill oval 0.7 1.7) '
                 '(layers "*.Cu" "*.Mask") (remove_unused_layers no))')
    L.append(')')
    return "\n".join(L) + "\n"


# --------------------------------------------------------------------------
# 3. 2-D geometry
# --------------------------------------------------------------------------
def sub(a, b): return (a[0] - b[0], a[1] - b[1])
def add(a, b): return (a[0] + b[0], a[1] + b[1])
def mul(a, k): return (a[0] * k, a[1] * k)
def norm(a):
    L = math.hypot(*a)
    return (a[0] / L, a[1] / L)
def left(u): return (u[1], -u[0])           # left-hand normal, KiCad Y-down screen frame
def plen(pts): return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


def dedupe(pts, eps=1e-9):
    out = [pts[0]]
    for p in pts[1:]:
        if math.dist(p, out[-1]) > eps:
            out.append(p)
    # drop collinear interior points
    res = [out[0]]
    for i in range(1, len(out) - 1):
        u, v = norm(sub(out[i], res[-1])), norm(sub(out[i + 1], out[i]))
        if abs(u[0] * v[1] - u[1] * v[0]) > 1e-9 or (u[0] * v[0] + u[1] * v[1]) < 0:
            res.append(out[i])
    res.append(out[-1])
    return res


def chamfer(pts, c=CHAMFER):
    """Replace every 90-degree corner by two 45-degree turns."""
    pts = dedupe(pts)
    out = [pts[0]]
    for i in range(1, len(pts) - 1):
        u, v = norm(sub(pts[i], pts[i - 1])), norm(sub(pts[i + 1], pts[i]))
        if abs(u[0] * v[0] + u[1] * v[1]) < 1e-6:                    # 90 deg
            lin, lout = math.dist(pts[i], pts[i - 1]), math.dist(pts[i + 1], pts[i])
            cc = min(c, lin / 2.01, lout / 2.01)
            out += [sub(pts[i], mul(u, cc)), add(pts[i], mul(v, cc))]
        else:
            out.append(pts[i])
    out.append(pts[-1])
    return out


def offset(pts, d):
    """Miter offset of a polyline, +d = left of travel direction."""
    segs = []
    for a, b in zip(pts, pts[1:]):
        n = left(norm(sub(b, a)))
        segs.append((add(a, mul(n, d)), add(b, mul(n, d))))
    out = [segs[0][0]]
    for (a1, b1), (a2, b2) in zip(segs, segs[1:]):
        d1, d2 = sub(b1, a1), sub(b2, a2)
        den = d1[0] * d2[1] - d1[1] * d2[0]
        if abs(den) < 1e-12:
            out.append(b1)
            continue
        t = ((a2[0] - a1[0]) * d2[1] - (a2[1] - a1[1]) * d2[0]) / den
        out.append(add(a1, mul(d1, t)))
    out.append(segs[-1][1])
    return out


def trombone(at, heading, side, depth, pitch=TROMBONE_PITCH):
    """Corner points of a pair trombone inserted at `at` on a straight run.
    side=+1 -> excursion to the left of travel. Adds 2*depth to the centreline
    and (L,R,R,L) turn order makes its P/N skew contribution exactly zero."""
    n = mul(left(heading), side)
    p1 = add(at, mul(n, depth))
    p2 = add(p1, mul(heading, pitch))
    p3 = add(at, mul(heading, pitch))
    return [at, p1, p2, p3]


def insert_bumps(pts, anchors, h, side=+1, flat=BUMP_FLAT):
    """Single-trace phase-compensation bumps (45-deg flanks, height h) at
    the given anchor points, which must lie on straight segments of pts."""
    pts = list(pts)
    for anc in anchors:
        best = None
        for i, (a, b) in enumerate(zip(pts, pts[1:])):
            ab = sub(b, a)
            L = math.hypot(*ab)
            t = ((anc[0] - a[0]) * ab[0] + (anc[1] - a[1]) * ab[1]) / (L * L)
            q = add(a, mul(ab, t))
            dd = math.dist(q, anc)
            if 0 <= t <= 1 and (best is None or dd < best[0]):
                best = (dd, i, t * L, L)
        dd, i, s, L = best
        assert dd < 0.2, f"bump anchor {anc} not on trace (closest {dd:.3f})"
        span = 2 * h + flat
        s0 = s - span / 2
        assert s0 > 0.05 and s0 + span < L - 0.05, f"bump at {anc} does not fit on segment (len {L:.2f})"
        a, b = pts[i], pts[i + 1]
        u = norm(sub(b, a))
        n = mul(left(u), side)
        A = add(a, mul(u, s0))
        B = add(add(A, mul(u, h)), mul(n, h))
        C = add(B, mul(u, flat))
        D = add(A, mul(u, span))
        pts[i + 1:i + 1] = [A, B, C, D]
    return pts


BUMP_EXTRA_PER_H = 2 * (math.sqrt(2) - 1)


# --------------------------------------------------------------------------
# 4. Route plan (all coordinates mm, carrier frame)
# --------------------------------------------------------------------------
# Y positions
J2_VIA_Y = 37.75               # TX0/TX2 F->B vias between the DF40 rows (pad edge 38.15)
J2_GND_VIA_Y = 39.35           # GND vias outboard of the J2 outer row (edge clearance 0.425)
J3_FRONT_VIA_Y = 35.80         # TX1/CLK B->F vias in front of J3 front row (pad edge 35.335)
LANE_CLK_GAP = 36.225          # F.Cu lanes inside the J2 row gap (inner-row pad edge 35.77)
LANE_TX1_GAP = 37.080          #   (TX2 via top edge 37.525)
COL_CLK_X, COL_TX1_X = 35.40, 36.255   # F.Cu column between J2 pin 101 (34.9) and J3 slot (36.75)
TOP_TX1_Y = 26.50              # F.Cu top lanes
TOP_CLK_Y = TOP_TX1_Y - (TMDS_PITCH + TMDS_W + INTERPAIR_EDGE + 0.0)   # 0.855 pitch
CLK_RUN_X = 42.40              # CLK F.Cu riser jogs east to clear the J3 pin-13 GND via
LANE_TX0_B = 31.00             # B.Cu lanes behind J3 (rear-row escape)
LANE_TX2_B = 32.40
GND7_VIA = (38.60, 31.70)      # J3 pin 7 (D1 shield, between D2/D0) -> channel between B lanes
GND13_VIA = (41.35, 32.30)     # J3 pin 13 (CLK shield)
J3_FRONT_GND = {"4": (38.85, 36.10), "10": (40.60, 36.55), "16": (42.35, 36.10)}

# Inter-pair trombone slots (filled by the solver): pair -> list of (at, heading, side)
TROMBONE_SLOTS = {
    "TX2": [((33.5, LANE_TX2_B), (-1, 0), +1), ((30.5, LANE_TX2_B), (-1, 0), +1), ((27.5, LANE_TX2_B), (-1, 0), +1),
            ((24.5, LANE_TX2_B), (-1, 0), +1)],                         # +1 = left of west = south
    "TX0": [((34.0, LANE_TX0_B), (-1, 0), -1), ((31.0, LANE_TX0_B), (-1, 0), -1)],   # north
    "TX1": [((39.85, 30.8), (0, -1), +1), ((39.85, 28.8), (0, -1), +1)],              # west
    "CLK": [],
}
TROMBONE_MAX_DEPTH = {"TX2": 3.6, "TX0": 9.0, "TX1": 2.3, "CLK": 0.0}

# Phase-bump anchors on the P trace (P is always the left/inner trace: every
# route turns a net 180 deg left between J3 and J2).
BUMP_ANCHORS = {
    "TX2": [(21.0 + TMDS_PITCH / 2, y) for y in (33.6, 34.5, 35.4, 36.3)],
    "TX0": [(18.6 + TMDS_PITCH / 2, y) for y in (33.6, 34.5, 35.4, 36.3)],
    "TX1": [(x, LANE_TX1_GAP + TMDS_PITCH / 2) for x in (33.0, 31.5, 30.0, 28.5)],
    "CLK": [(CLK_RUN_X - TMDS_PITCH / 2, y) for y in (31.0, 30.0, 29.0, 28.0)],
}


def centreline(pair: str, pads: dict, depths: list[float]) -> tuple[list, dict]:
    """Return (centreline corner list, meta) for the main coupled run."""
    j3p, j3n = pads["J3"][PAIRS[pair][2]], pads["J3"][PAIRS[pair][3]]
    j2p, j2n = pads["J2"][PAIRS[pair][0]], pads["J2"][PAIRS[pair][1]]
    x3 = (j3p[0] + j3n[0]) / 2
    x2 = (j2p[0] + j2n[0]) / 2
    slots = TROMBONE_SLOTS[pair]
    tromb = lambda i: trombone(slots[i][0], slots[i][1], slots[i][2], depths[i]) if depths[i] > 1e-6 else [slots[i][0]]
    if pair in ("TX2", "TX0"):
        lane = LANE_TX2_B if pair == "TX2" else LANE_TX0_B
        pts = [(x3, j3p[1] - 0.45), (x3, lane)]
        for i in range(len(slots)):
            pts += tromb(i)
        pts += [(x2, lane), (x2, J2_VIA_Y - 0.50)]
    elif pair == "TX1":
        pts = [(x3 + 0.05, J3_FRONT_VIA_Y - 0.50)]
        for i in range(len(slots)):
            pts += tromb(i)
        pts += [(x3 + 0.05, TOP_TX1_Y), (COL_TX1_X, TOP_TX1_Y), (COL_TX1_X, LANE_TX1_GAP),
                (x2, LANE_TX1_GAP), (x2, j2p[1] - 0.55)]
    else:  # CLK
        xs = x3 - 0.05
        pts = [(xs, J3_FRONT_VIA_Y - 0.50), (xs, 34.80), (CLK_RUN_X, 34.80 - (CLK_RUN_X - xs)),
               (CLK_RUN_X, TOP_CLK_Y), (COL_CLK_X, TOP_CLK_Y), (COL_CLK_X, LANE_CLK_GAP),
               (x2, LANE_CLK_GAP), (x2, j2p[1] - 0.55)]
    return chamfer(pts), dict(x3=x3, x2=x2)


def build_pair(pair: str, pads: dict, depths: list[float], bump_h: float | None):
    """Full geometry for one pair. Returns dict net -> list of (layer, pts),
    vias, and lengths."""
    pnet, nnet, p3, n3, layer = PAIRS[pair]
    cl, meta = centreline(pair, pads, depths)
    d = TMDS_PITCH / 2
    P, N = offset(cl, +d), offset(cl, -d)
    j3p, j3n = pads["J3"][p3], pads["J3"][n3]
    j2p, j2n = pads["J2"][pnet], pads["J2"][nnet]
    out = {pnet: [], nnet: []}
    vias = {}
    if layer == "B":
        # J3 rear pad -> coupled run (B.Cu) -> via between J2 rows -> F.Cu stub -> J2 pad
        x2 = meta["x2"]
        h_end = (0.0, 1.0)                        # heading south at the J2 end
        vP = add((x2, J2_VIA_Y), mul(left(h_end), VIA_PAIR_PITCH / 2))
        vN = add((x2, J2_VIA_Y), mul(left(h_end), -VIA_PAIR_PITCH / 2))
        Pm, Nm = [j3p] + P + [vP], [j3n] + N + [vN]
        main_layer = "B.Cu"
        stubs = {pnet: ("F.Cu", [vP, j2p]), nnet: ("F.Cu", [vN, j2n])}
    else:
        # J3 front pad -> B.Cu stub -> via in front of J3 -> coupled run (F.Cu) -> J2 pad
        xs = cl[0][0]
        h0 = (0.0, -1.0)                          # heading north at the J3 end
        vP = add((xs, J3_FRONT_VIA_Y), mul(left(h0), VIA_PAIR_PITCH / 2))
        vN = add((xs, J3_FRONT_VIA_Y), mul(left(h0), -VIA_PAIR_PITCH / 2))
        Pm, Nm = [vP] + P + [j2p], [vN] + N + [j2n]
        main_layer = "F.Cu"
        stubs = {pnet: ("B.Cu", [j3p, vP]), nnet: ("B.Cu", [j3n, vN])}
    # polarity sanity: P via/pad must be on the P side at both ends
    assert math.dist(Pm[1], Pm[0]) < math.dist(Pm[1], Nm[0]) + 1e-9, f"{pair}: P/N swapped at start"
    assert math.dist(Pm[-2], Pm[-1]) < math.dist(Pm[-2], Nm[-1]) + 1e-9, f"{pair}: P/N swapped at end"
    if bump_h:
        Pm = insert_bumps(Pm, BUMP_ANCHORS[pair], bump_h, side=+1)
    out[pnet] = [(main_layer, Pm), stubs[pnet]]
    out[nnet] = [(main_layer, Nm), stubs[nnet]]
    vias = {pnet: vP, nnet: vN}
    lens = {n: sum(plen(p) for _, p in segs) for n, segs in out.items()}
    return dict(tracks=out, vias=vias, lens=lens, centreline=cl)


def solve(pads: dict) -> dict:
    """Pick trombone depths and bump height so that all 8 nets match."""
    base = {p: build_pair(p, pads, [0.0] * len(TROMBONE_SLOTS[p]), None) for p in PAIRS}
    # N trace is never bumped: it carries the pair length.
    nlen = {p: base[p]["lens"][PAIRS[p][1]] for p in PAIRS}
    target = max(nlen.values())
    assert max(nlen, key=nlen.get) == "CLK", f"CLK must be the natural longest pair ({nlen})"
    depths = {}
    for p in PAIRS:
        need = target - nlen[p]
        k = len(TROMBONE_SLOTS[p])
        if k == 0:
            depths[p] = []
            continue
        # fewest trombones that respect the depth limit; equal depths
        for use in range(1, k + 1):
            if need / (2 * use) <= TROMBONE_MAX_DEPTH[p]:
                break
        else:
            raise SystemExit(f"{p}: need {need:.2f} mm, slots exhausted")
        depths[p] = [need / (2 * use)] * use + [0.0] * (k - use)
    # iterate once on measured lengths (chamfer interaction is not exactly 2*D)
    for _ in range(4):
        for p in PAIRS:
            if not depths[p]:
                continue
            r = build_pair(p, pads, depths[p], None)
            err = target - r["lens"][PAIRS[p][1]]
            use = sum(1 for d in depths[p] if d > 1e-6)
            depths[p] = [d + err / (2 * use) if d > 1e-6 else 0.0 for d in depths[p]]
    # P/N phase bumps
    result = {}
    for p in PAIRS:
        r = build_pair(p, pads, depths[p], None)
        deficit = r["lens"][PAIRS[p][1]] - r["lens"][PAIRS[p][0]]
        n = len(BUMP_ANCHORS[p])
        assert deficit >= 0, f"{p}: P longer than N ({deficit:.4f}) - bump side wrong"
        h = deficit / (n * BUMP_EXTRA_PER_H)
        for _ in range(3):
            assert h <= BUMP_H + 1e-9, f"{p}: bump {h:.3f} > {BUMP_H} mm"
            rb = build_pair(p, pads, depths[p], h)
            err = rb["lens"][PAIRS[p][1]] - rb["lens"][PAIRS[p][0]]
            h += err / (n * BUMP_EXTRA_PER_H)
        rb = build_pair(p, pads, depths[p], h)
        rb.update(depths=depths[p], bump_h=h, natural=nlen[p])
        result[p] = rb
    return result


# --------------------------------------------------------------------------
# 5. pcbnew helpers
# --------------------------------------------------------------------------
def pad_positions(board) -> dict:
    import pcbnew
    tm = pcbnew.ToMM
    fps = {f.GetReference(): f for f in board.GetFootprints()}
    out = {"J2": {}, "J3": {}}
    for pad in fps["J2"].Pads():
        if pad.GetNetname() in TMDS_NETS or pad.GetNetname() == "GND":
            out["J2"].setdefault(pad.GetNetname(), [])
            out["J2"][pad.GetNetname()].append((round(tm(pad.GetPosition().x), 4), round(tm(pad.GetPosition().y), 4)))
    out["J2"] = {k: (v[0] if k != "GND" else v) for k, v in out["J2"].items()}
    for pad in fps[J3_REF].Pads():
        p = (round(tm(pad.GetPosition().x), 4), round(tm(pad.GetPosition().y), 4))
        if pad.GetNumber() == "20":
            out["J3"].setdefault("20", []).append(p)
        else:
            out["J3"][pad.GetNumber()] = p
    return out


def remove_previous(board) -> None:
    import pcbnew
    for g in list(board.Groups()):
        if g.GetName() == GROUP:
            for it in list(g.GetItems()):
                board.Remove(it)
            board.Remove(g)
    for fp in list(board.GetFootprints()):
        if fp.GetReference() == J3_REF:
            board.Remove(fp)
    for z in list(board.Zones()):
        if z.GetZoneName() in ("GND_L2_REF", "GND_L3_REF"):
            board.Remove(z)


def cmd_generate(_a) -> int:
    import pcbnew
    mm = pcbnew.FromMM
    V = lambda p: pcbnew.VECTOR2I(mm(p[0]), mm(p[1]))
    LIB.mkdir(parents=True, exist_ok=True)
    (LIB / f"{J3_FP}.kicad_mod").write_text(j3_footprint())

    board = pcbnew.LoadBoard(str(PCB))
    remove_previous(board)
    nets = {}

    def net(name):
        if not name:
            return None
        n = board.FindNet(name)
        if n is None:
            n = pcbnew.NETINFO_ITEM(board, name)
            board.Add(n)
        nets[name] = n
        return n

    # --- J3
    fp = pcbnew.FootprintLoad(str(LIB), J3_FP)
    fp.SetReference(J3_REF)
    fp.SetValue("HDMI-101S")
    fp.SetFPIDAsString(f"radxa-cm4-nano-carrier:{J3_FP}")
    board.Add(fp)
    fp.SetPosition(V(J3_AT))
    fp.Flip(V(J3_AT), True)                   # to B.Cu, mirrored left/right
    for pad in fp.Pads():
        n = net(J3_NETS[pad.GetNumber()])
        if n is not None:
            pad.SetNet(n)
    fp.Reference().SetPosition(V((J3_AT[0], J3_AT[1] - 4.3)))
    fp.SetLocked(True)

    pads = pad_positions(board)
    plan = solve(pads)

    grp = pcbnew.PCB_GROUP(board)
    grp.SetName(GROUP)
    board.Add(grp)
    layer_id = {"F.Cu": pcbnew.F_Cu, "B.Cu": pcbnew.B_Cu}

    def track(pts, layer, n, w):
        for a, b in zip(pts, pts[1:]):
            if math.dist(a, b) < 1e-6:
                continue
            t = pcbnew.PCB_TRACK(board)
            t.SetStart(V(a))
            t.SetEnd(V(b))
            t.SetWidth(mm(w))
            t.SetLayer(layer_id[layer])
            t.SetNet(n)
            board.Add(t)
            grp.AddItem(t)

    def via(p, n, d=VIA_D, drill=VIA_DRILL):
        v = pcbnew.PCB_VIA(board)
        v.SetPosition(V(p))
        v.SetWidth(mm(d))
        v.SetDrill(mm(drill))
        v.SetViaType(pcbnew.VIATYPE_THROUGH)
        v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        v.SetNet(n)
        board.Add(v)
        grp.AddItem(v)

    for p, r in plan.items():
        for nn, segs in r["tracks"].items():
            for layer, pts in segs:
                track(pts, layer, net(nn), TMDS_W)
            via(r["vias"][nn], net(nn))

    # --- GND return / stitching vias
    gnd = net("GND")
    for (x, y) in pads["J2"]["GND"]:
        if 16.5 <= x <= 22.0 and y > 37.0:      # outer-row GND pins flanking the 4 pairs
            via((x, J2_GND_VIA_Y), gnd)
            track([(x, y), (x, J2_GND_VIA_Y)], "F.Cu", gnd, GND_STUB_W)
    tx2_p = plan["TX2"]["vias"]["HDMI0_TX2_P"]
    g = (tx2_p[0] + VIA_PAIR_PITCH + 0.10, J2_VIA_Y)   # beside the TX2 via pair, tied to pin 168
    via(g, gnd)
    p168 = min(pads["J2"]["GND"], key=lambda q: math.dist(q, g))
    track([p168, (p168[0], J2_VIA_Y + 0.30), g], "F.Cu", gnd, GND_STUB_W)
    j3 = pads["J3"]
    via(GND7_VIA, gnd)
    track([j3["7"], (j3["7"][0], GND7_VIA[1]), GND7_VIA], "B.Cu", gnd, GND_STUB_W)
    via(GND13_VIA, gnd)
    track([j3["13"], (j3["13"][0], GND13_VIA[1] + 0.45), GND13_VIA], "B.Cu", gnd, GND_STUB_W)
    for pin, pos in J3_FRONT_GND.items():
        via(pos, gnd)
        track([j3[pin], (j3[pin][0], j3[pin][1] + 0.55), pos], "B.Cu", gnd, 0.10 if pin == "10" else GND_STUB_W)

    # --- Solid GND reference planes on L2 (under F.Cu runs) and L3 (under B.Cu runs)
    for lname, lid in (("GND_L2_REF", pcbnew.In1_Cu), ("GND_L3_REF", pcbnew.In2_Cu)):
        z = pcbnew.ZONE(board)
        z.SetZoneName(lname)
        z.SetLayer(lid)
        z.SetNet(gnd)
        z.SetLocalClearance(mm(0.2))
        z.SetMinThickness(mm(0.2))
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
        z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
        ol = z.Outline()
        ol.NewOutline()
        for p in [(0.3, 0.3), (54.7, 0.3), (54.7, 39.7), (0.3, 39.7)]:
            ol.Append(mm(p[0]), mm(p[1]))
        board.Add(z)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())

    tb = board.GetTitleBlock()
    tb.SetRevision("11.0")
    tb.SetComment(2, "Rev 11: J3 Micro-HDMI D + HDMI0 TMDS 100R, 8 nets matched <= 0.15 mm (tools/hdmi_tmds.py)")
    board.Save(str(PCB))
    write_project_rules()

    print(f"J3 {J3_FP} at {J3_AT} (B.Cu). Target length {max(r['lens'][PAIRS[p][1]] for p, r in plan.items()):.4f} mm")
    for p, r in plan.items():
        print(f"  {p}: natural {r['natural']:.3f}  trombones {[round(d, 3) for d in r['depths'] if d]}  "
              f"bump h {r['bump_h']:.4f}  P {r['lens'][PAIRS[p][0]]:.4f}  N {r['lens'][PAIRS[p][1]]:.4f}")
    return 0


# --------------------------------------------------------------------------
# 6. Net class + DRC rules
# --------------------------------------------------------------------------
DRU_BEGIN = "# ---- BEGIN Rev 11 HDMI0 TMDS (generated by tools/hdmi_tmds.py) ----"
DRU_END = "# ---- END Rev 11 HDMI0 TMDS ----"


def dru_text() -> str:
    cond = " || ".join(f"A.NetName == '{n}'" for n in TMDS_NETS)
    return f"""{DRU_BEGIN}
# 100 ohm differential (W {TMDS_W} / S {TMDS_S} mm, FD-solved, see tools/hdmi_tmds.py).
# One length-matching group over ALL 8 TMDS nets: max - min <= {SKEW_MAX} mm. This
# bounds intra-pair skew (P vs N) and inter-pair skew with a single rule, because
# KiCad applies only one skew constraint per net.
(rule "hdmi0_tmds_skew"
  (condition "{cond}")
  (constraint skew (max {SKEW_MAX}mm)))

(rule "hdmi0_tmds_width"
  (condition "A.NetClass == 'HDMI_TMDS'")
  (constraint track_width (min {TMDS_W - 0.005:.3f}mm) (opt {TMDS_W}mm) (max {TMDS_W + 0.005:.3f}mm)))

(rule "hdmi0_tmds_dp_gap"
  (condition "A.NetClass == 'HDMI_TMDS' && A.inDiffPair('*')")
  (constraint diff_pair_gap (min {TMDS_S - 0.010:.3f}mm) (opt {TMDS_S}mm) (max {TMDS_S + 0.010:.3f}mm))
  (constraint diff_pair_uncoupled (max {MAX_UNCOUPLED}mm)))

# TMDS copper keeps {OTHER_CLEARANCE} mm from every non-TMDS item (GND stitching, pads, vias).
(rule "hdmi0_tmds_to_other"
  (condition "A.NetClass == 'HDMI_TMDS' && B.NetClass != 'HDMI_TMDS'")
  (constraint clearance (min {OTHER_CLEARANCE}mm)))

# Every TMDS net changes layer exactly once (F.Cu <-> B.Cu through via).
(rule "hdmi0_tmds_one_via"
  (condition "A.NetClass == 'HDMI_TMDS'")
  (constraint via_count (max 1)))
{DRU_END}
"""


def write_project_rules() -> None:
    txt = DRU.read_text()
    if DRU_BEGIN in txt:
        txt = txt[:txt.index(DRU_BEGIN)].rstrip() + "\n"
    DRU.write_text(txt.rstrip() + "\n\n" + dru_text())

    pro = json.loads(PRO.read_text())
    ns = pro["net_settings"]
    ns["classes"] = [c for c in ns["classes"] if c["name"] != "HDMI_TMDS"]
    ns["classes"].append({
        "bus_width": 12, "clearance": round(TMDS_S - 0.005, 3), "diff_pair_gap": TMDS_S, "diff_pair_via_gap": 0.10,
        "diff_pair_width": TMDS_W, "line_style": 0, "microvia_diameter": 0.3, "microvia_drill": 0.1,
        "name": "HDMI_TMDS", "pcb_color": "rgba(0, 0, 0, 0.000)", "schematic_color": "rgba(0, 0, 0, 0.000)",
        "track_width": TMDS_W, "via_diameter": VIA_D, "via_drill": VIA_DRILL, "wire_width": 6})
    pats = [p for p in (ns.get("netclass_patterns") or []) if p.get("netclass") != "HDMI_TMDS"]
    pats.append({"netclass": "HDMI_TMDS", "pattern": "HDMI0_*_[PN]"})
    ns["netclass_patterns"] = pats
    PRO.write_text(json.dumps(pro, indent=2) + "\n")


# --------------------------------------------------------------------------
# 7. verify
# --------------------------------------------------------------------------
def seg_seg_dist(a, b, c, d):
    def pd(p, a, b):
        ab = sub(b, a)
        L2 = ab[0] ** 2 + ab[1] ** 2
        t = 0 if L2 == 0 else max(0, min(1, ((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1]) / L2))
        return math.dist(p, add(a, mul(ab, t)))
    def ccw(p, q, r): return (r[1] - p[1]) * (q[0] - p[0]) - (q[1] - p[1]) * (r[0] - p[0])
    if (ccw(a, c, d) > 0) != (ccw(b, c, d) > 0) and (ccw(a, b, c) > 0) != (ccw(a, b, d) > 0):
        return 0.0
    return min(pd(a, c, d), pd(b, c, d), pd(c, a, b), pd(d, a, b))


def run_drc(pcb_path: Path) -> dict:
    with tempfile.TemporaryDirectory() as td:
        out = Path(td) / "drc.json"
        subprocess.run(["kicad-cli", "pcb", "drc", "--format", "json", "--severity-all", "--refill-zones",
                        "-o", str(out), str(pcb_path)], check=True, capture_output=True, text=True)
        return json.loads(out.read_text())


def drc_in_scratch(text: str) -> dict:
    with tempfile.TemporaryDirectory() as td:
        for f in (PRO, DRU, ROOT / "radxa-cm4-nano-carrier.kicad_sch", ROOT / "fp-lib-table", ROOT / "sym-lib-table"):
            if f.exists():
                shutil.copy(f, Path(td) / f.name)
        shutil.copytree(ROOT / "lib", Path(td) / "lib")
        p = Path(td) / PCB.name
        p.write_text(text)
        return run_drc(p)


def hdmi_items(v: dict) -> bool:
    s = json.dumps(v)
    return any(n in s for n in TMDS_NETS) or "J3" in s or "hdmi0" in s.lower()


def cmd_verify(a) -> int:
    import pcbnew
    tm = pcbnew.ToMM
    board = pcbnew.LoadBoard(str(PCB))
    rows = []
    add_ = lambda n, ok, d: rows.append((n, bool(ok), d))

    fps = {f.GetReference(): f for f in board.GetFootprints()}
    j3 = fps.get(J3_REF)
    add_("J3 Micro-HDMI D (XUNPU HDMI-101S, LCSC C720620) present on B.Cu", j3 is not None and j3.GetLayer() == pcbnew.B_Cu,
         f"{j3.GetFPIDAsString() if j3 else 'missing'}")
    if j3 is None:
        return report(rows, {}, a)
    bad = [(p.GetNumber(), p.GetNetname(), J3_NETS[p.GetNumber()]) for p in j3.Pads() if p.GetNetname() != J3_NETS[p.GetNumber()]]
    add_("J3 pinout = HDMI Type D (3/5 D2, 6/8 D1, 9/11 D0, 12/14 CLK, 4/7/10/13/16/shell GND, 1 HPD, 15 CEC, 17 SCL, 18 SDA, 19 +5V)",
         not bad, "ok" if not bad else str(bad[:4]))
    # mouth vs edge
    bb = j3.GetBoundingBox(False, False)
    shell_front = max(tm(p.GetBoundingBox().GetBottom()) for p in j3.Pads())
    add_("J3 copper >= 0.30 mm from Y=40 edge, mouth inside outline (zero overhang)",
         40.0 - shell_front >= 0.30 and J3_AT[1] + J3_MOUTH_Y <= 40.0,
         f"shell pad front {shell_front:.3f} (edge gap {40 - shell_front:.3f}); mouth Y {J3_AT[1] + J3_MOUTH_Y:.2f}")
    # THT shell legs vs J2 body on top side
    j2 = fps["J2"]
    legs = [tm(p.GetPosition().x) - 0.75 for p in j3.Pads() if p.GetNumber() == "20"]
    j2_body_right = tm(j2.GetPosition().x) + 22.6 / 2
    add_("J3 through-hole shell legs clear of J2 DF40 body on the top side", min(legs) > j2_body_right,
         f"leftmost leg copper X {min(legs):.2f} > J2 body end {j2_body_right:.2f}")

    # --- lengths per net measured from the saved board
    L, vias, layers, widths = {}, {}, {}, {}
    segs_by_net = {}
    for t in board.GetTracks():
        n = t.GetNetname()
        if n not in TMDS_NETS:
            continue
        if t.Type() == pcbnew.PCB_VIA_T:
            vias[n] = vias.get(n, 0) + 1
            continue
        a_ = (tm(t.GetStart().x), tm(t.GetStart().y))
        b_ = (tm(t.GetEnd().x), tm(t.GetEnd().y))
        L[n] = L.get(n, 0.0) + math.dist(a_, b_)
        layers.setdefault(n, set()).add(t.GetLayerName())
        widths.setdefault(n, set()).add(round(tm(t.GetWidth()), 4))
        segs_by_net.setdefault(n, []).append((t.GetLayerName(), a_, b_))
    missing = [n for n in TMDS_NETS if n not in L]
    add_("All 8 TMDS nets routed", not missing, f"missing {missing}" if missing else "8/8")
    if missing:
        return report(rows, L, a)
    add_(f"Track width {TMDS_W} mm on every TMDS segment", all(w == {TMDS_W} for w in widths.values()),
         str({n: sorted(w) for n, w in widths.items()}))
    add_("Exactly one F.Cu<->B.Cu via per TMDS net (via barrel cancels in all comparisons)",
         all(vias.get(n) == 1 for n in TMDS_NETS), str(vias))
    spread = max(L.values()) - min(L.values())
    add_(f"Inter-pair: all 8 nets within {SKEW_MAX} mm (max-min)", spread <= SKEW_MAX,
         f"{min(L.values()):.4f} .. {max(L.values()):.4f} mm, spread {spread * 1000:.1f} um = {spread * TPD_PS_PER_MM:.3f} ps")
    for p, (pn, nn, *_r) in PAIRS.items():
        sk = abs(L[pn] - L[nn])
        add_(f"Intra-pair {p}: |P-N| <= {SKEW_MAX} mm", sk <= SKEW_MAX,
             f"P {L[pn]:.4f}  N {L[nn]:.4f}  skew {sk * 1000:.2f} um = {sk * TPD_PS_PER_MM:.4f} ps")

    # --- coupling: P/N centreline spacing on parallel coupled segments
    worst_gap, coupled, unc = {}, {}, {}
    for p, (pn, nn, *_r) in PAIRS.items():
        cpl = 0.0
        g = []
        for (lp, a1, b1) in segs_by_net[pn]:
            for (ln, a2, b2) in segs_by_net[nn]:
                if lp != ln:
                    continue
                u1, u2 = norm(sub(b1, a1)), norm(sub(b2, a2))
                if abs(u1[0] * u2[1] - u1[1] * u2[0]) > 1e-3:
                    continue
                # perpendicular distance and overlap length
                nrm = left(u1)
                dperp = abs((a2[0] - a1[0]) * nrm[0] + (a2[1] - a1[1]) * nrm[1])
                s = sorted([0, math.dist(a1, b1)])
                t1 = (a2[0] - a1[0]) * u1[0] + (a2[1] - a1[1]) * u1[1]
                t2 = (b2[0] - a1[0]) * u1[0] + (b2[1] - a1[1]) * u1[1]
                ov = min(s[1], max(t1, t2)) - max(s[0], min(t1, t2))
                if ov > 0.05 and dperp < 0.6:
                    g.append(round(dperp - TMDS_W, 4))
                    if abs(dperp - TMDS_PITCH) < 0.01:
                        cpl += ov
        worst_gap[p] = (min(g), max(x for x in g if x < 0.2))
        coupled[p] = cpl
        unc[p] = L[nn] - cpl
    add_(f"Coupled-section edge gap = {TMDS_S} mm (+/-5 um) on every pair",
         all(abs(lo - TMDS_S) < 0.005 and abs(hi - TMDS_S) < 0.005 for lo, hi in worst_gap.values()),
         "; ".join(f"{p} {lo:.4f}..{hi:.4f}" for p, (lo, hi) in worst_gap.items()))
    add_(f"Coupled fraction >= 95 % of routed length", all(coupled[p] / L[PAIRS[p][1]] >= 0.95 for p in PAIRS),
         "; ".join(f"{p} {coupled[p] / L[PAIRS[p][1]] * 100:.1f} %" for p in PAIRS))

    # --- inter-pair spacing on the same layer (parallel runs) + crossings
    worst = (9.0, "")
    for (p1, q1), (p2, q2) in [((a1, b1), (a2, b2)) for i, (a1, b1) in enumerate(PAIRS.items())
                               for j, (a2, b2) in enumerate(PAIRS.items()) if j > i]:
        for n1 in q1[:2]:
            for n2 in q2[:2]:
                for (la, a_, b_) in segs_by_net[n1]:
                    for (lb, c_, d_) in segs_by_net[n2]:
                        if la == lb:
                            dd = seg_seg_dist(a_, b_, c_, d_) - TMDS_W
                            if dd < worst[0]:
                                worst = (dd, f"{n1}/{n2} on {la}")
    add_(f"Pair-to-pair copper gap >= {INTERPAIR_EDGE} mm (no crossings)", worst[0] >= INTERPAIR_EDGE - 1e-3,
         f"min {worst[0]:.3f} mm ({worst[1]})")

    # --- reference planes
    zn = {z.GetZoneName(): z for z in board.Zones()}
    ok = all(n in zn and zn[n].GetNetname() == "GND" for n in ("GND_L2_REF", "GND_L3_REF"))
    add_("Solid GND reference: L2 (In1) under F.Cu runs, L3 (In2) under B.Cu runs", ok,
         ", ".join(f"{n}:{zn[n].GetLayerName()}" for n in zn if n.startswith("GND_L")))

    # --- timing summary
    mean = sum(L.values()) / 8
    add_("Insertion length (FR4, odd mode) within HDMI 2.0 carrier budget (< 60 mm)", mean < 60.0,
         f"{mean:.2f} mm = {mean * TPD_PS_PER_MM:.1f} ps")

    # --- kicad-cli DRC + mutation tests
    drc = None
    if not a.no_kicad and shutil.which("kicad-cli"):
        drc = run_drc(PCB)
        v = [x for x in drc.get("violations", [])]
        hv = [x for x in v if hdmi_items(x)]
        add_("kicad-cli DRC: 0 violations on the whole board (all severities, custom rules incl. skew)", not v,
             "0" if not v else "; ".join(sorted({f"{x['type']}: {x['description']}" for x in v}))[:600])
        add_("kicad-cli DRC: 0 violations touching TMDS / J3", not hv, f"{len(hv)}")
        uc = [x for x in drc.get("unconnected_items", []) if hdmi_items(x) and any(
            n in json.dumps(x) for n in TMDS_NETS)]
        add_("kicad-cli: 0 unconnected TMDS items (J2 <-> J3 fully routed)", not uc, f"{len(uc)}")
        src = PCB.read_text()
        # pick a TMDS straight segment to lengthen
        net_codes = {board.FindNet(n).GetNetCode(): n for n in TMDS_NETS}
        m = None
        for mm_ in re.finditer(r'\(segment\s*\(start ([\d.\-]+) ([\d.\-]+)\)\s*\(end ([\d.\-]+) ([\d.\-]+)\)\s*'
                               r'\(width [\d.]+\)\s*\(layer "[^"]+"\)\s*\(net (\d+)\)', src):
            x1, y1, x2, y2 = map(float, mm_.groups()[:4])
            if int(mm_.group(5)) in net_codes and net_codes[int(mm_.group(5))].endswith("_N") and abs(y1 - y2) < 1e-6 and abs(x1 - x2) > 3:
                m = mm_
                break
        muts = {}
        if m:
            x1, y1, x2, y2 = map(float, m.groups()[:4])
            # 0.20 mm detour: lift a 0.10 mm-wide notch -> +0.20 mm length, same endpoints
            xm = (x1 + x2) / 2
            s = 1 if x2 > x1 else -1
            seg = m.group(0)
            w = re.search(r'\(width [\d.]+\)\s*\(layer "[^"]+"\)\s*\(net \d+\)', seg).group(0)
            dy = 0.10 if y1 < 30 else -0.10
            pts = [(x1, y1), (xm, y1), (xm, y1 + dy), (xm + s * 0.05, y1 + dy), (xm + s * 0.05, y1), (x2, y2)]
            new = "\n".join(f'(segment (start {a_[0]:.6f} {a_[1]:.6f}) (end {b_[0]:.6f} {b_[1]:.6f}) {w} (uuid "00000000-0000-0000-0000-0000000b{i:04d}"))'
                            for i, (a_, b_) in enumerate(zip(pts, pts[1:])))
            muts["TMDS N trace +0.20 mm -> skew"] = ("skew_out_of_range", _replace_segment(src, m, new))
        muts["foreign via inside TMDS 0.20 mm halo -> clearance"] = (
            "clearance", src.rstrip()[:-1] + f'  (via (at 28.0 {LANE_TX1_GAP + TMDS_PITCH / 2 + TMDS_W / 2 + 0.225 + 0.15:.4f}) (size 0.45) '
            '(drill 0.2) (layers "F.Cu" "B.Cu") (net 1) (uuid "00000000-0000-0000-0000-0000000c0001"))\n)\n')
        for name, (want, text) in muts.items():
            d = drc_in_scratch(text)
            types = sorted({x["type"] for x in d.get("violations", [])})
            add_(f"MUTATION caught: {name}", want in types, f"DRC types {types}")
    return report(rows, L, a, drc)


def _replace_segment(src: str, m, new: str) -> str:
    # m matched up to the net token of the segment; extend to the segment's closing paren
    start = m.start()
    depth, i = 0, start
    while True:
        ch = src[i]
        if ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
            if depth == 0:
                break
        i += 1
    return src[:start] + new + src[i + 1:]


def report(rows, L, a, drc=None) -> int:
    lines = ["# Rev 11.0 - HDMI0 TMDS length-matching verification report", "",
             "Generated by `python3 tools/hdmi_tmds.py verify` from the saved `.kicad_pcb` (lengths measured "
             "segment-by-segment by pcbnew, DRC by kicad-cli with the project `.kicad_dru`).", "",
             "| # | Check | Result | Detail |", "|---|---|---|---|"]
    for i, (n, ok, d) in enumerate(rows, 1):
        lines.append(f"| {i} | {n} | {'PASS' if ok else '**FAIL**'} | {str(d).replace('|', '/')} |")
    failed = [r for r in rows if not r[1]]
    lines += ["", f"**{len(rows) - len(failed)}/{len(rows)} checks passed.**", ""]
    if L:
        ref = max(L.values())
        lines += ["## Routed length per net (pad centre to pad centre, incl. stubs; 1 via each)", "",
                  "| Net | Length mm | vs longest um | Delta ps |", "|---|---|---|---|"]
        for n in TMDS_NETS:
            lines.append(f"| {n} | {L[n]:.4f} | {(L[n] - ref) * 1000:+.1f} | {(L[n] - ref) * TPD_PS_PER_MM:+.3f} |")
        lines += ["", f"Odd-mode delay {TPD_PS_PER_MM:.3f} ps/mm (eeff {EEFF_ODD}); HDMI 2.0 UI at "
                  f"{HDMI20_LANE_GBPS} Gbit/s = {UI_PS:.1f} ps; 0.15 mm = {0.15 * TPD_PS_PER_MM:.2f} ps "
                  f"= {0.15 * TPD_PS_PER_MM / UI_PS * 100:.2f} % UI.", ""]
    REPORT.parent.mkdir(exist_ok=True)
    REPORT.write_text("\n".join(lines) + "\n")
    for n, ok, d in rows:
        print(f"[{'PASS' if ok else 'FAIL'}] {n} :: {d}")
    print(f"\n{len(rows) - len(failed)}/{len(rows)} passed -> {REPORT.relative_to(ROOT)}")
    return 1 if failed else 0


def cmd_solve(_a) -> int:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import mdi_impedance as M
    su = M.STACKUP
    for s in (0.100, 0.125, 0.150):
        w = M.solve_w(s)
        r = M.fd_coupled(w, s, su.h_pp, su.t_outer, su.er_pp, su.t_mask, su.er_mask)
        print(f"S={s:.3f} -> W={w:.3f}  Zdiff {r['zdiff']:.2f}  Zcomm {r['zcomm']:.2f}  eeff_odd {r['eeff_odd']:.3f}")
    r = M.fd_coupled(TMDS_W, TMDS_S, su.h_pp, su.t_outer, su.er_pp, su.t_mask, su.er_mask)
    ok = abs(r["zdiff"] - Z_DIFF) <= 0.03 * Z_DIFF
    print(f"Frozen W={TMDS_W} S={TMDS_S}: Zdiff {r['zdiff']:.2f} ohm -> {'PASS' if ok else 'FAIL'} (+/-3 %)")
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest="cmd", required=True)
    sp.add_parser("generate").set_defaults(fn=cmd_generate)
    v = sp.add_parser("verify")
    v.add_argument("--no-kicad", action="store_true", help="skip kicad-cli DRC + mutation tests")
    v.set_defaults(fn=cmd_verify)
    sp.add_parser("solve").set_defaults(fn=cmd_solve)
    a = ap.parse_args()
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
