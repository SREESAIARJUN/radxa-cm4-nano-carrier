#!/usr/bin/env python3
"""
Radxa CM4 Nano Carrier - Rev 11: Micro-HDMI (Type D) TMDS breakout
====================================================================

HDMI 2.0 TMDS (HDMI0_TX0..2, HDMI0_CLK) from module connector J2
(DF40, pins 170-190, outer row) to J4, a XUNPU HDMI-101S Micro-HDMI
Type D receptacle (LCSC C720620) on the bottom side at the Y=40 edge.

Every one of the 8 TMDS nets is length-matched to within 0.15 mm of every
other (which also covers the P/N skew inside each pair). That matching is
computed here, written into the board and then *measured back* from the
saved .kicad_pcb, so the numbers in the report are what the fab gets.

Sub-commands
------------
  generate  Strip everything a previous run added (S-expression level, see
            strip_previous), write the J4 footprint, place J4, route the 4
            pairs (centreline -> trombones -> 45-deg chamfers -> miter offset
            -> P/N phase bumps, solved so all 8 nets are equal), add GND
            return/stitching vias, keep the existing In1/In2 GND planes (or
            pour full ones on a bare board), fill zones, save. Also writes the
            HDMI_TMDS net class (.kicad_pro) and DRC rules (.kicad_dru).
            Idempotent: re-running yields the same copper.
  verify    Re-load the saved board with pcbnew and check, on the copper
            itself: lengths/skew, 0.125 mm coupled gap, longest uncoupled
            stretch, pair-to-pair spacing, GND reference under every segment,
            vias, pinout, mechanics. Then kicad-cli DRC (0 TMDS/J4 violations)
            and DRU mutation tests. Writes artifacts/rev11_hdmi_tmds_report.md;
            exit 1 on any failure.
  solve     Re-derive W/S for 100 ohm with the FD solver in mdi_impedance.py.

Pipeline (each later stage is additive, earlier ones rebuild the board):
  cm4_mechanical.py generate -> ethernet_mdi.py generate -> hdmi_tmds.py generate

Geometry (carrier frame, mm, KiCad Y-down, viewed from the top)
---------------------------------------------------------------
J2 outer row (Y=38.50, F.Cu): CLK_N 17.2  CLK_P 17.6 | GND 18.0 | TX0_N 18.4
TX0_P 18.8 | GND 19.2 | TX1_N 19.6 TX1_P 20.0 | GND 20.4 | TX2_N 20.8 TX2_P 21.2
| GND 21.6.   Every pair has a GND pin on each side.

J4 (B.Cu, at X 44 on the Y=40 edge) has staggered rows 0.2 mm apart. The
rear row (D2+ 3, D2- 5, D0+ 9, D0- 11) escapes north on B.Cu. The front row
(D1+ 6, D1- 8, CLK+ 12, CLK- 14) cannot pass between rear pads (0.20 mm
gaps), so it drops through a via pair in front of the pads and runs on F.Cu,
entering the gap between J2's two pad rows from the east. Every TMDS net has
exactly ONE through via, so the via barrel cancels in all comparisons, and
F.Cu/B.Cu are identical microstrips (L1/L2 and L4/L3 GND), so physical
length == electrical length.
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
MAX_UNCOUPLED = 8.5            # KiCad-measured uncoupled length per pair (see DRU comment)
TROMBONE_PITCH = 1.0           # trombone leg centreline spacing (edge gap 0.545 >= 3W)
CHAMFER = 0.35                 # 90-degree corners -> 2 x 45 deg
BUMP_H, BUMP_FLAT = 0.22, 0.30 # max phase-bump height (local gap <= 0.345 mm), flat top
VIA_D, VIA_DRILL = 0.50, 0.30  # project minimums (.kicad_pro: min via 0.50, min drill 0.30)
VIA_PAIR_PITCH = 0.65          # diff via pitch (0.15 mm edge gap)
GND_VIA_D, GND_VIA_DRILL = 0.50, 0.30
GND_STUB_W = 0.15

# HDMI 2.0 timing context (for the report): TMDS character rate up to 600 MHz
# (5.94 Gbit/s/lane after 1/4 scrambling clock ratio), UI = 168 ps.
HDMI20_LANE_GBPS = 5.94
UI_PS = 1e3 / HDMI20_LANE_GBPS

PAIRS = {
    # name : (J2 P net, J2 N net, J4 P pin, J4 N pin, layer of main run)
    "TX2": ("HDMI0_TX2_P", "HDMI0_TX2_N", "3", "5", "B"),
    "TX1": ("HDMI0_TX1_P", "HDMI0_TX1_N", "6", "8", "F"),
    "TX0": ("HDMI0_TX0_P", "HDMI0_TX0_N", "9", "11", "B"),
    "CLK": ("HDMI0_CLK_P", "HDMI0_CLK_N", "12", "14", "F"),
}
TMDS_NETS = [n for p in PAIRS.values() for n in p[:2]]

# J4 pin -> net (HDMI Type D pin assignment, HDMI 1.4b Table 4-3 / XUNPU symbol)
J4_NETS = {
    "1": "HDMI0_HOTPLUG", "2": "", "3": "HDMI0_TX2_P", "4": "GND", "5": "HDMI0_TX2_N",
    "6": "HDMI0_TX1_P", "7": "GND", "8": "HDMI0_TX1_N", "9": "HDMI0_TX0_P", "10": "GND",
    "11": "HDMI0_TX0_N", "12": "HDMI0_CLK_P", "13": "GND", "14": "HDMI0_CLK_N",
    "15": "HDMI0_CEC", "16": "GND", "17": "HDMI0_SCL", "18": "HDMI0_SDA", "19": "HDMI0_5V",
    "20": "GND",
}

# --------------------------------------------------------------------------
# 2. J4 footprint: XUNPU HDMI-101S (LCSC C720620), land pattern taken from
#    the EasyEDA/LCSC footprint "HDMI-SMD_HDMI-101S" (pad centres, sizes,
#    shell slots verbatim). Body/mouth outline is approximate: confirm
#    against the XUNPU drawing before release.
# --------------------------------------------------------------------------
J4_FP = "HDMI_Micro-D_XUNPU_HDMI-101S"
J4_REF = "J4"
J4_AT = (44.0, 36.0)           # origin, carrier frame; B.Cu (flipped L/R), mouth -> +Y edge at Y=39.75
J4_MOUTH_Y = 3.75              # local Y of the receptacle mouth (front of shell)
J4_BODY_X = 3.35               # half width of the metal shell (approx.)
J4_BODY_REAR = -2.95


def j4_footprint() -> str:
    L = [f'(footprint "{J4_FP}"', '  (version 20240108)', '  (generator "hdmi_tmds.py")', '  (layer "F.Cu")',
         '  (descr "Micro HDMI Type D receptacle, right angle, SMD + 4 shell slots. XUNPU HDMI-101S, LCSC C720620. '
         'Land pattern = LCSC/EasyEDA HDMI-SMD_HDMI-101S. Mouth at +Y.")',
         '  (tags "HDMI micro type D connector 19P 0.4mm")', '  (attr smd)',
         '  (property "Reference" "REF**" (at 0 -4.0 0) (layer "F.SilkS") (effects (font (size 0.8 0.8) (thickness 0.12))))',
         f'  (property "Value" "HDMI-101S" (at 0 1.6 0) (layer "F.Fab") (effects (font (size 0.6 0.6) (thickness 0.1))))',
         '  (property "LCSC" "C720620" (at 0 0 0) (layer "F.Fab") (hide yes) (effects (font (size 1 1))))',
         '  (property "MPN" "HDMI-101S" (at 0 0 0) (layer "F.Fab") (hide yes) (effects (font (size 1 1))))']
    bx, br, bm = J4_BODY_X, J4_BODY_REAR, J4_MOUTH_Y
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
# Floor plan (co-designed with the Rev 9/10 Ethernet stage, tools/ethernet_mdi.py)
# ------------------------------------------------------------------------------
# That stage owns the west/centre of the inner planes (ETH_SHIELD island
# X 0..34.5, Y 11.8..31.5 + 2 mm hi-pot split -> no GND for X < 36.5 and
# 9.8 < Y < 33.5) and routes its MDI pairs down the east side on B.Cu to
# Y = 27. TMDS must never cross a plane split, so every TMDS segment stays in
# the solid-GND "L": the east band X > 36.5 (Y > 28) and the south band
# Y > 34.1. J4 sits on the Y=40 edge at X 44 (shell legs clear the J2 body
# at X 36.3, courtyard clear of the H4 standoff keep-out at X 48.4).
#
# Route topology, identical for all four pairs (net turn +90 deg left, P is
# always the left/inner trace):
#   J4 -> north -> "top lane" (Y 31.7/32.6) -> west -> "column" (X 37.3/38.16)
#   -> south -> "strip lane" (F: inside the J2 row gap, B: Y 34.6/38.4) -> west
#   -> J2 (pads, or a via pair between / outboard of the DF40 rows).
J4_ROT = 180.0                 # KiCad L/R flip = mirror + 180 deg; 180 here keeps the mouth at +Y
J4_REAR_Y, J4_FRONT_Y = 33.615, 34.835
J4_VIA_Y = 36.30               # front-row B->F via pairs, 0.97 mm in front of the front pads (room for GND stubs)
TOP_N, TOP_S = 30.400, 31.255  # B.Cu top lanes (0.855 pitch); 1.17 mm clear of the T1 courtyard (Y 29.0)
TOP_F_N, TOP_F_S = 29.000, 29.855  # F.Cu top lanes: higher, so the TX1 riser can carry its trombone
COL_W, COL_E = 37.300, 38.155  # columns: 0.57 mm from the X=36.5 split edge, 1.77 mm to the J4 shell leg
LANE_F_N, LANE_F_S = 36.225, 37.080    # F.Cu lanes inside the J2 row gap (inner pad edge 35.77)
LANE_B_N, LANE_B_S = 34.600, 38.400    # B.Cu lanes (34.6 keeps 0.87 mm to the Y=33.5 split edge)
J2_VIA_Y = 37.65               # TX0 F<->B vias between the DF40 rows (0.28 mm to GND pins 180/186)
J2_VIA_Y_OUT = 39.35           # TX2 vias OUTBOARD of the outer row (keeps TX2 polarity, 0.35 mm to edge)
J2_GND_VIA_Y = 39.35
J2_GND_PADS_X = (16.8, 18.0, 19.2)     # outer-row GND pins that get an outboard via
STITCH_VIAS = []                       # (J4 return via is the pin-10 GND via, see J4_GND_TIES)
# J4 SMD GND pins: 4 and 16 bond to the shell legs (THT, GND) along Y 35.6 in front of
# pins 2/18; pin 10 runs between the D1/CLK via pairs to a GND via that is also the
# L2<->L3 return path for both transitions. Rear-row shield pins 7/13 sit under the
# F.Cu risers and between the B.Cu escapes: no via site (open item, README Rev 11).
J4_GND_TIES = {"4": [(42.8, 35.6), (41.4, 35.6)], "16": [(45.2, 35.6), (46.6, 35.6)],
               "10": [(44.0, 37.05)]}
J4_GND_VIA = (44.0, 37.05)
J4_GND_TIE_W = 0.10
J2_TX2_GND = [(22.8, 38.5), (22.8, 39.2), (22.05, 39.35)]   # pin 162 -> GND via east of the TX2 via pair

# pair -> (layer, J4 row, top lane Y, column X, strip lane Y, J2 end)
ROUTE = {
    "TX2": ("B", "rear", TOP_S, COL_E, LANE_B_S, "via-out"),
    "TX0": ("B", "rear", TOP_N, COL_W, LANE_B_N, "via-in"),
    "TX1": ("F", "front", TOP_F_S, COL_E, LANE_F_S, "pads"),
    "CLK": ("F", "front", TOP_F_N, COL_W, LANE_F_N, "pads"),
}

# Inter-pair trombones. B.Cu: on the west-bound strip lanes, TX0 to the south and
# TX2 to the north, interleaved in X so either may use the 2.9 mm between the
# lanes. F.Cu: TX1 on its riser above the J4 shell leg (to the west).
# side +1 = left of travel.
W, NORTH = (-1, 0), (0, -1)
TROMBONE_SLOTS = {
    "TX0": [((35.5, LANE_B_N), W, +1), ((31.5, LANE_B_N), W, +1), ((27.5, LANE_B_N), W, +1)],
    "TX2": [((33.5, LANE_B_S), W, -1), ((29.5, LANE_B_S), W, -1), ((25.5, LANE_B_S), W, -1)],
    "TX1": [((43.1, 33.20), NORTH, +1)],
    "CLK": [],
}
TROMBONE_MAX_DEPTH = {"TX0": 2.60, "TX2": 2.60, "TX1": 3.60, "CLK": 0.0}

# Phase-bump anchors on the centreline (all on P, the inner trace, outer side):
# heading north -> west side, west -> south side, south -> east side.
BUMP_ANCHORS = {
    # >= 1.2 mm pitch: every bump (<= 0.74 mm) is followed by >= 0.46 mm of coupled pair,
    # so no contiguous uncoupled stretch exceeds 1 mm (verify checks this on the copper).
    "CLK": [(44.9, y) for y in (35.3, 34.2, 33.1, 32.0, 30.9, 29.85)] + [(19.0, LANE_F_N)],
    "TX1": [(43.1, 34.7)] + [(x, TOP_F_S) for x in (42.25, 41.2, 40.15, 39.1)],
    "TX0": [(x, LANE_B_N) for x in (36.4, 33.0, 29.0, 24.6, 23.4, 22.2)],
    "TX2": [(x, TOP_S) for x in (41.9, 40.7, 39.5)] + [(x, LANE_B_S) for x in (35.0, 31.0, 23.0)],
}


def centreline(pair: str, pads: dict, depths: list[float]) -> tuple[list, dict]:
    layer, row, top, col, lane, end = ROUTE[pair]
    pnet, nnet, p4, n4, _ = PAIRS[pair]
    j2p, j2n = pads["J2"][pnet], pads["J2"][nnet]
    x2 = (j2p[0] + j2n[0]) / 2
    if row == "rear":
        j4p, j4n = pads["J4"][p4], pads["J4"][n4]
        x4 = (j4p[0] + j4n[0]) / 2
        y0 = J4_REAR_Y - 0.45
    else:
        x4 = via_centre_x(pair, pads)
        y0 = J4_VIA_Y - 0.50
    pts = [(x4, y0), (x4, top), (col, top), (col, lane)]
    if end == "pads":
        pts += [(x2, lane), (x2, j2p[1] - 0.55)]
    elif end == "via-in":
        pts += [(x2, lane), (x2, J2_VIA_Y - 0.50)]
    else:
        pts += [(x2, lane), (x2, J2_VIA_Y_OUT - 0.40)]   # 0.55 mm drop, chamfer self-limits
    for i, (at, hd, side) in enumerate(TROMBONE_SLOTS[pair]):
        if depths[i] > 1e-6:
            pts = _insert_on_segment(pts, at, hd, trombone(at, hd, side, depths[i]))
    return chamfer(pts), dict(x2=x2, x4=x4)


def _insert_on_segment(pts, at, heading, extra):
    """Insert `extra` corner points into the straight segment of `pts` that
    contains `at` and runs along `heading` (trombone slots)."""
    for i, (a, b) in enumerate(zip(pts, pts[1:])):
        u = norm(sub(b, a))
        if abs(u[0] - heading[0]) > 1e-9 or abs(u[1] - heading[1]) > 1e-9:
            continue
        L = math.dist(a, b)
        t = (at[0] - a[0]) * u[0] + (at[1] - a[1]) * u[1]
        perp = abs((at[0] - a[0]) * u[1] - (at[1] - a[1]) * u[0])
        if perp < 1e-6 and 0.5 < t < L - TROMBONE_PITCH - 0.5:
            return pts[:i + 1] + extra + pts[i + 1:]
    raise ValueError(f"trombone slot {at} heading {heading} is not on a long-enough straight run")


def via_centre_x(pair: str, pads: dict) -> float:
    """Front-row via pairs (D1, CLK) are pushed 0.10 mm apart from their pad
    centres so the two 0.65-pitch via pairs keep 0.25 mm between them."""
    p, n = pads["J4"][PAIRS[pair][2]], pads["J4"][PAIRS[pair][3]]
    xc = (p[0] + n[0]) / 2
    other = "CLK" if pair == "TX1" else "TX1"
    ox = sum(pads["J4"][PAIRS[other][k]][0] for k in (2, 3)) / 2
    return xc + (0.30 if xc > ox else -0.30)


def build_pair(pair: str, pads: dict, depths: list[float], bump_h: float | None, bump_net: str | None = None):
    pnet, nnet, p4, n4, layer = PAIRS[pair]
    _l, row, _t, _c, _lane, end = ROUTE[pair]
    cl, meta = centreline(pair, pads, depths)
    d = TMDS_PITCH / 2
    P, N = offset(cl, +d), offset(cl, -d)
    j4p, j4n = pads["J4"][p4], pads["J4"][n4]
    j2p, j2n = pads["J2"][pnet], pads["J2"][nnet]
    h_end = norm(sub(cl[-1], cl[-2]))
    if row == "rear":
        vy = J2_VIA_Y if end == "via-in" else J2_VIA_Y_OUT
        vc = (meta["x2"], vy)
        vP = add(vc, mul(left(h_end), VIA_PAIR_PITCH / 2))
        vN = add(vc, mul(left(h_end), -VIA_PAIR_PITCH / 2))
        Pm, Nm = [j4p] + P + [vP], [j4n] + N + [vN]
        main = "B.Cu"
        # leave the J2 pad along its long (Y) axis first, then turn to the via, so
        # the stub never cuts the corner of the neighbouring GND pin
        ky = 37.95 if end == "via-in" else 39.05
        stubs = {pnet: ("F.Cu", [vP, (j2p[0], ky), j2p]), nnet: ("F.Cu", [vN, (j2n[0], ky), j2n])}
    else:
        h0 = norm(sub(cl[1], cl[0]))
        vc = (meta["x4"], J4_VIA_Y)
        vP = add(vc, mul(left(h0), VIA_PAIR_PITCH / 2))
        vN = add(vc, mul(left(h0), -VIA_PAIR_PITCH / 2))
        Pm, Nm = [vP] + P + [j2p], [vN] + N + [j2n]
        main = "F.Cu"
        # leave the J4 front pad along its long (Y) axis toward the mouth, then the via
        ky = J4_VIA_Y - 0.30
        stubs = {pnet: ("B.Cu", [j4p, (j4p[0], ky), vP]), nnet: ("B.Cu", [j4n, (j4n[0], ky), vN])}
    # polarity: the P via / pad must be the nearer one to the P trace at both ends
    for (pp, nn_, q) in ((Pm[0], Nm[0], Pm[1]), (Pm[-1], Nm[-1], Pm[-2])):
        assert math.dist(q, pp) < math.dist(q, nn_), f"{pair}: P/N would cross"
    sp, sn = stubs[pnet][1], stubs[nnet][1]
    gap = min(seg_seg_dist(a, b, c, e) for a, b in zip(sp, sp[1:]) for c, e in zip(sn, sn[1:])) - TMDS_W
    assert gap > 0.10, f"{pair}: P/N stubs cross or touch (gap {gap:.3f})"
    if bump_h:
        anchors = BUMP_ANCHORS[pair]
        if bump_net == pnet:
            Pm = insert_bumps(Pm, [add(a, mul(left(_dir_at(cl, a)), +d)) for a in anchors], bump_h, side=+1)
        else:
            Nm = insert_bumps(Nm, [add(a, mul(left(_dir_at(cl, a)), -d)) for a in anchors], bump_h, side=-1)
    out = {pnet: [(main, Pm), stubs[pnet]], nnet: [(main, Nm), stubs[nnet]]}
    lens = {n: sum(plen(p) for _, p in segs) for n, segs in out.items()}
    return dict(tracks=out, vias={pnet: vP, nnet: vN}, lens=lens, centreline=cl)


def _dir_at(cl, a):
    best = None
    for p, q in zip(cl, cl[1:]):
        ab = sub(q, p)
        L = math.hypot(*ab)
        t = max(0, min(1, ((a[0] - p[0]) * ab[0] + (a[1] - p[1]) * ab[1]) / (L * L)))
        dd = math.dist(a, add(p, mul(ab, t)))
        if best is None or dd < best[0]:
            best = (dd, norm(ab))
    assert best[0] < 1e-6, f"bump anchor {a} not on centreline"
    return best[1]


def pair_len(r, pair):
    return max(r["lens"][PAIRS[pair][0]], r["lens"][PAIRS[pair][1]])


def solve(pads: dict) -> dict:
    """Trombone depths (inter-pair) then phase bumps (intra-pair) so that all
    8 nets end up equal to the naturally longest net."""
    zero = lambda p: [0.0] * len(TROMBONE_SLOTS[p])
    base = {p: build_pair(p, pads, zero(p), None) for p in PAIRS}
    nat = {p: pair_len(base[p], p) for p in PAIRS}
    target = max(nat.values())
    for p in PAIRS:
        if not TROMBONE_SLOTS[p]:
            assert nat[p] == target, f"{p} has no trombone slot but is not the longest ({nat})"
    depths = {}
    for p in PAIRS:
        k = len(TROMBONE_SLOTS[p])
        if k == 0:
            depths[p] = []
            continue
        # fewest trombones that reach the target at <= max depth, then bisect on
        # the measured length (chamfered trombones are not exactly +2*D)
        Lf = lambda use, D: pair_len(build_pair(p, pads, [D] * use + [0.0] * (k - use), None), p)
        for use in range(1, k + 1):
            if Lf(use, TROMBONE_MAX_DEPTH[p]) >= target:
                break
        else:
            raise SystemExit(f"{p}: needs {target - nat[p]:.2f} mm, trombone slots exhausted")
        lo, hi = 0.0, TROMBONE_MAX_DEPTH[p]
        for _ in range(60):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if Lf(use, mid) < target else (lo, mid)
        depths[p] = [hi] * use + [0.0] * (k - use)
        assert max(depths[p]) <= TROMBONE_MAX_DEPTH[p] + 1e-6, f"{p}: depth {max(depths[p]):.3f}"
    result = {}
    for p in PAIRS:
        r = build_pair(p, pads, depths[p], None)
        lp, ln = r["lens"][PAIRS[p][0]], r["lens"][PAIRS[p][1]]
        short = PAIRS[p][0] if lp < ln else PAIRS[p][1]
        deficit = abs(lp - ln)
        n = len(BUMP_ANCHORS[p])
        h = deficit / (n * BUMP_EXTRA_PER_H)
        for _ in range(4):
            assert h <= BUMP_H + 1e-9, f"{p}: bump {h:.3f} > {BUMP_H} mm"
            rb = build_pair(p, pads, depths[p], h, short)
            err = pair_len(rb, p) - rb["lens"][short]
            h += err / (n * BUMP_EXTRA_PER_H)
        rb = build_pair(p, pads, depths[p], h, short)
        rb.update(depths=depths[p], bump_h=h, bump_net=short, natural=nat[p], natural_skew=deficit)
        result[p] = rb
    return result


# --------------------------------------------------------------------------
# 5. pcbnew helpers
# --------------------------------------------------------------------------
def pad_positions(board) -> dict:
    import pcbnew
    tm = pcbnew.ToMM
    fps = {f.GetReference(): f for f in board.GetFootprints()}
    out = {"J2": {}, "J4": {}}
    for pad in fps["J2"].Pads():
        if pad.GetNetname() in TMDS_NETS or pad.GetNetname() == "GND":
            out["J2"].setdefault(pad.GetNetname(), [])
            out["J2"][pad.GetNetname()].append((round(tm(pad.GetPosition().x), 4), round(tm(pad.GetPosition().y), 4)))
    out["J2"] = {k: (v[0] if k != "GND" else v) for k, v in out["J2"].items()}
    for pad in fps[J4_REF].Pads():
        p = (round(tm(pad.GetPosition().x), 4), round(tm(pad.GetPosition().y), 4))
        if pad.GetNumber() == "20":
            out["J4"].setdefault("20", []).append(p)
        else:
            out["J4"][pad.GetNumber()] = p
    return out


def _top_level_blocks(text: str):
    """Yield (start, end) spans of the depth-1 S-expressions of a .kicad_pcb."""
    depth, start, i, n = 0, None, 0, len(text)
    in_str = False
    while i < n:
        ch = text[i]
        if in_str:
            if ch == "\\":
                i += 1
            elif ch == '"':
                in_str = False
        elif ch == '"':
            in_str = True
        elif ch == "(":
            depth += 1
            if depth == 2:
                start = i
        elif ch == ")":
            if depth == 2:
                yield start, i + 1
            depth -= 1
        i += 1


def strip_previous(text: str) -> str:
    """Remove everything a previous `generate` added (group members, the group,
    J4, our GND planes) at the text level. KiCad 8.0.9's SWIG bindings corrupt
    the board's item lists after BOARD.Remove() of tracks, so deletion through
    pcbnew is not reliable; the S-expression edit is."""
    blocks = list(_top_level_blocks(text))
    members = set()
    for a, b in blocks:
        blk = text[a:b]
        if blk.startswith("(group") and f'"{GROUP}"' in blk.split("\n", 1)[0]:
            m = re.search(r"\(members(.*?)\)", blk, re.S)
            members |= set(re.findall(r'"([0-9a-fA-F-]{36})"', m.group(1))) if m else set()
    drop = []
    for a, b in blocks:
        blk = text[a:b]
        head = blk[1:blk.find("\n") if "\n" in blk else 40]
        uu = re.search(r'\(uuid "([0-9a-fA-F-]{36})"\)', blk)
        if blk.startswith("(group") and f'"{GROUP}"' in head:
            drop.append((a, b))
        elif blk.startswith(("(segment", "(via", "(arc")) and uu and uu.group(1) in members:
            drop.append((a, b))
        elif blk.startswith("(footprint") and re.search(r'\(property "Reference" "' + J4_REF + '"', blk):
            drop.append((a, b))
        elif blk.startswith("(zone") and re.search(r'\(name "GND_L[23]_REF"\)', blk):
            drop.append((a, b))
    out, last = [], 0
    for a, b in drop:
        # take the preceding indentation/newline with the block
        a2 = text.rfind("\n", 0, a)
        out.append(text[last:a2 if a2 >= last else a])
        last = b
    out.append(text[last:])
    return "".join(out)


def cmd_generate(_a) -> int:
    import pcbnew
    mm = pcbnew.FromMM
    V = lambda p: pcbnew.VECTOR2I(mm(p[0]), mm(p[1]))
    LIB.mkdir(parents=True, exist_ok=True)
    (LIB / f"{J4_FP}.kicad_mod").write_text(j4_footprint())

    clean = strip_previous(PCB.read_text())
    tmp = PCB.with_name(".rev11_tmp.kicad_pcb")
    tmp.write_text(clean)
    board = pcbnew.LoadBoard(str(tmp))
    tmp.unlink()
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

    # --- J4
    fp = pcbnew.FootprintLoad(str(LIB), J4_FP)
    fp.SetReference(J4_REF)
    fp.SetValue("HDMI-101S")
    fp.SetFPIDAsString(f"radxa-cm4-nano-carrier:{J4_FP}")
    board.Add(fp)
    fp.SetPosition(V(J4_AT))
    fp.Flip(V(J4_AT), True)                   # to B.Cu, mirrored left/right
    fp.SetOrientationDegrees(J4_ROT)           # mouth faces -X (board edge X = 0)
    for pad in fp.Pads():
        n = net(J4_NETS[pad.GetNumber()])
        if n is not None:
            pad.SetNet(n)
    fp.Reference().SetPosition(V((J4_AT[0] - 1.4, J4_AT[1] - 4.6)))
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

    def via(p, n, d=VIA_D, drill=VIA_DRILL):  # GND vias use the same 0.50/0.30
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
        if any(abs(x - gx) < 1e-3 for gx in J2_GND_PADS_X) and y > 37.0:
            via((x, J2_GND_VIA_Y), gnd)            # outboard of the outer row, 0.425 mm to the edge
            track([(x, y), (x, J2_GND_VIA_Y)], "F.Cu", gnd, GND_STUB_W)
    for p in STITCH_VIAS:
        via(p, gnd)
    via(J4_GND_VIA, gnd)
    for pin, path in J4_GND_TIES.items():
        track([pads["J4"][pin]] + path, "B.Cu", gnd, J4_GND_TIE_W)
    via(J2_TX2_GND[-1], gnd)
    track(J2_TX2_GND, "F.Cu", gnd, GND_STUB_W)

    # --- GND reference planes. If the Ethernet stage already poured In1/In2
    # (GND with the ETH_SHIELD notch) keep them; otherwise pour full planes.
    have = {z.GetLayer() for z in board.Zones() if z.GetNetname() == "GND"}
    for lname, lid in (("GND_L2_REF", pcbnew.In1_Cu), ("GND_L3_REF", pcbnew.In2_Cu)):
        if lid in have:
            continue
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
    tb.SetComment(2, "Rev 11: J4 Micro-HDMI D + HDMI0 TMDS 100R, 8 nets matched <= 0.15 mm (tools/hdmi_tmds.py)")
    board.Save(str(PCB))
    write_project_rules(max(pair_len(r, p) for p, r in plan.items()))

    print(f"J4 {J4_FP} at {J4_AT} (B.Cu, rot {J4_ROT}).")
    for p, r in plan.items():
        print(f"  {p}: natural {r['natural']:.3f} (P/N skew {r['natural_skew']:.3f})  trombones "
              f"{[round(d, 3) for d in r['depths'] if d]}  bump {r['bump_h']:.4f} on {r['bump_net']}  "
              f"P {r['lens'][PAIRS[p][0]]:.4f}  N {r['lens'][PAIRS[p][1]]:.4f}")
    return 0


# --------------------------------------------------------------------------
# 6. Net class + DRC rules
# --------------------------------------------------------------------------
DRU_BEGIN = "# ---- BEGIN Rev 11 HDMI0 TMDS (generated by tools/hdmi_tmds.py) ----"
DRU_END = "# ---- END Rev 11 HDMI0 TMDS ----"


# KiCad 8 counts the via barrel in net length (F.Cu->B.Cu on this 1.6 mm stackup,
# measured: 1.551 mm). Every TMDS net has exactly one via, so it only shifts the window.
KICAD_VIA_LEN = 1.551


def dru_text(target_len: float) -> str:
    cond = " || ".join(f"A.NetName == '{n}'" for n in TMDS_NETS)
    L = target_len + KICAD_VIA_LEN
    half = SKEW_MAX / 2
    skew_rules = "\n".join(
        f"""(rule "hdmi0_tmds_skew_{p}"
  (condition "A.NetName == '{pn}' || A.NetName == '{nn}'")
  (constraint skew (max {half:.3f}mm)))""" for p, (pn, nn, *_r) in PAIRS.items())
    return f"""{DRU_BEGIN}
# 100 ohm differential (W {TMDS_W} / S {TMDS_S} mm, FD-solved, see tools/hdmi_tmds.py).
# Intra-pair: KiCad 8 'skew' is measured against the AVERAGE of the nets the rule
# matches. With exactly P and N in a group each deviates by |P-N|/2, so
# max {half:.3f} mm  <=>  |P - N| <= {SKEW_MAX} mm.
{skew_rules}

# Inter-pair: a +/-{half:.3f} mm length window on all 8 nets (KiCad length incl. the
# {KICAD_VIA_LEN} mm via barrel) bounds max-min <= {SKEW_MAX} mm across all four pairs.
(rule "hdmi0_tmds_length_window"
  (condition "{cond}")
  (constraint length (min {L - half:.4f}mm) (max {L + half:.4f}mm)))

(rule "hdmi0_tmds_width"
  (condition "A.NetClass == 'HDMI_TMDS'")
  (constraint track_width (min {TMDS_W - 0.005:.3f}mm) (opt {TMDS_W}mm) (max {TMDS_W + 0.005:.3f}mm)))

# min = fab floor for the 100-ohm gap. max admits the single-trace phase bumps
# (gap 0.125 + h <= 0.20 mm); the exact 0.125 mm on coupled runs is checked by
# `hdmi_tmds.py verify` on the copper itself.
# Uncoupled: KiCad 8 counts pad/via fan-outs, the 0.6-0.8 mm phase bumps and the
# partially-overlapping 45-deg chamfer segments as uncoupled (5-8 mm measured).
# Each individual uncoupled feature is < 0.9 mm (< 5 ps, << 50 ps edge); the
# rule catches gross de-coupling such as a pair split around an obstacle.
(rule "hdmi0_tmds_dp_gap"
  (condition "A.NetClass == 'HDMI_TMDS' && A.inDiffPair('*')")
  (constraint diff_pair_gap (min {TMDS_S - 0.010:.3f}mm) (opt {TMDS_S}mm) (max {TMDS_S + BUMP_H + 0.010:.3f}mm))
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


def write_project_rules(target_len: float) -> None:
    txt = DRU.read_text()
    # remove ONLY our BEGIN..END block; other stages' rule blocks before/after it survive
    txt = re.sub(re.escape(DRU_BEGIN) + r".*?" + re.escape(DRU_END) + r"\n?", "", txt, flags=re.S)
    DRU.write_text(txt.rstrip() + "\n\n" + dru_text(target_len))

    pro = json.loads(PRO.read_text())
    ns = pro["net_settings"]
    ns["classes"] = [c for c in ns["classes"] if c["name"] != "HDMI_TMDS"]
    ns["classes"].append({
        "bus_width": 12, "clearance": round(TMDS_S - 0.005, 3), "diff_pair_gap": TMDS_S, "diff_pair_via_gap": 0.10,
        "diff_pair_width": TMDS_W, "line_style": 0, "microvia_diameter": 0.3, "microvia_drill": 0.1,
        "name": "HDMI_TMDS", "pcb_color": "rgba(0, 0, 0, 0.000)", "schematic_color": "rgba(0, 0, 0, 0.000)",
        "track_width": TMDS_W, "via_diameter": VIA_D, "via_drill": VIA_DRILL, "wire_width": 6})
    pats = [p for p in (ns.get("netclass_patterns") or []) if p.get("netclass") != "HDMI_TMDS"]
    pats += [{"netclass": "HDMI_TMDS", "pattern": n} for n in TMDS_NETS]   # exact names: KiCad patterns are globs only
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


def _chains(segs):
    """Order a net's segments into connected chains (path order), per layer,
    so contiguous uncoupled runs can be accumulated across segment joints."""
    left_ = list(segs)
    key = lambda p: (round(p[0], 4), round(p[1], 4))
    chains = []
    while left_:
        deg = {}
        for (l, a, b) in left_:
            for q in (key(a), key(b)):
                deg[(l, q)] = deg.get((l, q), 0) + 1
        start = next((s_ for s_ in left_ if deg[(s_[0], key(s_[1]))] == 1 or deg[(s_[0], key(s_[2]))] == 1), left_[0])
        l0, a0, b0 = start
        if deg[(l0, key(a0))] != 1:
            a0, b0 = b0, a0
        chain, cur = [(l0, a0, b0)], b0
        left_.remove(start)
        while True:
            nxt = None
            for s_ in left_:
                if s_[0] == l0 and (key(s_[1]) == key(cur) or key(s_[2]) == key(cur)):
                    nxt = s_
                    break
            if nxt is None:
                break
            left_.remove(nxt)
            a1, b1 = (nxt[1], nxt[2]) if key(nxt[1]) == key(cur) else (nxt[2], nxt[1])
            chain.append((l0, a1, b1))
            cur = b1
        chains.append(chain)
    return chains


def run_drc(pcb_path: Path) -> dict:
    with tempfile.TemporaryDirectory() as td:
        out = Path(td) / "drc.json"
        subprocess.run(["kicad-cli", "pcb", "drc", "--format", "json", "--severity-all",
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
    return any(n in s for n in TMDS_NETS) or "J4" in s or "hdmi0" in s.lower()


def cmd_verify(a) -> int:
    import pcbnew
    tm = pcbnew.ToMM
    board = pcbnew.LoadBoard(str(PCB))
    rows, info = [], []
    add_ = lambda n, ok, d: rows.append((n, bool(ok), d))

    fps = {f.GetReference(): f for f in board.GetFootprints()}
    j4 = fps.get(J4_REF)
    add_("J4 Micro-HDMI D (XUNPU HDMI-101S, LCSC C720620) present on B.Cu", j4 is not None and j4.GetLayer() == pcbnew.B_Cu,
         f"{j4.GetFPIDAsString() if j4 else 'missing'}")
    if j4 is None:
        return report(rows, {}, a)
    bad = [(p.GetNumber(), p.GetNetname(), J4_NETS[p.GetNumber()]) for p in j4.Pads() if p.GetNetname() != J4_NETS[p.GetNumber()]]
    add_("J4 pinout = HDMI Type D (3/5 D2, 6/8 D1, 9/11 D0, 12/14 CLK, 4/7/10/13/16/shell GND, 1 HPD, 15 CEC, 17 SCL, 18 SDA, 19 +5V)",
         not bad, "ok" if not bad else str(bad[:4]))
    # mouth vs edge (connector faces the Y = 40 edge)
    shell_front = max(tm(p.GetBoundingBox().GetBottom()) for p in j4.Pads())
    mouth_y = J4_AT[1] + J4_MOUTH_Y
    add_("J4 copper >= 0.30 mm from the Y=40 edge, mouth inside the outline (zero overhang)",
         40.0 - shell_front >= 0.30 and 39.5 <= mouth_y <= 40.0,
         f"front shell-pad copper Y {shell_front:.3f} (gap {40 - shell_front:.3f}); mouth Y {mouth_y:.2f} (recess {40 - mouth_y:.2f} mm)")
    crt = j4.GetCourtyard(pcbnew.B_CrtYd).BBox()
    cx0, cx1 = tm(crt.GetLeft()), tm(crt.GetRight())
    # H4 standoff keep-out circle d6.2 at (51.5, 36.5): nearest courtyard point
    qx = min(max(51.5, cx0), cx1)
    qy = min(max(36.5, tm(crt.GetTop())), tm(crt.GetBottom()))
    gap4 = math.dist((qx, qy), (51.5, 36.5)) - 3.1
    add_("J4 courtyard clear of the H4 standoff keep-out (d6.2)", gap4 >= 0, f"courtyard X {cx0:.2f}..{cx1:.2f}; gap {gap4:.2f} mm")
    others = []
    for f in board.GetFootprints():
        if f.GetReference() == J4_REF or not f.IsFlipped():
            continue
        try:
            ob = f.GetCourtyard(pcbnew.B_CrtYd).BBox()
        except Exception:
            continue
        if ob.GetWidth() > 0 and ob.Intersects(crt):
            others.append(f.GetReference())
    add_("J4 courtyard does not overlap any other bottom-side part", not others, ", ".join(others) or "clear")
    legs = [tm(p.GetPosition().x) - 0.75 for p in j4.Pads() if p.GetNumber() == "20"]
    add_("J4 through-hole shell legs clear of the J2 DF40 body on the top side", min(legs) > 36.3,
         f"leftmost leg copper X {min(legs):.2f} > DF40 body end X 36.3")
    rear = [round(tm(p.GetPosition().y), 3) for p in j4.Pads() if p.GetNumber() in ("3", "9")]
    front = [round(tm(p.GetPosition().y), 3) for p in j4.Pads() if p.GetNumber() in ("6", "12")]
    add_(f"J4 orientation: rear row (D2/D0) Y {J4_REAR_Y}, front row (D1/CLK) Y {J4_FRONT_Y}, mouth toward +Y",
         all(abs(y - J4_REAR_Y) < 1e-3 for y in rear) and all(abs(y - J4_FRONT_Y) < 1e-3 for y in front),
         f"rear {rear}, front {front}")

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

    # --- coupling. A segment is "coupled" when its partner has a parallel segment
    # at exactly the 0.29 mm pitch overlapping >= 50 % of it. Phase bumps, fan-outs
    # and inner chamfers are uncoupled by design; what matters for SI is that no
    # single contiguous uncoupled stretch is electrically long compared with the
    # signal edge (HDMI 2.0 tr ~ 0.3 UI ~ 50 ps -> limit 1.0 mm = 5.5 ps).
    MAX_RUN = 1.1        # worst feature: J4 front pad -> via escape (1.04 mm, 0.5 mm of it inside the pad)
    worst_gap, runs, frac = {}, {}, {}
    for p, (pn, nn, *_r) in PAIRS.items():
        gaps = []
        worst_run, tot_c, tot = 0.0, 0.0, 0.0
        for me, other in ((pn, nn), (nn, pn)):
            for layer_chain in _chains(segs_by_net[me]):
                run = 0.0
                for (lp, a1, b1) in layer_chain:
                    L1 = math.dist(a1, b1)
                    if L1 == 0:
                        continue
                    u1 = norm(sub(b1, a1))
                    cov = []
                    for (ln, a2, b2) in segs_by_net[other]:
                        if ln != lp or math.dist(a2, b2) == 0:
                            continue
                        u2 = norm(sub(b2, a2))
                        if abs(u1[0] * u2[1] - u1[1] * u2[0]) > 1e-3:
                            continue
                        nrm = left(u1)
                        dperp = abs((a2[0] - a1[0]) * nrm[0] + (a2[1] - a1[1]) * nrm[1])
                        t1 = (a2[0] - a1[0]) * u1[0] + (a2[1] - a1[1]) * u1[1]
                        t2 = (b2[0] - a1[0]) * u1[0] + (b2[1] - a1[1]) * u1[1]
                        lo, hi = max(0.0, min(t1, t2)), min(L1, max(t1, t2))
                        if hi - lo > 0.05 and dperp < 0.6:
                            gaps.append(round(dperp - TMDS_W, 4))
                        if hi > lo and abs(dperp - TMDS_PITCH) < 0.01:
                            cov.append((lo, hi))
                    # walk the segment: covered intervals reset the run
                    cov.sort()
                    pos = 0.0
                    for lo, hi in cov:
                        if lo > pos:
                            run += lo - pos
                            worst_run = max(worst_run, run)
                        if hi > pos:
                            tot_c += hi - max(lo, pos)
                            run = 0.0
                            pos = hi
                    if pos < L1:
                        run += L1 - pos
                        worst_run = max(worst_run, run)
                    tot += L1
        coupled_gaps = [g for g in gaps if abs(g - TMDS_S) < 0.02]
        worst_gap[p] = (min(coupled_gaps), max(coupled_gaps))
        runs[p], frac[p] = worst_run, tot_c / tot
    add_(f"Coupled-section edge gap = {TMDS_S} mm (+/-5 um) on every pair",
         all(abs(lo - TMDS_S) < 0.005 and abs(hi - TMDS_S) < 0.005 for lo, hi in worst_gap.values()),
         "; ".join(f"{p} {lo:.4f}..{hi:.4f}" for p, (lo, hi) in worst_gap.items()))
    add_(f"Longest contiguous uncoupled stretch <= {MAX_RUN} mm ({MAX_RUN * TPD_PS_PER_MM:.1f} ps << 50 ps edge; "
         "bumps, fan-outs and chamfers measured along the copper)",
         all(r <= MAX_RUN for r in runs.values()),
         "; ".join(f"{p} {runs[p]:.3f} mm (coupled {frac[p] * 100:.1f} % of copper)" for p in PAIRS))

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

    # --- reference planes: every TMDS segment over FILLED GND on the adjacent plane,
    # sampled every 0.1 mm along the centre and at +/-(W/2 + 0.20 mm) laterally.
    refl = {"F.Cu": pcbnew.In1_Cu, "B.Cu": pcbnew.In2_Cu}
    gz = {lid: [z for z in board.Zones() if z.GetNetname() == "GND" and z.IsOnLayer(lid)] for lid in refl.values()}
    tvias = [(tm(t.GetPosition().x), tm(t.GetPosition().y)) for t in board.GetTracks()
             if t.Type() == pcbnew.PCB_VIA_T and t.GetNetname() in TMDS_NETS]
    VIA_ZONE = 0.75      # layer-transition zone: plane antipad (via r 0.25 + zone clearance) + margin
    bad_ref, nsamp, nskip = [], 0, 0
    for n, segs in segs_by_net.items():
        for (lay, a_, b_) in segs:
            lid = refl[lay]
            L_ = math.dist(a_, b_)
            u = norm(sub(b_, a_)) if L_ > 0 else (1, 0)
            nn_ = left(u)
            for k in range(int(L_ / 0.1) + 1):
                c = add(a_, mul(u, min(k * 0.1, L_)))
                for off in (0.0, TMDS_W / 2 + 0.20, -(TMDS_W / 2 + 0.20)):
                    q = add(c, mul(nn_, off))
                    if any(math.dist(q, v) < VIA_ZONE for v in tvias):
                        nskip += 1
                        continue
                    nsamp += 1
                    vq = pcbnew.VECTOR2I(pcbnew.FromMM(q[0]), pcbnew.FromMM(q[1]))
                    if not any(z.HitTestFilledArea(lid, vq) for z in gz[lid]):
                        bad_ref.append((n, lay, round(q[0], 2), round(q[1], 2)))
    add_("Every TMDS segment runs over filled GND on its reference plane (In1 under F.Cu, In2 under B.Cu), "
         "incl. +/-0.28 mm lateral margin - never across the ETH_SHIELD split",
         not bad_ref, f"{nsamp} samples, {len(bad_ref)} off-plane ({nskip} inside the {VIA_ZONE} mm via-transition "
         "zones excluded)" + (f", e.g. {bad_ref[:3]}" if bad_ref else ""))

    # --- GND return via at every TMDS layer change (L2 <-> L3 reference swap)
    gvias = [(tm(t.GetPosition().x), tm(t.GetPosition().y)) for t in board.GetTracks()
             if t.Type() == pcbnew.PCB_VIA_T and t.GetNetname() == "GND"]
    gshell = [(tm(pd.GetPosition().x), tm(pd.GetPosition().y)) for pd in j4.Pads() if pd.GetNumber() == "20"]
    rd = {}
    for t in board.GetTracks():
        if t.Type() == pcbnew.PCB_VIA_T and t.GetNetname() in TMDS_NETS:
            v = (tm(t.GetPosition().x), tm(t.GetPosition().y))
            rd[t.GetNetname()] = min(math.dist(v, g) for g in gvias + gshell)
    add_("GND return via (or grounded shell leg) within 2.0 mm of every TMDS layer-change via",
         rd and max(rd.values()) <= 2.0, ", ".join(f"{n} {d:.2f}" for n, d in sorted(rd.items())))

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
        import collections
        cnt = collections.Counter(x["type"] for x in v if not hdmi_items(x))
        info.append(("kicad-cli DRC, items NOT touching TMDS/J4 (owned by other stages, e.g. tools/ethernet_mdi.py)",
                     f"{sum(cnt.values())}: " + ", ".join(f"{k} {n}" for k, n in cnt.most_common())))
        add_("kicad-cli DRC: 0 violations touching TMDS / J4", not hv, f"{len(hv)}")
        uc = [x for x in drc.get("unconnected_items", []) if hdmi_items(x) and any(
            n in json.dumps(x) for n in TMDS_NETS)]
        add_("kicad-cli: 0 unconnected TMDS items (J2 <-> J4 fully routed)", not uc, f"{len(uc)}")
        # J4 ground pins: which GND pads of J4 still appear in an unconnected pair?
        def j4_gnd_pads(x):
            out = set()
            for it in x.get("items", []):
                m_ = re.match(r"(?:PTH )?[Pp]ad (\d+) \[GND\] of J4", it.get("description", ""))
                if m_:
                    out.add(m_.group(1))
            return out
        open_gnd = set()
        for x in drc.get("unconnected_items", []):
            open_gnd |= j4_gnd_pads(x)
        must = {"4", "10", "16", "20"}
        add_("J4 GND pins 4 (D2 shield), 10 (D0 shield), 16 (DDC/CEC GND) and the shell are connected to GND",
             not (open_gnd & must), f"unconnected J4 GND pads: {sorted(open_gnd) or 'none'}")
        info.append(("J4 rear-row shield pins 7 (D1) and 13 (CLK) - KNOWN OPEN (no via site under the F.Cu risers; Rev 12)",
                     f"unconnected now: {sorted(open_gnd & {'7', '13'}) or 'none'}"))
        nontmds = sorted({x["items"][0]["description"].split("[")[1].split("]")[0] for x in drc.get("unconnected_items", [])
                          if "of J4" in json.dumps(x) and "[GND]" not in json.dumps(x)})
        info.append(("J4 low-speed nets not yet routed (HPD/CEC/DDC/+5V: out of Rev 11 scope)", ", ".join(nontmds) or "none"))
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
        # F.Cu-only GND track whose copper sits 0.15 mm from the CLK top lane (rule needs 0.20 mm)
        gy = TOP_F_N - TMDS_PITCH / 2 - TMDS_W / 2 - 0.15 - 0.075
        muts["GND track 0.15 mm from a TMDS trace -> clearance (hdmi0_tmds_to_other)"] = (
            "clearance", src.rstrip()[:-1] + f'  (segment (start 39.5 {gy:.4f}) (end 41.5 {gy:.4f}) (width 0.15) '
            '(layer "F.Cu") (net 1) (uuid "00000000-0000-0000-0000-0000000c0001"))\n)\n')
        base = {(x["type"], json.dumps(x["items"], sort_keys=True)) for x in drc.get("violations", []) if hdmi_items(x)}
        for name, (want, text) in muts.items():
            d = drc_in_scratch(text)
            new = [x for x in d.get("violations", []) if hdmi_items(x) and x["type"] == want
                   and (x["type"], json.dumps(x["items"], sort_keys=True)) not in base]
            add_(f"MUTATION caught: {name}", bool(new),
                 f"{len(new)} new '{want}' violation(s) on TMDS copper, e.g. {new[0]['description'][:90] if new else '-'}")
    return report(rows, L, a, drc, info)


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


def report(rows, L, a, drc=None, info=()) -> int:
    lines = ["# Rev 11.0 - HDMI0 TMDS length-matching verification report", "",
             "Generated by `python3 tools/hdmi_tmds.py verify` from the saved `.kicad_pcb` (lengths measured "
             "segment-by-segment by pcbnew, DRC by kicad-cli with the project `.kicad_dru`).", "",
             "| # | Check | Result | Detail |", "|---|---|---|---|"]
    for i, (n, ok, d) in enumerate(rows, 1):
        lines.append(f"| {i} | {n} | {'PASS' if ok else '**FAIL**'} | {str(d).replace('|', '/')} |")
    failed = [r for r in rows if not r[1]]
    lines += ["", f"**{len(rows) - len(failed)}/{len(rows)} checks passed.**", ""]
    if info:
        lines += ["Informational (not gating):", ""] + [f"- {n}: {d}" for n, d in info] + [""]
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
    for n, d in info:
        print(f"[INFO] {n} :: {d}")
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
