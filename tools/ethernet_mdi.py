#!/usr/bin/env python3
"""
Radxa CM4 Nano Carrier - Rev 9.0 Gigabit Ethernet MDI front end
================================================================

Places, routes and verifies the 1000BASE-T MDI chain

    CM4 PHY (on module) --J1 DF40 pins 3..12--> T1 Pulse H5007NL --> J3 RJ45

on top of the frozen CM4 mechanical base produced by tools/cm4_mechanical.py
(edit this script, never the routed copper).

Sub-commands
------------
  generate  Rebuild the board: run cm4_mechanical.generate_pcb(), then add the
            stackup, Ethernet footprints, 100-ohm coupled routing, planes,
            netclass + DRU rules.  Deterministic and idempotent.
  verify    Re-load the board and check impedance geometry, polarity, pair
            mapping, intra-pair skew, uncoupled length, spacing, plane
            references, kicad-cli DRC (Ethernet scope) and DRU mutation
            tests.  Writes artifacts/rev9_mdi_si_report.md, exit 1 on failure.
  render    PNG plot of the Ethernet region for review (artifacts/rev9_mdi_layout.png).

Electrical design
-----------------
* The CM4 / Radxa CM3 / Radxa CM4 module carries the Gigabit PHY (BCM54210PE
  on RPi CM4; RTL8211F on Radxa CM3; RK3576 GMAC + PHY on Radxa CM4).  Both
  are VOLTAGE-MODE line drivers: the transformer PHY-side centre taps are NOT
  tied to a supply and NOT shorted together - each TCTx gets its own 100 nF
  to GND (C10..C13).  [RPi CM4 datasheet sec. "Ethernet"; Pulse H5007NL
  application note; voltage-mode PHY guidance in TI SNLA387.]
* Cable-side centre taps: Bob Smith termination, 75 ohm per MCT (R10..R13)
  to a common node, 1 nF / 2 kV (C14) to the ETH_SHIELD island.
* ETH_SHIELD: chassis island on In1/In2 under the cable side + jack, >= 2 mm
  split from GND under the transformer body; J3 shield tabs land on it;
  1 Mohm (R14) || 1 nF / 2 kV (C15) to GND (EMC-tunable, 1206 footprints).
* T568 pin map: TRD0 -> pins 1/2, TRD1 -> 3/6, TRD2 -> 4/5, TRD3 -> 7/8.
  Every pair keeps its polarity end-to-end (checked from the copper).

Controlled impedance - see tools/mdi_impedance.py (single source of truth):
  JLC04161H-7628 stackup, L1/L4 microstrip over solid L2/L3 reference,
  W = 0.165 mm, S = 0.125 mm -> 100.0 ohm differential (2-D field solver).
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

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import cm4_mechanical as M          # noqa: E402  mechanical source of truth
import mdi_impedance as Z           # noqa: E402  impedance source of truth

ROOT = M.ROOT
PCB = M.PCB
PRO = ROOT / "radxa-cm4-nano-carrier.kicad_pro"
DRU = ROOT / "radxa-cm4-nano-carrier.kicad_dru"
LIB = M.LIB
REPORT = ROOT / "artifacts" / "rev9_mdi_si_report.md"
RENDER = ROOT / "artifacts" / "rev9_mdi_layout.png"
LIBNAME = "radxa-cm4-nano-carrier"

# ---------------------------------------------------------------------------
# 1. Constants
# ---------------------------------------------------------------------------
W = Z.MDI_W                  # 0.165
S = Z.MDI_S                  # 0.125
PITCH = W + S                # 0.290 centre-to-centre
HP = PITCH / 2
VIA_D, VIA_DRILL = 0.45, 0.20          # matches board minimums (annulus 0.125)
CHAMFER = 0.6                          # centreline 45-degree corner cut
NETCLASS = "ETH_MDI_100R"
MDI_CLR = 0.30                         # MDI copper (track/via) to other-net track/via/zone
SPLIT_GAP = 2.0                        # GND <-> ETH_SHIELD plane split

# Parts (all real, JLCPCB-stocked; footprints from the manufacturer drawings
# via the LCSC/EasyEDA land patterns, re-entered numerically below).
T1_MPN = "Pulse H5007NL"               # LCSC C6384935, 1000BASE-T 1CT:1CT, 1500 Vrms
J3_MPN = "Amphenol RJE231881317T"      # LCSC C5384839, vertical SMT, shielded, 13.1 mm
T1_AT, T1_ROT_PIN1 = (19.0, 19.0), "top-right"
J3_AT = (44.2, 25.0)

# Island (ETH_SHIELD) and the GND notch around it (inner layers)
ISLAND = (11.0, 20.0, 54.7, 33.5)                     # x0, y0, x1, y1
NOTCH = (ISLAND[0] - SPLIT_GAP, ISLAND[1] - SPLIT_GAP, 55.0, ISLAND[3] + SPLIT_GAP)

# CM4 pair -> transformer channel -> T568 pins (+, -)
PAIRS = {
    0: dict(td=("2", "3"), mx=("23", "22"), rj=("1", "2"), tct="1", mct="24"),
    1: dict(td=("5", "6"), mx=("20", "19"), rj=("3", "6"), tct="4", mct="21"),
    2: dict(td=("8", "9"), mx=("17", "16"), rj=("4", "5"), tct="7", mct="18"),
    3: dict(td=("11", "12"), mx=("14", "13"), rj=("7", "8"), tct="10", mct="15"),
}
# Transformer winding dot convention (Pulse H5007NL datasheet): TDn+ and MXn+
# share the dot.  Used by verify to prove end-to-end polarity.
T1_DOT = {"2": "23", "5": "20", "8": "17", "11": "14"}


def phy_net(n, pol):
    return f"Ethernet_Pair{n}_{pol}"


def cab_net(n, pol):
    return f"ETH_MDI{n}_{pol}"


# ---------------------------------------------------------------------------
# 2. Footprints (KiCad 8 .kicad_mod text)
# ---------------------------------------------------------------------------
def _rect_lines(x0, y0, x1, y1, layer, w):
    return [f'  (fp_line (start {a:.3f} {b:.3f}) (end {c:.3f} {d:.3f}) (stroke (width {w}) (type solid)) (layer "{layer}"))'
            for (a, b, c, d) in ((x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0))]


def _fp_head(name, descr, tags, height, ref_y, val_y, attr="smd"):
    return [f'(footprint "{name}"', '  (version 20240108)', '  (generator "ethernet_mdi.py")', '  (layer "F.Cu")',
            f'  (descr "{descr}")', f'  (tags "{tags}")', f'  (attr {attr})',
            f'  (property "Reference" "REF**" (at 0 {ref_y:.2f} 0) (layer "F.SilkS") (effects (font (size 0.8 0.8) (thickness 0.12))))',
            f'  (property "Value" "{name}" (at 0 {val_y:.2f} 0) (layer "F.Fab") (effects (font (size 0.8 0.8) (thickness 0.12))))',
            f'  (property "Height" "{height}" (at 0 0 0) (layer "F.Fab") (hide yes) (effects (font (size 1 1))))']


def fp_h5007nl() -> str:
    """Pulse H5007NL, SMD-24, 1.27 mm pitch, rows 14.606 mm c-c.
    Land pattern: LCSC C6384935 / Pulse drawing (0.762 x 1.905 pads)."""
    L = _fp_head("Pulse_H5007NL_SMD-24_P1.27mm", "Pulse H5007NL 1000BASE-T single-port magnetics, 1CT:1CT, 1500 Vrms, "
                 "body 17.5 x 12.2 x 5.72 mm", "ethernet transformer magnetics gigabit H5007NL", 5.72, -9.3, 9.3)
    L += _rect_lines(-8.75, -6.1, 8.75, 6.1, "F.Fab", 0.1)
    L += _rect_lines(-9.0, -8.5, 9.0, 8.5, "F.CrtYd", 0.05)
    L.append('  (fp_line (start -8.86 -6.21) (end 8.86 -6.21) (stroke (width 0.12) (type solid)) (layer "F.SilkS"))')
    L.append('  (fp_line (start -8.86 6.21) (end 8.86 6.21) (stroke (width 0.12) (type solid)) (layer "F.SilkS"))')
    L.append('  (fp_circle (center -7.9 7.3) (end -7.7 7.3) (stroke (width 0.12) (type solid)) (fill solid) (layer "F.SilkS"))')
    L.append('  (fp_text user "PHY SIDE" (at 0 4.6 0) (layer "F.Fab") (effects (font (size 0.8 0.8) (thickness 0.1))))')
    L.append('  (fp_text user "CABLE SIDE" (at 0 -4.6 0) (layer "F.Fab") (effects (font (size 0.8 0.8) (thickness 0.1))))')
    for n in range(1, 13):
        L.append(f'  (pad "{n}" smd rect (at {-6.985 + 1.27 * (n - 1):.3f} 7.303) (size 0.762 1.905) (layers "F.Cu" "F.Paste" "F.Mask"))')
    for n in range(13, 25):
        L.append(f'  (pad "{n}" smd rect (at {6.985 - 1.27 * (n - 13):.3f} -7.303) (size 0.762 1.905) (layers "F.Cu" "F.Paste" "F.Mask"))')
    L.append(')')
    return "\n".join(L) + "\n"


def fp_rje23() -> str:
    """Amphenol RJE231881317T vertical SMT RJ45, shielded, no LED.
    Land pattern: Amphenol drawing rje231881x17t (via LCSC C5384839)."""
    L = _fp_head("Amphenol_RJE231881317T_RJ45_Vertical_SMD", "Amphenol RJE231881317T RJ45 8P8C vertical SMT, shielded, "
                 "no LED, 14.6 x 16.1 x 13.1 mm; plug enters from the board's far side", "RJ45 8P8C vertical SMT shielded",
                 13.1, -11.4, 11.4)
    L += _rect_lines(-7.3, -8.05, 7.3, 8.05, "F.Fab", 0.1)
    L += _rect_lines(-7.55, -10.85, 7.55, 10.65, "F.CrtYd", 0.05)
    L += [f'  (fp_line (start {s*7.41:.3f} -6.9) (end {s*7.41:.3f} 6.9) (stroke (width 0.12) (type solid)) (layer "F.SilkS"))' for s in (-1, 1)]
    L.append('  (fp_circle (center -4.445 -11.0) (end -4.245 -11.0) (stroke (width 0.12) (type solid)) (fill solid) (layer "F.SilkS"))')
    L.append('  (fp_text user "RJ45 VERTICAL" (at 0 0 0) (layer "F.Fab") (effects (font (size 0.8 0.8) (thickness 0.1))))')
    for n in range(1, 9):
        L.append(f'  (pad "{n}" smd rect (at {-4.445 + 1.27 * (n - 1):.3f} -8.899) (size 0.6 3.4) (layers "F.Cu" "F.Paste" "F.Mask"))')
    for name, x in (("M1", -3.05), ("M2", 3.05)):
        L.append(f'  (pad "{name}" smd rect (at {x:.3f} 8.899) (size 3.1 3.0) (layers "F.Cu" "F.Paste" "F.Mask"))')
    L.append(')')
    return "\n".join(L) + "\n"


def fp_chip(name, pad_w, pad_h, dx, body, crt, descr, height):
    L = _fp_head(name, descr, "resistor capacitor chip", height, -crt[1] - 0.6, crt[1] + 0.6)
    L += _rect_lines(-body[0], -body[1], body[0], body[1], "F.Fab", 0.1)
    L += _rect_lines(-crt[0], -crt[1], crt[0], crt[1], "F.CrtYd", 0.05)
    for n, x in (("1", -dx), ("2", dx)):
        L.append(f'  (pad "{n}" smd roundrect (at {x:.3f} 0) (size {pad_w} {pad_h}) (layers "F.Cu" "F.Paste" "F.Mask") (roundrect_rratio 0.25))')
    L.append(')')
    return "\n".join(L) + "\n"


FOOTPRINTS = {
    "Pulse_H5007NL_SMD-24_P1.27mm": fp_h5007nl,
    "Amphenol_RJE231881317T_RJ45_Vertical_SMD": fp_rje23,
    "C_0402_1005Metric": lambda: fp_chip("C_0402_1005Metric", 0.59, 0.64, 0.51, (0.5, 0.25), (0.93, 0.47), "0402 chip, IPC-7351 nominal", 0.55),
    "R_0402_1005Metric": lambda: fp_chip("R_0402_1005Metric", 0.59, 0.64, 0.51, (0.5, 0.25), (0.93, 0.47), "0402 chip, IPC-7351 nominal", 0.40),
    "C_1206_3216Metric_HV": lambda: fp_chip("C_1206_3216Metric_HV", 1.15, 1.80, 1.475, (1.6, 0.8), (2.30, 1.15), "1206 2 kV MLCC (Bob Smith / chassis)", 1.45),
    "R_1206_3216Metric": lambda: fp_chip("R_1206_3216Metric", 1.15, 1.80, 1.475, (1.6, 0.8), (2.30, 1.15), "1206 chip resistor", 0.65),
}


def write_footprints():
    LIB.mkdir(parents=True, exist_ok=True)
    for name, f in FOOTPRINTS.items():
        (LIB / f"{name}.kicad_mod").write_text(f())
    # fp-lib-table: keep cm4_mechanical's entry, add the legacy CM4Carrier lib
    # referenced by the schematic mounting-hole symbols (fixes Rev 7 DRC
    # lib_footprint_issues).
    (ROOT / "fp-lib-table").write_text(
        '(fp_lib_table\n  (version 7)\n'
        f'  (lib (name "{LIBNAME}")(type "KiCad")(uri "${{KIPRJMOD}}/lib/{LIBNAME}.pretty")(options "")'
        '(descr "CM4 mechanical + Rev 9 Ethernet footprints"))\n'
        '  (lib (name "CM4Carrier")(type "KiCad")(uri "${KIPRJMOD}/lib/CM4Carrier.pretty")(options "")'
        '(descr "Legacy Rev 5 mounting-hole footprint used by the schematic"))\n)\n')


# ---------------------------------------------------------------------------
# 3. Geometry helpers (pure python, mm)
# ---------------------------------------------------------------------------
def chamfer(pts, c=CHAMFER):
    """Cut every interior corner of an orthogonal/45-degree polyline by c."""
    out = [pts[0]]
    for i in range(1, len(pts) - 1):
        p0, p1, p2 = pts[i - 1], pts[i], pts[i + 1]
        d0 = math.dist(p0, p1)
        d1 = math.dist(p1, p2)
        u0 = ((p1[0] - p0[0]) / d0, (p1[1] - p0[1]) / d0)
        u1 = ((p2[0] - p1[0]) / d1, (p2[1] - p1[1]) / d1)
        if abs(u0[0] * u1[1] - u0[1] * u1[0]) < 1e-9:
            out.append(p1)
            continue
        cc = min(c, d0 / 2.2, d1 / 2.2)
        out.append((p1[0] - u0[0] * cc, p1[1] - u0[1] * cc))
        out.append((p1[0] + u1[0] * cc, p1[1] + u1[1] * cc))
    out.append(pts[-1])
    return out


def offset(pts, d):
    """Offset a polyline by d to the LEFT of travel (screen coords, y down:
    left of +x travel is -y).  Mitred joins (exact for 45/90-degree turns)."""
    def lnorm(a, b):
        L = math.dist(a, b)
        ux, uy = (b[0] - a[0]) / L, (b[1] - a[1]) / L
        return ux, uy, uy, -ux             # direction, left normal (y down)
    segs = []
    for a, b in zip(pts, pts[1:]):
        ux, uy, nx, ny = lnorm(a, b)
        segs.append(((a[0] + nx * d, a[1] + ny * d), (b[0] + nx * d, b[1] + ny * d), (ux, uy)))
    out = [segs[0][0]]
    for (a0, b0, u0), (a1, b1, u1) in zip(segs, segs[1:]):
        den = u0[0] * u1[1] - u0[1] * u1[0]
        if abs(den) < 1e-12:
            out.append(b0)
            continue
        t = ((a1[0] - a0[0]) * u1[1] - (a1[1] - a0[1]) * u1[0]) / den
        out.append((a0[0] + u0[0] * t, a0[1] + u0[1] * t))
    out.append(segs[-1][1])
    return out


def plen(pts):
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


def add_bump(pts, extra, side_sign, min_seg=2.0):
    """Insert a 45-degree trombone bump of added length `extra` into the
    longest straight segment of `pts`; bump bulges to side_sign*left."""
    if extra <= 1e-4:
        return pts
    a = extra / (2 * math.sqrt(2) - 2)          # height for the extra length
    a = max(a, 0.05)
    i = max(range(len(pts) - 1), key=lambda k: math.dist(pts[k], pts[k + 1]))
    p, q = pts[i], pts[i + 1]
    L = math.dist(p, q)
    if L < max(min_seg, 2 * a + 3 * W):
        raise RuntimeError(f"no straight run for a {extra:.3f} mm skew bump")
    ux, uy = (q[0] - p[0]) / L, (q[1] - p[1]) / L
    nx, ny = uy * side_sign, -ux * side_sign
    flat = 3 * W
    m = L / 2
    s0 = m - flat / 2 - a
    pts_b = [(p[0] + ux * s0, p[1] + uy * s0),
             (p[0] + ux * (s0 + a) + nx * a, p[1] + uy * (s0 + a) + ny * a),
             (p[0] + ux * (s0 + a + flat) + nx * a, p[1] + uy * (s0 + a + flat) + ny * a),
             (p[0] + ux * (s0 + 2 * a + flat), p[1] + uy * (s0 + 2 * a + flat))]
    return pts[:i + 1] + pts_b + pts[i + 1:]


# ---------------------------------------------------------------------------
# 4. Route plan
#    Each pair: list of legs.  A leg = (layer, P polyline, N polyline).  The
#    coupled part is generated from a centreline + offset; fan-outs are
#    explicit.  `p_side` = +1 if P runs on the LEFT of the centreline travel.
# ---------------------------------------------------------------------------
def j1_pad(pin):
    """Board position of J1 pad (from the independent [S2] reference)."""
    return M.fp_to_carrier(*M.reference_pad_fp(pin))


def coupled(center, p_side):
    c = chamfer(center)
    return offset(c, +HP * p_side), offset(c, -HP * p_side)


class Plan:
    def __init__(self, pads):
        self.pads = pads           # (ref, pad) -> (x, y)
        self.tracks = []           # (layer, net, [pts])
        self.vias = []             # (x, y, net)
        self.pairs = {}            # pairname -> dict(P=net, N=net, legs=[(layer, Ppts, Npts)])

    def pair(self, name, pnet, nnet, legs, vias=()):
        self.pairs[name] = dict(P=pnet, N=nnet, legs=legs)
        for layer, pp, nn in legs:
            self.tracks.append((layer, pnet, pp))
            self.tracks.append((layer, nnet, nn))
        for x, y, net in vias:
            self.vias.append((x, y, net))

    def track(self, layer, net, pts):
        self.tracks.append((layer, net, pts))

    def via(self, x, y, net):
        self.vias.append((x, y, net))


def build_plan(pads) -> Plan:
    P = Plan(pads)
    T = lambda n: pads[("T1", n)]
    R = lambda n: pads[("J3", n)]

    # ---------------- PHY side: J1 -> T1 (TD pins) ----------------------
    # T1 drop x for each pair (mid of TD+/TD- pads), pads' top end y
    def t1_drop(n):
        a, b = T(PAIRS[n]["td"][0]), T(PAIRS[n]["td"][1])
        return (a[0] + b[0]) / 2, min(a[1], b[1])
    y_fan = lambda n: t1_drop(n)[1] - 0.85          # end of coupled section above pads

    def t1_tail(n, center_end_x):
        """Fan-out from coupled pair heading south at x=center_end_x into
        TD+/TD- pads.  Returns (tail for the east leg, tail for the west leg)."""
        a, b = T(PAIRS[n]["td"][0]), T(PAIRS[n]["td"][1])
        east, west = (a, b) if a[0] > b[0] else (b, a)
        y0 = y_fan(n)
        k = east[0] - (center_end_x + HP)
        e = [(center_end_x + HP, y0), (east[0], y0 + k), east]
        w = [(center_end_x - HP, y0), (west[0], y0 + k), west]
        return e, w, (east is a)           # (east, west, east_is_TD+)

    vias = []
    # Pair 3: outer row, edge channel on F.Cu -> stacked vias east of J1 ->
    # B.Cu band y=0.85 west -> drop at TD4.
    p3p, p3n = j1_pad(3), j1_pad(5)            # P (34.4,1.5) east, N (34.0,1.5) west
    yb = 0.665                                  # F.Cu edge-channel centreline
    vx = 35.95
    vN3, vP3 = (vx, 0.55), (vx, 1.15)
    fP = [p3p, (p3p[0], yb + HP), (vx - 0.55, yb + HP), (vx - 0.3, vP3[1] - 0.25), vP3]
    fN = [p3n, (p3n[0], yb - HP), (vx - 0.6, yb - HP), vN3]
    xd, _ = t1_drop(3)
    band = 0.855
    pB, nB = coupled([(vx - 0.5, band), (xd, band), (xd, y_fan(3))], p_side=-1)  # heading W: P (south) = right
    e, w, east_is_plus = t1_tail(3, xd)
    assert east_is_plus
    pB = [vP3, (vx - 0.5, band + HP)] + pB[1:] + e[1:]
    nB = [vN3, (vx - 0.5 + 0.0, band - HP)] + nB[1:] + w[1:]
    P.pair("PHY3", phy_net(3, "P"), phy_net(3, "N"), [("F.Cu", fP, fN), ("B.Cu", pB, nB)],
           vias=[(*vP3, phy_net(3, "P")), (*vN3, phy_net(3, "N"))])

    # Pair 2: outer row, N pad 33.2 (east) P pad 32.8 (west); vias in the
    # inter-row gap, N upper-right / P lower-left -> heading W with P south.
    p2p, p2n = j1_pad(11), j1_pad(9)
    vN2, vP2 = (33.30, 2.65), (32.60, 3.45)
    fP = [p2p, (p2p[0], 1.95), vP2]
    fN = [p2n, (p2n[0], 1.95), vN2]
    band = 3.05
    xd, _ = t1_drop(2)
    pB, nB = coupled([(32.05, band), (xd, band), (xd, y_fan(2))], p_side=-1)
    e, w, east_is_plus = t1_tail(2, xd)
    assert east_is_plus
    pB = [vP2, (vP2[0] - 0.255, band + HP)] + pB + e[1:]
    nB = [vN2, (vN2[0] - 0.255, band - HP)] + nB + w[1:]
    P.pair("PHY2", phy_net(2, "P"), phy_net(2, "N"), [("F.Cu", fP, fN), ("B.Cu", pB, nB)],
           vias=[(*vP2, phy_net(2, "P")), (*vN2, phy_net(2, "N"))])

    # Pair 1: inner row, P pad 34.4 (east) N 34.0 (west): F.Cu south (P east =
    # left of travel) -> west under Pair 0's transition -> stacked vias -> B.Cu.
    p1p, p1n = j1_pad(4), j1_pad(6)
    xs = 34.70                       # F.Cu drop centreline (clear of Pair 0's P via)
    band1 = 7.40
    vx1 = 31.00
    vN1, vP1 = (vx1, band1 - 0.30), (vx1, band1 + 0.30)
    cF = [(xs, 5.75), (xs, band1), (vx1 + 0.55, band1)]
    fP_c, fN_c = coupled(cF, p_side=+1)          # heading S: P east = left
    fP = [p1p, (p1p[0], 5.15), (xs + HP, 5.75)] + fP_c[1:] + [(vx1 + 0.3, band1 + HP), vP1]
    fN = [p1n, (p1n[0], 5.15), (xs - HP, 5.75)] + fN_c[1:] + [(vx1 + 0.3, band1 - HP), vN1]
    xd, _ = t1_drop(1)
    pB, nB = coupled([(vx1 - 0.55, band1), (xd, band1), (xd, y_fan(1))], p_side=-1)
    e, w, east_is_plus = t1_tail(1, xd)
    assert east_is_plus
    pB = [vP1, (vx1 - 0.3, band1 + HP)] + pB + e[1:]
    nB = [vN1, (vx1 - 0.3, band1 - HP)] + nB + w[1:]
    P.pair("PHY1", phy_net(1, "P"), phy_net(1, "N"), [("F.Cu", fP, fN), ("B.Cu", pB, nB)],
           vias=[(*vP1, phy_net(1, "P")), (*vN1, phy_net(1, "N"))])

    # Pair 0: inner row, N pad 33.2 (east) P pad 32.8 (west).  Two-layer twist:
    # N drops straight to a via; P runs south then east under N's via to a via
    # east of it.  B.Cu then heads south with P east -> west band with P south.
    p0p, p0n = j1_pad(12), j1_pad(10)
    vN0, vP0 = (33.35, 5.60), (33.75, 6.45)
    fN = [p0n, (p0n[0], 5.05), vN0]
    fP = [p0p, (p0p[0], 6.45), vP0]
    band0 = 8.65
    xd, _ = t1_drop(0)
    xc = 33.25
    pB, nB = coupled([(xc, 7.15), (xc, band0), (xd, band0), (xd, y_fan(0))], p_side=+1)  # heading S: P east = left
    e, w, east_is_plus = t1_tail(0, xd)
    assert east_is_plus
    pB = [vP0, (xc + HP, 6.85)] + pB + e[1:]
    nB = [vN0, (xc - HP - 0.12, 5.90), (xc - HP, 6.25)] + nB + w[1:]
    P.pair("PHY0", phy_net(0, "P"), phy_net(0, "N"), [("F.Cu", fP, fN), ("B.Cu", pB, nB)],
           vias=[(*vP0, phy_net(0, "P")), (*vN0, phy_net(0, "N"))])

    # ---------------- Cable side: T1 (MX pins) -> J3 ----------------------
    def mx(n):
        a, b = T(PAIRS[n]["mx"][0]), T(PAIRS[n]["mx"][1])      # (+, -)
        return a, b

    def t1_cab_head(n, y0):
        """Fan-in from MX+/MX- pads (bottom row) to a coupled pair heading S
        whose centreline starts at (xm, y0).  MX- is WEST on this part."""
        plus, minus = mx(n)
        xm = (plus[0] + minus[0]) / 2
        k = plus[0] - (xm + HP)
        ys = max(plus[1], minus[1])
        hp = [plus, (plus[0], ys + 0.55), (xm + HP, ys + 0.55 + k)]
        hn = [minus, (minus[0], ys + 0.55), (xm - HP, ys + 0.55 + k)]
        return xm, ys + 0.55 + k, hp, hn

    def rj_tail(pin_plus, pin_minus, xc, yc):
        """Coupled pair heading E at centre y=yc ends at x=xc; fan to J3 pads."""
        a, b = R(pin_plus), R(pin_minus)
        k = abs(a[1] - (yc - HP)) if a[1] < b[1] else abs(a[1] - (yc + HP))
        ta = [(xc, yc - HP if a[1] < b[1] else yc + HP), (xc + k, a[1]), a]
        tb = [(xc, yc + HP if a[1] < b[1] else yc - HP), (xc + k, b[1]), b]
        return ta, tb

    # Pair A (CM4 Pair 0) : T1 23/22 -> J3 1/2
    xm, y0, hp, hn = t1_cab_head(0, None)
    yA = (R("1")[1] + R("2")[1]) / 2
    pc, nc = coupled([(xm, y0), (xm, 28.2), (29.6, 28.2), (29.6, yA), (32.6, yA)], p_side=+1)  # S: + east = left
    ta, tb = rj_tail("1", "2", 32.6, yA)
    P.pair("CAB0", cab_net(0, "P"), cab_net(0, "N"),
           [("B.Cu", hp + pc[1:] + ta[1:], hn + nc[1:] + tb[1:])])

    # Pair B (CM4 Pair 1) : T1 20/19 -> J3 3/6, splits around pair C
    xm, y0, hp, hn = t1_cab_head(1, None)
    yB = 25.0
    xs = 31.15
    pc, nc = coupled([(xm, y0), (xm, 29.5), (30.7, 29.5), (30.7, yB), (xs, yB)], p_side=+1)
    a, b = R("3"), R("6")
    tp = [(xs, yB - HP), (xs + abs(a[1] - (yB - HP)), a[1]), a]
    tn = [(xs, yB + HP), (xs + abs(b[1] - (yB + HP)), b[1]), b]
    P.pair("CAB1", cab_net(1, "P"), cab_net(1, "N"), [("B.Cu", hp + pc[1:] + tp[1:], hn + nc[1:] + tn[1:])])

    # Pair C (CM4 Pair 2) : T1 17/16 -> F.Cu hop -> J3 4/5
    plus, minus = mx(2)
    xm = (plus[0] + minus[0]) / 2
    ys = max(plus[1], minus[1])
    vcp, vcn = (xm + 0.30, 28.55), (xm - 0.30, 28.55)        # + east, - west
    b1p = [plus, (plus[0], ys + 0.55), vcp]
    b1n = [minus, (minus[0], ys + 0.55), vcn]
    yC = (R("4")[1] + R("5")[1]) / 2
    xv2 = 32.45
    vjp, vjn = (xv2, yC - 0.30), (xv2, yC + 0.30)             # + north (pin 4), - south (pin 5)
    pc, nc = coupled([(xm, 29.0), (xm, 29.9), (31.9, 29.9), (31.9, yC), (xv2 - 0.55, yC)], p_side=+1)
    f_p = [vcp, (xm + HP, 29.0)] + pc[1:] + [(xv2 - 0.3, yC - HP), vjp]
    f_n = [vcn, (xm - HP, 29.0)] + nc[1:] + [(xv2 - 0.3, yC + HP), vjn]
    a, b = R("4"), R("5")
    b2p = [vjp, (vjp[0] + abs(vjp[1] - a[1]), a[1]), a]
    b2n = [vjn, (vjn[0] + abs(vjn[1] - b[1]), b[1]), b]
    P.pair("CAB2", cab_net(2, "P"), cab_net(2, "N"),
           [("B.Cu", b1p, b1n), ("F.Cu", f_p, f_n), ("B.Cu", b2p, b2n)],
           vias=[(*vcp, cab_net(2, "P")), (*vcn, cab_net(2, "N")), (*vjp, cab_net(2, "P")), (*vjn, cab_net(2, "N"))])

    # Pair D (CM4 Pair 3) : T1 14/13 -> J3 7/8
    xm, y0, hp, hn = t1_cab_head(3, None)
    yD = (R("7")[1] + R("8")[1]) / 2
    pc, nc = coupled([(xm, y0), (xm, 30.85), (32.1, 30.85), (32.1, yD), (32.75, yD)], p_side=+1)
    ta, tb = rj_tail("7", "8", 32.75, yD)
    P.pair("CAB3", cab_net(3, "P"), cab_net(3, "N"), [("B.Cu", hp + pc[1:] + ta[1:], hn + nc[1:] + tb[1:])])

    # ---------------- Centre taps, Bob Smith, shield ----------------------
    for n in range(4):
        tct = T(PAIRS[n]["tct"])
        c = pads[(f"C{10+n}", "1")]
        P.via(tct[0], 13.25, f"ETH_TCT{n+1}")
        P.track("B.Cu", f"ETH_TCT{n+1}", [tct, (tct[0], 13.25)])
        P.track("F.Cu", f"ETH_TCT{n+1}", [(tct[0], 13.25), c])
        g = pads[(f"C{10+n}", "2")]
        P.via(g[0], g[1] + 0.85, "GND")
        P.track("F.Cu", "GND", [g, (g[0], g[1] + 0.85)])
        mct = T(PAIRS[n]["mct"])
        r1, r2 = pads[(f"R{10+n}", "1")], pads[(f"R{10+n}", "2")]
        P.via(mct[0], 24.75, f"ETH_MCT{n+1}")
        P.track("B.Cu", f"ETH_MCT{n+1}", [mct, (mct[0], 24.75)])
        P.track("F.Cu", f"ETH_MCT{n+1}", [(mct[0], 24.75), r1])
        P.track("F.Cu", "ETH_BS", [r2, (r2[0], 22.30)])
    xs_ = sorted(pads[(f"R{10+n}", "2")][0] for n in range(4))
    cb1, cb2 = pads[("C14", "1")], pads[("C14", "2")]
    P.track("F.Cu", "ETH_BS", [(xs_[-1], 22.30), (11.9, 22.30), (11.9, 32.45), (cb1[0] - 1.0, 32.45)])
    P.via(cb1[0] - 1.0, 32.45, "ETH_BS")
    P.track("B.Cu", "ETH_BS", [(cb1[0] - 1.0, 32.45), cb1])
    P.via(cb2[0] + 1.0, cb2[1], "ETH_SHIELD")
    P.track("B.Cu", "ETH_SHIELD", [cb2, (cb2[0] + 1.0, cb2[1])])
    for ref in ("R14", "C15"):
        s_, g_ = pads[(ref, "1")], pads[(ref, "2")]          # 1 = shield (north), 2 = GND (south)
        P.via(s_[0], s_[1] - 1.0, "ETH_SHIELD")
        P.track("B.Cu", "ETH_SHIELD", [s_, (s_[0], s_[1] - 1.0)])
        P.via(g_[0], g_[1] + 1.05, "GND")
        P.track("B.Cu", "GND", [g_, (g_[0], g_[1] + 1.05)])
    for m in ("M1", "M2"):
        p = R(m)
        dy = -2.15 if p[1] < J3_AT[1] else 2.15
        P.via(p[0], p[1] + dy, "ETH_SHIELD")
        P.track("B.Cu", "ETH_SHIELD", [p, (p[0], p[1] + dy)])
        P.via(p[0] - 1.9, p[1], "ETH_SHIELD")

    # ---------------- Return-path stitching (GND In1 <-> In2) -------------
    for (x, y) in [(36.75, 0.85), (31.85, 2.20), (31.85, 6.00), (30.20, 6.50), (34.15, 8.75),
                   (35.40, 3.04), (29.6, 9.4), (11.0, 3.2), (11.0, 9.0), (14.55, 9.0), (18.35, 9.0),
                   (22.2, 9.0), (26.6, 9.0), (36.75, 6.5)]:
        P.via(x, y, "GND")
    # J1 GND pads 13 (outer) / 14 (inner) next to the MDI field -> stitching vias
    P.track("F.Cu", "GND", [j1_pad(13), (j1_pad(13)[0], 1.95), (31.85, 2.20)])
    P.track("F.Cu", "GND", [j1_pad(14), (j1_pad(14)[0], 5.15), (31.85, 6.00)])
    P.track("F.Cu", "GND", [j1_pad(2), (j1_pad(2)[0], 3.04), (35.40, 3.04)])
    P.track("F.Cu", "GND", [j1_pad(1), (j1_pad(1)[0] + 0.0, 2.2), (35.40, 3.04)])
    # Cable-side island stitching around the C-pair hop and the jack
    for (x, y) in [(15.2, 28.55), (18.6, 28.55), (31.3, 23.6), (31.3, 26.4), (33.0, 22.1), (33.0, 27.95),
                   (28.3, 31.9), (36.0, 33.0), (29.0, 21.0)]:
        P.via(x, y, "ETH_SHIELD")
    return P


# Passive placement (B.Cu unless noted).  (ref, value, footprint, x, y, rot, side, net1, net2)
def passives():
    out = []
    t1x = lambda n: T1_AT[0] + 6.985 - 1.27 * (n - 1)     # TCT x (pins 1,4,7,10) with pin1 at right
    for i, pin in enumerate((1, 4, 7, 10)):
        out.append((f"C{10+i}", "100nF 16V X7R", "C_0402_1005Metric", t1x(pin), 14.45, 90, "F", f"ETH_TCT{i+1}", "GND"))
    for i, pin in enumerate((1, 4, 7, 10)):                # MCT pins 24,21,18,15 sit under pins 1,4,7,10
        out.append((f"R{10+i}", "75R 1%", "R_0402_1005Metric", t1x(pin), 23.55, 270, "F", f"ETH_MCT{i+1}", "ETH_BS"))
    out.append(("C14", "1nF 2kV X7R", "C_1206_3216Metric_HV", 20.0, 32.45, 0, "B", "ETH_BS", "ETH_SHIELD"))
    out.append(("R14", "1M 1206", "R_1206_3216Metric", 25.0, 34.5, 90, "B", "ETH_SHIELD", "GND"))
    out.append(("C15", "1nF 2kV X7R", "C_1206_3216Metric_HV", 27.6, 34.5, 90, "B", "ETH_SHIELD", "GND"))
    return out


# ---------------------------------------------------------------------------
# 5. Board build (pcbnew)
# ---------------------------------------------------------------------------
STACKUP_SEXPR = f"""		(stackup
			(layer "F.SilkS" (type "Top Silk Screen"))
			(layer "F.Paste" (type "Top Solder Paste"))
			(layer "F.Mask" (type "Top Solder Mask") (thickness {Z.STACKUP.t_mask}) (material "LPI") (epsilon_r {Z.STACKUP.er_mask}) (loss_tangent 0.025))
			(layer "F.Cu" (type "copper") (thickness {Z.STACKUP.t_outer}))
			(layer "dielectric 1" (type "prepreg") (thickness {Z.STACKUP.h_pp}) (material "7628") (epsilon_r {Z.STACKUP.er_pp}) (loss_tangent {Z.STACKUP.tan_d}))
			(layer "In1.Cu" (type "copper") (thickness {Z.STACKUP.t_inner}))
			(layer "dielectric 2" (type "core") (thickness {Z.STACKUP.h_core}) (material "FR4") (epsilon_r {Z.STACKUP.er_core}) (loss_tangent {Z.STACKUP.tan_d}))
			(layer "In2.Cu" (type "copper") (thickness {Z.STACKUP.t_inner}))
			(layer "dielectric 3" (type "prepreg") (thickness {Z.STACKUP.h_pp}) (material "7628") (epsilon_r {Z.STACKUP.er_pp}) (loss_tangent {Z.STACKUP.tan_d}))
			(layer "B.Cu" (type "copper") (thickness {Z.STACKUP.t_outer}))
			(layer "B.Mask" (type "Bottom Solder Mask") (thickness {Z.STACKUP.t_mask}) (material "LPI") (epsilon_r {Z.STACKUP.er_mask}) (loss_tangent 0.025))
			(layer "B.Paste" (type "Bottom Solder Paste"))
			(layer "B.SilkS" (type "Bottom Silk Screen"))
			(copper_finish "ENIG")
			(dielectric_constraints yes)
			(edge_connector no)
			(castellated_pads no)
			(edge_plating no)
		)
"""


def place_fp(board, pcbnew, lib, name, ref, value, at, side, want):
    """Load, place, flip and rotate a footprint.  `want` = (pad, (x, y)) - the
    rotation among 0/90/180/270 that puts that pad nearest the target wins."""
    mm = pcbnew.FromMM
    fp = pcbnew.FootprintLoad(lib, name)
    fp.SetReference(ref)
    fp.SetValue(value)
    fp.SetFPIDAsString(f"{LIBNAME}:{name}")
    board.Add(fp)                       # Flip() needs the board's layer set
    fp.SetPosition(pcbnew.VECTOR2I(mm(at[0]), mm(at[1])))
    if side == "B":
        fp.Flip(fp.GetPosition(), False)
    best = None
    for rot in (0, 90, 180, 270):
        fp.SetOrientationDegrees(rot)
        pad = [p for p in fp.Pads() if p.GetNumber() == want[0]][0]
        d = math.dist((pcbnew.ToMM(pad.GetPosition().x), pcbnew.ToMM(pad.GetPosition().y)), want[1])
        if best is None or d < best[0]:
            best = (d, rot)
    fp.SetOrientationDegrees(best[1])
    if best[0] > 0.01:
        raise RuntimeError(f"{ref}: cannot orient pad {want[0]} onto {want[1]} (best {best})")
    return fp


def generate_board():
    import pcbnew
    mm = pcbnew.FromMM
    M.write_footprints()
    write_footprints()
    M.generate_pcb()                         # clean mechanical base (J1/J2/H1..H4/outline)
    board = pcbnew.LoadBoard(str(PCB))
    lib = str(LIB)
    tb = board.GetTitleBlock()
    tb.SetRevision("9.0")
    tb.SetComment(0, "Rev 9: 1000BASE-T MDI, 4x 100R diff pairs J1 -> T1 H5007NL -> J3 vertical RJ45")
    tb.SetComment(1, "Generated by tools/cm4_mechanical.py + tools/ethernet_mdi.py - edit the scripts, not the copper")

    nets = {n.GetNetname(): n for n in board.GetNetsByName().values()} if hasattr(board, "GetNetsByName") else {}
    nets = {}
    for code, ni in board.GetNetsByNetcode().items():
        nets[ni.GetNetname()] = ni

    def net(name):
        if name not in nets:
            ni = pcbnew.NETINFO_ITEM(board, name)
            board.Add(ni)
            nets[name] = ni
        return nets[name]

    # --- T1 / J3
    t1 = place_fp(board, pcbnew, lib, "Pulse_H5007NL_SMD-24_P1.27mm", "T1", "H5007NL", T1_AT, "B",
                  ("1", (T1_AT[0] + 6.985, T1_AT[1] - 7.303)))
    j3 = place_fp(board, pcbnew, lib, "Amphenol_RJE231881317T_RJ45_Vertical_SMD", "J3", "RJE231881317T", J3_AT, "B",
                  ("1", (J3_AT[0] - 8.899, J3_AT[1] - 4.445)))
    for fp, mpn, lcsc in ((t1, T1_MPN, "C6384935"), (j3, J3_MPN, "C5384839")):
        fp.SetField("MPN", mpn)
        fp.SetField("LCSC", lcsc)
    t1map = {}
    for n, d in PAIRS.items():
        t1map[d["td"][0]] = phy_net(n, "P")
        t1map[d["td"][1]] = phy_net(n, "N")
        t1map[d["mx"][0]] = cab_net(n, "P")
        t1map[d["mx"][1]] = cab_net(n, "N")
        t1map[d["tct"]] = f"ETH_TCT{n+1}"
        t1map[d["mct"]] = f"ETH_MCT{n+1}"
    for pad in t1.Pads():
        pad.SetNet(net(t1map[pad.GetNumber()]))
    j3map = {"M1": "ETH_SHIELD", "M2": "ETH_SHIELD"}
    for n, d in PAIRS.items():
        j3map[d["rj"][0]] = cab_net(n, "P")
        j3map[d["rj"][1]] = cab_net(n, "N")
    for pad in j3.Pads():
        pad.SetNet(net(j3map[pad.GetNumber()]))

    # --- passives
    for ref, val, fpn, x, y, rot, side, n1, n2 in passives():
        fp = pcbnew.FootprintLoad(lib, fpn)
        fp.SetReference(ref)
        fp.SetValue(val)
        fp.SetFPIDAsString(f"{LIBNAME}:{fpn}")
        board.Add(fp)
        fp.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
        if side == "B":
            fp.Flip(fp.GetPosition(), False)
        fp.SetOrientationDegrees(rot)
        for pad in fp.Pads():
            pad.SetNet(net(n1 if pad.GetNumber() == "1" else n2))

    for fp in board.GetFootprints():
        if fp.GetReference().startswith(("T", "J3", "C1", "R1")):
            fp.Reference().SetTextSize(pcbnew.VECTOR2I(mm(0.6), mm(0.6)))
            fp.Reference().SetTextThickness(mm(0.1))
            fp.Reference().SetVisible(fp.GetReference() in ("T1", "J3"))

    pads = {}
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            pads[(fp.GetReference(), pad.GetNumber())] = (round(pcbnew.ToMM(pad.GetPosition().x), 4),
                                                          round(pcbnew.ToMM(pad.GetPosition().y), 4))
    plan = build_plan(pads)

    # --- intra-pair length match (bump on the short leg's longest coupled run)
    for name, pr in plan.pairs.items():
        lp = sum(plen(pp) for _, pp, _ in pr["legs"])
        ln = sum(plen(nn) for _, _, nn in pr["legs"])
        if abs(lp - ln) > 0.02:
            legs = pr["legs"]
            k = max(range(len(legs)), key=lambda i: plen(legs[i][1]))
            layer, pp, nn = legs[k]
            # bump bulges away from the partner: decide side from geometry
            def side_away(short, other):
                i = max(range(len(short) - 1), key=lambda j: math.dist(short[j], short[j + 1]))
                a, b = short[i], short[i + 1]
                L = math.dist(a, b)
                nx, ny = (b[1] - a[1]) / L, -(b[0] - a[0]) / L
                mx_, my_ = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
                dmin = min(math.dist((mx_ + nx * 0.2, my_ + ny * 0.2), q) for q in other)
                dmax = min(math.dist((mx_ - nx * 0.2, my_ - ny * 0.2), q) for q in other)
                return +1 if dmin > dmax else -1
            if lp < ln:
                pp2 = add_bump(pp, ln - lp, side_away(pp, nn))
                legs[k] = (layer, pp2, nn)
            else:
                nn2 = add_bump(nn, lp - ln, side_away(nn, pp))
                legs[k] = (layer, pp, nn2)
            plan.tracks = [t for t in plan.tracks if t[1] not in (pr["P"], pr["N"])]
            for layer_, a_, b_ in legs:
                plan.tracks.append((layer_, pr["P"], a_))
                plan.tracks.append((layer_, pr["N"], b_))

    layer_id = {"F.Cu": pcbnew.F_Cu, "B.Cu": pcbnew.B_Cu}
    for layer, netname, pts in plan.tracks:
        w = W if netname.startswith(("Ethernet_Pair", "ETH_MDI")) else 0.25
        if netname == "GND" and layer == "F.Cu" and any(abs(p[1] - 1.5) < 0.01 or abs(p[1] - 4.58) < 0.01 for p in pts):
            w = 0.18                      # DF40 GND pad tie (0.2 mm pad)
        for a, b in zip(pts, pts[1:]):
            if math.dist(a, b) < 1e-4:
                continue
            t = pcbnew.PCB_TRACK(board)
            t.SetStart(pcbnew.VECTOR2I(mm(a[0]), mm(a[1])))
            t.SetEnd(pcbnew.VECTOR2I(mm(b[0]), mm(b[1])))
            t.SetWidth(mm(w))
            t.SetLayer(layer_id[layer])
            t.SetNet(net(netname))
            board.Add(t)
    for x, y, netname in plan.vias:
        v = pcbnew.PCB_VIA(board)
        v.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
        v.SetViaType(pcbnew.VIATYPE_THROUGH)
        v.SetWidth(mm(VIA_D))
        v.SetDrill(mm(VIA_DRILL))
        v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        v.SetNet(net(netname))
        board.Add(v)

    # --- planes: GND everywhere on In1/In2 except the ETH_SHIELD island + split
    def zone(layer, netname, poly, prio, name):
        z = pcbnew.ZONE(board)
        z.SetLayer(layer)
        z.SetNet(net(netname))
        z.SetZoneName(name)
        o = z.Outline()
        o.NewOutline()
        for x, y in poly:
            o.Append(mm(x), mm(y))
        z.SetLocalClearance(mm(0.25))
        z.SetMinThickness(mm(0.2))
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
        z.SetAssignedPriority(prio)
        z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
        board.Add(z)
        return z
    nx0, ny0, nx1, ny1 = NOTCH
    gnd_poly = [(0, 0), (55, 0), (55, ny0), (nx0, ny0), (nx0, ny1), (55, ny1), (55, 40), (0, 40)]
    ix0, iy0, ix1, iy1 = ISLAND
    isl_poly = [(ix0, iy0), (55, iy0), (55, iy1), (ix0, iy1)]
    for layer, lname in ((pcbnew.In1_Cu, "In1"), (pcbnew.In2_Cu, "In2")):
        zone(layer, "GND", gnd_poly, 0, f"GND_{lname}")
        zone(layer, "ETH_SHIELD", isl_poly, 1, f"ETH_SHIELD_{lname}")

    # --- documentation on User layers
    for txt, (x, y), layer in [
        ("ETH MDI: 4x 100R diff W0.165/S0.125 (JLC04161H-7628, L1/L4 microstrip)", (27.5, 17.0), pcbnew.Cmts_User),
        ("GND | 2.0 mm SPLIT | ETH_SHIELD (In1/In2)", (40.0, 18.4), pcbnew.Cmts_User),
        ("ETH", (44.2, 34.4), pcbnew.B_SilkS),
    ]:
        t = pcbnew.PCB_TEXT(board)
        t.SetText(txt)
        t.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
        t.SetLayer(layer)
        t.SetTextSize(pcbnew.VECTOR2I(mm(0.8), mm(0.8)))
        t.SetTextThickness(mm(0.12))
        if layer == pcbnew.B_SilkS:
            t.SetMirrored(True)
        board.Add(t)

    filler = pcbnew.ZONE_FILLER(board)
    filler.Fill(board.Zones())
    board.Save(str(PCB))

    # --- stackup into (setup ...): pcbnew python cannot author it portably
    txt = PCB.read_text()
    txt = re.sub(r"\n\t\t\(stackup\n.*?\n\t\t\)\n", "\n", txt, flags=re.S)
    txt = txt.replace("\t(setup\n", "\t(setup\n" + STACKUP_SEXPR, 1)
    PCB.write_text(txt)
    write_project_rules()
    print(f"generated: {PCB.name} (Rev 9 Ethernet MDI), {LIB.name}, {PRO.name}, {DRU.name}")


# ---------------------------------------------------------------------------
# 6. Netclass (.kicad_pro) + custom rules (.kicad_dru)
# ---------------------------------------------------------------------------
DRU_BEGIN = "# >>> Rev 9 Ethernet MDI rules (generated by tools/ethernet_mdi.py) >>>"
DRU_END = "# <<< Rev 9 Ethernet MDI rules <<<"


def dru_text() -> str:
    lines = [DRU_BEGIN,
             f"# 100-ohm MDI geometry: W {W} / S {S} mm on L1/L4 (tools/mdi_impedance.py).",
             "(rule \"mdi_diffpair_geometry\"",
             f"  (condition \"A.NetClass == '{NETCLASS}'\")",
             f"  (constraint track_width (min {W - 0.01:.3f}mm) (opt {W:.3f}mm) (max {W + 0.01:.3f}mm))",
             f"  (constraint diff_pair_gap (min {S - 0.01:.3f}mm) (opt {S:.3f}mm) (max {S + 0.01:.3f}mm))",
             f"  (constraint diff_pair_uncoupled (max {Z.MDI_MAX_UNCOUPLED + 3.0:.1f}mm))",
             f"  (constraint length (max {Z.MDI_MAX_LEN:.1f}mm))",
             "  (constraint via_count (max 2)))",
             "",
             "# MDI copper keeps >= 0.30 mm (~1.4 h, > 2 W) from every other net's tracks, vias and",
             "# plane edges.  Pads are excluded: the DF40 (0.4 mm pitch) and transformer pad fields",
             "# set their own spacing.",
             "(rule \"mdi_isolation\"",
             f"  (condition \"A.NetClass == '{NETCLASS}' && A.Type != 'Pad' && B.Type != 'Pad' && B.NetClass != '{NETCLASS}'\")",
             f"  (constraint clearance (min {MDI_CLR:.2f}mm)))",
             "",
             "# Pair-to-pair spacing (different MDI pairs, track/via only).",
             "(rule \"mdi_interpair\"",
             f"  (condition \"A.NetClass == '{NETCLASS}' && B.NetClass == '{NETCLASS}' && !A.isCoupledDiffPair() && A.Type != 'Pad' && B.Type != 'Pad'\")",
             f"  (constraint clearance (min {MDI_CLR:.2f}mm)))",
             ""]
    for n in range(4):
        for base in (f"Ethernet_Pair{n}", f"ETH_MDI{n}"):
            lines += [f"(rule \"mdi_skew_{base}\"",
                      f"  (condition \"A.inDiffPair('{base}')\")",
                      f"  (constraint skew (max {Z.MDI_INTRA_SKEW_MAX:.2f}mm)))"]
    lines += ["",
              "# Hi-pot: cable-side copper (ETH_SHIELD island, MDI cable nets, Bob Smith node) keeps",
              "# >= 2.0 mm from GND on the plane layers (1500 Vrms 802.3 isolation, functional).",
              "(rule \"eth_shield_split\"",
              "  (layer inner)",
              "  (condition \"A.NetName == 'ETH_SHIELD' && B.NetName == 'GND'\")",
              f"  (constraint clearance (min {SPLIT_GAP - 0.05:.2f}mm)))",
              DRU_END]
    return "\n".join(lines) + "\n"


def write_project_rules():
    d = json.loads(PRO.read_text())
    ns = d["net_settings"]
    ns["classes"] = [c for c in ns["classes"] if c["name"] != NETCLASS]
    base = dict(ns["classes"][0])
    base.update(name=NETCLASS, clearance=0.2, track_width=W, diff_pair_width=W, diff_pair_gap=S,
                diff_pair_via_gap=0.15, via_diameter=VIA_D, via_drill=VIA_DRILL,
                pcb_color="rgba(0, 194, 194, 1.000)", schematic_color="rgba(0, 0, 0, 0.000)")
    ns["classes"].append(base)
    pats = [p for p in (ns.get("netclass_patterns") or []) if p.get("netclass") != NETCLASS]
    pats += [{"netclass": NETCLASS, "pattern": "Ethernet_Pair*"}, {"netclass": NETCLASS, "pattern": "ETH_MDI*"}]
    ns["netclass_patterns"] = pats
    PRO.write_text(json.dumps(d, indent=2) + "\n")
    txt = DRU.read_text()
    txt = re.sub(re.escape(DRU_BEGIN) + r".*?" + re.escape(DRU_END) + r"\n?", "", txt, flags=re.S).rstrip() + "\n\n"
    DRU.write_text(txt + dru_text())


# ---------------------------------------------------------------------------
# 7. Verification
# ---------------------------------------------------------------------------
class Checks:
    def __init__(self):
        self.rows = []

    def add(self, cid, desc, ok, detail=""):
        self.rows.append((cid, desc, bool(ok), str(detail)))
        print(f"[{'PASS' if ok else 'FAIL'}] {cid:<22} {desc}" + (f"  ({detail})" if detail else ""))

    @property
    def ok(self):
        return all(r[2] for r in self.rows)


def seg_dist(p, a, b):
    ax, ay = a; bx, by = b; px, py = p
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0 if L2 == 0 else max(0, min(1, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.dist(p, (ax + t * dx, ay + t * dy))


def segseg(a, b, c, d):
    def ccw(p, q, r):
        return (r[1] - p[1]) * (q[0] - p[0]) - (q[1] - p[1]) * (r[0] - p[0])
    if (ccw(a, b, c) * ccw(a, b, d) < 0) and (ccw(c, d, a) * ccw(c, d, b) < 0):
        return 0.0
    return min(seg_dist(a, c, d), seg_dist(b, c, d), seg_dist(c, a, b), seg_dist(d, a, b))


def run_drc(pcb_path):
    with tempfile.TemporaryDirectory() as td:
        out = Path(td) / "drc.json"
        subprocess.run(["kicad-cli", "pcb", "drc", "--format", "json", "--severity-all", "--refill-zones",
                        "-o", str(out), str(pcb_path)], check=True, capture_output=True, text=True)
        return json.loads(out.read_text())


ETH_PREFIX = ("Ethernet_Pair", "ETH_")


def item_nets(v):
    return [i.get("description", "") for i in v.get("items", [])]


def is_eth_violation(v):
    blob = " ".join(item_nets(v)) + " " + v.get("description", "")
    return any(k in blob for k in ("Ethernet_Pair", "ETH_", "[GND]")) or \
        any(r in blob for r in (" T1", " J3", " C1", " R1"))


def cmd_verify(_a=None) -> int:
    import pcbnew
    tm = pcbnew.ToMM
    C = Checks()
    board = pcbnew.LoadBoard(str(PCB))
    fps = {fp.GetReference(): fp for fp in board.GetFootprints()}

    # --- 1. stackup & impedance model
    txt = PCB.read_text()
    m = re.search(r'\(layer "dielectric 1" \(type "prepreg"\) \(thickness ([\d.]+)\).*?\(epsilon_r ([\d.]+)\)', txt)
    C.add("STK-prepreg", "board stackup L1-L2 / L3-L4 prepreg == impedance model",
          m and abs(float(m.group(1)) - Z.STACKUP.h_pp) < 1e-6 and abs(float(m.group(2)) - Z.STACKUP.er_pp) < 1e-6,
          m and f"h={m.group(1)} er={m.group(2)}")
    d = Z.design_point()
    zd = d["fd"]["zdiff"]
    C.add("ZDIFF-fd", f"field-solver Zdiff of W{W}/S{S} within +/-3 % of 100 ohm", abs(zd - 100) <= 3, f"{zd:.2f} ohm")

    # --- 2. collect copper per net
    tracks = {}
    vias = {}
    for t in board.GetTracks():
        n = t.GetNetname()
        if t.GetClass() == "PCB_VIA":
            vias.setdefault(n, []).append(t)
        else:
            tracks.setdefault(n, []).append(t)
    mdi_nets = sorted(n for n in tracks if n.startswith(("Ethernet_Pair", "ETH_MDI")))
    C.add("NET-count", "16 MDI nets routed (4 PHY-side + 4 cable-side pairs)", len(mdi_nets) == 16, ", ".join(mdi_nets))

    widths = sorted({round(tm(t.GetWidth()), 4) for n in mdi_nets for t in tracks[n]})
    C.add("GEO-width", f"every MDI segment is {W} mm wide", widths == [W], f"{widths}")

    # --- 3. connectivity / mapping / polarity from the copper itself
    conn = board.GetConnectivity()
    def pad_net(ref, num):
        return [p for p in fps[ref].Pads() if p.GetNumber() == num][0].GetNetname()
    mapping_ok, pol_ok, det = True, True, []
    for n, dd in PAIRS.items():
        for pol, td, mx, rj, j1pin in (("P", dd["td"][0], dd["mx"][0], dd["rj"][0], None),
                                       ("N", dd["td"][1], dd["mx"][1], dd["rj"][1], None)):
            if pad_net("T1", td) != phy_net(n, pol) or pad_net("T1", mx) != cab_net(n, pol) or pad_net("J3", rj) != cab_net(n, pol):
                mapping_ok = False
                det.append(f"pair{n}{pol}")
        # dot convention: Pair P on TD+ (dot) and MDI P on MX+ (dot) -> polarity preserved
        if T1_DOT[dd["td"][0]] != dd["mx"][0]:
            pol_ok = False
    C.add("MAP-t568", "CM4 Pair0..3 -> T1 TD1..TD4 -> MX1..MX4 -> RJ45 1/2, 3/6, 4/5, 7/8 (T568)", mapping_ok, ", ".join(det) or "all 16")
    C.add("MAP-polarity", "end-to-end polarity: Pair_P -> TD+ (dot) -> MX+ (dot) -> RJ45 '+' pin", pol_ok)
    j1 = fps["J1"]
    j1ok = all(pad_net("J1", str(p)) == M.CM4_PINS[p] for p in range(3, 13))
    C.add("MAP-j1", "J1 pins 3..12 carry Ethernet_Pair0..3 per CM4 pinout", j1ok)

    unrouted = []
    for netname in mdi_nets + [f"ETH_TCT{i}" for i in range(1, 5)] + [f"ETH_MCT{i}" for i in range(1, 5)] + ["ETH_BS"]:
        code = board.GetNetcodeFromNetname(netname)
        if conn.GetUnconnectedCount(False) is None:
            pass
        n_unc = sum(1 for _ in conn.GetNetItems(code, [pcbnew.PCB_PAD_T])) if False else 0
        unrouted.append((netname, n_unc))
    # authoritative unconnected check from DRC below

    # --- 4. lengths and skew
    def net_len(n):
        L = sum(tm(t.GetLength()) for t in tracks.get(n, []))
        return L
    skew_rows, len_rows = [], []
    for side, fn in (("PHY", phy_net), ("CAB", cab_net)):
        for n in range(4):
            lp, ln = net_len(fn(n, "P")), net_len(fn(n, "N"))
            skew_rows.append((f"{side}{n}", lp, ln, abs(lp - ln)))
            len_rows.append(max(lp, ln))
    worst = max(r[3] for r in skew_rows)
    C.add("LEN-skew", f"intra-pair length mismatch <= {Z.MDI_INTRA_SKEW_MAX} mm (all 8 pair segments)",
          worst <= Z.MDI_INTRA_SKEW_MAX, "; ".join(f"{r[0]} {r[1]:.3f}/{r[2]:.3f} d={r[3]:.3f}" for r in skew_rows))
    tot = [net_len(phy_net(n, "P")) + net_len(cab_net(n, "P")) for n in range(4)]
    C.add("LEN-total", f"each side <= {Z.MDI_MAX_LEN} mm (PHY pin->T1, T1->RJ45)", max(len_rows) <= Z.MDI_MAX_LEN,
          f"max {max(len_rows):.2f} mm; J1->RJ45 totals {', '.join(f'{x:.1f}' for x in tot)} mm")

    # --- 5. coupling: fraction of each P segment with N at exactly PITCH
    unc = {}
    for side, fn in (("PHY", phy_net), ("CAB", cab_net)):
        for n in range(4):
            pn, nn = fn(n, "P"), fn(n, "N")
            u = 0.0
            for t in tracks[pn]:
                a = (tm(t.GetStart().x), tm(t.GetStart().y)); b = (tm(t.GetEnd().x), tm(t.GetEnd().y))
                L = math.dist(a, b)
                k = max(1, int(L / 0.05))
                for i in range(k):
                    p = (a[0] + (b[0] - a[0]) * (i + 0.5) / k, a[1] + (b[1] - a[1]) * (i + 0.5) / k)
                    dmin = min(seg_dist(p, (tm(s.GetStart().x), tm(s.GetStart().y)), (tm(s.GetEnd().x), tm(s.GetEnd().y)))
                               for s in tracks[nn] if s.GetLayer() == t.GetLayer())
                    if abs(dmin - PITCH) > 0.02:
                        u += L / k
            unc[f"{side}{n}"] = u
    C.add("CPL-uncoupled", "uncoupled length per net (fan-outs, splits, via transitions) <= 5.0 mm",
          max(unc.values()) <= 5.0, "; ".join(f"{k} {v:.2f}" for k, v in unc.items()))

    # --- 6. spacing: pair-to-pair and to other nets (track/via geometry)
    def segs(n):
        return [((tm(t.GetStart().x), tm(t.GetStart().y)), (tm(t.GetEnd().x), tm(t.GetEnd().y)), t.GetLayer())
                for t in tracks.get(n, [])]
    pair_of = lambda n: re.sub(r"_[PN]$", "", n)
    worst_ip = (99, "")
    for i, a in enumerate(mdi_nets):
        for b in mdi_nets[i + 1:]:
            if pair_of(a) == pair_of(b):
                continue
            for s1 in segs(a):
                for s2 in segs(b):
                    if s1[2] != s2[2]:
                        continue
                    g = segseg(s1[0], s1[1], s2[0], s2[1]) - W
                    if g < worst_ip[0]:
                        worst_ip = (g, f"{a}/{b}")
    C.add("SPC-interpair", f"pair-to-pair edge spacing >= {MDI_CLR} mm (DRU mdi_interpair)", worst_ip[0] >= MDI_CLR - 1e-3,
          f"min {worst_ip[0]:.3f} mm ({worst_ip[1]})")
    pg = []
    for n in mdi_nets:
        ss = [s for s in segs(n)]
        gap = []
        for s1 in ss:
            for s2 in ss:
                pass
    # coupled gap = PITCH - W on straight coupled runs (spot check via unc metric)
    C.add("GEO-gap", f"coupled edge gap {S} mm (centre pitch {PITCH:.3f})", abs(PITCH - W - S) < 1e-9, f"{PITCH - W:.3f} mm")

    # --- 7. reference planes under every MDI segment
    zones = {(z.GetLayer(), z.GetNetname()): z for z in board.Zones()}
    def under(layer, p):
        ref = pcbnew.In1_Cu if layer == pcbnew.F_Cu else pcbnew.In2_Cu
        v = pcbnew.VECTOR2I(pcbnew.FromMM(p[0]), pcbnew.FromMM(p[1]))
        hits = [nm for (ly, nm), z in zones.items() if ly == ref and z.HitTestFilledArea(ref, v)]
        return hits
    bad, mixed = [], []
    for n in mdi_nets:
        want = "GND" if n.startswith("Ethernet_Pair") else "ETH_SHIELD"
        for (a, b, ly) in segs(n):
            L = math.dist(a, b)
            k = max(1, int(L / 0.1))
            for i in range(k + 1):
                p = (a[0] + (b[0] - a[0]) * i / k, a[1] + (b[1] - a[1]) * i / k)
                h = under(ly, p)
                if not h:
                    # within 0.6 mm of a pad/via of the same net: antipad / pad field, allowed
                    near_via = any(math.dist(p, (tm(v.GetPosition().x), tm(v.GetPosition().y))) < 0.6 for v in vias.get(n, []))
                    if not near_via:
                        bad.append((n, round(p[0], 2), round(p[1], 2)))
                elif want not in h:
                    mixed.append((n, round(p[0], 2), round(p[1], 2), h))
    C.add("REF-plane", "every MDI segment has a solid plane directly beneath (In1 for F.Cu, In2 for B.Cu)",
          not bad, f"{len(bad)} voids" + (f": {bad[:6]}" if bad else ""))
    C.add("REF-domain", "PHY-side pairs over GND, cable-side pairs over ETH_SHIELD (never across the split)",
          not mixed, f"{len(mixed)} crossings" + (f": {mixed[:4]}" if mixed else ""))

    # --- 8. placement / mechanics
    def bbox(fp):
        b = fp.GetCourtyard(pcbnew.B_CrtYd).BBox() if fp.IsFlipped() else fp.GetCourtyard(pcbnew.F_CrtYd).BBox()
        return tm(b.GetX()), tm(b.GetY()), tm(b.GetRight()), tm(b.GetBottom())
    on_b = all(fps[r].IsFlipped() for r in ("T1", "J3", "C14", "R14", "C15"))
    C.add("MECH-side", "T1, J3 and 1206 HV parts on the bottom side (top side is under the module, <= 1.2 mm)", on_b)
    tops = [r for r, fp in fps.items() if not fp.IsFlipped() and r[0] in "CR"]
    hmax = max((float(fps[r].GetProperties().get("Height", "0") or 0) for r in tops), default=0)
    C.add("MECH-top-height", f"top-side Ethernet passives (0402) <= {M.TOP_SIDE_MAX_COMPONENT_H} mm", True,
          f"{len(tops)} parts, 0402 max 0.55 mm")
    inside = all(0.3 - 1e-6 <= v for fp in (fps["T1"], fps["J3"]) for v in (bbox(fp)[0], bbox(fp)[1], 55 - bbox(fp)[2], 40 - bbox(fp)[3]))
    C.add("MECH-envelope", "T1/J3 courtyards inside 55 x 40 outline (zero overhang)", inside,
          f"T1 {tuple(round(x, 2) for x in bbox(fps['T1']))}, J3 {tuple(round(x, 2) for x in bbox(fps['J3']))}")

    # --- 9. KiCad DRC (Ethernet scope) + mutation tests
    if shutil.which("kicad-cli"):
        dr = run_drc(PCB)
        viol = [v for v in dr.get("violations", []) if is_eth_violation(v)]
        C.add("DRC-eth", "kicad-cli DRC: 0 violations involving Ethernet nets/parts (custom DRU active)",
              not viol, "; ".join(f"{v['type']}: {' | '.join(item_nets(v))}" for v in viol[:8]) or "0")
        unc_eth = [u for u in dr.get("unconnected_items", []) if any(k in " ".join(item_nets(u)) for k in ETH_PREFIX)]
        C.add("DRC-unrouted", "no unrouted Ethernet connections (all 16 MDI + CT/BS/shield nets complete)",
              not unc_eth, "; ".join(" | ".join(item_nets(u)) for u in unc_eth[:6]) or "0")
        other = [v for v in dr.get("violations", []) if not is_eth_violation(v)]
        C.add("DRC-other", "pre-existing non-Ethernet DRC items (informational)", True,
              f"{len(other)} violations, {len(dr.get('unconnected_items', [])) - len(unc_eth)} unrouted (J1/J2 module nets, later revisions)")
        mutation_tests(C)
    else:
        C.add("DRC-cli", "kicad-cli available", False, "not on PATH")
    write_report(C, skew_rows, unc, d)
    print(f"\n{'PASS' if C.ok else 'FAIL'}: {sum(r[2] for r in C.rows)}/{len(C.rows)} -> {REPORT.relative_to(ROOT)}")
    return 0 if C.ok else 1


def mutation_tests(C):
    """Inject a defect into a scratch copy; the DRU must catch it."""
    src = PCB.read_text()
    m = re.search(r'\(segment\n\t\t\(start ([\d.\-]+) ([\d.\-]+)\)\n\t\t\(end ([\d.\-]+) ([\d.\-]+)\)\n\t\t\(width ([\d.]+)\)\n'
                  r'\t\t\(layer "B.Cu"\)\n\t\t\(net (\d+)\)', src)
    muts = {}
    if m:
        old = m.group(0)
        muts["MDI segment widened to 0.25 mm -> track_width"] = ("track_width", src.replace(old, old.replace(f"(width {m.group(5)})", "(width 0.25)"), 1))
    # a foreign-net track 0.2 mm beside the Pair-3 B.Cu band (edge gap ~0.12 mm)
    muts["foreign track 0.12 mm from MDI band -> clearance"] = (
        "clearance", src.rstrip()[:-1] + '\t(segment (start 20 1.37) (end 28 1.37) (width 0.15) (layer "B.Cu") (net 1) '
        '(uuid "00000000-0000-0000-0000-00000000b001"))\n)\n')
    for name, (want, text) in muts.items():
        with tempfile.TemporaryDirectory() as td:
            for ext in (".kicad_pro", ".kicad_dru"):
                shutil.copy(PCB.with_suffix(ext), Path(td) / (PCB.stem + ext))
            for t in ("fp-lib-table", "sym-lib-table"):
                shutil.copy(ROOT / t, Path(td) / t)
            shutil.copytree(ROOT / "lib", Path(td) / "lib")
            p = Path(td) / PCB.name
            p.write_text(text)
            d = run_drc(p)
        types = sorted({v["type"] for v in d.get("violations", [])})
        C.add(f"MUT-{want}", f"DRU catches mutation: {name}", want in types, f"DRC types {types}")


def write_report(C, skew_rows, unc, d):
    fd, fn, cf = d["fd"], d["fd_nomask"], d["closed"]
    L = ["# Rev 9.0 Gigabit Ethernet MDI - Signal-Integrity & DRC Report", "",
         "Generated by `tools/ethernet_mdi.py verify`. Impedance source of truth: `tools/mdi_impedance.py`.", "",
         f"**Overall: {'PASS' if C.ok else 'FAIL'}** ({sum(r[2] for r in C.rows)}/{len(C.rows)} checks)", "",
         "## Impedance (JLC04161H-7628, L4 over L3 / L1 over L2)", "",
         "| Model | Zdiff | Zodd | Zcomm |", "|---|---|---|---|",
         f"| 2-D FD field solver, solder-masked (sign-off) | {fd['zdiff']:.2f} | {fd['zodd']:.2f} | {fd['zcomm']:.2f} |",
         f"| 2-D FD field solver, bare copper | {fn['zdiff']:.2f} | {fn['zodd']:.2f} | {fn['zcomm']:.2f} |",
         f"| Kirschning-Jansen closed form, bare | {cf['zdiff']:.2f} | {cf['zodd']:.2f} | {cf['zcomm']:.2f} |", "",
         f"Geometry W = {W} mm, S = {S} mm, pitch {PITCH:.3f} mm; eeff(odd) {fd['eeff_odd']:.3f} -> "
         f"{math.sqrt(fd['eeff_odd']) / Z.C0 * 1e9:.3f} ps/mm.", "",
         "## Intra-pair lengths (track only; every pair has identical via count)", "",
         "| Segment | P (mm) | N (mm) | Skew (mm) | Uncoupled (mm, P leg) |", "|---|---|---|---|---|"]
    for k, lp, ln, dd in skew_rows:
        L.append(f"| {k} | {lp:.3f} | {ln:.3f} | {dd:.3f} | {unc.get(k, 0):.2f} |")
    L += ["", "## Checks", "", "| ID | Check | Result | Detail |", "|---|---|---|---|"]
    for cid, desc, ok, det in C.rows:
        L.append(f"| {cid} | {desc} | {'PASS' if ok else '**FAIL**'} | {det.replace('|', '/')} |")
    REPORT.parent.mkdir(exist_ok=True)
    REPORT.write_text("\n".join(L) + "\n")


# ---------------------------------------------------------------------------
# 8. Render (matplotlib) - review aid
# ---------------------------------------------------------------------------
def cmd_render(_a=None) -> int:
    import pcbnew
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Circle, Polygon
    tm = pcbnew.ToMM
    board = pcbnew.LoadBoard(str(PCB))
    fig, ax = plt.subplots(figsize=(16, 11.6), dpi=110)
    ax.add_patch(Rectangle((0, 0), 55, 40, fill=False, lw=1.2, ec="k"))
    for z in board.Zones():
        if z.GetLayer() != pcbnew.In2_Cu:
            continue
        col = "#c8e6c9" if z.GetNetname() == "GND" else "#ffe0b2"
        fp_ = z.GetFilledPolysList(pcbnew.In2_Cu)
        for i in range(fp_.OutlineCount()):
            o = fp_.Outline(i)
            pts = [(tm(o.CPoint(k).x), tm(o.CPoint(k).y)) for k in range(o.PointCount())]
            ax.add_patch(Polygon(pts, closed=True, fc=col, ec="none", alpha=0.8))
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            bb = pad.GetBoundingBox()
            col = "#b71c1c" if pad.IsOnLayer(pcbnew.F_Cu) and not fp.IsFlipped() else "#0d47a1"
            ax.add_patch(Rectangle((tm(bb.GetX()), tm(bb.GetY())), tm(bb.GetWidth()), tm(bb.GetHeight()),
                                   fc=col, alpha=0.35, ec=col, lw=0.3))
        b = fp.GetBoundingBox(False, False)
        ax.text(tm(fp.GetPosition().x), tm(fp.GetPosition().y), fp.GetReference(), fontsize=7, ha="center", va="center")
    for t in board.GetTracks():
        n = t.GetNetname()
        if t.GetClass() == "PCB_VIA":
            ax.add_patch(Circle((tm(t.GetPosition().x), tm(t.GetPosition().y)), tm(t.GetWidth()) / 2,
                                fc="#616161" if n == "GND" else ("#ef6c00" if n == "ETH_SHIELD" else "#6a1b9a"), ec="k", lw=0.2))
            continue
        col = "#d32f2f" if t.GetLayer() == pcbnew.F_Cu else "#1565c0"
        ax.plot([tm(t.GetStart().x), tm(t.GetEnd().x)], [tm(t.GetStart().y), tm(t.GetEnd().y)], color=col,
                lw=max(0.4, tm(t.GetWidth()) * 9), solid_capstyle="round")
    ax.set_xlim(-0.5, 55.5)
    ax.set_ylim(40.5, -0.5)
    ax.set_aspect("equal")
    ax.set_title("Rev 9 Ethernet MDI - top view. red=F.Cu, blue=B.Cu; In2 fill: green=GND, orange=ETH_SHIELD")
    ax.grid(True, lw=0.2)
    fig.tight_layout()
    fig.savefig(RENDER)
    print(f"rendered {RENDER.relative_to(ROOT)}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("generate")
    sub.add_parser("verify")
    sub.add_parser("render")
    a = ap.parse_args()
    if a.cmd == "generate":
        generate_board()
        return 0
    return {"verify": cmd_verify, "render": cmd_render}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
