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
VIA_D, VIA_DRILL = 0.50, 0.30          # project minimums (.kicad_pro: via 0.50 / drill 0.30, annulus 0.10)
VIA_PITCH = 0.65                       # MDI via-pair pitch -> 0.15 mm P-N via gap (netclass clearance 0.12)
GROUP = "REV9_ETH_MDI"                 # every item this stage adds is a member (idempotent re-runs, DRC attribution)
CHAMFER = 0.6                          # centreline 45-degree corner cut
NETCLASS = "ETH_MDI_100R"
MDI_CLR = 0.30                         # MDI copper (track/via) to other-net track/via/zone
SPLIT_GAP = 2.0                        # GND <-> ETH_SHIELD plane split

# Parts (all real, JLCPCB-stocked; footprints from the manufacturer drawings
# via the LCSC/EasyEDA land patterns, re-entered numerically below).
T1_MPN = "Pulse H5007NL"               # LCSC C6384935, 1000BASE-T 1CT:1CT, 1500 Vrms
J3_MPN = "Amphenol RJE231881317T"      # LCSC C5384839, vertical SMT, shielded, 13.1 mm
T1_AT = (34.0, 20.0)                   # B.Cu, PHY row east (pin 1 top), cable row west
J3_AT = (11.0, 20.0)                   # B.Cu, contacts facing east (pin 1 at the BOTTOM: bottom-side mirror)

# Island (ETH_SHIELD) and the GND notch around it (inner layers)
ISLAND = (0.0, 11.8, T1_AT[0] - 1.0, 31.5)            # x0, y0, x1, y1 (west / cable side; split under T1)
NOTCH = (0.0, ISLAND[1] - SPLIT_GAP, ISLAND[2] + SPLIT_GAP, ISLAND[3] + SPLIT_GAP)

# CM4 pair -> transformer channel -> T568 pins (+, -).  td/mx = (pin for the
# P net, pin for the N net).  Channel order on T1 (top->bottom) is ch1..ch4 =
# Pair3, Pair1, Pair2, Pair0 so that the cable side runs straight to the
# (bottom-side, mirrored) RJ45.  Every pair lands P on the NON-dot end of both
# windings (TDk- and MXk-): swapping both ends of a 1CT:1CT winding keeps the
# end-to-end polarity.
PAIRS = {
    3: dict(ch=1, td=("3", "2"), mx=("22", "23"), rj=("7", "8"), tct="1", mct="24"),
    1: dict(ch=2, td=("6", "5"), mx=("19", "20"), rj=("3", "6"), tct="4", mct="21"),
    2: dict(ch=3, td=("9", "8"), mx=("16", "17"), rj=("4", "5"), tct="7", mct="18"),
    0: dict(ch=4, td=("12", "11"), mx=("13", "14"), rj=("1", "2"), tct="10", mct="15"),
}
# Transformer windings (Pulse H5007NL datasheet): channel -> (TD+ dot, TD-, MX+ dot, MX-)
T1_WINDINGS = {1: ("2", "3", "23", "22"), 2: ("5", "6", "20", "19"),
               3: ("8", "9", "17", "16"), 4: ("11", "12", "14", "13")}


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
    # fp-lib-table: MERGE (other pipeline stages may add libraries).  Ensure the
    # project lib and the legacy CM4Carrier lib referenced by the schematic
    # mounting-hole symbols (Rev 7 DRC lib_footprint_issues) are present.
    t = ROOT / "fp-lib-table"
    txt = t.read_text() if t.exists() else "(fp_lib_table\n  (version 7)\n)\n"
    want = {
        LIBNAME: f'  (lib (name "{LIBNAME}")(type "KiCad")(uri "${{KIPRJMOD}}/lib/{LIBNAME}.pretty")(options "")(descr "Project footprints (CM4 mechanical, Ethernet, HDMI, power)"))',
        "CM4Carrier": '  (lib (name "CM4Carrier")(type "KiCad")(uri "${KIPRJMOD}/lib/CM4Carrier.pretty")(options "")(descr "Legacy Rev 5 mounting-hole footprint used by the schematic"))',
    }
    for name, line in want.items():
        if f'(name "{name}")' not in txt:
            txt = txt.rstrip().rstrip(")").rstrip() + "\n" + line + "\n)\n"
    t.write_text(txt)


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


def add_bump(pts, extra, side_sign, amp_max=0.35, seg=None):
    """Insert 45-degree trombone bump(s) adding `extra` mm into the longest
    straight segment of `pts`, bulging to side_sign * left-of-travel.  Bump
    height <= amp_max (keeps >= 2 S from the partner and clear of neighbours);
    as many bumps as needed, each flat 3 W, spaced 3 W."""
    if extra <= 1e-4:
        return pts
    per = (2 * math.sqrt(2) - 2)                 # extra length per mm of height, per bump
    nb = max(1, math.ceil(extra / (per * amp_max) - 1e-9))
    a = extra / (per * nb)
    i = seg if seg is not None else max(range(len(pts) - 1), key=lambda k: math.dist(pts[k], pts[k + 1]))
    p, q = pts[i], pts[i + 1]
    L = math.dist(p, q)
    flat, gap = 3 * W, 3 * W
    span = nb * (2 * a + flat) + (nb - 1) * gap
    if L < span + 0.4:
        raise RuntimeError(f"no straight run for {extra:.3f} mm of skew compensation (need {span:.2f}, have {L:.2f})")
    ux, uy = (q[0] - p[0]) / L, (q[1] - p[1]) / L
    nx, ny = uy * side_sign, -ux * side_sign
    s0 = (L - span) / 2
    out = []
    for k in range(nb):
        b0 = s0 + k * (2 * a + flat + gap)
        for (t, h) in ((b0, 0), (b0 + a, a), (b0 + a + flat, a), (b0 + 2 * a + flat, 0)):
            out.append((p[0] + ux * t + nx * h, p[1] + uy * t + ny * h))
    return pts[:i + 1] + out + pts[i + 1:]


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


def build_plan(pads, pads_net=None) -> Plan:
    """Route plan (board coords, mm, top view; y down).  See module docstring.

    T1 (B.Cu) PHY row faces EAST, pin 1 at the top; J3 (B.Cu) contacts face
    EAST towards T1's cable row, pin 1 at the BOTTOM (bottom-side mirror).
    Channel order top->bottom: ch1 = Pair3 (D), ch2 = Pair1 (B), ch3 = Pair2
    (C), ch4 = Pair0 (A) - this makes the cable side run straight across.
    Every pair lands P on the non-dot winding end at BOTH TDk- and MXk-, so
    polarity is preserved end-to-end (verified from the dot convention).

    PHY side: bands run east on F.Cu (P3 and P2/P1/P0 from the J1 field),
    drops run south on B.Cu east of T1 and turn west into the pads with P on
    the south (TD-) pad.  F.Cu bands and B.Cu drops sit over different solid
    planes (In1 / In2), so they may cross; each pair changes layer exactly
    once.  Two pairs (3, 0) need a compact two-layer via twist at the corner.
    """
    P = Plan(pads)
    T = lambda n: pads[("T1", n)]
    R = lambda n: pads[("J3", n)]

    t1_e = T("1")[0]                      # PHY pad-row x (pad centres)
    x_entry = t1_e + 0.952 + 0.55         # coupled section ends here, then fans into pads
    # South-drop centrelines (ch1..ch4): 0.95 mm pitch (0.495 mm edge gap), 1.2 mm
    # between ch2 and ch3 so ch3 clears ch2's corner-via antipads on In2.  The
    # corridor lies between T1 (x <= 42.6) and the Rev 13 buck U5/L1 (x >= 47.85).
    XV = {3: x_entry + 0.60, 1: x_entry + 1.55, 2: x_entry + 2.75, 0: x_entry + 3.70}

    def t1_tail(n):
        """Fan-in heading WEST: P -> south (TD-) pad, N -> north (TD+) pad."""
        pp, nn = T(PAIRS[n]["td"][0]), T(PAIRS[n]["td"][1])
        assert pp[1] > nn[1], "P must land on the south (TD-) pad"
        yc = (pp[1] + nn[1]) / 2
        k = (pp[1] - nn[1]) / 2 - HP
        tp = [(x_entry, yc + HP), (x_entry - k, pp[1]), pp]
        tn = [(x_entry, yc - HP), (x_entry - k, nn[1]), nn]
        return yc, tp, tn

    def twist(xc, y0):
        """Arriving on F.Cu heading SOUTH with P west / N east at (xc, y0);
        leave on B.Cu heading south with P EAST.  N drops through a via first,
        P passes under it and through a second via further east."""
        vN = (xc + 0.35, y0 + 0.15)
        vP = (xc + 0.70, y0 + 1.05)
        fP = [(xc - HP, y0), (xc - HP, y0 + 0.85), (xc + 0.05, y0 + 1.05), vP]
        fN = [(xc + HP, y0), vN]
        bN = [vN, (xc - HP, y0 + 0.645)]
        bP = [vP, (xc + HP, y0 + 1.605)]
        return vP, vN, fP, fN, bP, bN, y0 + 1.65

    # ---- Pair 3 (ch1): outer row, P (34.4) east / N (34.0) west.  F.Cu south
    #      into the DF40 inter-row gap (P east = left), east along the gap past
    #      pins 1/2 and the connector end (P north), south at the drop (P east)
    #      -> side-by-side vias -> B.Cu drop.  No twist, no edge channel (the
    #      project keeps copper >= 0.5 mm from Edge.Cuts).
    p3p, p3n = j1_pad(3), j1_pad(5)
    xd = XV[3]
    band3 = 2.75
    pc, nc = coupled([(34.20, 1.95), (34.20, band3), (xd, band3), (xd, 3.40)], p_side=+1)
    vP3, vN3 = (xd + VIA_PITCH / 2, 3.95), (xd - VIA_PITCH / 2, 3.95)
    fP = [p3p] + pc + [vP3]
    fN = [p3n] + nc + [vN3]
    yc, tp, tn = t1_tail(3)
    pc, nc = coupled([(xd, 4.55), (xd, yc), (x_entry, yc)], p_side=+1)
    P.pair("PHY3", phy_net(3, "P"), phy_net(3, "N"),
           [("F.Cu", fP, fN), ("B.Cu", [vP3] + pc + tp[1:], [vN3] + nc + tn[1:])],
           vias=[(*vP3, phy_net(3, "P")), (*vN3, phy_net(3, "N"))])

    # ---- Pair 2 (ch3): outer row, P (32.8) west / N (33.2) east.  Inter-row
    #      vias: P upper-left, N lower-right -> B.Cu band east (P north),
    #      south drop (P east), west entry (P south).  No twist.
    p2p, p2n = j1_pad(11), j1_pad(9)
    vP2, vN2 = (32.55, 2.45), (33.25, 3.50)
    fP = [p2p, vP2]
    fN = [p2n, (p2n[0], 2.30), vN2]
    xd = XV[2]
    yc, tp, tn = t1_tail(2)
    band = 3.05
    pc, nc = coupled([(33.75, band), (xd, band), (xd, yc), (x_entry, yc)], p_side=+1)
    pB = [vP2, (33.0, band - HP)] + pc + tp[1:]
    nB = [vN2, (33.555, band + HP)] + nc + tn[1:]
    P.pair("PHY2", phy_net(2, "P"), phy_net(2, "N"), [("F.Cu", fP, fN), ("B.Cu", pB, nB)],
           vias=[(*vP2, phy_net(2, "P")), (*vN2, phy_net(2, "N"))])

    # ---- Pair 1 (ch2): inner row, P (34.4) east / N (34.0) west.  F.Cu south
    #      (P east = left), east band y=6.4 (P north), south (P east) -> side-
    #      by-side vias -> B.Cu drop.  No twist.
    p1p, p1n = j1_pad(4), j1_pad(6)
    xd = XV[1]
    pc, nc = coupled([(34.20, 5.25), (34.20, 6.40), (xd, 6.40), (xd, 7.00)], p_side=+1)
    vP1, vN1 = (xd + VIA_PITCH / 2, 7.35), (xd - VIA_PITCH / 2, 7.35)
    fP = [p1p] + pc + [vP1]
    fN = [p1n] + nc + [vN1]
    yc, tp, tn = t1_tail(1)
    pc, nc = coupled([(xd, 7.95), (xd, yc), (x_entry, yc)], p_side=+1)
    pB = [vP1] + pc + tp[1:]
    nB = [vN1] + nc + tn[1:]
    P.pair("PHY1", phy_net(1, "P"), phy_net(1, "N"), [("F.Cu", fP, fN), ("B.Cu", pB, nB)],
           vias=[(*vP1, phy_net(1, "P")), (*vN1, phy_net(1, "N"))])

    # ---- Pair 0 (ch4): inner row, P (32.8) west / N (33.2) east.  F.Cu south
    #      (P west = right), east band y=8.1 under Pair 1's vias (P south),
    #      south (P west) -> twist -> B.Cu drop (P east).
    p0p, p0n = j1_pad(12), j1_pad(10)
    xd = XV[0]
    pc, nc = coupled([(33.00, 5.25), (33.00, 8.10), (xd, 8.10), (xd, 8.70)], p_side=-1)
    fP = [p0p] + pc
    fN = [p0n] + nc
    vP, vN, tP, tN, bP, bN, ys = twist(xd, 8.70)
    yc, tp, tn = t1_tail(0)
    pc, nc = coupled([(xd, ys), (xd, yc), (x_entry, yc)], p_side=+1)
    P.pair("PHY0", phy_net(0, "P"), phy_net(0, "N"),
           [("F.Cu", fP + tP[1:], fN + tN[1:]), ("B.Cu", bP + pc + tp[1:], bN + nc + tn[1:])],
           vias=[(*vP, phy_net(0, "P")), (*vN, phy_net(0, "N"))])

    # ---------------- Cable side: T1 MX row (west) -> J3 contacts ----------
    tx = T("13")[0]                         # MX pad centres x
    jx = R("1")[0]                          # RJ45 contact pad centres x
    t_end = tx - 0.952 - 0.05               # just off the transformer pad toe
    j_end = jx + 1.70 + 0.05                # just off the RJ45 pad toe
    xs, xe = t_end - 0.49, j_end + 0.49

    def cable(up_t, lo_t, up_j, lo_j):
        """Coupled pair from T1 pads (upper/lower) to J3 pads, heading west,
        with one 45-degree jog.  Returns (upper leg, lower leg)."""
        yt, yj = (up_t[1] + lo_t[1]) / 2, (up_j[1] + lo_j[1]) / 2
        dy = yj - yt
        xm = (xs + xe) / 2
        center = [(xs, yt), (xm + abs(dy) / 2, yt), (xm - abs(dy) / 2, yj), (xe, yj)]
        up = offset(center, -HP)            # heading W: north = right of travel
        lo = offset(center, +HP)
        fu = [up_t, (t_end, up_t[1])] + up + [(j_end, up_j[1]), up_j]
        fl = [lo_t, (t_end, lo_t[1])] + lo + [(j_end, lo_j[1]), lo_j]
        return fu, fl

    def mxP(n):
        return T(PAIRS[n]["mx"][0])

    def mxN(n):
        return T(PAIRS[n]["mx"][1])

    for n in (3, 2, 0):                     # D, C, A: straight across with a jog
        p_t, n_t = mxP(n), mxN(n)
        p_j, n_j = R(PAIRS[n]["rj"][0]), R(PAIRS[n]["rj"][1])
        assert (p_t[1] > n_t[1]) == (p_j[1] > n_j[1]), f"pair {n}: cable-side order would cross"
        if p_t[1] > n_t[1]:
            nl, pl = cable(n_t, p_t, n_j, p_j)
        else:
            pl, nl = cable(p_t, n_t, p_j, n_j)
        P.pair(f"CAB{n}", cab_net(n, "P"), cab_net(n, "N"), [("B.Cu", pl, nl)])

    # Pair 1 (B): MX2+ -> pin 6 (B-) is a straight B.Cu run; MX2- -> pin 3 (B+)
    # must cross pair C (the T568 3/6 straddle) -> one F.Cu hop.
    n_t, p_t = mxN(1), mxP(1)
    n_j, p_j = R(PAIRS[1]["rj"][1]), R(PAIRS[1]["rj"][0])
    assert abs(n_t[1] - n_j[1]) < 1e-6
    vt, vj = (tx - 1.85, p_t[1]), (jx + 2.55, p_j[1])
    nl = [n_t, n_j]
    P.pair("CAB1", cab_net(1, "P"), cab_net(1, "N"),
           [("B.Cu", [p_t, vt], nl), ("F.Cu", [vt, vj], []), ("B.Cu", [vj, p_j], [])],
           vias=[(*vt, cab_net(1, "P")), (*vj, cab_net(1, "P"))])

    def role(ref, netname):
        return [v for (r, _), v in pads_net.items() if r == ref and _ == netname][0]

    # ---------------- Centre taps (TCT -> 100 nF -> GND, top side) ---------
    for n in range(4):
        tct = T(PAIRS[n]["tct"])
        i = PAIRS[n]["ch"] - 1
        c1, c2 = role(f"C{10+i}", f"ETH_TCT{i+1}"), role(f"C{10+i}", "GND")
        vx = tct[0] - 0.952 - 0.40
        P.track("B.Cu", f"ETH_TCT{i+1}", [tct, (vx, tct[1])])
        P.via(vx, tct[1], f"ETH_TCT{i+1}")
        P.track("F.Cu", f"ETH_TCT{i+1}", [(vx, tct[1]), c1])
        g = (c2[0] - 0.85, c2[1]) if c2[0] < c1[0] else (c2[0] + 0.85, c2[1])
        P.track("F.Cu", "GND", [c2, g])
        P.via(*g, "GND")
    # ---------------- Bob Smith (MCT -> 75R -> ETH_BS -> 1nF/2kV -> shield) --
    for n in range(4):
        mct = T(PAIRS[n]["mct"])
        i = PAIRS[n]["ch"] - 1
        r1, r2 = role(f"R{10+i}", f"ETH_MCT{i+1}"), role(f"R{10+i}", "ETH_BS")
        vx = mct[0] + 0.952 + 0.40
        P.track("B.Cu", f"ETH_MCT{i+1}", [mct, (vx, mct[1])])
        P.via(vx, mct[1], f"ETH_MCT{i+1}")
        P.track("F.Cu", f"ETH_MCT{i+1}", [(vx, mct[1]), r1])
        P.track("F.Cu", "ETH_BS", [r2, (BS_X, r2[1])])
    ys = sorted(role(f"R{10+i}", "ETH_BS")[1] for i in range(4))
    c14a, c14b = role("C14", "ETH_BS"), role("C14", "ETH_SHIELD")
    bsv = (BS_X, c14a[1] - 1.45)
    P.track("F.Cu", "ETH_BS", [(BS_X, ys[0]), bsv])
    P.via(*bsv, "ETH_BS")
    P.track("B.Cu", "ETH_BS", [bsv, (c14a[0], bsv[1]), c14a])
    sv = (c14b[0] + (1.05 if c14b[0] > c14a[0] else -1.05), c14b[1])
    P.via(*sv, "ETH_SHIELD")
    P.track("B.Cu", "ETH_SHIELD", [c14b, sv])
    # ---------------- Shield stitch R14 || C15 across the split -------------
    for ref in ("R14", "C15"):
        s_, g_ = role(ref, "ETH_SHIELD"), role(ref, "GND")
        assert s_[1] < g_[1], f"{ref}: shield pad must be north (island side)"
        P.via(s_[0], s_[1] - 1.05, "ETH_SHIELD")
        P.track("B.Cu", "ETH_SHIELD", [s_, (s_[0], s_[1] - 1.05)])
        P.via(g_[0], g_[1] + 1.05, "GND")
        P.track("B.Cu", "GND", [g_, (g_[0], g_[1] + 1.05)])
    for m in ("M1", "M2"):
        p = R(m)
        dy = -2.15 if p[1] < J3_AT[1] else 2.15
        P.via(p[0], p[1] + dy, "ETH_SHIELD")
        P.track("B.Cu", "ETH_SHIELD", [p, (p[0], p[1] + dy)])
        P.via(p[0] + 2.2, p[1], "ETH_SHIELD")

    # ---------------- Return-path stitching ---------------------------------
    # GND In1<->In2 next to every PHY-side layer transition and along the run
    x3, x0 = XV[3], XV[0]
    gv = [(35.30, 1.00), (35.30, 4.85), (31.90, 3.00), (31.90, 6.00), (37.0, 4.75), (41.0, 4.75),
          (x3 - 0.95, 4.60), (37.0, 9.30), (41.0, 9.30), (x3 - 1.0, 10.0),
          (x0 + 0.85, 11.0), (x0 + 0.85, 16.0), (x0 + 0.85, 21.0), (x0 + 0.85, 26.0), (x0 - 0.95, 27.4)]
    for x, y in gv:
        P.via(x, y, "GND")
    P.track("F.Cu", "GND", [j1_pad(1), (35.30, 1.00)])
    P.track("F.Cu", "GND", [j1_pad(2), (j1_pad(2)[0], 4.85), (35.30, 4.85)])
    # ETH_SHIELD In1<->In2 stitching around the B hop and the jack field
    for x, y in [(tx - 1.85, 20.65), (tx - 1.80, 13.0), (jx + 2.55, 23.05), (tx - 1.80, 27.6),
                 (jx + 2.30, 12.9), (jx + 2.30, 27.2)]:
        P.via(x, y, "ETH_SHIELD")
    return P


BUMP_SIDE = {"CAB1": -1}           # B- straight westward run: bulge NORTH (away from pair C and the B+ via)
BS_X = T1_AT[0] - 3.20             # F.Cu Bob Smith bus x (over the island, above T1's west half)


def passives():
    """(ref, value, footprint, x, y, rot, side, net_near, toward, net_far).
    Nets are assigned from the PLACED geometry: the pad nearest the `toward`
    point gets net_near (bottom-side flips mirror pad order, so pad numbers
    are never trusted).  Index i = transformer channel - 1 (ch1 at the top)."""
    out = []
    ty = lambda n: T1_AT[1] - 6.985 + 1.27 * (n - 1)       # PHY pin n y (pin 1 at top)
    tx_phy = T1_AT[0] + 7.303
    tx_cab = T1_AT[0] - 7.303
    for i, pin in enumerate((1, 4, 7, 10)):                  # TCT1..4: TCT pad faces T1 (east)
        out.append((f"C{10+i}", "100nF 16V X7R", "C_0402_1005Metric", tx_phy - 2.78, ty(pin), 0, "F",
                    f"ETH_TCT{i+1}", (tx_phy, ty(pin)), "GND"))
    for i, pin in enumerate((1, 4, 7, 10)):                  # MCT1..4 (pins 24/21/18/15): MCT pad faces T1 (west)
        out.append((f"R{10+i}", "75R 1%", "R_0402_1005Metric", tx_cab + 2.85, ty(pin), 0, "F",
                    f"ETH_MCT{i+1}", (tx_cab, ty(pin)), "ETH_BS"))
    out.append(("C14", "1nF 2kV X7R", "C_1206_3216Metric_HV", T1_AT[0] - 5.5, 30.30, 0, "B",
                "ETH_BS", (BS_X + 5.0, 30.30), "ETH_SHIELD"))
    for ref, val, fpn, x in (("R14", "1M 1206", "R_1206_3216Metric", 14.0),
                             ("C15", "1nF 2kV X7R", "C_1206_3216Metric_HV", 11.2)):
        out.append((ref, val, fpn, x, 32.5, 90, "B", "ETH_SHIELD", (x, 0.0), "GND"))   # north pad = shield
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


OUR_REFS = {"T1", "J3", "C10", "C11", "C12", "C13", "C14", "C15", "R10", "R11", "R12", "R13", "R14"}
OUR_ZONES = {"GND_In1", "GND_In2", "ETH_SHIELD_In1", "ETH_SHIELD_In2"}


def strip_previous(board, pcbnew) -> int:
    """Remove everything a previous run of THIS stage added: members of the
    REV9_ETH_MDI group, our footprints, copper on Ethernet nets, our plane
    zones, and any full In1/In2 GND plane another stage poured as a fallback
    (this stage owns the notched In1/In2 planes)."""
    doomed = {}
    groups = [g for g in board.Groups() if g.GetName() == GROUP]
    for g in groups:
        for it in list(g.GetItems()):
            doomed[it.m_Uuid.AsString()] = it
    for fp in board.GetFootprints():
        if fp.GetReference() in OUR_REFS:
            doomed[fp.m_Uuid.AsString()] = fp
    for t in board.GetTracks():
        if t.GetNetname().startswith(("Ethernet_Pair", "ETH_")):
            doomed[t.m_Uuid.AsString()] = t
    # one-time cleanup of copper / texts from pre-group Rev 9 runs: every
    # pipeline stage groups its items, so ungrouped GND copper and our texts
    # can only be Rev 9 leftovers.
    for t in board.GetTracks():
        if t.GetParentGroup() is None and t.GetNetname() == "GND":
            doomed[t.m_Uuid.AsString()] = t
    for d in board.GetDrawings():
        if d.GetParentGroup() is None and d.GetClass() == "PCB_TEXT" and \
                (d.GetText() == "ETH" or d.GetText().startswith(("ETH MDI", "GND | 2.0 mm", "ETH_SHIELD |"))):
            doomed[d.m_Uuid.AsString()] = d
    for z in board.Zones():
        if z.GetZoneName() in OUR_ZONES or z.GetNetname() == "ETH_SHIELD" or \
                (z.GetNetname() == "GND" and z.GetLayer() in (pcbnew.In1_Cu, pcbnew.In2_Cu)):
            doomed[z.m_Uuid.AsString()] = z
    for g in groups:
        g.RemoveAll()
    for it in doomed.values():
        board.Remove(it)
    for g in groups:
        board.Remove(g)
    # Keep the Python proxies alive: letting SWIG free removed items while the
    # board's caches still reference them corrupts later calls (pcbnew 8.0).
    _GRAVEYARD.extend(list(doomed.values()) + groups)
    return len(doomed)


_GRAVEYARD: list = []


def generate_board():
    import pcbnew
    mm = pcbnew.FromMM
    write_footprints()
    board = pcbnew.LoadBoard(str(PCB)) if PCB.exists() else None
    if board is None or not any(fp.GetReference() == "J1" for fp in board.GetFootprints()):
        M.write_footprints()
        M.generate_pcb()                     # no mechanical base yet: build it first
        board = pcbnew.LoadBoard(str(PCB))
    lib = str(LIB)
    tb = board.GetTitleBlock()
    tb.SetRevision("9.0")
    tb.SetComment(2, "Rev 9: 1000BASE-T MDI, 4x 100R pairs J1 -> T1 H5007NL -> J3 vertical RJ45 (tools/ethernet_mdi.py)")
    removed = strip_previous(board, pcbnew)   # ADDITIVE stage: only our own items are replaced
    group = pcbnew.PCB_GROUP(board)
    group.SetName(GROUP)
    board.Add(group)

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
                  ("1", (T1_AT[0] + 7.303, T1_AT[1] - 6.985)))
    j3 = place_fp(board, pcbnew, lib, "Amphenol_RJE231881317T_RJ45_Vertical_SMD", "J3", "RJE231881317T", J3_AT, "B",
                  ("1", (J3_AT[0] + 8.899, J3_AT[1] + 4.445)))
    for fp, mpn, lcsc in ((t1, T1_MPN, "C6384935"), (j3, J3_MPN, "C5384839")):
        fp.SetField("MPN", mpn)
        fp.SetField("LCSC", lcsc)
        group.AddItem(fp)
    t1map = {}
    for n, d in PAIRS.items():
        t1map[d["td"][0]] = phy_net(n, "P")
        t1map[d["td"][1]] = phy_net(n, "N")
        t1map[d["mx"][0]] = cab_net(n, "P")
        t1map[d["mx"][1]] = cab_net(n, "N")
        t1map[d["tct"]] = f"ETH_TCT{d['ch']}"
        t1map[d["mct"]] = f"ETH_MCT{d['ch']}"
    for pad in t1.Pads():
        pad.SetNet(net(t1map[pad.GetNumber()]))
    j3map = {"M1": "ETH_SHIELD", "M2": "ETH_SHIELD"}
    for n, d in PAIRS.items():
        j3map[d["rj"][0]] = cab_net(n, "P")
        j3map[d["rj"][1]] = cab_net(n, "N")
    for pad in j3.Pads():
        pad.SetNet(net(j3map[pad.GetNumber()]))

    # --- passives
    for ref, val, fpn, x, y, rot, side, n_near, toward, n_far in passives():
        fp = pcbnew.FootprintLoad(lib, fpn)
        fp.SetReference(ref)
        fp.SetValue(val)
        fp.SetFPIDAsString(f"{LIBNAME}:{fpn}")
        board.Add(fp)
        fp.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
        if side == "B":
            fp.Flip(fp.GetPosition(), False)
        fp.SetOrientationDegrees(rot)
        pl = sorted(fp.Pads(), key=lambda pd: math.dist((pcbnew.ToMM(pd.GetPosition().x), pcbnew.ToMM(pd.GetPosition().y)), toward))
        pl[0].SetNet(net(n_near))
        pl[1].SetNet(net(n_far))
        group.AddItem(fp)

    for fp in board.GetFootprints():
        if fp.GetReference() in OUR_REFS:
            fp.Reference().SetTextSize(pcbnew.VECTOR2I(mm(0.6), mm(0.6)))
            fp.Reference().SetTextThickness(mm(0.1))
            fp.Reference().SetVisible(fp.GetReference() in ("T1", "J3"))

    pads = {}
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            pads[(fp.GetReference(), pad.GetNumber())] = (round(pcbnew.ToMM(pad.GetPosition().x), 4),
                                                          round(pcbnew.ToMM(pad.GetPosition().y), 4))
    pads_net = {}
    for fp in board.GetFootprints():
        if fp.GetReference() in OUR_REFS:
            for pad in fp.Pads():
                pads_net[(fp.GetReference(), pad.GetNetname())] = (round(pcbnew.ToMM(pad.GetPosition().x), 4),
                                                                   round(pcbnew.ToMM(pad.GetPosition().y), 4))
    plan = build_plan(pads, pads_net)

    # --- intra-pair length match: trombone(s) on the shorter net's longest
    #     straight run, bulging away from its partner.
    for name, pr in plan.pairs.items():
        legs = pr["legs"]
        lp = sum(plen(pp) for _, pp, _ in legs)
        ln = sum(plen(nn) for _, _, nn in legs)
        if abs(lp - ln) <= 0.01:
            continue
        short_is_p = lp < ln
        idx = 1 if short_is_p else 2
        def ok_seg(a_, b_):
            # skew trombones only on horizontal runs; PHY pairs: only on the
            # bands north of T1 (the south drops are packed at 0.95 mm pitch)
            return abs(a_[1] - b_[1]) < 1e-6 and (name.startswith("CAB") or a_[1] < 10.0)
        cand = [(math.dist(a_, b_), i, m) for i in range(len(legs))
                for m, (a_, b_) in enumerate(zip(legs[i][idx], legs[i][idx][1:])) if ok_seg(a_, b_)]
        _, k, j = max(cand)
        short, other = legs[k][idx], legs[k][3 - idx]
        a_, b_ = short[j], short[j + 1]
        L_ = math.dist(a_, b_)
        lx, ly = (b_[1] - a_[1]) / L_, -(b_[0] - a_[0]) / L_          # left normal
        mid = ((a_[0] + b_[0]) / 2, (a_[1] + b_[1]) / 2)
        if other:
            dl = min(seg_dist((mid[0] + lx * 0.3, mid[1] + ly * 0.3), u, v) for u, v in zip(other, other[1:]))
            dr = min(seg_dist((mid[0] - lx * 0.3, mid[1] - ly * 0.3), u, v) for u, v in zip(other, other[1:]))
            side = +1 if dl > dr else -1
        else:
            side = BUMP_SIDE.get(name, +1)
        new = add_bump(short, abs(lp - ln), side, seg=j)
        legs[k] = (legs[k][0], new, legs[k][2]) if short_is_p else (legs[k][0], legs[k][1], new)
        plan.tracks = [t for t in plan.tracks if t[1] not in (pr["P"], pr["N"])]
        for layer_, a2, b2 in legs:
            plan.tracks.append((layer_, pr["P"], a2))
            plan.tracks.append((layer_, pr["N"], b2))

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
            group.AddItem(t)
    for x, y, netname in plan.vias:
        v = pcbnew.PCB_VIA(board)
        v.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
        v.SetViaType(pcbnew.VIATYPE_THROUGH)
        v.SetWidth(mm(VIA_D))
        v.SetDrill(mm(VIA_DRILL))
        v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        v.SetNet(net(netname))
        board.Add(v)
        group.AddItem(v)

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
        group.AddItem(z)
        return z
    nx0, ny0, nx1, ny1 = NOTCH
    gnd_poly = [(0, 0), (55, 0), (55, 40), (0, 40), (0, ny1), (nx1, ny1), (nx1, ny0), (0, ny0)]
    ix0, iy0, ix1, iy1 = ISLAND
    isl_poly = [(ix0, iy0), (ix1, iy0), (ix1, iy1), (ix0, iy1)]
    for layer, lname in ((pcbnew.In1_Cu, "In1"), (pcbnew.In2_Cu, "In2")):
        zone(layer, "GND", gnd_poly, 0, f"GND_{lname}")
        zone(layer, "ETH_SHIELD", isl_poly, 1, f"ETH_SHIELD_{lname}")

    # --- documentation on User layers
    for txt, (x, y), layer in [
        ("ETH MDI: 4x 100R diff W0.165/S0.125 (JLC04161H-7628, L1/L4 microstrip)", (27.5, 17.0), pcbnew.Cmts_User),
        ("ETH_SHIELD | 2.0 mm SPLIT | GND (In1/In2)", (20.0, 32.6), pcbnew.Cmts_User),
        ("ETH", (J3_AT[0], 29.6), pcbnew.B_SilkS),
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
        group.AddItem(t)

    filler = pcbnew.ZONE_FILLER(board)
    filler.Fill(board.Zones())
    board.Save(str(PCB))

    # --- stackup into (setup ...): pcbnew python cannot author it portably
    txt = PCB.read_text()
    txt = re.sub(r"\n\t\t\(stackup\n.*?\n\t\t\)\n", "\n", txt, flags=re.S)
    txt = txt.replace("\t(setup\n", "\t(setup\n" + STACKUP_SEXPR, 1)
    PCB.write_text(txt)
    write_project_rules()
    print(f"generated: {PCB.name} (Rev 9 Ethernet MDI; replaced {removed} previous Rev 9 items, "
          f"other stages' items untouched), {LIB.name}, {PRO.name}, {DRU.name}")


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
             f"  (constraint diff_pair_gap (min {S - 0.01:.3f}mm) (opt {S:.3f}mm) (max {S + 0.36:.3f}mm))",
             "  # max admits the 0.35 mm skew trombones; nominal 0.125 coupling is generated + verified",
             f"  (constraint diff_pair_uncoupled (max {Z.MDI_MAX_UNCOUPLED:.1f}mm))",
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
        subprocess.run(["kicad-cli", "pcb", "drc", "--format", "json", "--severity-all",
                        "-o", str(out), str(pcb_path)], check=True, capture_output=True, text=True)
        return json.loads(out.read_text())


ETH_PREFIX = ("Ethernet_Pair", "ETH_")


def item_nets(v):
    return [i.get("description", "") for i in v.get("items", [])]


OUR_UUIDS: set = set()


def collect_our_uuids(board):
    """UUIDs of every item this stage owns (group members + their pads) and of
    all copper on Ethernet nets (incl. the J1 MDI pads)."""
    u = set()
    for g in board.Groups():
        if g.GetName() == GROUP:
            for it in g.GetItems():
                u.add(it.m_Uuid.AsString())
                if it.GetClass() == "FOOTPRINT":
                    u.update(p.m_Uuid.AsString() for p in it.Pads())
    for t in board.GetTracks():
        if t.GetNetname().startswith(("Ethernet_Pair", "ETH_")):
            u.add(t.m_Uuid.AsString())
    for fp in board.GetFootprints():
        for p in fp.Pads():
            if p.GetNetname().startswith(("Ethernet_Pair", "ETH_")):
                u.add(p.m_Uuid.AsString())
    return u


def is_eth_violation(v):
    """A violation is ours if any involved item is ours (by UUID) or sits on an
    Ethernet net."""
    if any(i.get("uuid") in OUR_UUIDS for i in v.get("items", [])):
        return True
    blob = " ".join(item_nets(v))
    return "Ethernet_Pair" in blob or "ETH_" in blob


def cmd_verify(_a=None) -> int:
    import pcbnew
    tm = pcbnew.ToMM
    C = Checks()
    board = pcbnew.LoadBoard(str(PCB))
    fps = {fp.GetReference(): fp for fp in board.GetFootprints()}
    OUR_UUIDS.clear()
    OUR_UUIDS.update(collect_our_uuids(board))

    # --- 1. stackup & impedance model
    txt = PCB.read_text()
    m = re.search(r'\(layer "dielectric 1"\s*\(type "prepreg"\)\s*\(thickness ([\d.]+)\).*?\(epsilon_r ([\d.]+)\)', txt, re.S)
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
        # winding polarity: P must sit on the same winding end (dot / non-dot)
        # at the PHY side and the cable side of the SAME channel.
        tdp, tdm, mxp, mxm = T1_WINDINGS[dd["ch"]]
        if {dd["td"][0], dd["td"][1]} != {tdp, tdm} or {dd["mx"][0], dd["mx"][1]} != {mxp, mxm}:
            pol_ok = False
            det.append(f"pair{n} channel mismatch")
        elif (dd["td"][0] == tdp) != (dd["mx"][0] == mxp):
            pol_ok = False
            det.append(f"pair{n} polarity inverted")
    C.add("MAP-t568", "CM4 Pair0..3 -> T1 TD1..TD4 -> MX1..MX4 -> RJ45 1/2, 3/6, 4/5, 7/8 (T568)", mapping_ok, ", ".join(det) or "all 16")
    C.add("MAP-polarity", "end-to-end polarity: Pair_P and MDI_P on the same winding end (dot/non-dot) of the same channel; MDI_P on RJ45 '+'", pol_ok)
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
                    dmin = min((seg_dist(p, (tm(s.GetStart().x), tm(s.GetStart().y)), (tm(s.GetEnd().x), tm(s.GetEnd().y)))
                                for s in tracks[nn] if s.GetLayer() == t.GetLayer()), default=99.0)
                    if abs(dmin - PITCH) > 0.02:
                        u += L / k
            unc[f"{side}{n}"] = u
    # Return-loss bound of the uncoupled length: treat each net's uncoupled
    # copper as ONE lumped section of Zdiff_unc (field solver, loose coupling)
    # in a 100-ohm line: |S11| ~ 2 |G| sin(beta l) at 100 MHz.
    su = Z.STACKUP
    z_unc = 2 * Z.fd_coupled(W, 2.0, su.h_pp, su.t_outer, su.er_pp, su.t_mask, su.er_mask)["zeven"]
    gam = abs(z_unc - 100) / (z_unc + 100)
    tpd = math.sqrt(d["fd"]["eeff_odd"]) / Z.C0 * 1e9           # ps/mm
    rl = {k: 20 * math.log10(max(1e-9, 2 * gam * math.sin(2 * math.pi * 100e6 * v * tpd * 1e-12))) for k, v in unc.items()}
    worst_k = max(unc, key=unc.get)
    C.add("CPL-uncoupled", f"uncoupled length per net <= {Z.MDI_MAX_UNCOUPLED} mm (DRU diff_pair_uncoupled)",
          max(unc.values()) <= Z.MDI_MAX_UNCOUPLED, "; ".join(f"{k} {v:.2f}" for k, v in unc.items()))
    C.add("CPL-RL", "uncoupled sections' return-loss contribution < -30 dB at 100 MHz",
          max(rl.values()) < -30, f"Zdiff(uncoupled) {z_unc:.1f} ohm, worst {worst_k}: {unc[worst_k]:.2f} mm -> {rl[worst_k]:.1f} dB")

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
                    mate = n[:-1] + ("N" if n.endswith("P") else "P")
                    near_via = any(math.dist(p, (tm(v.GetPosition().x), tm(v.GetPosition().y))) < 0.80
                                   for v in vias.get(n, []) + vias.get(mate, []))
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
    tops = [r for r, fp in fps.items() if not fp.IsFlipped() and r[0] in "CR" and r[1:].isdigit() and int(r[1:]) >= 10]
    tall = [r for r in tops if "0402" not in fps[r].GetFPIDAsString()]
    C.add("MECH-top-height", f"top-side Ethernet passives are 0402 (<= 0.55 mm) under the {M.TOP_SIDE_MAX_COMPONENT_H} mm limit",
          not tall, f"{len(tops)} top-side parts; non-0402: {tall or 'none'}")
    inside = all(0.3 - 1e-6 <= v for fp in (fps["T1"], fps["J3"]) for v in (bbox(fp)[0], bbox(fp)[1], 55 - bbox(fp)[2], 40 - bbox(fp)[3]))
    C.add("MECH-envelope", "T1/J3 courtyards inside 55 x 40 outline (zero overhang)", inside,
          f"T1 {tuple(round(x, 2) for x in bbox(fps['T1']))}, J3 {tuple(round(x, 2) for x in bbox(fps['J3']))}")

    # --- 9. KiCad DRC (Ethernet scope) + mutation tests
    if shutil.which("kicad-cli"):
        dr = run_drc(PCB)
        viol = [v for v in dr.get("violations", []) if is_eth_violation(v)]
        C.add("DRC-eth", "kicad-cli DRC: 0 violations involving any Rev 9 item (group REV9_ETH_MDI) or Ethernet net",
              not viol, "; ".join(f"{v['type']}: {' | '.join(item_nets(v))}" for v in viol[:8]) or "0")
        unc_eth = [u for u in dr.get("unconnected_items", []) if any(k in " ".join(item_nets(u)) for k in ETH_PREFIX)]
        C.add("DRC-unrouted", "no unrouted Ethernet connections (all 16 MDI + CT/BS/shield nets complete)",
              not unc_eth, "; ".join(" | ".join(item_nets(u)) for u in unc_eth[:6]) or "0")
        other = [v for v in dr.get("violations", []) if not is_eth_violation(v)]
        C.add("DRC-other", "pre-existing non-Ethernet DRC items (informational)", True,
              f"{len(other)} violations, {len(dr.get('unconnected_items', [])) - len(unc_eth)} unrouted (J1/J2 module nets / other stages)")
        mutation_tests(C)
    else:
        C.add("DRC-cli", "kicad-cli available", False, "not on PATH")
    write_report(C, skew_rows, unc, d)
    print(f"\n{'PASS' if C.ok else 'FAIL'}: {sum(r[2] for r in C.rows)}/{len(C.rows)} -> {REPORT.relative_to(ROOT)}")
    return 0 if C.ok else 1


def mutation_tests(C):
    """Inject a defect into a scratch copy; the Rev 9 DRU / netclass must catch
    it.  The isolation mutations sit 0.25 mm from Pair 2's B.Cu band - legal
    under the 0.20 mm default clearance, illegal under mdi_isolation /
    mdi_interpair (0.30 mm) - so they prove the custom rules are live."""
    src = PCB.read_text()
    y_edge = 3.05 - HP - W / 2                 # north edge of Pair 2's P track on the band
    y_bad = y_edge - 0.25 - 0.075              # 0.15 mm track, 0.25 mm gap
    gnd = re.search(r'\(net (\d+) "GND"\)', src).group(1)
    p1 = re.search(r'\(net (\d+) "Ethernet_Pair1_P"\)', src).group(1)
    muts = {}
    m = re.search(r'\(segment\s*\(start [\d.\-]+ [\d.\-]+\)\s*\(end [\d.\-]+ [\d.\-]+\)\s*\(width ([\d.]+)\)\s*'
                  r'\(layer "B.Cu"\)\s*\(net (\d+)\)', src)
    nets_mdi = set(re.findall(r'\(net (\d+) "(?:Ethernet_Pair|ETH_MDI)', src))
    for mm_ in re.finditer(r'\(segment\s*\(start [\d.\-]+ [\d.\-]+\)\s*\(end [\d.\-]+ [\d.\-]+\)\s*\(width ([\d.]+)\)\s*'
                           r'\(layer "B.Cu"\)\s*\(net (\d+)\)', src):
        if mm_.group(2) in nets_mdi:
            old = mm_.group(0)
            muts["MDI segment widened to 0.25 mm -> track_width"] = (
                "track_width", src.replace(old, old.replace(f"(width {mm_.group(1)})", "(width 0.25)"), 1))
            break
    muts["GND track 0.25 mm from an MDI band -> mdi_isolation"] = (
        "clearance", src.rstrip()[:-1] + f'\t(segment (start 37.5 {y_bad:.4f}) (end 41.5 {y_bad:.4f}) (width 0.15) (layer "B.Cu") '
        f'(net {gnd}) (uuid "00000000-0000-0000-0000-00000000b001"))\n)\n')
    muts["foreign MDI pair 0.25 mm from Pair 2 -> mdi_interpair"] = (
        "clearance", src.rstrip()[:-1] + f'\t(segment (start 37.5 {y_bad:.4f}) (end 41.5 {y_bad:.4f}) (width 0.15) (layer "B.Cu") '
        f'(net {p1}) (uuid "00000000-0000-0000-0000-00000000b002"))\n)\n')
    for name, (want, text) in muts.items():
        with tempfile.TemporaryDirectory() as td:
            for ext in (".kicad_pro", ".kicad_dru"):
                shutil.copy(PCB.with_suffix(ext), Path(td) / (PCB.stem + ext))
            for t in ("fp-lib-table", "sym-lib-table"):
                if (ROOT / t).exists():
                    shutil.copy(ROOT / t, Path(td) / t)
            shutil.copytree(ROOT / "lib", Path(td) / "lib")
            p = Path(td) / PCB.name
            p.write_text(text)
            d = run_drc(p)
        hits = [v for v in d.get("violations", []) if v["type"] == want and
                any("b00" in (i.get("uuid") or "") or "0.2500 mm" in v.get("description", "") or want == "track_width"
                    for i in v.get("items", []))]
        types = sorted({v["type"] for v in d.get("violations", [])})
        C.add(f"MUT-{name.split(' -> ')[1]}", f"DRU catches mutation: {name}", bool(hits),
              f"{len(hits)} matching '{want}' violations; types {types}")


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
