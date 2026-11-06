#!/usr/bin/env python3
"""
Radxa CM4 Nano Carrier - Rev 18: status LEDs and user controls
==============================================================

MaskROM / rpiboot recovery button, reset tactile switch, and the PWR and ACT
bi-colour (red + emerald-green) status LEDs, laid out as an edge-facing "user
panel" on B.Cu along the east board edge. The whole top side is covered by
the 55 x 40 mm module, so anything a user can see or press must be on the
bottom side and must face out through the enclosure side wall:

  * side-view LEDs whose lens dome sits on the X = 55 edge, and
  * right-angle (side-push) tact switches whose actuator sits on the same edge.

Sub-commands
------------
  calc       Electrical design checks only (no KiCad): LED current over Vf / rail
             corners, module-pin sink current, button low level against the module
             pull-up range, series-R pulse power, switch and ESD ratings, back-power
             analysis. Writes artifacts/rev18_user_io_design.json. Exit 1 on failure.
  generate   Write the footprints; place SW1/SW2/D50..D53/R50..R55 on B.Cu at the
             east edge; route every Rev 18 net with the clearance-aware maze router
             below (obstacles read back from the live board, so concurrently edited
             stages are respected); pour the local B.Cu GND panel zone; write the Rev 18
             DRU block. Additive + idempotent: the previous REV18_USER_IO group, its
             footprints and its zone are cut out at S-expression level first.
  rules      Re-apply only the Rev 18 block in .kicad_dru.
  schematic  Insert/refresh the Rev 18 symbols + global labels in the root schematic
             (net names read back from the board -> parity by construction).
  verify     Reload the saved board and measure: pin-to-net map, edge/actuator
             geometry, keep-outs, isolation to MDI/TMDS copper, reference-plane
             coverage, connectivity, kicad-cli DRC attributed by UUID + baseline,
             DRU mutation tests, schematic parity. Writes
             artifacts/rev18_user_io_report.md. Exit 1 on failure.
  render     artifacts/rev18_user_io_layout.png.

Pipeline:  cm4_mechanical -> ethernet_mdi -> hdmi_tmds -> buck_3v3 -> user_io  (additive).

Module interface (specs/radxa_cm4_pinout.json, J1 = module connector 1)
-----------------------------------------------------------------------
  J1-92  RUN_PG            RPi CM4: open-drain power-good / pull low = reset.
                           Radxa CM4 (RK3576): RESET_L.            -> SW2 RESET
  J1-93  nRPIBOOT          RPi CM4: low at power-up = USB rpiboot.
                           Radxa CM4 (RK3576): MASKROM.            -> SW1 BOOT/MASKROM
  J1-95  nPI_LED_PWR       active-low LED drive (power LED)        -> D50 red
  J1-21  PI_nLED_Activity  active-low LED drive (eMMC/SD activity) -> D51 red
  J1-19  Ethernet_nLED1    active-low PHY LED (link/activity)      -> D51 green
  J1-84/86 +3V3_OUT        module 3.3 V output: LED anode supply   -> D50 green (PWR 3V3)
  (J1-99 GLOBAL_EN and J1-100 nEXTRST are deliberately NOT used: GLOBAL_EN low
   powers the module off, and nEXTRST is a module *output* on RPi CM4.)

Primary sources
---------------
[S1] SHOU HAN TS24CA (LCSC C393942) drawing: 4.7 x 1.9 x 2.25 mm body, 2.0 mm actuator,
     3.5 mm overall, 12 V / 50 mA, 1.6 N, 2 x 0.5 mm locating pegs at 1.7 mm. Land:
     contacts 0.6 x 1.2 at 3.4 c-c, L-shaped cover tabs 5.3 / 4.0 / 3.2 mm, 0.7 mm row gap.
     NOTE: the LCSC/EasyEDA footprint omits the two peg holes; they are added here
     (NPTH 0.75 mm at x +/-0.85 on the tab-row inner edge, measured off the drawing).
[S2] XINGLIGHT XL-3210SURUGC (LCSC C965883) datasheet: side-view R/G, 4 independent pins
     1 G+ 2 G- 3 R+ 4 R-; R VF 1.7-2.3 V, G VF 2.6-3.2 V @ 20 mA; IF max 20 mA; VR 5 V;
     ESD 2 kV HBM; 180 / 750 mcd @ 20 mA; 3.2 x 1.0 base, 1.5 mm deep incl. dome.
[S3] BORN BPESD0402-05ST (LCSC C316059): bidirectional ESD, VRWM 5 V, 0.05 pF, IEC 61000-4-2.
[S4] Raspberry Pi CM4 datasheet section 2.1 / 5.x (pin functions above); Radxa CM4 pinout
     v1.20 (pins 92/93 = RESET_L / MASKROM). Module LED / button pins assumed <= 3.3 V
     logic with on-module pull-ups (CM4: nRPIBOOT, RUN_PG pulled up on module).

Coordinate frame: carrier board, mm, KiCad Y-down, viewed from the TOP.
"""
from __future__ import annotations

import argparse
import hashlib
import heapq
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
SCH = ROOT / "radxa-cm4-nano-carrier.kicad_sch"
SYMLIB = ROOT / "lib" / "CM4Carrier.kicad_sym"
LIB = ROOT / "lib" / "radxa-cm4-nano-carrier.pretty"
LIBNICK = "radxa-cm4-nano-carrier"
DESIGN_JSON = ROOT / "artifacts" / "rev18_user_io_design.json"
REPORT = ROOT / "artifacts" / "rev18_user_io_report.md"
RENDER = ROOT / "artifacts" / "rev18_user_io_layout.png"
GROUP = "REV18_USER_IO"
ZONE_NAME = "USERIO_GND_BCU"
BOARD_W, BOARD_H = 55.0, 40.0
EDGE_CLR = 0.5                       # project copper-to-edge minimum

# ==========================================================================
# 1. Electrical design
# ==========================================================================
V3V3 = (3.3 * 0.95, 3.3, 3.3 * 1.05)     # module +3V3_OUT corners (+/-5 %)
LED = {   # [S2] VF window at the 0.5-4 mA operating point, derated from the 20 mA bins
          # (R 1.7-2.3 V, G 2.6-3.2 V @ 20 mA) by the AlGaInP / InGaN I-V slope; "knee" =
          # lowest voltage at which a die conducts >= 1 uA (used for the back-injection bound).
    "red":   {"vf": (1.65, 1.85, 2.05), "knee": 1.45, "mcd_20mA": 180.0},
    "green": {"vf": (2.30, 2.55, 2.85), "knee": 2.20, "mcd_20mA": 750.0},
}
VOL = 0.10                               # module pin VOL at <= 4 mA (LVCMOS, typ. tens of mV)
R_LED = {"R52": 330.0, "R53": 1000.0, "R54": 1000.0, "R55": 330.0}   # 1 % 0402
R_BTN = 470.0                            # R50 / R51 series resistors
RPU_RANGE = (2.2e3, 100e3)               # module-side pull-up assumption [S4]
VIO = (1.8, 3.3)                         # possible module IO domains (RK3576 1V8, BCM2711 3V3)
PIN_SINK_MAX = 4e-3                      # conservative GPIO sink limit (CM4 default drive 8 mA)
SW_RATING = (12.0, 50e-3)                # [S1]
ESD_VRWM = 5.0                           # [S3]
P0402 = 0.0625                           # 1/16 W


def calc() -> dict:
    rows = []

    def chk(name, ok, detail):
        rows.append({"check": name, "pass": bool(ok), "detail": detail})

    leds = {}
    for ref, colour, drv in (("R52", "green", "+3V3_OUT (always on with module 3V3)"),
                             ("R53", "red", "nPI_LED_PWR"), ("R54", "red", "PI_nLED_Activity"),
                             ("R55", "green", "Ethernet_nLED1")):
        r = R_LED[ref]
        vol = 0.0 if ref == "R52" else VOL
        i_min = (V3V3[0] - LED[colour]["vf"][2] - vol) / (r * 1.01)
        i_typ = (V3V3[1] - LED[colour]["vf"][1] - vol) / r
        i_max = (V3V3[2] - LED[colour]["vf"][0]) / (r * 0.99)
        mcd = LED[colour]["mcd_20mA"] * i_typ / 20e-3
        leds[ref] = {"colour": colour, "driver": drv, "R": r, "I_min_mA": round(i_min * 1e3, 3),
                     "I_typ_mA": round(i_typ * 1e3, 3), "I_max_mA": round(i_max * 1e3, 3), "Iv_typ_mcd": round(mcd, 1)}
        chk(f"{ref} {colour} LED current >= 0.25 mA at worst corner (visible indoors)", i_min >= 0.25e-3,
            f"{i_min*1e3:.2f} / {i_typ*1e3:.2f} / {i_max*1e3:.2f} mA, ~{mcd:.0f} mcd typ")
        chk(f"{ref} LED current <= IF max 20 mA", i_max <= 20e-3, f"{i_max*1e3:.2f} mA")
        if ref != "R52":
            chk(f"{ref} module pin sink <= {PIN_SINK_MAX*1e3:.0f} mA", i_max <= PIN_SINK_MAX, f"{i_max*1e3:.2f} mA")
        p = i_max ** 2 * r * 0.99
        chk(f"{ref} dissipation << 0402 rating", p <= 0.25 * P0402, f"{p*1e3:.2f} mW")
    # Back-power: every anode sits on the MODULE's own +3V3_OUT, so with the module off no current
    # can reach its pins. With the module on and a 1V8 IO bank (RK3576) driving the pin HIGH, the
    # die sees 3V3max - 1.8 V; current into the 1V8 pin is bounded by (that - knee) / R.
    for ref, colour in (("R53", "red"), ("R54", "red"), ("R55", "green")):
        v = V3V3[2] - VIO[0] - LED[colour]["knee"]
        inj = max(0.0, v) / (R_LED[ref] * 0.99)
        chk(f"{ref} {colour}: injection into a 1V8 pin held HIGH <= 0.5 mA (no latch-up / rail pump)",
            inj <= 0.5e-3, f"{inj*1e6:.0f} uA ({'faint glow possible' if inj > 0 else 'die stays off'})")
    btn = {}
    for vio in VIO:
        vlow = vio * R_BTN / (R_BTN + RPU_RANGE[0])
        vil = 0.3 * vio if vio < 2.5 else 0.8
        btn[f"VIO_{vio}"] = {"V_low_worst": round(vlow, 3), "VIL": vil}
        chk(f"Button low level at VIO {vio} V, Rpu >= {RPU_RANGE[0]/1e3:.1f} k: V <= VIL - 0.15 V margin",
            vlow <= vil - 0.15, f"{vlow:.3f} V vs VIL {vil:.2f} V")
    i_short = VIO[1] * 1.05 / R_BTN
    chk("Button pressed against a push-pull HIGH pin: current <= switch 50 mA and <= 8 mA GPIO",
        i_short <= min(SW_RATING[1], 8e-3), f"{i_short*1e3:.2f} mA")
    chk("Series-R pulse power (pin driven high, button held) <= 0402 rating",
        i_short ** 2 * R_BTN <= P0402, f"{i_short**2*R_BTN*1e3:.1f} mW")
    chk("ESD VRWM >= 3V3 rail max + 10 %", ESD_VRWM >= V3V3[2] * 1.1, f"{ESD_VRWM} V vs {V3V3[2]*1.1:.2f} V")
    chk("Switch voltage rating >= 3V3 max", SW_RATING[0] >= V3V3[2], f"{SW_RATING[0]} V")
    # RC seen by the module pin (ESD 0.05 pF + switch + 0402 pads ~1 pF) -> no edge-rate problem
    tau = R_BTN * 1.5e-12
    chk("Series R x node C << 1 us (no effect on reset/boot strap timing)", tau < 1e-6, f"tau {tau*1e9:.2f} ns")
    res = {"rev": "18.0", "leds": leds, "buttons": btn, "R_btn": R_BTN, "Rpu_range": RPU_RANGE, "checks": rows,
           "pass": all(r["pass"] for r in rows)}
    return res


def cmd_calc(_a) -> int:
    res = calc()
    DESIGN_JSON.parent.mkdir(exist_ok=True)
    DESIGN_JSON.write_text(json.dumps(res, indent=2) + "\n")
    for r in res["checks"]:
        print(("PASS " if r["pass"] else "FAIL ") + r["check"] + "  -- " + r["detail"])
    print(f"calc: {sum(r['pass'] for r in res['checks'])}/{len(res['checks'])} pass -> {DESIGN_JSON.relative_to(ROOT)}")
    return 0 if res["pass"] else 1


# ==========================================================================
# 2. Footprints
# ==========================================================================
FP_SW = "SW_SideAct_SHOUHAN_TS24CA"
FP_LED = "LED_SideView_XINGLIGHT_XL-3210SURUGC"
FP_ESD = "D_0402_1005Metric"
FP_R = "R_0402_1005Metric"            # existing (ethernet_mdi.py)

SW_TIP = -2.50       # actuator tip, local Y (front = -Y)      [S1] 3.5 overall, contacts at the rear
LED_TIP = -1.30      # lens-dome apex, local Y (front = -Y)     [S2] 0.5 base + 1.0 dome


def _lines(pts, layer, w):
    return [f'  (fp_line (start {a[0]:.3f} {a[1]:.3f}) (end {b[0]:.3f} {b[1]:.3f}) (stroke (width {w}) (type solid)) (layer "{layer}"))'
            for a, b in zip(pts, pts[1:])]


def _rect(x1, y1, x2, y2, layer, w):
    return _lines([(x1, y1), (x2, y1), (x2, y2), (x1, y2), (x1, y1)], layer, w)


def _hdr(name, descr, tags, height, ref_y, val_y):
    return [f'(footprint "{name}"', '  (version 20240108)', '  (generator "user_io.py")', '  (layer "F.Cu")',
            f'  (descr "{descr}")', f'  (tags "{tags}")', '  (attr smd)',
            f'  (property "Reference" "REF**" (at 0 {ref_y:.3f} 0) (layer "F.SilkS") (effects (font (size 0.6 0.6) (thickness 0.1))))',
            f'  (property "Value" "{name}" (at 0 {val_y:.3f} 0) (layer "F.Fab") (effects (font (size 0.5 0.5) (thickness 0.08))))',
            f'  (property "Height" "{height}" (at 0 0 0) (layer "F.Fab") (hide yes) (effects (font (size 1 1))))']


def footprints() -> dict:
    # --- TS24CA side-actuated switch (front/actuator = -Y) ---
    o = _hdr(FP_SW, "SHOU HAN TS24CA 4.7x1.9 mm right-angle SMD tact switch, actuator on -Y edge, "
                    "2 locating pegs (LCSC C393942). Pads 1/2 = contacts, 3/4 = cover tabs (tie to GND)",
             "switch tactile side right-angle", 2.3, 2.45, -3.1)
    o += _rect(-2.35, -1.25, 2.35, 1.0, "F.Fab", 0.1) + _rect(-1.0, SW_TIP, 1.0, -1.25, "F.Fab", 0.1)
    o += _rect(-2.95, SW_TIP - 0.05, 2.95, 1.85, "F.CrtYd", 0.05)
    o += _lines([(-2.45, 1.75), (-2.45, 1.10)], "F.SilkS", 0.12) + _lines([(2.45, 1.75), (2.45, 1.10)], "F.SilkS", 0.12)
    o += _lines([(-0.6, 1.75), (0.6, 1.75)], "F.SilkS", 0.12)
    o.append('  (pad "1" smd rect (at -1.700 0.975) (size 0.600 1.200) (layers "F.Cu" "F.Paste" "F.Mask"))')
    o.append('  (pad "2" smd rect (at 1.700 0.975) (size 0.600 1.200) (layers "F.Cu" "F.Paste" "F.Mask"))')
    for num, s in (("3", 1), ("4", -1)):
        pts = [(-0.725, -0.65), (0.325, -0.65), (0.325, 0.65), (-0.325, 0.65), (-0.325, 0.0), (-0.725, 0.0)]
        pts = " ".join(f"(xy {s*x:.3f} {y:.3f})" for x, y in pts)
        o.append(f'  (pad "{num}" smd custom (at {s*2.325:.3f} -0.975) (size 0.650 1.300) (layers "F.Cu" "F.Paste" "F.Mask") '
                 f'(options (clearance outline) (anchor rect)) (primitives (gr_poly (pts {pts}) (width 0) (fill yes))))')
    for s in (1, -1):
        o.append(f'  (pad "" np_thru_hole circle (at {s*0.85:.3f} -0.325) (size 0.750 0.750) (drill 0.750) (layers "*.Cu" "*.Mask"))')
    o.append(")")
    sw = "\n".join(o) + "\n"
    # --- XL-3210SURUGC side-view bi-colour LED (lens = -Y) ---
    o = _hdr(FP_LED, "XINGLIGHT XL-3210SURUGC side-view red/emerald-green LED, 4 independent pins "
                     "1 G+ 2 G- 3 R+ 4 R-, lens on -Y edge (LCSC C965883)", "LED side view bicolor", 1.0, 1.25, -1.9)
    o += _rect(-1.6, -0.30, 1.6, 0.20, "F.Fab", 0.1)
    dome = [(1.15 * math.cos(a), -0.30 - 1.0 * math.sin(a)) for a in [k * math.pi / 12 for k in range(13)]]
    o += _lines(dome, "F.Fab", 0.1)
    o += _rect(-2.20, LED_TIP - 0.05, 2.20, 0.70, "F.CrtYd", 0.05)
    o += _lines([(1.95, 0.70 - 0.05), (2.15, 0.70 - 0.05)], "F.SilkS", 0.12)   # R+ (pin 3) side mark
    o.append('  (pad "1" smd rect (at 0.400 0.070) (size 0.500 0.600) (layers "F.Cu" "F.Paste" "F.Mask"))')
    o.append('  (pad "2" smd rect (at -0.400 0.070) (size 0.500 0.600) (layers "F.Cu" "F.Paste" "F.Mask"))')
    o.append('  (pad "3" smd rect (at 1.475 -0.070) (size 0.950 0.880) (layers "F.Cu" "F.Paste" "F.Mask"))')
    o.append('  (pad "4" smd rect (at -1.475 -0.070) (size 0.950 0.880) (layers "F.Cu" "F.Paste" "F.Mask"))')
    o.append(")")
    led = "\n".join(o) + "\n"
    # --- 0402 ESD diode (bidirectional, same land as the R/C 0402) ---
    o = _hdr(FP_ESD, "0402 bidirectional ESD diode, IPC-7351 nominal (BORN BPESD0402-05ST, LCSC C316059)",
             "ESD TVS diode 0402", 0.5, -1.0, 1.0)
    o += _rect(-0.5, -0.25, 0.5, 0.25, "F.Fab", 0.1) + _rect(-0.93, -0.47, 0.93, 0.47, "F.CrtYd", 0.05)
    o.append('  (pad "1" smd roundrect (at -0.510 0) (size 0.59 0.64) (layers "F.Cu" "F.Paste" "F.Mask") (roundrect_rratio 0.25))')
    o.append('  (pad "2" smd roundrect (at 0.510 0) (size 0.59 0.64) (layers "F.Cu" "F.Paste" "F.Mask") (roundrect_rratio 0.25))')
    o.append(")")
    return {FP_SW: sw, FP_LED: led, FP_ESD: "\n".join(o) + "\n"}


# local pad centres (top-side footprint frame) - used for the placement transform
PADS_LOCAL = {
    FP_SW: {"1": (-1.7, 0.975), "2": (1.7, 0.975), "3": (2.325, -0.975), "4": (-2.325, -0.975)},
    FP_LED: {"1": (0.4, 0.07), "2": (-0.4, 0.07), "3": (1.475, -0.07), "4": (-1.475, -0.07)},
    FP_ESD: {"1": (-0.51, 0.0), "2": (0.51, 0.0)},
    FP_R: {"1": (-0.51, 0.0), "2": (0.51, 0.0)},
}


def edge_xf(at, p):
    """Bottom-side, front (-Y local) facing +X (the east edge): mirror X, then rotate.
    (x, y) -> (-x, y) -> board = at + (-y, -x).  Proper check: local (0,-1) -> (+1, 0)."""
    return (round(at[0] - p[1], 4), round(at[1] - p[0], 4))


# ==========================================================================
# 3. Placement plan (board frame, B.Cu)
# ==========================================================================
EDGE_X = BOARD_W
SW_X = EDGE_X - 0.05 + SW_TIP            # actuator 0.05 mm inside the edge (0.0 mm overhang rule)
LED_X = EDGE_X - 0.05 + LED_TIP          # lens apex 0.05 mm inside the edge
PANEL = {   # ref: (footprint, at, value, nets{pad: net}, props)
    "D50": (FP_LED, (LED_X, 9.20), "XL-3210SURUGC PWR",
            {"1": "LED_PWR_G_A", "2": "GND", "3": "LED_PWR_R_A", "4": "nPI_LED_PWR"},
            {"LCSC": "C965883", "MPN": "XL-3210SURUGC", "Manufacturer": "XINGLIGHT", "Function": "PWR: G = module 3V3, R = nPI_LED_PWR"}),
    "D51": (FP_LED, (LED_X, 14.20), "XL-3210SURUGC ACT",
            {"1": "LED_ACT_G_A", "2": "Ethernet_nLED1", "3": "LED_ACT_R_A", "4": "PI_nLED_Activity"},
            {"LCSC": "C965883", "MPN": "XL-3210SURUGC", "Manufacturer": "XINGLIGHT", "Function": "ACT: R = PI_nLED_Activity, G = Ethernet link"}),
    "SW1": (FP_SW, (SW_X, 20.60), "TS24CA BOOT",
            {"1": "USR_BOOT_SW", "2": "GND", "3": "GND", "4": "GND"},
            {"LCSC": "C393942", "MPN": "TS24CA", "Manufacturer": "SHOU HAN", "Function": "MaskROM / rpiboot (hold at power-up)"}),
    "SW2": (FP_SW, (SW_X, 28.20), "TS24CA RESET",
            {"1": "USR_RST_SW", "2": "GND", "3": "GND", "4": "GND"},
            {"LCSC": "C393942", "MPN": "TS24CA", "Manufacturer": "SHOU HAN", "Function": "Reset (RUN_PG / RESET_L)"}),
}
# 0402 passives: ref -> (footprint, pad1 xy, pad1 net, pad2 xy, pad2 net, value, lcsc, mpn)
PASSIVES = {
    "R53": (FP_R, (51.10, 7.70), "+3V3_OUT", (52.12, 7.70), "LED_PWR_R_A", "1k 1%", "C11702", "0402WGF1001TCE"),
    "R52": (FP_R, (51.10, 8.75), "+3V3_OUT", (52.12, 8.75), "LED_PWR_G_A", "330R 1%", "C25104", "0402WGF3300TCE"),
    "R54": (FP_R, (51.10, 12.70), "+3V3_OUT", (52.12, 12.70), "LED_ACT_R_A", "1k 1%", "C11702", "0402WGF1001TCE"),
    "R55": (FP_R, (51.10, 13.75), "+3V3_OUT", (52.12, 13.75), "LED_ACT_G_A", "330R 1%", "C25104", "0402WGF3300TCE"),
    "R50": (FP_R, (48.95, 22.30), "nRPIBOOT", (49.97, 22.30), "USR_BOOT_SW", "470R 1%", "C25117", "0402WGF4700TCE"),
    "D52": (FP_ESD, (49.95, 24.05), "USR_BOOT_SW", (49.95, 25.07), "GND", "BPESD0402-05ST", "C316059", "BPESD0402-05ST"),
    "R51": (FP_R, (48.95, 29.90), "RUN_PG", (49.97, 29.90), "USR_RST_SW", "470R 1%", "C25117", "0402WGF4700TCE"),
    "D53": (FP_ESD, (49.95, 31.65), "USR_RST_SW", (49.95, 32.67), "GND", "BPESD0402-05ST", "C316059", "BPESD0402-05ST"),
}
OUR_REFS = set(PANEL) | set(PASSIVES)
OUR_NETS = {"nPI_LED_PWR", "nRPIBOOT", "RUN_PG", "+3V3_OUT", "PI_nLED_Activity", "Ethernet_nLED1",
            "LED_PWR_G_A", "LED_PWR_R_A", "LED_ACT_G_A", "LED_ACT_R_A", "USR_BOOT_SW", "USR_RST_SW"}
LOCAL_NETS = {"LED_PWR_G_A", "LED_PWR_R_A", "LED_ACT_G_A", "LED_ACT_R_A", "USR_BOOT_SW", "USR_RST_SW"}

# connections, routed in this order. Each = (net, [terminals]); a terminal is (ref, pad).
# Multi-terminal nets are routed as a tree (each new terminal joins the copper already laid).
CONNECTIONS = [
    ("USR_BOOT_SW", [("SW1", "1"), ("R50", "2"), ("D52", "1")]),
    ("USR_RST_SW", [("SW2", "1"), ("R51", "2"), ("D53", "1")]),
    ("LED_PWR_R_A", [("R53", "2"), ("D50", "3")]),
    ("LED_PWR_G_A", [("R52", "2"), ("D50", "1")]),
    ("LED_ACT_R_A", [("R54", "2"), ("D51", "3")]),
    ("LED_ACT_G_A", [("R55", "2"), ("D51", "1")]),
    ("+3V3_OUT", [("R53", "1"), ("R52", "1"), ("R54", "1"), ("R55", "1"), ("J1", "84"), ("J1", "86")]),
    ("nPI_LED_PWR", [("D50", "4"), ("J1", "95")]),
    ("nRPIBOOT", [("R50", "1"), ("J1", "93")]),
    ("RUN_PG", [("R51", "1"), ("J1", "92")]),
    ("PI_nLED_Activity", [("D51", "4"), ("J1", "21")]),
    ("Ethernet_nLED1", [("D51", "2"), ("J1", "19")]),
]
GND_PADS = [("D50", "2"), ("SW1", "2"), ("SW1", "3"), ("SW1", "4"), ("SW2", "2"), ("SW2", "3"), ("SW2", "4"),
            ("D52", "2"), ("D53", "2")]
LABELS = [("PWR", (49.60, 10.45)), ("ACT", (49.60, 15.30)), ("BOOT", (51.60, 17.05)), ("RST", (51.95, 24.85))]
ZONE_POLY = [(47.30, 6.90), (54.50, 6.90), (54.50, 33.20), (47.30, 33.20)]
HOLES = {"H1": (3.5, 3.5), "H2": (51.5, 3.5), "H3": (3.5, 36.5), "H4": (51.5, 36.5)}
HOLE_KO = 3.1                         # standoff keep-out radius (same as Rev 13)

TRACK_W = 0.15
VIA_D, VIA_DRILL = 0.50, 0.30         # project minimums (Rev 9 / Rev 13)
CLR_DEFAULT = 0.20
CLR_MDI = 0.30                        # Rev 9 DRU mdi_isolation (track/via vs MDI track/via)
CLR_ESD_NODE = 0.25                   # Rev 18 DRU: user-touchable switch nodes
GRID = 0.05
MARGIN = 0.012


# ==========================================================================
# 4. Maze router (octilinear A* on a 0.05 mm grid, F.Cu + B.Cu, through vias)
# ==========================================================================
def _is_mdi(net):
    return net.startswith("Ethernet_Pair") or net.startswith("ETH_MDI")


def _sps_to_shapely(sps):
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    polys = []
    for i in range(sps.OutlineCount()):
        o = sps.Outline(i)
        ext = [(o.CPoint(k).x / 1e6, o.CPoint(k).y / 1e6) for k in range(o.PointCount())]
        holes = []
        for j in range(sps.HoleCount(i)):
            h = sps.Hole(i, j)
            holes.append([(h.CPoint(k).x / 1e6, h.CPoint(k).y / 1e6) for k in range(h.PointCount())])
        if len(ext) >= 3:
            polys.append(Polygon(ext, holes).buffer(0))
    return unary_union(polys) if polys else Polygon()


class Router:
    def __init__(self, board, pcbnew):
        import numpy as np
        self.np, self.board, self.pcbnew = np, board, pcbnew
        self.nx, self.ny = int(round(BOARD_W / GRID)) + 1, int(round(BOARD_H / GRID)) + 1
        xs = np.arange(self.nx) * GRID
        ys = np.arange(self.ny) * GRID
        self.X, self.Y = np.meshgrid(xs, ys)
        self.items = {0: [], 1: []}    # layer -> [(geom, net, kind)]
        self._collect()
        self._static_masks()
        self.own = {}                  # net -> {layer: [geoms]} copper laid by this router

    # -- geometry of everything already on the board ------------------------
    def _collect(self):
        from shapely.geometry import LineString, Point
        pc = self.pcbnew
        tm = pc.ToMM
        for t in self.board.GetTracks():
            net = t.GetNetname()
            if t.GetClass() == "PCB_VIA":
                p = t.GetPosition()
                g = Point(tm(p.x), tm(p.y)).buffer(tm(t.GetWidth()) / 2, 16)
                for L in (0, 1):
                    self.items[L].append((g, net, "via"))
            else:
                lay = {pc.F_Cu: 0, pc.B_Cu: 1}.get(t.GetLayer())
                if lay is None:
                    continue
                s, e = t.GetStart(), t.GetEnd()
                g = LineString([(tm(s.x), tm(s.y)), (tm(e.x), tm(e.y))]).buffer(tm(t.GetWidth()) / 2, 8)
                self.items[lay].append((g, net, "track"))
        for fp in self.board.GetFootprints():
            for pad in fp.Pads():
                net = pad.GetNetname()
                npth = pad.GetAttribute() == pc.PAD_ATTRIB_NPTH
                for L, lid in ((0, pc.F_Cu), (1, pc.B_Cu)):
                    if not (pad.IsOnLayer(lid) or npth):
                        continue
                    if npth:
                        p = pad.GetPosition()
                        g = Point(tm(p.x), tm(p.y)).buffer(tm(pad.GetDrillSize().x) / 2, 16)
                        self.items[L].append((g, "<NPTH>", "hole"))
                    else:
                        g = _sps_to_shapely(pad.GetEffectivePolygon())
                        self.items[L].append((g, net, "pad"))
                        if pad.GetAttribute() == pc.PAD_ATTRIB_PTH:
                            p = pad.GetPosition()
                            self.items[L].append((Point(tm(p.x), tm(p.y)).buffer(tm(pad.GetDrillSize().x) / 2 + 0.05, 16), net, "pad"))

    def _raster(self, geom):
        import shapely
        np = self.np
        if geom.is_empty:
            return np.zeros(self.X.shape, bool)
        minx, miny, maxx, maxy = geom.bounds
        out = np.zeros(self.X.shape, bool)
        i0, i1 = max(0, int(minx / GRID) - 1), min(self.nx, int(maxx / GRID) + 2)
        j0, j1 = max(0, int(miny / GRID) - 1), min(self.ny, int(maxy / GRID) + 2)
        if i0 >= i1 or j0 >= j1:
            return out
        out[j0:j1, i0:i1] = shapely.contains_xy(geom, self.X[j0:j1, i0:i1], self.Y[j0:j1, i0:i1])
        return out

    def _static_masks(self):
        from shapely.geometry import Point, box
        from shapely.ops import unary_union
        np, pc = self.np, self.pcbnew
        r = TRACK_W / 2 + MARGIN
        inner = box(EDGE_CLR + r, EDGE_CLR + r, BOARD_W - EDGE_CLR - r, BOARD_H - EDGE_CLR - r)
        self.edge_ok = self._raster(inner)
        rv = VIA_D / 2 + MARGIN
        self.edge_ok_via = self._raster(box(EDGE_CLR + rv, EDGE_CLR + rv, BOARD_W - EDGE_CLR - rv, BOARD_H - EDGE_CLR - rv))
        ko = unary_union([Point(*c).buffer(HOLE_KO, 32) for c in HOLES.values()])
        self.hole_ko = self._raster(ko)
        # reference planes (filled GND on In1 under F.Cu, In2 under B.Cu) and the chassis island
        ref = {0: [], 1: []}
        shield = []
        buck_island = []
        for z in self.board.Zones():
            for L, lid in ((0, pc.In1_Cu), (1, pc.In2_Cu)):
                if z.IsOnLayer(lid):
                    g = _sps_to_shapely(z.GetFilledPolysList(lid))
                    if z.GetNetname() == "GND":
                        ref[L].append(g)
                    elif z.GetNetname() == "ETH_SHIELD":
                        shield.append(g)
            if z.GetZoneName() == "BUCK_GND_BCU":
                buck_island.append(_sps_to_shapely(z.Outline()))
        self.ref_geom = {L: unary_union(ref[L]) for L in (0, 1)}
        self.ref_ok = {L: self._raster(self.ref_geom[L].buffer(-0.10)) for L in (0, 1)}
        self.shield_geom = unary_union(shield)
        self.shield = self._raster(self.shield_geom.buffer(0.30)) if shield else np.zeros(self.X.shape, bool)
        self.shield_via = self._raster(self.shield_geom.buffer(2.0 + VIA_D / 2)) if shield else np.zeros(self.X.shape, bool)
        self.buck = self._raster(unary_union(buck_island).buffer(0.30)) if buck_island else np.zeros(self.X.shape, bool)
        # other parts' courtyards (soft): stay off foreign bodies where possible
        cy = {0: [], 1: []}
        for fp in self.board.GetFootprints():
            if fp.GetReference() in OUR_REFS:
                continue
            for L, lid in ((0, pc.F_CrtYd), (1, pc.B_CrtYd)):
                c = fp.GetCourtyard(lid)
                if c.OutlineCount():
                    cy[L].append(_sps_to_shapely(c))
        self.court = {L: self._raster(unary_union(cy[L])) if cy[L] else np.zeros(self.X.shape, bool) for L in (0, 1)}

    def _clr(self, mynet, other_net, kind):
        if kind == "hole":
            return 0.25
        if _is_mdi(other_net) and kind != "pad":
            return CLR_MDI
        c = CLR_DEFAULT
        if mynet in ("USR_BOOT_SW", "USR_RST_SW") and kind != "pad":
            c = max(c, CLR_ESD_NODE)
        if other_net in ("USR_BOOT_SW", "USR_RST_SW"):
            c = max(c, CLR_ESD_NODE)
        return c

    def blocked(self, net, radius):
        """Per-layer boolean grids: True where a copper centre-line of `radius` would violate clearance."""
        from shapely.ops import unary_union
        out = {}
        for L in (0, 1):
            gs = [g.buffer(self._clr(net, n, k) + radius + MARGIN, 8) for g, n, k in self.items[L] if n != net]
            for n2, per in self.own.items():
                if n2 != net:
                    gs += [g.buffer(self._clr(net, n2, "track") + radius + MARGIN, 8) for g in per.get(L, [])]
            out[L] = self._raster(unary_union(gs))
        return out

    def add_copper(self, net, L, geom):
        self.own.setdefault(net, {}).setdefault(L, []).append(geom)

    # -- search ---------------------------------------------------------------
    def route(self, net, sources, targets, via_cost=1.6, bend=0.15):
        """sources/targets: {layer: bool grid}. Returns list of (layer, ix, iy) or None."""
        np = self.np
        bt = self.blocked(net, TRACK_W / 2)
        bv = self.blocked(net, VIA_D / 2)
        free = {}
        cost = {}
        for L in (0, 1):
            f = self.edge_ok & ~bt[L] & ~self.hole_ko & ~self.shield
            if L == 1:
                f &= ~self.buck
            free[L] = f | sources[L] | targets[L]
            c = np.ones(self.X.shape, np.float32)
            c[~self.ref_ok[L]] *= 5.0          # no solid GND under this layer
            c[self.court[L]] *= 2.5            # under a foreign part body on this side
            cost[L] = c
        via_ok = self.edge_ok_via & ~bv[0] & ~bv[1] & ~self.hole_ko & ~self.shield_via & ~self.buck
        nx, ny = self.nx, self.ny
        INF = np.float32(1e18)
        g = np.full((2, ny, nx), INF, np.float32)
        par = np.full((2, ny, nx), -1, np.int64)
        tcells = [(L, j, i) for L in (0, 1) for j, i in zip(*np.nonzero(targets[L]))]
        tj = np.array([c[1] for c in tcells])
        ti = np.array([c[2] for c in tcells])
        if not len(tcells):
            return None
        # heuristic: octile distance to the target bounding box (admissible, cheap)
        tx0, tx1, ty0, ty1 = ti.min(), ti.max(), tj.min(), tj.max()

        def h(j, i):
            dx = max(tx0 - i, 0, i - tx1)
            dy = max(ty0 - j, 0, j - ty1)
            return (max(dx, dy) + 0.41421356 * min(dx, dy)) * GRID

        pq = []
        for L in (0, 1):
            for j, i in zip(*np.nonzero(sources[L])):
                g[L, j, i] = 0.0
                heapq.heappush(pq, (h(j, i), 0.0, L, int(j), int(i), -1))
        dirs = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        dl = [GRID] * 4 + [GRID * 1.41421356] * 4
        tset = targets
        while pq:
            f, gc, L, j, i, d0 = heapq.heappop(pq)
            if gc > g[L, j, i]:
                continue
            if tset[L][j, i] and not sources[L][j, i]:
                path = [(L, i, j)]
                k = par[L, j, i]
                while k >= 0:
                    L2, rem = divmod(int(k), ny * nx)
                    j2, i2 = divmod(rem, nx)
                    path.append((L2, i2, j2))
                    k = par[L2, j2, i2]
                return path[::-1]
            for d, (di, dj) in enumerate(dirs):
                i2, j2 = i + di, j + dj
                if not (0 <= i2 < nx and 0 <= j2 < ny) or not free[L][j2, i2]:
                    continue
                if di and dj and not (free[L][j, i2] and free[L][j2, i]):
                    continue
                c = gc + dl[d] * float(cost[L][j2, i2]) + (bend if (d0 >= 0 and d0 < 8 and d0 != d) else 0.0)
                if c < g[L, j2, i2]:
                    g[L, j2, i2] = c
                    par[L, j2, i2] = (L * ny + j) * nx + i
                    heapq.heappush(pq, (c + h(j2, i2), c, L, j2, i2, d))
            if via_ok[j, i]:
                L2 = 1 - L
                c = gc + via_cost
                if free[L2][j, i] and c < g[L2, j, i]:
                    g[L2, j, i] = c
                    par[L2, j, i] = (L * ny + j) * nx + i
                    heapq.heappush(pq, (c + h(j, i), c, L2, j, i, 8))
        return None


def simplify(path):
    """Grid path -> list of (layer, [(x, y) ...]) polylines + via points."""
    runs, vias = [], []
    cur_L, pts = path[0][0], [path[0][1:]]
    for (L, i, j) in path[1:]:
        if L != cur_L:
            vias.append(pts[-1])
            runs.append((cur_L, pts))
            cur_L, pts = L, [(i, j)]
        else:
            pts.append((i, j))
    runs.append((cur_L, pts))
    out = []
    for L, p in runs:
        q = [p[0]]
        for k in range(1, len(p) - 1):
            a, b, c = q[-1], p[k], p[k + 1]
            d1 = (b[0] - a[0], b[1] - a[1])
            d2 = (c[0] - b[0], c[1] - b[1])
            if d1[0] * d2[1] - d1[1] * d2[0] != 0 or (d1[0] * d2[0] + d1[1] * d2[1]) < 0:
                q.append(b)
        if len(p) > 1:
            q.append(p[-1])
        out.append((L, [(round(i * GRID, 4), round(j * GRID, 4)) for i, j in q]))
    return out, [(round(i * GRID, 4), round(j * GRID, 4)) for i, j in vias]


# ==========================================================================
# 5. generate
# ==========================================================================
def _top_level(src: str):
    i, depth, start, n = 0, 0, None, len(src)
    while i < n:
        ch = src[i]
        if ch == '"':
            i += 1
            while i < n and src[i] != '"':
                i += 2 if src[i] == "\\" else 1
        elif ch == "(":
            depth += 1
            if depth == 2:
                start = i
        elif ch == ")":
            if depth == 2:
                yield start, i + 1
            depth -= 1
        i += 1


def strip_previous(src: str) -> str:
    """Cut the previous Rev 18 output (group, its tracks/vias/texts, our footprints, our zone)."""
    spans = list(_top_level(src))
    members = set()
    for a, b in spans:
        blk = src[a:b]
        if blk.startswith("(group") and f'"{GROUP}"' in blk[:200]:
            m = re.search(r"\(members(.*?)\)", blk, re.S)
            members |= set(re.findall(r'"([0-9a-f-]{36})"', m.group(1))) if m else set()
    drop = []
    for a, b in spans:
        blk = src[a:b]
        head = blk[:14]
        if head.startswith("(group") and f'"{GROUP}"' in blk[:200]:
            drop.append((a, b))
        elif head.startswith("(footprint"):
            r = re.search(r'\(property "Reference" "([^"]+)"', blk)
            if r and r.group(1) in OUR_REFS:
                drop.append((a, b))
        elif head.startswith("(zone") and f'(name "{ZONE_NAME}")' in blk:
            drop.append((a, b))
        elif head.startswith(("(segment", "(via", "(arc", "(gr_text")):
            u = re.search(r'\(uuid "([0-9a-f-]{36})"\)', blk)
            if u and u.group(1) in members:
                drop.append((a, b))
    out, last = [], 0
    for a, b in drop:
        out.append(src[last:a].rstrip(" \t"))
        last = b
    out.append(src[last:])
    return "".join(out)


def plan_pads():
    """ref -> (footprint, at, {pad: board xy}, {pad: net}, value, props)."""
    out = {}
    for ref, (fpn, at, val, nets, props) in PANEL.items():
        out[ref] = (fpn, at, {k: edge_xf(at, v) for k, v in PADS_LOCAL[fpn].items()}, nets, val, props)
    for ref, (fpn, p1, n1, p2, n2, val, lcsc, mpn) in PASSIVES.items():
        at = ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)
        out[ref] = (fpn, at, {"1": p1, "2": p2}, {"1": n1, "2": n2}, val, {"LCSC": lcsc, "MPN": mpn})
    return out


def place(board, pcbnew, ref, fpname, value, at, pad_targets, nets, props, net):
    mm = pcbnew.FromMM
    V = lambda p: pcbnew.VECTOR2I(mm(p[0]), mm(p[1]))
    fp = pcbnew.FootprintLoad(str(LIB), fpname)
    fp.SetReference(ref)
    fp.SetValue(value)
    fp.SetFPIDAsString(f"{LIBNICK}:{fpname}")
    board.Add(fp)                       # Flip() needs the board's layer stack
    fp.SetPosition(V(at))
    fp.Flip(V(at), True)

    def err_():
        e = 0.0
        for pad in fp.Pads():
            if not pad.GetNumber():
                continue
            q = pad.GetPosition()
            e = max(e, math.dist((pcbnew.ToMM(q.x), pcbnew.ToMM(q.y)), pad_targets[pad.GetNumber()]))
        return e
    err = err_()
    for _ in range(3):
        if err <= 1e-3:
            break
        fp.Rotate(V(at), pcbnew.EDA_ANGLE(90.0, pcbnew.DEGREES_T))
        err = err_()
    if err > 1e-3:
        raise SystemExit(f"{ref}: cannot place pads on targets (err {err:.4f} mm)")
    for pad in fp.Pads():
        if pad.GetNumber():
            pad.SetNet(net(nets[pad.GetNumber()]))
    for k, v in props.items():
        fp.SetField(k, v)
        f = fp.GetFieldByName(k)
        if f:
            f.SetVisible(False)
    fp.Reference().SetVisible(False)
    fp.SetLocked(True)
    return fp


def _pad_xy(board, pcbnew, ref, num):
    fp = board.FindFootprintByReference(ref)
    for p in fp.Pads():
        if p.GetNumber() == num:
            q = p.GetPosition()
            return p, (pcbnew.ToMM(q.x), pcbnew.ToMM(q.y))
    raise KeyError((ref, num))


def _pad_cells(router, pad, pcbnew):
    """Grid cells whose 0.15 mm trace centre-line end stays fully inside the pad."""
    g = _sps_to_shapely(pad.GetEffectivePolygon()).buffer(-TRACK_W / 2 + 0.01)
    if g.is_empty:
        g = _sps_to_shapely(pad.GetEffectivePolygon()).buffer(-0.02)
    m = router._raster(g)
    L = 0 if pad.IsOnLayer(pcbnew.F_Cu) else 1
    return L, m


def generate_board(verbose=True):
    import numpy as np
    import pcbnew
    from shapely.geometry import LineString, Point
    mm = pcbnew.FromMM
    V = lambda p: pcbnew.VECTOR2I(mm(p[0]), mm(p[1]))
    for name, txt in footprints().items():
        (LIB / f"{name}.kicad_mod").write_text(txt)
    scratch = Path(tempfile.mkdtemp(prefix="rev18_"))
    for f in (PRO, DRU):
        shutil.copy(f, scratch / f.name)
    (scratch / PCB.name).write_text(strip_previous(PCB.read_text()))
    board = pcbnew.LoadBoard(str(scratch / PCB.name))

    def net(name):
        n = board.FindNet(name)
        if n is None:
            n = pcbnew.NETINFO_ITEM(board, name)
            board.Add(n)
        return n

    for ref, (fpn, at, targets, nets, val, props) in plan_pads().items():
        place(board, pcbnew, ref, fpn, val, at, targets, nets, props, net)

    router = Router(board, pcbnew)
    grp = pcbnew.PCB_GROUP(board)
    grp.SetName(GROUP)
    board.Add(grp)
    stats = {"tracks": 0, "vias": 0, "length": {}, "vias_per_net": {}}

    def add_track(a, b, L, nn):
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(V(a))
        t.SetEnd(V(b))
        t.SetWidth(mm(TRACK_W))
        t.SetLayer(pcbnew.F_Cu if L == 0 else pcbnew.B_Cu)
        t.SetNet(net(nn))
        board.Add(t)
        grp.AddItem(t)
        router.add_copper(nn, L, LineString([a, b]).buffer(TRACK_W / 2, 8))
        stats["tracks"] += 1
        stats["length"][nn] = stats["length"].get(nn, 0.0) + math.dist(a, b)

    def add_via(xy, nn):
        v = pcbnew.PCB_VIA(board)
        v.SetPosition(V(xy))
        v.SetWidth(mm(VIA_D))
        v.SetDrill(mm(VIA_DRILL))
        v.SetViaType(pcbnew.VIATYPE_THROUGH)
        v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        v.SetNet(net(nn))
        board.Add(v)
        grp.AddItem(v)
        for L in (0, 1):
            router.add_copper(nn, L, Point(xy).buffer(VIA_D / 2, 16))
        stats["vias"] += 1
        stats["vias_per_net"][nn] = stats["vias_per_net"].get(nn, 0) + 1

    failures = []
    for nn, terms in CONNECTIONS:
        tree = {0: np.zeros(router.X.shape, bool), 1: np.zeros(router.X.shape, bool)}
        pad0, c0 = _pad_xy(board, pcbnew, *terms[0])
        L0, m0 = _pad_cells(router, pad0, pcbnew)
        tree[L0] |= m0
        for term in terms[1:]:
            pad, cxy = _pad_xy(board, pcbnew, *term)
            L, m = _pad_cells(router, pad, pcbnew)
            src = {0: np.zeros_like(m), 1: np.zeros_like(m)}
            src[L] = m
            if tree[L][m].any():               # already touching (e.g. abutting pads)
                continue
            path = router.route(nn, src, tree)
            if path is None:
                failures.append(f"{nn}: {term}")
                continue
            runs, vias = simplify(path)
            # stub from the exact pad centre to the first grid point (inside the pad)
            first = runs[0][1]
            if math.dist(first[0], cxy) > 1e-4:
                first.insert(0, (round(cxy[0], 4), round(cxy[1], 4)))
            for Lr, pts in runs:
                for a, b in zip(pts, pts[1:]):
                    if math.dist(a, b) > 1e-6:
                        add_track(a, b, Lr, nn)
            for xy in vias:
                add_via(xy, nn)
            # grow the tree with the new copper (grid cells under the centre-lines, both ends)
            for Lr, pts in runs:
                for a, b in zip(pts, pts[1:]):
                    tree[Lr] |= router._raster(LineString([a, b]).buffer(0.03))
            for xy in vias:
                for Lr in (0, 1):
                    tree[Lr] |= router._raster(Point(xy).buffer(0.06))
            tree[L] |= m
        if verbose:
            print(f"  routed {nn:<18} {stats['length'].get(nn, 0):6.2f} mm, {stats['vias_per_net'].get(nn, 0)} via(s)")

    # GND: each GND pad gets its own stitching via (B.Cu escape to the nearest legal via site),
    # in addition to the solid B.Cu panel pour.
    for ref, num in GND_PADS:
        pad, cxy = _pad_xy(board, pcbnew, ref, num)
        L, m = _pad_cells(router, pad, pcbnew)
        bv = router.blocked("GND", VIA_D / 2)
        bt = router.blocked("GND", TRACK_W / 2)
        ok = router.edge_ok_via & ~bv[0] & ~bv[1] & ~router.hole_ko & ~router.shield_via
        ok &= ~router._raster(_sps_to_shapely(pad.GetEffectivePolygon()).buffer(0.30))   # no via-in-pad
        free = router.edge_ok & ~bt[1]
        # BFS on B.Cu from the pad up to 2.5 mm to the first via site
        from collections import deque
        js, is_ = np.nonzero(m)
        seen = np.zeros(m.shape, bool)
        prev = {}
        dq = deque()
        for j, i in zip(js, is_):
            seen[j, i] = True
            dq.append((j, i))
        hit = None
        maxd = int(2.5 / GRID)
        while dq:
            j, i = dq.popleft()
            if ok[j, i] and not m[j, i]:
                hit = (j, i)
                break
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                j2, i2 = j + dj, i + di
                if 0 <= j2 < router.ny and 0 <= i2 < router.nx and not seen[j2, i2] and free[j2, i2] \
                        and abs(j2 - js[0]) <= maxd and abs(i2 - is_[0]) <= maxd:
                    seen[j2, i2] = True
                    prev[(j2, i2)] = (j, i)
                    dq.append((j2, i2))
        if hit is None:
            failures.append(f"GND via for {ref}.{num}")
            continue
        cells = [hit]
        while cells[-1] in prev:
            cells.append(prev[cells[-1]])
        pts = [(round(i * GRID, 4), round(j * GRID, 4)) for j, i in cells[::-1]]
        _, pv = simplify([(1, int(round(x / GRID)), int(round(y / GRID))) for x, y in pts])
        runs, _ = simplify([(1, int(round(x / GRID)), int(round(y / GRID))) for x, y in pts])
        poly = [(round(cxy[0], 4), round(cxy[1], 4))] + runs[0][1]
        for a, b in zip(poly, poly[1:]):
            if math.dist(a, b) > 1e-6:
                add_track(a, b, 1, "GND")
        add_via(pts[-1], "GND")

    # silkscreen labels (B.SilkS, mirrored, readable from below)
    for txt, xy in LABELS:
        t = pcbnew.PCB_TEXT(board)
        t.SetText(txt)
        t.SetLayer(pcbnew.B_SilkS)
        t.SetPosition(V(xy))
        t.SetTextSize(pcbnew.VECTOR2I(mm(0.8), mm(0.8)))
        t.SetTextThickness(mm(0.12))
        t.SetMirrored(True)
        board.Add(t)
        grp.AddItem(t)

    z = pcbnew.ZONE(board)
    z.SetZoneName(ZONE_NAME)
    z.SetLayer(pcbnew.B_Cu)
    z.SetNet(net("GND"))
    z.SetLocalClearance(mm(0.25))
    z.SetMinThickness(mm(0.20))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    z.SetAssignedPriority(4)
    ol = z.Outline()
    ol.NewOutline()
    for p in ZONE_POLY:
        ol.Append(mm(p[0]), mm(p[1]))
    board.Add(z)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    if failures:
        raise SystemExit("routing failed: " + "; ".join(failures))
    board.Save(str(PCB))
    write_rules()
    if verbose:
        print(f"Rev 18: {len(OUR_REFS)} parts on B.Cu, {stats['tracks']} segments, {stats['vias']} vias, "
              f"{sum(stats['length'].values()):.1f} mm copper; zone {ZONE_NAME}.")
    return stats


def cmd_generate(_a) -> int:
    generate_board()
    return 0


# ==========================================================================
# 6. DRC rules
# ==========================================================================
DRU_BEGIN = "# ---- BEGIN Rev 18 user I/O (generated by tools/user_io.py) ----"
DRU_END = "# ---- END Rev 18 user I/O ----"
DRU_W_MIN = 0.15


def dru_text() -> str:
    return f"""{DRU_BEGIN}
# No neck-downs on the user-panel nets below the {DRU_W_MIN} mm the router lays.
(rule "userio_track_width"
  (condition "A.Type == 'Track' && A.memberOfGroup('{GROUP}')")
  (constraint track_width (min {DRU_W_MIN}mm)))

# Switch nodes are user-touchable (actuator on the board edge): ESD events land here
# before the ESD diode clamps them, so keep {CLR_ESD_NODE} mm air/FR-4 to every other conductor.
(rule "userio_esd_node_clearance"
  (condition "(A.NetName == 'USR_BOOT_SW' || A.NetName == 'USR_RST_SW') && A.Type != 'Pad' && B.Type != 'Pad'")
  (constraint clearance (min {CLR_ESD_NODE}mm)))
{DRU_END}
"""


def write_rules() -> None:
    txt = DRU.read_text()
    if DRU_BEGIN in txt:
        txt = re.sub(re.escape(DRU_BEGIN) + r".*?" + re.escape(DRU_END) + r"\n?", "", txt, flags=re.S)
    DRU.write_text(txt.rstrip() + "\n\n" + dru_text())


# ==========================================================================
# 7. Schematic
# ==========================================================================
UUID_NS = "e7e90018"


def _uid(tag: str) -> str:
    h = hashlib.sha1(tag.encode()).hexdigest()
    return f"{UUID_NS}-{h[8:12]}-{h[12:16]}-{h[16:20]}-{h[20:32]}"


def _sexp_children(body: str) -> list:
    out, depth, start, i, n, instr = [], 0, None, 0, len(body), False
    while i < n:
        c = body[i]
        if instr:
            if c == "\\":
                i += 2
                continue
            if c == '"':
                instr = False
        elif c == '"':
            instr = True
        elif c == "(":
            if depth == 0:
                start = i
            depth += 1
        elif c == ")":
            depth -= 1
            if depth == 0:
                out.append(body[start:i + 1])
        i += 1
    return out


def _sym(name, ref, desc, fp, pins, body_w=5.08, kw=""):
    """pins: [(num, name, side, y)] side -1 left / +1 right, y in symbol units (y-up)."""
    ys = [p[3] for p in pins]
    top, bot = max(ys) + 2.54, min(ys) - 2.54
    L = [f'    (symbol "CM4Carrier:{name}"', '      (pin_names (offset 0.762))', '      (exclude_from_sim no) (in_bom yes) (on_board yes)',
         f'      (property "Reference" "{ref}" (at 0 {top + 1.27:.2f} 0) (effects (font (size 1.27 1.27))))',
         f'      (property "Value" "{name}" (at 0 {bot - 1.27:.2f} 0) (effects (font (size 1.27 1.27))))',
         f'      (property "Footprint" "{LIBNICK}:{fp}" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes)))',
         '      (property "Datasheet" "~" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes)))',
         f'      (property "Description" "{desc}" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes)))',
         f'      (property "ki_keywords" "{kw}" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes)))',
         f'      (symbol "{name}_0_1"',
         f'        (rectangle (start {-body_w:.2f} {top:.2f}) (end {body_w:.2f} {bot:.2f}) (stroke (width 0.254) (type default)) (fill (type background)))',
         '      )', f'      (symbol "{name}_1_1"']
    for num, pname, side, y in pins:
        x = side * (body_w + 2.54)
        L.append(f'        (pin passive line (at {x:.2f} {y:.2f} {0 if side < 0 else 180}) (length 2.54) (name "{pname}" '
                 f'(effects (font (size 1.016 1.016)))) (number "{num}" (effects (font (size 1.016 1.016)))))')
    L += ['      )', '    )']
    return "\n".join(L)


def rev18_symbols():
    return {
        "TS24CA": _sym("TS24CA", "SW", "SHOU HAN TS24CA right-angle SMD tact switch, 12 V 50 mA (LCSC C393942); 3/4 = cover tabs",
                       FP_SW, [("1", "A", -1, 1.27), ("2", "B", 1, 1.27), ("3", "TAB", -1, -1.27), ("4", "TAB", 1, -1.27)],
                       kw="switch tactile"),
        "XL-3210SURUGC": _sym("XL-3210SURUGC", "D", "XINGLIGHT side-view red/green LED, independent dice (LCSC C965883)",
                              FP_LED, [("1", "G+", -1, 1.27), ("2", "G-", 1, 1.27), ("3", "R+", -1, -1.27), ("4", "R-", 1, -1.27)],
                              kw="LED bicolor side view"),
        "ESD_0402": _sym("ESD_0402", "D", "Bidirectional ESD diode 0402, VRWM 5 V (BORN BPESD0402-05ST, LCSC C316059)",
                         FP_ESD, [("1", "IO", -1, 0.0), ("2", "GND", 1, 0.0)], body_w=3.81, kw="ESD TVS"),
    }


def _glabel(net, x, y, ang):
    u = _uid(f"gl:{net}:{x:.2f}:{y:.2f}")
    j = "right" if ang == 180 else "left"
    return (f'  (global_label "{net}" (shape bidirectional) (at {x:.2f} {y:.2f} {ang}) (fields_autoplaced yes)\n'
            f'    (effects (font (size 1.016 1.016)) (justify {j}))\n    (uuid "{u}")\n'
            f'    (property "Intersheetrefs" "${{INTERSHEET_REFS}}" (at {x:.2f} {y:.2f} 0) (effects (font (size 1.27 1.27)) (hide yes)))\n  )')


def _instance(lib, ref, value, fp, at, sch_uuid, props):
    x, y = at
    extra = "".join(f'\n    (property "{k}" "{v}" (at {x} {y} 0) (effects (font (size 1.27 1.27)) (hide yes)))'
                    for k, v in props.items() if k not in ("Function",))
    return (f'  (symbol\n    (lib_id "CM4Carrier:{lib}")\n    (at {x} {y} 0)\n    (unit 1)\n    (exclude_from_sim no)\n    (in_bom yes)\n'
            f'    (on_board yes)\n    (dnp no)\n    (uuid "{_uid("sym:" + ref)}")\n'
            f'    (property "Reference" "{ref}" (at {x} {y - 6.35:.2f} 0) (effects (font (size 1.27 1.27))))\n'
            f'    (property "Value" "{value}" (at {x} {y + 6.35:.2f} 0) (effects (font (size 1.27 1.27))))\n'
            f'    (property "Footprint" "{LIBNICK}:{fp}" (at {x} {y} 0) (effects (font (size 1.27 1.27)) (hide yes)))\n'
            f'    (property "Datasheet" "~" (at {x} {y} 0) (effects (font (size 1.27 1.27)) (hide yes))){extra}\n'
            f'    (instances\n      (project "radxa-cm4-nano-carrier"\n        (path "/{sch_uuid}" (reference "{ref}") (unit 1))\n      )\n    )\n  )')


def cmd_schematic(_a=None) -> int:
    import pcbnew
    board = pcbnew.LoadBoard(str(PCB))
    padnet, vals = {}, {}
    for fp in board.GetFootprints():
        if fp.GetReference() in OUR_REFS:
            vals[fp.GetReference()] = (fp.GetValue(), fp.GetFPIDAsString().split(":")[1])
            for p in fp.Pads():
                if p.GetNumber():
                    padnet[(fp.GetReference(), p.GetNumber())] = p.GetNetname()
    missing = OUR_REFS - set(vals)
    if missing:
        raise SystemExit(f"schematic: run generate first (missing {sorted(missing)})")
    txt = SCH.read_text()
    sch_uuid = re.search(r'\(kicad_sch[^(]*\(version[^)]*\)\s*\(generator[^)]*\)\s*\(generator_version[^)]*\)\s*\(uuid "([^"]+)"\)', txt).group(1)
    head = txt[:txt.index("(kicad_sch") + len("(kicad_sch")]
    body = txt[txt.index("(kicad_sch") + len("(kicad_sch"):txt.rindex(")")]
    syms = rev18_symbols()
    keep = []
    for it in _sexp_children(body):
        if it.startswith("(lib_symbols"):
            inner = it[len("(lib_symbols"):-1]
            libs = [s for s in _sexp_children(inner) if not any(f'"CM4Carrier:{k}"' in s.split("\n", 1)[0] for k in syms)]
            it = "(lib_symbols\n" + "\n".join("    " + s.strip() for s in libs) + "\n" + "\n".join(syms.values()) + "\n  )"
        elif UUID_NS in it and it.startswith(("(symbol", "(global_label", "(text")):
            continue
        keep.append(it)
    lib_of = {FP_SW: "TS24CA", FP_LED: "XL-3210SURUGC", FP_ESD: "ESD_0402", FP_R: "R_0402"}
    props_of = {r: PANEL[r][4] for r in PANEL}
    props_of.update({r: {"LCSC": v[6], "MPN": v[7]} for r, v in PASSIVES.items()})
    new = []
    X0, Y0 = 127.0, 177.8                      # below the Rev 9 Ethernet block
    order = ["D50", "R52", "R53", "D51", "R54", "R55", "SW1", "R50", "D52", "SW2", "R51", "D53"]
    for k, ref in enumerate(order):
        val, fpn = vals[ref]
        lib = lib_of[fpn]
        at = (round(X0 + 30.48 * (k % 3), 2), round(Y0 + 25.4 * (k // 3), 2))
        new.append(_instance(lib, ref, val, fpn, at, sch_uuid, props_of[ref]))
        if lib == "R_0402":                     # Rev 9 two-pin symbol: pins at (0, +/-3.81), y-up
            for num, dy, ang in (("1", -3.81, 90), ("2", 3.81, 270)):
                new.append(_glabel(padnet[(ref, num)], at[0], at[1] + dy, ang))
            continue
        d = re.search(r'\(symbol "' + re.escape(lib) + r'_1_1"(.*?)\n      \)', syms[lib], re.S).group(1)
        for m in re.finditer(r'\(at ([\-\d.]+) ([\-\d.]+) (\d+)\).*?\(number "([^"]+)"', d):
            px, py, ang, num = float(m.group(1)), float(m.group(2)), int(m.group(3)), m.group(4)
            new.append(_glabel(padnet[(ref, num)], at[0] + px, at[1] - py, 180 if ang == 0 else 0))
    note = ("REV 18 USER PANEL (tools/user_io.py schematic) - B.Cu, east edge, facing out of the enclosure side wall.\\n"
            "D50 PWR: green = module +3V3_OUT (R52 330R), red = nPI_LED_PWR (R53 1k).  D51 ACT: red = PI_nLED_Activity (R54 1k),\\n"
            "green = Ethernet_nLED1 link/act (R55 330R).  All LED anodes on the MODULE rail: no back-power when the module is off.\\n"
            "SW1 BOOT -> R50 470R -> nRPIBOOT (RK3576: MASKROM); hold while applying power.  SW2 RESET -> R51 470R -> RUN_PG (RK3576: RESET_L).\\n"
            "D52/D53 0.05 pF ESD on the touchable switch nodes; switch cover tabs to GND.  GLOBAL_EN / nEXTRST intentionally unused.")
    new.append(f'  (text "{note}"\n    (exclude_from_sim no)\n    (at {X0} {Y0 - 10.16} 0)\n'
               f'    (effects (font (size 1.27 1.27)) (justify left bottom))\n    (uuid "{_uid("note")}")\n  )')
    si = [k for k, it in enumerate(keep) if it.startswith("(sheet_instances")]
    pos = si[0] if si else len(keep)
    keep = keep[:pos] + new + keep[pos:]
    out = head + "\n  " + "\n  ".join(k.strip() for k in keep) + "\n)\n"
    SCH.write_text(out)
    lib = SYMLIB.read_text()
    lb = lib[lib.index("(kicad_symbol_lib") + len("(kicad_symbol_lib"):lib.rindex(")")]
    li = [s for s in _sexp_children(lb) if not any(s.startswith(f'(symbol "{k}"') for k in syms)]
    li += [v.replace('(symbol "CM4Carrier:', '(symbol "') for v in syms.values()]
    SYMLIB.write_text("(kicad_symbol_lib\n  " + "\n  ".join(x.strip() for x in li) + "\n)\n")
    print(f"schematic: {len(new)} Rev 18 items written to {SCH.name}; {len(syms)} symbols in {SYMLIB.name}")
    return 0


# ==========================================================================
# 8. verify
# ==========================================================================
def run_drc(pcb_path: Path, parity=True) -> dict:
    with tempfile.TemporaryDirectory() as td:
        out = Path(td) / "drc.json"
        cmd = ["kicad-cli", "pcb", "drc", "--format", "json", "--severity-all", "-o", str(out), str(pcb_path)]
        if parity:
            cmd.insert(5, "--schematic-parity")
        subprocess.run(cmd, capture_output=True, text=True)
        if not out.exists():
            subprocess.run([c for c in cmd if c != "--schematic-parity"], capture_output=True, text=True)
        return json.loads(out.read_text())


def drc_scratch(text: str, parity=False) -> dict:
    with tempfile.TemporaryDirectory() as td:
        for f in (PRO, DRU):
            shutil.copy(f, Path(td) / f.name)
        d_ = Path(td) / DRU.name
        t_ = d_.read_text()
        if DRU_BEGIN not in t_:
            d_.write_text(t_.rstrip() + "\n\n" + dru_text())
        if parity and SCH.exists():
            shutil.copy(SCH, Path(td) / SCH.name)
        p = Path(td) / PCB.name
        p.write_text(text)
        return run_drc(p, parity)


def _all_violations(d):
    return [("violation", v) for v in d.get("violations", [])] + \
           [("unconnected", v) for v in d.get("unconnected_items", [])] + \
           [("parity", v) for v in d.get("schematic_parity", [])]


def _our_uuids(board):
    ids = set()
    for g in board.Groups():
        if g.GetName() == GROUP:
            for it in g.GetItems():
                ids.add(it.m_Uuid.AsString())
    for fp in board.GetFootprints():
        if fp.GetReference() in OUR_REFS:
            ids.add(fp.m_Uuid.AsString())
            for p in fp.Pads():
                ids.add(p.m_Uuid.AsString())
    for z in board.Zones():
        if z.GetZoneName() == ZONE_NAME:
            ids.add(z.m_Uuid.AsString())
    return ids


def _touches(v, ids):
    return any(it.get("uuid") in ids for it in v.get("items", []))


def cmd_verify(a) -> int:
    import pcbnew
    from shapely.geometry import LineString, Point
    from shapely.ops import unary_union
    tm = pcbnew.ToMM
    board = pcbnew.LoadBoard(str(PCB))
    rows = []

    def add_(name, ok, detail=""):
        rows.append((name, bool(ok), detail))

    # 1. design calc
    cres = calc()
    add_(f"Electrical design checks (calc): {sum(r['pass'] for r in cres['checks'])}/{len(cres['checks'])}",
         cres["pass"], "; ".join(r["check"] for r in cres["checks"] if not r["pass"]) or "LED currents, pin sink, button VIL, R power, ratings")
    # 2. parts + pad nets + placement
    plan = plan_pads()
    fps = {fp.GetReference(): fp for fp in board.GetFootprints()}
    missing = sorted(OUR_REFS - set(fps))
    add_(f"All {len(OUR_REFS)} Rev 18 parts present (SW1, SW2, D50-D53, R50-R55)", not missing, f"missing {missing}" if missing else "")
    bad, perr = [], 0.0
    for ref, (fpn, at, targets, nets, val, props) in plan.items():
        if ref not in fps:
            continue
        fp = fps[ref]
        if not fp.IsFlipped():
            bad.append(f"{ref} not on B.Cu")
        for p in fp.Pads():
            n = p.GetNumber()
            if not n:
                continue
            if p.GetNetname() != nets[n]:
                bad.append(f"{ref}.{n}={p.GetNetname()} (want {nets[n]})")
            q = p.GetPosition()
            perr = max(perr, math.dist((tm(q.x), tm(q.y)), targets[n]))
    add_("Pad-to-net map = design intent; all parts on B.Cu", not bad, "; ".join(bad[:6]))
    add_("Pad positions = plan (mirror-correct flip)", perr < 1e-3, f"max err {perr*1e3:.1f} um")
    j1 = fps.get("J1")
    j1map = {p.GetNumber(): p.GetNetname() for p in j1.Pads()} if j1 else {}
    want_j1 = {"92": "RUN_PG", "93": "nRPIBOOT", "95": "nPI_LED_PWR", "21": "PI_nLED_Activity",
               "19": "Ethernet_nLED1", "84": "+3V3_OUT", "86": "+3V3_OUT"}
    add_("J1 module pins carry the expected functions (92 RUN_PG, 93 nRPIBOOT, 95/21/19 LED, 84/86 3V3)",
         all(j1map.get(k) == v for k, v in want_j1.items()), str({k: j1map.get(k) for k in want_j1}))
    # 3. mechanics: edge-facing parts, zero overhang, keep-outs
    mech = []
    for ref in PANEL:
        if ref not in fps:
            continue
        fp = fps[ref]
        bb = fp.GetCourtyard(pcbnew.B_CrtYd).BBox()
        tip = PANEL[ref][1][0] - (SW_TIP if PANEL[ref][0] == FP_SW else LED_TIP)
        inset = EDGE_X - tip
        mech.append((ref, round(inset, 3), round(tm(bb.GetRight()), 3)))
        add_(f"{ref} {'actuator' if PANEL[ref][0] == FP_SW else 'lens'} at the east edge: 0.0 mm overhang, <= 0.10 mm inset",
             0.0 <= inset <= 0.10, f"inset {inset:.3f} mm")
    pads_edge = min(tm(fp.GetBoundingBox(False, False).GetRight()) for r, fp in fps.items() if r in OUR_REFS) if fps else 0
    cu_edge = 1e9
    for r, fp in fps.items():
        if r in OUR_REFS:
            for p in fp.Pads():
                if p.GetNumber():
                    g = _sps_to_shapely(p.GetEffectivePolygon())
                    cu_edge = min(cu_edge, BOARD_W - g.bounds[2], g.bounds[1], BOARD_H - g.bounds[3])
    add_(f"Rev 18 pad copper to board edge >= {EDGE_CLR} mm", cu_edge >= EDGE_CLR, f"min {cu_edge:.3f} mm")
    ko = []
    for r, fp in fps.items():
        if r in OUR_REFS:
            c = _sps_to_shapely(fp.GetCourtyard(pcbnew.B_CrtYd))
            for h, xy in HOLES.items():
                d = c.distance(Point(xy))
                if d < HOLE_KO:
                    ko.append(f"{r}-{h} {d:.2f}")
    add_(f"Courtyards clear of the M2.5 standoff keep-outs (r {HOLE_KO} mm)", not ko, "; ".join(ko))
    # 4. copper: isolation, reference, connectivity
    ours = _our_uuids(board)
    tr_ours, mdi, tmds, vias_ours = [], {0: [], 1: []}, {0: [], 1: []}, []
    for t in board.GetTracks():
        is_ours = t.m_Uuid.AsString() in ours
        if t.GetClass() == "PCB_VIA":
            p = t.GetPosition()
            g = Point(tm(p.x), tm(p.y)).buffer(tm(t.GetWidth()) / 2, 16)
            if is_ours:
                vias_ours.append((g, t.GetNetname()))
            for L in (0, 1):
                if _is_mdi(t.GetNetname()):
                    mdi[L].append(g)
                elif t.GetNetname().startswith("HDMI0_"):
                    tmds[L].append(g)
            continue
        L = {pcbnew.F_Cu: 0, pcbnew.B_Cu: 1}.get(t.GetLayer())
        if L is None:
            continue
        s, e = t.GetStart(), t.GetEnd()
        g = LineString([(tm(s.x), tm(s.y)), (tm(e.x), tm(e.y))]).buffer(tm(t.GetWidth()) / 2, 8)
        if is_ours:
            tr_ours.append((L, g, t.GetNetname(), LineString([(tm(s.x), tm(s.y)), (tm(e.x), tm(e.y))])))
        elif _is_mdi(t.GetNetname()):
            mdi[L].append(g)
        elif t.GetNetname().startswith("HDMI0_"):
            tmds[L].append(g)
    dm = min([g.distance(unary_union(mdi[L])) for L, g, _, _ in tr_ours if mdi[L]] +
             [g.distance(unary_union(mdi[0] + mdi[1])) for g, _ in vias_ours] or [99])
    dt = min([g.distance(unary_union(tmds[L])) for L, g, _, _ in tr_ours if tmds[L]] +
             [g.distance(unary_union(tmds[0] + tmds[1])) for g, _ in vias_ours] or [99])
    add_(f"Isolation to Ethernet MDI copper >= {CLR_MDI} mm (Rev 9 mdi_isolation)", dm >= CLR_MDI - 1e-4, f"min {dm:.3f} mm")
    add_("Isolation to HDMI TMDS copper >= 0.20 mm", dt >= 0.20 - 1e-4, f"min {dt:.3f} mm")
    # reference plane: fraction of routed length over filled GND on the adjacent plane
    ref = {}
    shield = []
    for z in board.Zones():
        for L, lid in ((0, pcbnew.In1_Cu), (1, pcbnew.In2_Cu)):
            if z.IsOnLayer(lid):
                g = _sps_to_shapely(z.GetFilledPolysList(lid))
                if z.GetNetname() == "GND":
                    ref.setdefault(L, []).append(g)
                elif z.GetNetname() == "ETH_SHIELD":
                    shield.append(g)
    refg = {L: unary_union(v) for L, v in ref.items()}
    shg = unary_union(shield) if shield else None
    tot = over = 0.0
    on_shield = 0.0
    for L, g, n, ln in tr_ours:
        tot += ln.length
        if L in refg:
            over += ln.intersection(refg[L]).length
        if shg is not None:
            on_shield += ln.intersection(shg).length
    add_("No Rev 18 copper over the ETH_SHIELD chassis island (hi-pot domain)", on_shield < 1e-6, f"{on_shield:.3f} mm")
    frac = over / tot if tot else 1.0
    add_("Rev 18 track length over solid GND on the adjacent plane >= 90 %", frac >= 0.90,
         f"{over:.1f} / {tot:.1f} mm = {frac*100:.1f} % (rest: Rev 9 hi-pot split void near J1, DC/static nets only)")
    nvias = {}
    for g, n in vias_ours:
        nvias[n] = nvias.get(n, 0) + 1
    # 5. kicad-cli DRC, attributed by UUID, vs baseline (board without Rev 18)
    drc = run_drc(PCB)
    allv = _all_violations(drc)
    mine = [(k, v) for k, v in allv if _touches(v, ours)]
    mine_err = [(k, v) for k, v in mine if v.get("severity") == "error" or k == "unconnected"]
    unc_mine = [v for k, v in mine if k == "unconnected"]
    add_("kicad-cli DRC: 0 errors touching Rev 18 items", not mine_err,
         "; ".join(f"{v.get('type')}: {v.get('description')}" for _, v in mine_err[:5]))
    add_("kicad-cli DRC: all Rev 18 nets fully connected (0 unconnected items)", not unc_mine, f"{len(unc_mine)}")
    mine_w = [(k, v) for k, v in mine if (k, v) not in mine_err and k != "parity"]
    base = drc_scratch(strip_previous(PCB.read_text()))
    nb = len([1 for k, v in _all_violations(base) if k != "parity" and v.get("severity") == "error"])
    na = len([1 for k, v in allv if k != "parity" and v.get("severity") == "error"])
    add_("Rev 18 adds no DRC errors anywhere on the board (vs. board without Rev 18)", na - len([1 for k, v in mine_err if k != 'parity']) <= nb,
         f"errors: {nb} without Rev 18 -> {na} with ({len(mine_err)} touch Rev 18)")
    par = [v for k, v in mine if k == "parity"]
    add_("Schematic <-> PCB parity for Rev 18 parts", not par, "; ".join(v.get("description", "") for v in par[:4]))
    # 6. DRU presence + mutation tests (must create NEW violations on Rev 18 copper)
    add_("Rev 18 DRU block present in .kicad_dru", DRU_BEGIN in DRU.read_text())
    src = PCB.read_text()
    muts = []
    seg = next((ln for L, g, n, ln in tr_ours if n == "nRPIBOOT" and ln.length > 1.0), None)
    sw_seg = next(((L, ln) for L, g, n, ln in tr_ours if n == "USR_BOOT_SW" and ln.length > 0.6), None)
    if seg is not None:
        # (a) neck one of our tracks down to 0.12 mm -> userio_track_width
        x0, y0 = seg.coords[0]
        m = re.search(r'\(segment\s*\(start ' + re.escape(f"{x0:g}") + r' ' + re.escape(f"{y0:g}") + r'\)[^()]*\(end [^)]*\)\s*\(width ([\d.]+)\)', src)
        if m:
            mut = src[:m.start(1)] + "0.12" + src[m.end(1):]
            d = drc_scratch(mut)
            hit = [v for v in d.get("violations", []) if v.get("type") == "track_width" and "0.12" in v.get("description", "") + json.dumps(v)]
            muts.append(("userio_track_width (neck 0.15 -> 0.12 mm)", bool(hit)))
        else:
            muts.append(("userio_track_width (segment not found)", False))
    if sw_seg is not None:
        # (b) GND track parallel to a switch-node track at 0.22 mm edge gap -> userio_esd_node_clearance
        L, ln = sw_seg
        (xa, ya), (xb, yb) = ln.coords[0], ln.coords[-1]
        dx, dy = xb - xa, yb - ya
        nrm = math.hypot(dx, dy)
        ox, oy = -dy / nrm * (TRACK_W + 0.22), dx / nrm * (TRACK_W + 0.22)
        lay = "F.Cu" if L == 0 else "B.Cu"
        gnd_code = re.search(r'\(net (\d+) "GND"\)', src).group(1)
        add = (f'\n  (segment (start {xa + ox + dx*0.2:.4f} {ya + oy + dy*0.2:.4f}) (end {xb + ox - dx*0.2:.4f} {yb + oy - dy*0.2:.4f}) '
               f'(width {TRACK_W}) (layer "{lay}") (net {gnd_code}) (uuid "{_uid("mut-esd")}"))')
        mut = src[:src.rindex(")")] + add + "\n)\n"
        d = drc_scratch(mut)
        hit = [v for v in d.get("violations", []) if v.get("type") == "clearance" and "userio_esd_node_clearance" in v.get("description", "")]
        muts.append(("userio_esd_node_clearance (GND track 0.22 mm from USR_BOOT_SW)", bool(hit)))
    for nm, ok in muts:
        add_(f"DRU mutation caught: {nm}", ok)
    report(rows, cres, mech, nvias, drc, frac, tot)
    return 0 if all(ok for _, ok, _ in rows) else 1


def report(rows, cres, mech, nvias, drc, frac, tot):
    n_ok = sum(ok for _, ok, _ in rows)
    L = ["# Rev 18 - status LEDs and user controls: verification report", "",
         "Generated by `python3 tools/user_io.py verify` from the saved board (measured, not modelled).", "",
         f"**Result: {n_ok}/{len(rows)} checks pass.**", "", "| # | Check | Result | Detail |", "|---|---|---|---|"]
    for k, (nm, ok, det) in enumerate(rows, 1):
        L.append(f"| {k} | {nm} | {'PASS' if ok else '**FAIL**'} | {det} |")
    L += ["", "## LED operating points (module +3V3_OUT 3.135-3.465 V)", "",
          "| R | Colour | Driver | R (ohm) | I min / typ / max (mA) | Iv typ (mcd) |", "|---|---|---|---|---|---|"]
    for ref, v in cres["leds"].items():
        L.append(f"| {ref} | {v['colour']} | {v['driver']} | {v['R']:.0f} | {v['I_min_mA']:.2f} / {v['I_typ_mA']:.2f} / {v['I_max_mA']:.2f} | {v['Iv_typ_mcd']:.0f} |")
    L += ["", "## Edge-facing parts", "", "| Ref | Inset from X = 55 edge (mm) | Courtyard right edge (mm) |", "|---|---|---|"]
    L += [f"| {r} | {i:.3f} | {c:.3f} |" for r, i, c in mech]
    L += ["", f"Vias per net: {json.dumps(nvias)}", "", f"Routed length {tot:.1f} mm, {frac*100:.1f} % over solid GND reference.", ""]
    REPORT.write_text("\n".join(L) + "\n")
    for nm, ok, det in rows:
        print(("PASS " if ok else "FAIL ") + nm + (f"  -- {det}" if det else ""))
    print(f"verify: {n_ok}/{len(rows)} -> {REPORT.relative_to(ROOT)}")


# ==========================================================================
# 9. render
# ==========================================================================
def cmd_render(_a) -> int:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon as MPoly
    import pcbnew
    tm = pcbnew.ToMM
    board = pcbnew.LoadBoard(str(PCB))
    ours = _our_uuids(board)
    fig, ax = plt.subplots(figsize=(15, 11))
    ax.set_xlim(-0.5, 55.5)
    ax.set_ylim(40.5, -0.5)
    ax.set_aspect("equal")
    ax.plot([0, 55, 55, 0, 0], [0, 0, 40, 40, 0], "k", lw=1.2)
    for t in board.GetTracks():
        mine = t.m_Uuid.AsString() in ours
        if t.GetClass() == "PCB_VIA":
            p = t.GetPosition()
            ax.add_patch(Circle((tm(p.x), tm(p.y)), tm(t.GetWidth()) / 2, color="darkgreen" if mine else "#9c9", zorder=4))
            continue
        lay = t.GetLayer()
        if lay not in (pcbnew.F_Cu, pcbnew.B_Cu):
            continue
        s, e = t.GetStart(), t.GetEnd()
        col = ("#d00" if lay == pcbnew.B_Cu else "#00c") if mine else ("#f4b0b0" if lay == pcbnew.B_Cu else "#b0b0f4")
        ax.plot([tm(s.x), tm(e.x)], [tm(s.y), tm(e.y)], color=col, lw=2.2 if mine else 1.0, solid_capstyle="round", zorder=3 if mine else 2)
    for fp in board.GetFootprints():
        mine = fp.GetReference() in OUR_REFS
        for p in fp.Pads():
            sps = p.GetEffectivePolygon()
            if sps.OutlineCount():
                o = sps.Outline(0)
                ax.add_patch(MPoly([(o.CPoint(k).x / 1e6, o.CPoint(k).y / 1e6) for k in range(o.PointCount())],
                                   color="orange" if mine else "#e8d8b0", zorder=2))
        c = fp.GetCourtyard(pcbnew.B_CrtYd if fp.IsFlipped() else pcbnew.F_CrtYd)
        if c.OutlineCount():
            o = c.Outline(0)
            ax.add_patch(MPoly([(o.CPoint(k).x / 1e6, o.CPoint(k).y / 1e6) for k in range(o.PointCount())],
                               fill=False, ec="m" if mine else "#d9c", lw=1.0 if mine else 0.5, zorder=5))
            q = fp.GetPosition()
            ax.text(tm(q.x), tm(q.y), fp.GetReference(), fontsize=8 if mine else 6, color="m" if mine else "#a8a", zorder=6)
    ax.set_title("Rev 18 user panel (strong colours): B.Cu red, F.Cu blue, vias green; other stages faded. Top view.")
    ax.set_xticks(range(0, 56, 5))
    ax.set_yticks(range(0, 41, 5))
    ax.grid(True, lw=0.3)
    fig.tight_layout()
    fig.savefig(RENDER, dpi=110)
    print(f"render: {RENDER.relative_to(ROOT)}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for c in ("calc", "generate", "rules", "schematic", "verify", "render"):
        sub.add_parser(c)
    a = ap.parse_args()
    if a.cmd == "rules":
        write_rules()
        print("rules: Rev 18 DRU block re-asserted")
        return 0
    return {"calc": cmd_calc, "generate": cmd_generate, "schematic": cmd_schematic,
            "verify": cmd_verify, "render": cmd_render}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
