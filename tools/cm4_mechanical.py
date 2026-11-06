#!/usr/bin/env python3
"""
Radxa CM4 Nano Carrier - CM4 form-factor mechanical source of truth (Rev 6.0)
==============================================================================

Single source of truth for everything that must physically line up with the
compute module: board outline, 4x mounting holes and the two Hirose DF40
100-pin board-to-board receptacles (J1 = module connector 1, pins 1-100;
J2 = module connector 2, pins 101-200).

Sub-commands
------------
  generate   Write footprints, place them into the .kicad_pcb (via pcbnew),
             regenerate board_outline.dxf and specs/radxa_cm4_pinout.json.
  verify     Re-load the saved board with pcbnew and check, pad-by-pad, every
             one of the 200 DF40 pads plus holes against an *independent*
             expression of the reference geometry, then run keep-out,
             anti-rotation, tolerance-stack and pinout checks. Writes
             artifacts/rev6_df40_alignment_report.md. Exit code 1 on failure.

Primary sources (all values below were extracted and independently re-checked)
------------------------------------------------------------------------------
[S1] Raspberry Pi CM4 datasheet, Fig. 4 "CM4 viewed from the top"
     https://datasheets.raspberrypi.com/cm4/cm4-datasheet.pdf
     55 x 40 mm, 4x M2.5 holes inset 3.5 mm, connector centreline 3 mm from
     long edge, connector centre 21.5 mm from the antenna-side hole column.
[S2] Raspberry Pi CM4IO KiCad design, footprint
     CM4IO.pretty/Raspberry-Pi-4-Compute-Module.kicad_mod
     https://datasheets.raspberrypi.com/cm4io/CM4IO-KiCAD.zip
     Origin = antenna-side hole on the connector-1 edge. Holes NPTH d2.7 at
     (0,0) (33,0) (0,-48) (33,-48). Pins 1..99 odd x=-2.00, even x=+1.08,
     y = -31.3 + 0.4*k. Pins 101..199 odd x=31.92, even x=35.00.
[S3] Radxa CM4 (RK3576) product brief RAD-DOC-0151 sec. 7 and Radxa CM3
     datasheet Fig. 13: 55 x 40, 4x d2.6 holes on 48.0 x 33.0 pitch,
     connector centreline 3.0 from long edge, 25.0 from short edge.
[S4] Hirose DF40 catalog ed. 2020.2(6): DF40C-100DS / DF40HC(3.0)-100DS
     A=22.6, B=19.6, land 0.2 x 0.7 rows at +/-1.54 (outer 3.78 / inner
     2.38), self-alignment 0.33 mm, not polarised, stencil 0.12 mm / 80 %.

Coordinate frame used on this carrier (KiCad, Y down, viewed from top/F.Cu)
---------------------------------------------------------------------------
  Origin = board corner nearest the module antenna / U.FL cut-out on the
  connector-1 edge.  X along 55 mm, Y along 40 mm.  This is the datasheet
  [S1] top view with the antenna on the LEFT and connector 1 on the TOP edge.
  Transform from [S2] footprint coords:  X = 3.5 - y_fp ,  Y = 3.5 + x_fp
  (matrix [[0,-1],[1,0]], det=+1: a pure rotation, never a mirror).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PCB = ROOT / "radxa-cm4-nano-carrier.kicad_pcb"
LIB = ROOT / "lib" / "radxa-cm4-nano-carrier.pretty"
DXF = ROOT / "board_outline.dxf"
PINOUT = ROOT / "specs" / "radxa_cm4_pinout.json"
REPORT = ROOT / "artifacts" / "rev6_df40_alignment_report.md"

# --------------------------------------------------------------------------
# 1. Mechanical constants (mm)
# --------------------------------------------------------------------------
BOARD_W, BOARD_H = 55.0, 40.0
HOLE_INSET = 3.5                                   # [S1][S3]
HOLE_DRILL = 2.7                                   # [S2] NPTH; module hole d2.6 [S3]
HOLE_KEEPOUT_D = 6.2                               # 5 mm A/F hex standoff corner dia 5.77 + 0.4
HOLES = [(HOLE_INSET, HOLE_INSET), (BOARD_W - HOLE_INSET, HOLE_INSET),
         (HOLE_INSET, BOARD_H - HOLE_INSET), (BOARD_W - HOLE_INSET, BOARD_H - HOLE_INSET)]

# DF40 receptacle land pattern [S4]
DF40_PITCH = 0.4
DF40_PAIRS = 50
DF40_ROW_HALF = 1.54                               # row centre = +/-1.54 -> 3.08 c-c
DF40_PAD_W, DF40_PAD_L = 0.20, 0.70
DF40_A = 22.6                                      # overall length
DF40_CRTYD = (11.8, 2.39)                          # half extents, matches KiCad lib
DF40_SELF_ALIGN = 0.33                             # [S4] mating self-alignment range

# Selected stack: DF40HC(3.0)-100DS-0.4V(51) -> 3.0 mm board-to-board, 1.5 mm
# component clearance under the module [S1]. (DF40C-100DS = 1.5 mm stack,
# ZERO clearance: impossible here because the whole carrier top side lies
# under the module - the carrier is the module outline.)
STACK = {
    "DF40HC(3.0)-100DS-0.4V(51)": dict(stack=3.0, body_w=2.94, height=2.9, clearance=1.5),
    "DF40C-100DS-0.4V(51)": dict(stack=1.5, body_w=2.88, height=1.45, clearance=0.0),
}
SELECTED_RECEPTACLE = "DF40HC(3.0)-100DS-0.4V(51)"
TOP_SIDE_MAX_COMPONENT_H = 1.2                     # 1.5 clearance - 0.3 tolerance margin

# Connector placement: centre and rotation in carrier coords.  With the
# footprint convention pin1 at local (-9.8, +1.54) and rotation 180 deg, pin 1
# lands at centre + (9.8, -1.54) i.e. right end, outer (edge) row - exactly
# as [S1] Fig. 4 draws "Pin1" for connector 1 and "Pin101" for connector 2.
CONN_EDGE_OFFSET = 3.04                            # [S2]: 3.5 - 0.46 (datasheet rounds to 3)
CONN_X = HOLE_INSET + 21.5                         # [S1] = 25.0 from antenna-side edge
J1 = dict(ref="J1", first_pin=1, at=(CONN_X, CONN_EDGE_OFFSET), rot=180.0)
J2 = dict(ref="J2", first_pin=101, at=(CONN_X, BOARD_H - CONN_EDGE_OFFSET), rot=180.0)


# --------------------------------------------------------------------------
# 2. Independent reference: the RPi CM4IO module footprint [S2], expressed in
#    ITS OWN frame and transformed.  verify() compares the board against this,
#    NOT against the placement constants above, so a mistake in J1/J2 'at'
#    or 'rot' is caught.
# --------------------------------------------------------------------------
def reference_pad_fp(pin: int) -> tuple[float, float]:
    """Pad centre of `pin` in the CM4IO module-footprint frame [S2]."""
    if 1 <= pin <= 100:
        k = (pin - 1) // 2
        x = -2.00 if pin % 2 else 1.08
    elif 101 <= pin <= 200:
        k = (pin - 101) // 2
        x = 31.92 if pin % 2 else 35.00
    else:
        raise ValueError(pin)
    return x, -31.3 + DF40_PITCH * k


def fp_to_carrier(x_fp: float, y_fp: float) -> tuple[float, float]:
    return HOLE_INSET - y_fp, HOLE_INSET + x_fp


REFERENCE_HOLES_FP = [(0.0, 0.0), (33.0, 0.0), (0.0, -48.0), (33.0, -48.0)]


# --------------------------------------------------------------------------
# 3. Footprint generation (.kicad_mod text, KiCad 8 format)
# --------------------------------------------------------------------------
def df40_footprint(name: str, descr: str, body_w: float, height: float) -> str:
    cx, cy = DF40_CRTYD
    hb = body_w / 2
    ha = DF40_A / 2
    L = [f'(footprint "{name}"', '  (version 20240108)', '  (generator "cm4_mechanical.py")',
         '  (layer "F.Cu")', f'  (descr "{descr}")',
         '  (tags "Hirose DF40 0.4mm board-to-board receptacle CM4")',
         '  (attr smd)',
         # 80 % stencil area per Hirose: 0.9 x 0.9 linear = 0.81
         '  (solder_paste_ratio -0.1)',
         '  (property "Reference" "REF**" (at 0 -3.2 0) (layer "F.SilkS") (effects (font (size 0.8 0.8) (thickness 0.12))))',
         f'  (property "Value" "{name}" (at 0 3.2 0) (layer "F.Fab") (effects (font (size 0.8 0.8) (thickness 0.12))))',
         f'  (property "Height" "{height}" (at 0 0 0) (layer "F.Fab") (hide yes) (effects (font (size 1 1))))']
    # Fab body
    for (x1, y1, x2, y2) in [(-ha, -hb, ha, -hb), (ha, -hb, ha, hb), (ha, hb, -ha, hb), (-ha, hb, -ha, -hb)]:
        L.append(f'  (fp_line (start {x1:.3f} {y1:.3f}) (end {x2:.3f} {y2:.3f}) (stroke (width 0.1) (type solid)) (layer "F.Fab"))')
    # Courtyard
    for (x1, y1, x2, y2) in [(-cx, -cy, cx, -cy), (cx, -cy, cx, cy), (cx, cy, -cx, cy), (-cx, cy, -cx, -cy)]:
        L.append(f'  (fp_line (start {x1:.3f} {y1:.3f}) (end {x2:.3f} {y2:.3f}) (stroke (width 0.05) (type solid)) (layer "F.CrtYd"))')
    # Silk: end brackets only (outside pad field) + pin-1 arrow; the part is
    # NOT polarised [S4] so the pin-1 marker is the only orientation cue.
    sx, sy = ha + 0.12, hb + 0.12
    for s in (-1, 1):
        L.append(f'  (fp_line (start {s*sx:.3f} {-sy:.3f}) (end {s*sx:.3f} {sy:.3f}) (stroke (width 0.12) (type solid)) (layer "F.SilkS"))')
    p1x = -DF40_PITCH * (DF40_PAIRS - 1) / 2
    L.append(f'  (fp_poly (pts (xy {p1x:.3f} {DF40_ROW_HALF+0.55:.3f}) (xy {p1x-0.3:.3f} {DF40_ROW_HALF+0.95:.3f}) '
             f'(xy {p1x+0.3:.3f} {DF40_ROW_HALF+0.95:.3f})) (stroke (width 0.1) (type solid)) (fill solid) (layer "F.SilkS"))')
    L.append(f'  (fp_text user "1" (at {p1x-0.9:.3f} {DF40_ROW_HALF:.3f} 0) (layer "F.Fab") (effects (font (size 0.5 0.5) (thickness 0.08))))')
    for i in range(DF40_PAIRS):
        x = p1x + DF40_PITCH * i
        for pin, y in ((2 * i + 1, DF40_ROW_HALF), (2 * i + 2, -DF40_ROW_HALF)):
            L.append(f'  (pad "{pin}" smd rect (at {x:.3f} {y:.3f}) (size {DF40_PAD_W} {DF40_PAD_L}) '
                     '(layers "F.Cu" "F.Paste" "F.Mask"))')
    L.append(f'  (model "${{KICAD8_3DMODEL_DIR}}/Connector_Hirose.3dshapes/{name}.wrl" (offset (xyz 0 0 0)) (scale (xyz 1 1 1)) (rotate (xyz 0 0 0)))')
    L.append(')')
    return "\n".join(L) + "\n"


def hole_footprint() -> str:
    r = HOLE_KEEPOUT_D / 2
    return "\n".join([
        '(footprint "MountingHole_2.7mm_M2.5_CM4"', '  (version 20240108)', '  (generator "cm4_mechanical.py")',
        '  (layer "F.Cu")', '  (descr "CM4 module mounting hole, NPTH d2.7 (RPi CM4IO), d6.2 standoff keep-out")',
        '  (attr exclude_from_pos_files exclude_from_bom)',
        '  (property "Reference" "H**" (at 0 -3.6 0) (layer "F.Fab") (effects (font (size 0.6 0.6) (thickness 0.1))))',
        '  (property "Value" "M2.5 x 3.0 mm standoff" (at 0 3.6 0) (layer "F.Fab") (effects (font (size 0.6 0.6) (thickness 0.1))))',
        f'  (fp_circle (center 0 0) (end {r:.3f} 0) (stroke (width 0.05) (type solid)) (fill none) (layer "F.CrtYd"))',
        f'  (fp_circle (center 0 0) (end {r:.3f} 0) (stroke (width 0.05) (type solid)) (fill none) (layer "B.CrtYd"))',
        f'  (fp_circle (center 0 0) (end 2.5 0) (stroke (width 0.1) (type solid)) (fill none) (layer "F.Fab"))',
        f'  (pad "" np_thru_hole circle (at 0 0) (size {HOLE_DRILL} {HOLE_DRILL}) (drill {HOLE_DRILL}) '
        '(layers "*.Cu" "*.Mask") (clearance 0.5))',
        ')']) + "\n"


FOOTPRINTS = {
    "Hirose_DF40HC-3.0-100DS-0.4V_2x50_P0.4mm":
        ("Hirose DF40HC(3.0)-100DS-0.4V(51) receptacle, 3.0 mm stack with DF40C-100DP (CM4 module plug)", 2.94, 2.9),
    "Hirose_DF40C-100DS-0.4V_2x50_P0.4mm":
        ("Hirose DF40C-100DS-0.4V(51) receptacle, 1.5 mm stack with DF40C-100DP - ZERO under-module clearance", 2.88, 1.45),
}
SELECTED_FP = "Hirose_DF40HC-3.0-100DS-0.4V_2x50_P0.4mm"


# --------------------------------------------------------------------------
# 4. Pinout (RPi CM4 standard [S2] + Radxa deviations [S3])
# --------------------------------------------------------------------------
RADXA_CM3_DEVIATIONS = {
    16: "NC", 18: "NC", 64: "NC", 68: "NC", 70: "NC", 72: "NC", 73: "NC",
    76: "SDMMC0_DET_L", 80: "I2C0_SCL (PMIC, 1V8, pulled up on module)",
    82: "I2C0_SDA (PMIC, 1V8, pulled up on module)", 92: "PMIC_SLEEP_H (V1.3)",
    93: "NC", 94: "NC", 96: "NC", 99: "PWRON_KEY", 100: "PMIC_RESET_KEY",
}
RADXA_CM4_RK3576_DEVIATIONS = {
    16: "UART5_RX_M0_1V8", 18: "UART5_TX_M0_1V8",
    64: "GPIO/I2C4 (1V8) - see Radxa CM4 pinout v1.20", 68: "GPIO/I2C4 (1V8) - see Radxa CM4 pinout v1.20",
    70: "GPIO/I2C4 (1V8) - see Radxa CM4 pinout v1.20", 72: "GPIO/I2C4 (1V8) - see Radxa CM4 pinout v1.20",
    73: "GPIO/I2C4 (1V8) - see Radxa CM4 pinout v1.20",
    92: "RESET_L", 93: "MASKROM", 94: "PDM1", 96: "PDM1", 99: "PWRON_L", 100: "EXT_RST",
}
EXPECTED_POWER = {
    "+5V_IN": [77, 79, 81, 83, 85, 87],
    "+3V3_OUT": [84, 86],
    "+1V8_OUT": [88, 90],
    "GPIO_VREF": [78],
}
EXPECTED_GND = sorted([1, 2, 7, 8, 13, 14, 22, 23, 32, 33, 42, 43, 52, 53, 59, 60, 65, 66, 71, 74, 98,
                       107, 108, 113, 114, 119, 120, 125, 126, 131, 132, 137, 138, 144, 150, 155, 156,
                       161, 162, 167, 168, 173, 174, 179, 180, 185, 186, 191, 192, 197, 198])

# RPi CM4 pin function names, extracted from the CM4IOv5 board module footprint
# pinfunction fields [S2] (net-name normalised: spaces/parentheses removed).
_CM4 = """GND GND Ethernet_Pair3_P Ethernet_Pair1_P Ethernet_Pair3_N Ethernet_Pair1_N GND GND
Ethernet_Pair2_N Ethernet_Pair0_N Ethernet_Pair2_P Ethernet_Pair0_P GND GND Ethernet_nLED3
Ethernet_SYNC_IN Ethernet_nLED2 Ethernet_SYNC_OUT Ethernet_nLED1 EEPROM_nWP PI_nLED_Activity GND GND
GPIO26 GPIO21 GPIO19 GPIO20 GPIO13 GPIO16 GPIO6 GPIO12 GND GND GPIO5 ID_SC ID_SD GPIO7 GPIO11 GPIO8
GPIO9 GPIO25 GND GND GPIO10 GPIO24 GPIO22 GPIO23 GPIO27 GPIO18 GPIO17 GPIO15 GND GND GPIO4 GPIO14
GPIO3 SD_CLK GPIO2 GND GND SD_DAT3 SD_CMD SD_DAT0 SD_DAT5 GND GND SD_DAT1 SD_DAT4 SD_DAT2 SD_DAT7 GND
SD_DAT6 SD_VDD_OVERRIDE GND SD_PWR_ON RESERVED_76 +5V_IN GPIO_VREF +5V_IN SCL0 +5V_IN SDA0 +5V_IN
+3V3_OUT +5V_IN +3V3_OUT +5V_IN +1V8_OUT WL_nDISABLE +1V8_OUT BT_nDISABLE RUN_PG nRPIBOOT AnalogIP1
nPI_LED_PWR AnalogIP0 CAMERA_GPIO GND GLOBAL_EN nEXTRST
USB_OTG_ID PCIE_CLK_nREQ USB2_N RESERVED_104 USB2_P RESERVED_106 GND GND PCIE_nRST PCIE_CLK_P
VDAC_COMP PCIE_CLK_N GND GND CAM1_D0_N PCIE_RX_P CAM1_D0_P PCIE_RX_N GND GND CAM1_D1_N PCIE_TX_P
CAM1_D1_P PCIE_TX_N GND GND CAM1_C_N CAM0_D0_N CAM1_C_P CAM0_D0_P GND GND CAM1_D2_N CAM0_D1_N
CAM1_D2_P CAM0_D1_P GND GND CAM1_D3_N CAM0_C_N CAM1_D3_P CAM0_C_P HDMI1_HOTPLUG GND HDMI1_SDA
HDMI1_TX2_P HDMI1_SCL HDMI1_TX2_N HDMI1_CEC GND HDMI0_CEC HDMI1_TX1_P HDMI0_HOTPLUG HDMI1_TX1_N GND
GND DSI0_D0_N HDMI1_TX0_P DSI0_D0_P HDMI1_TX0_N GND GND DSI0_D1_N HDMI1_CLK_P DSI0_D1_P HDMI1_CLK_N
GND GND DSI0_C_N HDMI0_TX2_P DSI0_C_P HDMI0_TX2_N GND GND DSI1_D0_N HDMI0_TX1_P DSI1_D0_P
HDMI0_TX1_N GND GND DSI1_D1_N HDMI0_TX0_P DSI1_D1_P HDMI0_TX0_N GND GND DSI1_C_N HDMI0_CLK_P
DSI1_C_P HDMI0_CLK_N GND GND DSI1_D2_N DSI1_D3_N DSI1_D2_P DSI1_D3_P GND GND HDMI0_SDA HDMI0_SCL"""
CM4_PINS = {i + 1: n for i, n in enumerate(_CM4.split())}
assert len(CM4_PINS) == 200, len(CM4_PINS)


def build_pinout() -> dict:
    def conn(lo, hi):
        return {str(p): CM4_PINS[p] for p in range(lo, hi + 1)}
    return {
        "schema": "radxa-cm4-nano-carrier/pinout/v2 (Rev 6.0)",
        "module": "Raspberry Pi CM4 form factor - Radxa CM3 (RK3566) / Radxa CM4 (RK3576)",
        "generated_by": "tools/cm4_mechanical.py generate  (do not hand-edit)",
        "sources": [
            "https://datasheets.raspberrypi.com/cm4/cm4-datasheet.pdf",
            "https://datasheets.raspberrypi.com/cm4io/CM4IO-KiCAD.zip (CM4IOv5.kicad_pcb module pinfunction)",
            "https://dl.radxa.com/cm4/docs/radxa_cm4_product_brief.pdf",
            "Radxa CM3 datasheet (J69 pin table), Radxa CM4 pinout v1.20",
        ],
        "mechanical": {
            "frame": "KiCad carrier frame, origin = corner at antenna/U.FL end of connector-1 edge, X along 55, Y along 40 (down), top view",
            "width_mm": BOARD_W, "height_mm": BOARD_H,
            "mounting_holes": [{"ref": f"H{i+1}", "x": x, "y": y, "drill": HOLE_DRILL, "type": "NPTH",
                                "module_hole_dia": 2.6} for i, (x, y) in enumerate(HOLES)],
            "hole_pitch_mm": [BOARD_W - 2 * HOLE_INSET, BOARD_H - 2 * HOLE_INSET],
            "connectors": [
                {"ref": "J1", "module_connector": 1, "pins": "1-100", "center": list(J1["at"]), "rotation": J1["rot"],
                 "pin1": list(fp_to_carrier(*reference_pad_fp(1)))},
                {"ref": "J2", "module_connector": 2, "pins": "101-200", "center": list(J2["at"]), "rotation": J2["rot"],
                 "pin1": list(fp_to_carrier(*reference_pad_fp(101)))},
            ],
            "connector_center_to_center_mm": round(J2["at"][1] - J1["at"][1], 3),
            "receptacle": SELECTED_RECEPTACLE, "module_plug": "DF40C-100DP-0.4V(51)",
            "stack_height_mm": STACK[SELECTED_RECEPTACLE]["stack"],
            "standoff": "M2.5 female-female, 3.0 mm length",
            "top_side_max_component_height_mm": TOP_SIDE_MAX_COMPONENT_H,
        },
        "power_pins": EXPECTED_POWER,
        "gnd_pins": EXPECTED_GND,
        "connector_1": {"name": "Module connector 1 -> carrier J1 (" + SELECTED_RECEPTACLE + ")", "pins": conn(1, 100)},
        "connector_2": {"name": "Module connector 2 -> carrier J2 (" + SELECTED_RECEPTACLE + ")", "pins": conn(101, 200)},
        "radxa_deviations": {
            "note": "Signal pins whose function differs from RPi CM4. Power/GND pin numbers are identical on all three modules.",
            "radxa_cm3_rk3566": {str(k): v for k, v in RADXA_CM3_DEVIATIONS.items()},
            "radxa_cm4_rk3576": {str(k): v for k, v in RADXA_CM4_RK3576_DEVIATIONS.items()},
            "third_connector": "Radxa CM3/CM4 carry a 3rd 100-pin plug on a short edge; not mated on this carrier "
                               "(unmated plug 1.14 mm tall vs 3.0 mm stack -> 1.86 mm free, no keep-out conflict).",
        },
    }


# --------------------------------------------------------------------------
# 5. generate
# --------------------------------------------------------------------------
def write_footprints() -> None:
    LIB.mkdir(parents=True, exist_ok=True)
    for name, (descr, bw, h) in FOOTPRINTS.items():
        (LIB / f"{name}.kicad_mod").write_text(df40_footprint(name, descr, bw, h))
    (LIB / "MountingHole_2.7mm_M2.5_CM4.kicad_mod").write_text(hole_footprint())
    (ROOT / "fp-lib-table").write_text(
        '(fp_lib_table\n  (version 7)\n  (lib (name "radxa-cm4-nano-carrier")(type "KiCad")'
        '(uri "${KIPRJMOD}/lib/radxa-cm4-nano-carrier.pretty")(options "")(descr "Rev 6 CM4 mechanical footprints"))\n)\n')


def generate_pcb() -> None:
    import pcbnew
    mm = pcbnew.FromMM
    board = pcbnew.BOARD()
    board.SetCopperLayerCount(4)
    ds = board.GetDesignSettings()
    ds.SetBoardThickness(mm(1.6))
    ds.m_MinClearance = mm(0.1)
    ds.m_TrackMinWidth = mm(0.1)
    ds.m_CopperEdgeClearance = mm(0.3)
    ds.m_HoleClearance = mm(0.25)
    ds.m_ViasMinSize = mm(0.45)
    ds.m_ViasMinAnnularWidth = mm(0.12)
    ds.m_MinThroughDrill = mm(0.2)
    ds.m_HoleToHoleMin = mm(0.25)
    ds.m_SolderMaskExpansion = mm(0.05)
    ds.m_SolderMaskMinWidth = mm(0.0)
    tb = board.GetTitleBlock()
    tb.SetTitle("Radxa CM4 Nano Carrier")
    tb.SetRevision("6.0")
    tb.SetCompany("Sreesai Arjun Engineering")
    tb.SetComment(0, "Rev 6: DF40 J1/J2 + holes placed to RPi CM4 / Radxa CM3/CM4 mechanical drawing")
    tb.SetComment(1, "Generated by tools/cm4_mechanical.py - edit the script, not the placement")

    # Edge cuts
    pts = [(0, 0), (BOARD_W, 0), (BOARD_W, BOARD_H), (0, BOARD_H)]
    for a, b in zip(pts, pts[1:] + pts[:1]):
        s = pcbnew.PCB_SHAPE(board)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(pcbnew.VECTOR2I(mm(a[0]), mm(a[1])))
        s.SetEnd(pcbnew.VECTOR2I(mm(b[0]), mm(b[1])))
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(mm(0.1))
        board.Add(s)

    # Nets from pinout
    nets = {}
    def net(name):
        if name not in nets:
            n = pcbnew.NETINFO_ITEM(board, name)
            board.Add(n)
            nets[name] = n
        return nets[name]

    lib = str(LIB)
    for spec in (J1, J2):
        fp = pcbnew.FootprintLoad(lib, SELECTED_FP)
        fp.SetReference(spec["ref"])
        fp.SetValue(SELECTED_RECEPTACLE)
        fp.SetFPIDAsString(f"radxa-cm4-nano-carrier:{SELECTED_FP}")
        board.Add(fp)
        fp.SetPosition(pcbnew.VECTOR2I(mm(spec["at"][0]), mm(spec["at"][1])))
        fp.SetOrientationDegrees(spec["rot"])
        for pad in fp.Pads():
            mod_pin = int(pad.GetNumber()) + spec["first_pin"] - 1
            pad.SetNet(net(CM4_PINS[mod_pin]))
        fp.SetLocked(True)
        fp.Reference().SetTextAngleDegrees(0)
        rx = spec["at"][0] - 13.6
        fp.Reference().SetPosition(pcbnew.VECTOR2I(mm(rx), mm(spec["at"][1])))

    for i, (x, y) in enumerate(HOLES):
        fp = pcbnew.FootprintLoad(lib, "MountingHole_2.7mm_M2.5_CM4")
        fp.SetReference(f"H{i+1}")
        fp.SetFPIDAsString("radxa-cm4-nano-carrier:MountingHole_2.7mm_M2.5_CM4")
        board.Add(fp)
        fp.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
        fp.SetLocked(True)

    # Module outline + anti-rotation note on User.Drawings / F.Fab
    for a, b in zip(pts, pts[1:] + pts[:1]):
        s = pcbnew.PCB_SHAPE(board)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(pcbnew.VECTOR2I(mm(a[0]), mm(a[1])))
        s.SetEnd(pcbnew.VECTOR2I(mm(b[0]), mm(b[1])))
        s.SetLayer(pcbnew.Dwgs_User)
        s.SetWidth(mm(0.1))
        board.Add(s)
    for txt, (x, y), layer in [
        ("CM4 MODULE OUTLINE 55x40 (U.FL/ANTENNA END = X0)", (27.5, 20.0), pcbnew.Dwgs_User),
        (f"TOP-SIDE PARTS <= {TOP_SIDE_MAX_COMPONENT_H} mm (DF40HC 3.0 stack)", (27.5, 22.0), pcbnew.Dwgs_User),
        ("CM4", (8.5, 20.0), pcbnew.F_SilkS),
    ]:
        t = pcbnew.PCB_TEXT(board)
        t.SetText(txt)
        t.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
        t.SetLayer(layer)
        t.SetTextSize(pcbnew.VECTOR2I(mm(0.8), mm(0.8)))
        t.SetTextThickness(mm(0.12))
        board.Add(t)

    board.Save(str(PCB))


def generate_dxf() -> None:
    import ezdxf
    doc = ezdxf.new("R2010", setup=True)
    doc.units = ezdxf.units.MM
    msp = doc.modelspace()
    for name, color in [("EDGE_CUTS", 7), ("HOLES", 1), ("HOLE_KEEPOUT", 8), ("CONNECTORS", 3),
                        ("CONNECTOR_PADS", 2), ("PIN1", 6), ("COMPONENTS_BOTTOM", 5), ("NOTES", 7)]:
        doc.layers.add(name, color=color)
    # DXF is Y-up: mirror KiCad Y so the drawing reads as the top view.
    f = lambda x, y: (x, BOARD_H - y)
    msp.add_lwpolyline([f(0, 0), f(BOARD_W, 0), f(BOARD_W, BOARD_H), f(0, BOARD_H)], close=True,
                       dxfattribs={"layer": "EDGE_CUTS"})
    for x, y in HOLES:
        msp.add_circle(f(x, y), HOLE_DRILL / 2, dxfattribs={"layer": "HOLES"})
        msp.add_circle(f(x, y), HOLE_KEEPOUT_D / 2, dxfattribs={"layer": "HOLE_KEEPOUT"})
    bw = STACK[SELECTED_RECEPTACLE]["body_w"] / 2
    for spec in (J1, J2):
        cx, cy = spec["at"]
        msp.add_lwpolyline([f(cx - DF40_A / 2, cy - bw), f(cx + DF40_A / 2, cy - bw),
                            f(cx + DF40_A / 2, cy + bw), f(cx - DF40_A / 2, cy + bw)], close=True,
                           dxfattribs={"layer": "CONNECTORS"})
        for p in range(spec["first_pin"], spec["first_pin"] + 100):
            x, y = fp_to_carrier(*reference_pad_fp(p))
            msp.add_lwpolyline([f(x - DF40_PAD_W / 2, y - DF40_PAD_L / 2), f(x + DF40_PAD_W / 2, y - DF40_PAD_L / 2),
                                f(x + DF40_PAD_W / 2, y + DF40_PAD_L / 2), f(x - DF40_PAD_W / 2, y + DF40_PAD_L / 2)],
                               close=True, dxfattribs={"layer": "CONNECTOR_PADS"})
        x, y = fp_to_carrier(*reference_pad_fp(spec["first_pin"]))
        msp.add_circle(f(x, y), 0.3, dxfattribs={"layer": "PIN1"})
        msp.add_text(f'{spec["ref"]} PIN {spec["first_pin"]}', height=0.6,
                     dxfattribs={"layer": "PIN1"}).set_placement(f(x + 0.6, y))
    # Peripheral connectors: the whole top side is under the module, so all
    # tall I/O moves to the bottom side (envelopes carried over from Rev 5).
    for (x1, y1, x2, y2, label) in [(7.5, 32.5, 16.5, 40.0, "USB-C (BOTTOM)"),
                                    (39.85, 32.95, 48.15, 40.0, "uHDMI J4 (BOTTOM, Rev 11)"),   # = J4 courtyard, tools/hdmi_tmds.py
                                    (40.0, 12.0, 55.0, 28.0, "ETH/IO (BOTTOM)")]:
        msp.add_lwpolyline([f(x1, y1), f(x2, y1), f(x2, y2), f(x1, y2)], close=True,
                           dxfattribs={"layer": "COMPONENTS_BOTTOM"})
        msp.add_text(label, height=0.8, dxfattribs={"layer": "COMPONENTS_BOTTOM"}).set_placement(f(x1 + 0.3, y1 + 1.5))
    msp.add_text(f"REV 6.0 - CM4 FORM FACTOR, TOP VIEW, {SELECTED_RECEPTACLE}", height=0.8,
                 dxfattribs={"layer": "NOTES"}).set_placement(f(10.0, 20.0))
    doc.saveas(DXF)


def cmd_generate(_args) -> int:
    write_footprints()
    generate_pcb()
    generate_dxf()
    PINOUT.write_text(json.dumps(build_pinout(), indent=2) + "\n")
    print(f"generated: {LIB.relative_to(ROOT)}, {PCB.name}, {DXF.name}, {PINOUT.relative_to(ROOT)}")
    return 0


# --------------------------------------------------------------------------
# 6. verify
# --------------------------------------------------------------------------
class Checks:
    def __init__(self):
        self.rows = []

    def add(self, name, ok, detail):
        self.rows.append((name, bool(ok), detail))
        return ok

    @property
    def failed(self):
        return [r for r in self.rows if not r[1]]


def rect_overlap(a, b):
    return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]


def circle_rect_dist(cx, cy, r):
    x0, y0, x1, y1 = r
    dx = max(x0 - cx, 0, cx - x1)
    dy = max(y0 - cy, 0, cy - y1)
    return math.hypot(dx, dy)


def cmd_verify(_args) -> int:
    import pcbnew
    tm = pcbnew.ToMM
    board = pcbnew.LoadBoard(str(PCB))
    C = Checks()
    TOL = 0.001

    # --- Board outline
    bb = board.GetBoardEdgesBoundingBox()
    w, h = tm(bb.GetWidth()), tm(bb.GetHeight())
    # bbox includes edge line width
    C.add("Board outline 55.0 x 40.0 mm", abs(w - BOARD_W) < 0.2 and abs(h - BOARD_H) < 0.2
          and abs(tm(bb.GetX())) < 0.1 and abs(tm(bb.GetY())) < 0.1, f"bbox {w:.3f} x {h:.3f} @ ({tm(bb.GetX()):.3f},{tm(bb.GetY()):.3f})")

    fps = {fp.GetReference(): fp for fp in board.GetFootprints()}

    # --- Holes vs reference [S2] transformed
    ref_holes = sorted(fp_to_carrier(*p) for p in REFERENCE_HOLES_FP)
    got = []
    for r in ("H1", "H2", "H3", "H4"):
        fp = fps.get(r)
        if not fp:
            C.add(f"{r} present", False, "missing")
            continue
        pad = list(fp.Pads())[0]
        got.append((round(tm(pad.GetPosition().x), 4), round(tm(pad.GetPosition().y), 4)))
        C.add(f"{r} NPTH drill {HOLE_DRILL} mm", pad.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH
              and abs(tm(pad.GetDrillSize().x) - HOLE_DRILL) < TOL, f"drill {tm(pad.GetDrillSize().x):.3f}")
    worst = max((min(math.dist(g, r) for g in got) for r in ref_holes), default=99)
    C.add("4 holes coincide with CM4 hole pattern (48.0 x 33.0 pitch, 3.5 inset)", len(got) == 4 and worst < TOL,
          f"max error {worst*1000:.1f} um; holes {sorted(got)}")
    C.add("Module hole d2.6 passes M2.5 through carrier d2.7 (no interference)", HOLE_DRILL >= 2.6, "d2.7 >= d2.6")

    # --- 200 pads, absolute position, size, layer, net
    pad_err = []
    net_err = []
    size_err = []
    for spec in (J1, J2):
        fp = fps.get(spec["ref"])
        if not fp:
            C.add(f'{spec["ref"]} present', False, "missing")
            continue
        C.add(f'{spec["ref"]} on F.Cu, value {SELECTED_RECEPTACLE}', fp.GetLayer() == pcbnew.F_Cu and fp.GetValue() == SELECTED_RECEPTACLE,
              f'layer {fp.GetLayerName()}, value {fp.GetValue()}')
        pads = list(fp.Pads())
        C.add(f'{spec["ref"]} has 100 pads', len(pads) == 100, f"{len(pads)} pads")
        for pad in pads:
            mod_pin = int(pad.GetNumber()) + spec["first_pin"] - 1
            ex, ey = fp_to_carrier(*reference_pad_fp(mod_pin))
            px, py = tm(pad.GetPosition().x), tm(pad.GetPosition().y)
            e = math.hypot(px - ex, py - ey)
            if e > TOL:
                pad_err.append((mod_pin, px, py, ex, ey))
            bbx = pad.GetBoundingBox()
            sx, sy = tm(bbx.GetWidth()), tm(bbx.GetHeight())
            if abs(sx - DF40_PAD_W) > 0.01 or abs(sy - DF40_PAD_L) > 0.01:
                size_err.append((mod_pin, sx, sy))
            if pad.GetNetname() != CM4_PINS[mod_pin]:
                net_err.append((mod_pin, pad.GetNetname(), CM4_PINS[mod_pin]))
    C.add("All 200 DF40 pad centres match CM4IO module footprint (tol 1 um)", not pad_err,
          "0 deviations" if not pad_err else f"{len(pad_err)} off, e.g. {pad_err[:3]}")
    C.add("All 200 pads are 0.20 x 0.70 mm, long axis across the row (Hirose land)", not size_err,
          "ok" if not size_err else f"{size_err[:3]}")
    C.add("All 200 pad nets match CM4 pin function", not net_err, "ok" if not net_err else f"{net_err[:3]}")

    # --- Named landmark pins vs datasheet Fig. 4 text
    lm = {1: (34.8, 1.5), 2: (34.8, 4.58), 99: (15.2, 1.5), 100: (15.2, 4.58),
          101: (34.8, 35.42), 102: (34.8, 38.5), 199: (15.2, 35.42), 200: (15.2, 38.5)}
    lm_bad = []
    for spec in (J1, J2):
        fp = fps.get(spec["ref"])
        for pad in fp.Pads() if fp else []:
            mp = int(pad.GetNumber()) + spec["first_pin"] - 1
            if mp in lm and math.dist(lm[mp], (tm(pad.GetPosition().x), tm(pad.GetPosition().y))) > TOL:
                lm_bad.append(mp)
    C.add("Landmarks: pin 1/101 at right (X=34.8), pin 1 outer row, pin 101 inner row, pin 102 outer row", not lm_bad,
          f"bad {lm_bad}" if lm_bad else "matches datasheet Fig. 4 labelling")

    # --- Derived spacings
    c2c = J2["at"][1] - J1["at"][1]
    C.add("Connector centre-to-centre 33.92 mm (Rev<=5 spec said 14.8 mm: WRONG)", abs(c2c - 33.92) < TOL, f"{c2c:.3f} mm")
    C.add("Connector centre 25.0 mm from antenna-side edge (3.5 + 21.5)", abs(J1["at"][0] - 25.0) < TOL and abs(J2["at"][0] - 25.0) < TOL,
          f"J1 X={J1['at'][0]}, J2 X={J2['at'][0]}")

    # --- Anti-rotation (connector is not polarised [S4])
    rot = [(BOARD_W - x, BOARD_H - y) for (x, y) in (J1["at"], J2["at"])]
    mis = min(abs(rot[0][0] - J2["at"][0]), abs(rot[1][0] - J1["at"][0]))
    pad_span = DF40_PITCH * (DF40_PAIRS - 1)
    C.add("Module rotated 180 deg cannot mate: X offset > self-alignment", mis > DF40_SELF_ALIGN,
          f"180 deg rotation shifts connector by {mis:.2f} mm (> {DF40_SELF_ALIGN} mm self-align); hole pattern is "
          "symmetric so the connector offset (25.0 vs 30.0 mm) is the only mechanical key - silkscreen pin-1 + U.FL marker required")

    # --- Courtyards inside board, clear of hole keep-outs, clear of each other
    crt = {}
    for spec in (J1, J2):
        cx, cy = spec["at"]
        crt[spec["ref"]] = (cx - DF40_CRTYD[0], cy - DF40_CRTYD[1], cx + DF40_CRTYD[0], cy + DF40_CRTYD[1])
    for r, b in crt.items():
        C.add(f"{r} courtyard inside board outline", b[0] >= 0 and b[1] >= 0 and b[2] <= BOARD_W and b[3] <= BOARD_H,
              f"{tuple(round(v, 2) for v in b)}; min edge margin {min(b[0], b[1], BOARD_W-b[2], BOARD_H-b[3]):.2f} mm")
        dmin = min(circle_rect_dist(x, y, b) - HOLE_KEEPOUT_D / 2 for x, y in HOLES)
        C.add(f"{r} courtyard clear of d{HOLE_KEEPOUT_D} standoff keep-outs", dmin > 0, f"min gap {dmin:.2f} mm")
    C.add("J1/J2 courtyards do not overlap", not rect_overlap(crt["J1"], crt["J2"]), "")
    outer_pad_edge = J1["at"][1] - DF40_ROW_HALF - DF40_PAD_L / 2
    C.add("Copper-to-edge >= 0.30 mm (outer pad row)", outer_pad_edge >= 0.3, f"{outer_pad_edge:.2f} mm")
    hole_edge = HOLE_INSET - HOLE_KEEPOUT_D / 2
    C.add("Standoff keep-out fully on board (Rev 5 R1 blocker)", hole_edge >= 0, f"keep-out to edge {hole_edge:.2f} mm")

    # --- Tolerance chains.  The module is LOCATED BY ITS TWO DF40 PAIRS, not
    # by the screws (holes are clearance holes), so two independent chains:
    #  A) connector mating: relative X/Y offset between carrier receptacle and
    #     module plug must stay inside the 0.33 mm self-alignment [S4]. Judged
    #     WORST-CASE (a mis-mate bends 0.12 mm contacts).
    #  B) screw fit: with the module positioned by its connectors, the module
    #     hole (d2.6) and the carrier hole (d2.7) must both pass an M2.5
    #     (d2.5 max). Available radial float 0.05 + 0.10 = 0.15 mm. Judged RSS
    #     (statistical, non-destructive: a bind is caught at assembly), plus
    #     the assembly rule "mate module first, then torque standoffs".
    # Assumed process capability (state on fab/assembly drawing):
    #  NC drill position +/-0.075, artwork/pad registration +/-0.025,
    #  SMT placement of 0.4 mm-pitch DF40 +/-0.05 (vision-aligned).
    chain_a = {"carrier DF40 placement": 0.05, "module DF40 placement": 0.05,
               "carrier pad artwork registration": 0.025, "module pad artwork registration": 0.025,
               "carrier DF40 pitch-span B half-tol (+/-0.08/2)": 0.04, "module DF40 pitch-span B half-tol": 0.04}
    chain_b = {"carrier hole drill position": 0.075, "module hole drill position": 0.075,
               "carrier DF40 placement (connector datum)": 0.05, "module DF40 placement (connector datum)": 0.05}
    float_b = (2.6 - 2.5) / 2 + (HOLE_DRILL - 2.5) / 2
    wc_a = sum(chain_a.values()); rss_a = math.sqrt(sum(v * v for v in chain_a.values()))
    wc_b = sum(chain_b.values()); rss_b = math.sqrt(sum(v * v for v in chain_b.values()))
    C.add("Chain A: DF40 mating offset (worst case) < 0.33 mm self-alignment", wc_a < DF40_SELF_ALIGN,
          f"WC {wc_a:.3f} mm, RSS {rss_a:.3f} mm, WC margin {DF40_SELF_ALIGN-wc_a:.3f} mm")
    C.add(f"Chain B: M2.5 screw passes both holes (RSS < {float_b:.2f} mm radial float)", rss_b < float_b,
          f"RSS {rss_b:.3f} mm, WC {wc_b:.3f} mm ({'WC also passes' if wc_b <= float_b else 'WC exceeds float: mate module BEFORE torquing standoffs so carrier-hole float absorbs it'})")
    contrib = {**{"A: " + k: v for k, v in chain_a.items()}, **{"B: " + k: v for k, v in chain_b.items()}}

    # --- Stack height / standoff
    st = STACK[SELECTED_RECEPTACLE]
    C.add("Stack height selected allows top-side parts", st["clearance"] > TOP_SIDE_MAX_COMPONENT_H,
          f"{SELECTED_RECEPTACLE}: {st['stack']} mm stack, {st['clearance']} mm clearance, limit {TOP_SIDE_MAX_COMPONENT_H} mm")

    # --- Pinout JSON
    pj = json.loads(PINOUT.read_text())
    pins = {**pj["connector_1"]["pins"], **pj["connector_2"]["pins"]}
    perr = []
    for rail, lst in EXPECTED_POWER.items():
        for p in lst:
            if pins.get(str(p)) != rail:
                perr.append((p, pins.get(str(p)), rail))
    for p in EXPECTED_GND:
        if pins.get(str(p)) != "GND":
            perr.append((p, pins.get(str(p)), "GND"))
    extra_gnd = [int(k) for k, v in pins.items() if v == "GND" and int(k) not in EXPECTED_GND]
    five_v = sorted(int(k) for k, v in pins.items() if v == "+5V_IN")
    C.add("Pinout JSON: +5V_IN only on 77/79/81/83/85/87; 3V3 out 84/86; 1V8 out 88/90", not perr and not extra_gnd and len(pins) == 200,
          f"+5V pins {five_v}; errors {perr[:4]} extra GND {extra_gnd}")
    mj = pj["mechanical"]
    C.add("Pinout JSON mechanical section == board", sorted((h["x"], h["y"]) for h in mj["mounting_holes"]) == sorted(HOLES)
          and abs(mj["connector_center_to_center_mm"] - 33.92) < TOL, "holes + connector pitch")

    # --- DXF agrees with board
    import ezdxf
    d = ezdxf.readfile(DXF)
    dh = sorted((round(e.dxf.center.x, 3), round(BOARD_H - e.dxf.center.y, 3)) for e in d.modelspace().query('CIRCLE[layer=="HOLES"]'))
    C.add("board_outline.dxf holes == board holes", dh == sorted(HOLES), f"{dh}")
    npads = len(d.modelspace().query('LWPOLYLINE[layer=="CONNECTOR_PADS"]'))
    C.add("board_outline.dxf carries all 200 pad outlines", npads == 200, f"{npads}")

    # --- Report
    lines = ["# Rev 6.0 - DF40 / CM4 mechanical alignment verification report", "",
             "Generated by `python3 tools/cm4_mechanical.py verify` from the saved `.kicad_pcb` (pcbnew "
             f"{pcbnew.Version()}). Reference geometry: RPi CM4 datasheet Fig. 4, CM4IO KiCad module footprint, "
             "Radxa CM4 product brief sec. 7, Hirose DF40 catalog 2020.2.", "",
             "| # | Check | Result | Detail |", "|---|---|---|---|"]
    for i, (n, ok, det) in enumerate(C.rows, 1):
        lines.append(f"| {i} | {n} | {'PASS' if ok else '**FAIL**'} | {str(det).replace('|', '/')} |")
    lines += ["", f"**{len(C.rows) - len(C.failed)}/{len(C.rows)} checks passed.**", "",
              "## Placement table (carrier frame, mm, KiCad Y-down, top view)", "",
              "| Item | X | Y | Notes |", "|---|---|---|---|"]
    for i, (x, y) in enumerate(HOLES, 1):
        lines.append(f"| H{i} | {x:.2f} | {y:.2f} | NPTH d{HOLE_DRILL}, keep-out d{HOLE_KEEPOUT_D} |")
    for spec in (J1, J2):
        p1 = fp_to_carrier(*reference_pad_fp(spec["first_pin"]))
        lines.append(f"| {spec['ref']} centre | {spec['at'][0]:.2f} | {spec['at'][1]:.2f} | rot {spec['rot']:.0f} deg, "
                     f"pin {spec['first_pin']} at ({p1[0]:.2f}, {p1[1]:.2f}) |")
    lines += ["", "## Tolerance chains", "",
              "Chain A = connector mating (module is located by the DF40 pairs; judged worst-case). "
              "Chain B = M2.5 screw fit through d2.6 module / d2.7 carrier holes (judged RSS; assembly rule: "
              "mate module first, then torque standoffs).", "", "| Contributor | +/- mm |", "|---|---|"]
    lines += [f"| {k} | {v:.3f} |" for k, v in contrib.items()]
    lines += [f"| **Chain A worst case / RSS** | **{wc_a:.3f}** / {rss_a:.3f} (limit {DF40_SELF_ALIGN}) |",
              f"| **Chain B worst case / RSS** | {wc_b:.3f} / **{rss_b:.3f}** (limit {float_b:.3f}) |", ""]
    REPORT.parent.mkdir(exist_ok=True)
    REPORT.write_text("\n".join(lines) + "\n")

    for n, ok, det in C.rows:
        print(f"[{'PASS' if ok else 'FAIL'}] {n} :: {det}")
    print(f"\n{len(C.rows) - len(C.failed)}/{len(C.rows)} passed -> {REPORT.relative_to(ROOT)}")
    return 1 if C.failed else 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest="cmd", required=True)
    sp.add_parser("generate").set_defaults(fn=cmd_generate)
    sp.add_parser("verify").set_defaults(fn=cmd_verify)
    a = ap.parse_args()
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
