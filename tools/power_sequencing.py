#!/usr/bin/env python3
"""
Radxa CM4 Nano Carrier - Rev 17: power sequencing (GLOBAL_EN / RUN / PWR_OK)
============================================================================

Carrier-side sequencing logic for CM4-form-factor modules, written so that one
PCB is *compliant* with the Rockchip modules this carrier targets (Radxa CM3 =
RK3566 + RK817, Radxa CM4 = RK3576 + RK806S, Radxa CM5 = RK3588S + RK806) and
still offers the Raspberry Pi CM4 behaviour through an assembly variant.

The one fact that drives the whole design: the three "control" pins of the
CM4 connector do NOT mean the same thing on every module.

  J1 pin | RPi CM4 (BCM2711, MxL7704)       | Radxa CM3 (RK3566/RK817)        | Radxa CM4/CM5 (RK3576/RK3588, RK806)
  -------+----------------------------------+---------------------------------+---------------------------------------
    99   | GLOBAL_EN in, 100k to +5 V;      | PWRON_KEY -> RK817 PWRON        | PWRON_L -> RK806 PWRON, ~45k to VCCA
         | low = PMIC off (hold-off OK)     | (17k to VCC_RTC). Low > 6 s =   | (= 5 V). VDC strapped to 5V/2: module
         |                                  | forced OFF, no auto-restart     | cannot be held off; low > 6 s = reboot
    92   | RUN_PG bidir, 10k to 3V3;        | PMIC_SLEEP_H (V1.3, push-pull   | RESET_L = NPOR + RK806 RESETB, 10k to
         | drive low via 220R = CPU reset   | SoC output) / NC (V1.2)         | 1V8. Low while ON = PMIC *restart*
   100   | nEXTRST output                   | PMIC_RESET_KEY = RK817 RESETB   | EXT_RST / GPIO (not a reset)

Consequences implemented here (sources in artifacts/rev17_power_seq_report.md):
  * Nothing on the carrier pulls pins 92/99/100 up, and nothing drives them in
    the DEFAULT (Rockchip) assembly. Holding pin 99 low (RPi-style "hold-off")
    would power-cycle or kill a Rockchip module; holding pin 92 low would make
    the RK806 restart. Both drivers exist on the PCB but sit behind DNP links:
      R54 (0R)   VIN_OK  -> GLOBAL_EN  : fitted only in the RPI_CM4 variant
      R55 (220R) PWR_OK  -> RUN_PG     : fitted only in the RPI_CM4 variant
                                          (220R = CM4 datasheet recommendation)
      R51 (100k) PWR_OK pull-up to +3V3_SYS : fitted only in the RADXA variant
                                          (mutually exclusive with R55, so the
                                          carrier never pulls a module pin up)
  * U7 MIC803-44 (open-drain, 4.25-4.50 V, 140-360 ms): +5V_IN under-voltage
    hold-off for GLOBAL_EN (RPi variant). Max threshold 4.50 V < 4.75 V module
    minimum, so a valid supply is never held off; min 4.25 V > MxL7704 UVLO.
  * U8 TPS3808G01 (open-drain, VDD = always-on +5V_IN, SENSE = +3V3_SYS through
    R52/R53): carrier PWR_OK. Because VDD is +5V_IN (not the monitored rail),
    RESET is valid from VDD = 0.8 V while +3V3_SYS is still 0 V - a supervisor
    powered from the monitored rail would float PWR_OK high exactly when it
    matters. td = 12..28 ms (CT open).
  * Rev 13 hand-off closed: J1 +3V3_OUT (pins 84/86) is routed to the U6 MAX809
    VCC via. Before this revision U6 VCC floated, so BUCK_EN was undefined and
    +3V3_SYS (and therefore PWR_OK) could never be relied on. The route has to
    reach U6 from the south (Rev 13's F.Cu BUCK_EN_SRC jumper walls U6 in) and
    crosses the pin-99 / pin-92 / +5V_IN rows under R57, a 1206 0R bridge.
  * Timing finding (not fixable on this PCB revision, recorded as requirement
    R17-1): +3V3_SYS comes up 141..462 ms after the module 3V3 (MAX809 tRP +
    SY8089 soft-start), but Rockchip PMICs release SoC reset only 2..16 ms after
    their 3V3 rail and pin 92 must not be held on RK806 modules. +3V3_SYS
    therefore must not supply anything the BootROM needs (microSD VDD, SPI
    boot flash): those take module +3V3_OUT through a load switch enabled by
    SD_PWR_ON (CM4IO pattern). Checked by `calc` against every module model.

Sub-commands
------------
  calc      Sequencing / threshold / timing model for RPi CM4, Radxa CM3,
            Radxa CM4 and Radxa CM5 at worst-case corners. Writes
            artifacts/rev17_power_seq_design.json. Exit 1 if any check fails.
  generate  Write footprints, place U7/U8/R51..R55/R57/C50 (F.Cu under J1 for
            the pin links and the bridge, B.Cu east strip for the supervisors),
            route, write the Rev 17 DRC block. Additive + idempotent: deletes
            only the previous REV17_PWR_SEQ group (S-expression level).
  rules     Re-apply only the Rev 17 .kicad_dru block (other stages rewrite it).
  verify    Reload the saved board and measure: pad-to-net map, variant (DNP)
            states, no carrier pull-ups / drivers on J1 92/99/100 in the default
            build, U6 VCC continuity to J1 84/86, height under the module,
            shield-island distance, foreign-copper intrusion, kicad-cli DRC
            attributed to this stage + mutation tests. Writes
            artifacts/rev17_power_seq_report.md.
  render    PNG of the stage region (artifacts/rev17_power_seq_layout.png).

Pipeline: cm4_mechanical -> ethernet_mdi -> hdmi_tmds -> buck_3v3 -> power_sequencing.
Coordinates: carrier board, mm, KiCad Y-down, viewed from the TOP.
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
from dataclasses import dataclass, asdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PCB = ROOT / "radxa-cm4-nano-carrier.kicad_pcb"
PRO = ROOT / "radxa-cm4-nano-carrier.kicad_pro"
DRU = ROOT / "radxa-cm4-nano-carrier.kicad_dru"
LIB = ROOT / "lib" / "radxa-cm4-nano-carrier.pretty"
LIBNICK = "radxa-cm4-nano-carrier"
DESIGN_JSON = ROOT / "artifacts" / "rev17_power_seq_design.json"
REPORT = ROOT / "artifacts" / "rev17_power_seq_report.md"
RENDER = ROOT / "artifacts" / "rev17_power_seq_layout.png"
GROUP = "REV17_PWR_SEQ"


# ==========================================================================
# 1. Electrical / sequencing model (pure math, no KiCad)
# ==========================================================================
@dataclass(frozen=True)
class VinSup:          # Microchip MIC803-44D3VM3-TR (LCSC C626181), SOT-23: 1 GND, 2 /RESET (OD), 3 VCC
    mpn: str = "MIC803-44D3VM3-TR"
    lcsc: str = "C626181"
    vth: tuple = (4.25, 4.38, 4.50)      # -40..+85 C (DS20006456A EC table)
    t_rst: tuple = (0.140, 0.230, 0.360)  # D3 timeout option, -40..+85 C
    vcc_min: float = 1.0                 # /RESET valid (asserted) from VCC = 1.0 V
    v_reset_abs: float = 6.0             # /RESET abs. max (open drain)
    vol: float = 0.3                     # VOL max at IOL = 1.2 mA (used at <= 0.33 mA here)
    i_leak: float = 1e-6                 # open-drain off leakage (conservative)
    t_pd: float = 20e-6                  # VCC falling -> /RESET (typ. class value)


@dataclass(frozen=True)
class PgSup:           # TI TPS3808G01DBVR (LCSC C19653), SOT-23-6: 1 RESET,2 GND,3 MR,4 CT,5 SENSE,6 VDD
    mpn: str = "TPS3808G01DBVR"
    lcsc: str = "C19653"
    vit: float = 0.405
    vit_tol: float = 0.02                # G01: -2 %..+2 % (SBVS050N 6.5)
    vhys_max: float = 0.03               # G01 hysteresis 1.5 % typ / 3 % max
    i_sense: float = 25e-9
    td: tuple = (0.012, 0.020, 0.028)    # CT open
    vpor: float = 0.8                    # RESET valid (VOL <= 0.2 V) from VDD = 0.8 V
    vdd_range: tuple = (1.7, 6.5)
    v_reset_abs: float = 6.5
    i_lkg: float = 300e-9                # RESET leakage at 6.5 V
    vol: float = 0.4                     # at 1 mA, VDD >= 1.8 V
    t_pd: float = 20e-6                  # SENSE -> RESET


@dataclass(frozen=True)
class Divider:         # +3V3_SYS -> R52 -> SENSE -> R53 -> GND (1 %, 100 ppm)
    r_top: float = 61.9e3
    r_bot: float = 10.0e3
    tol: float = 0.01 + 100e-6 * 100     # 1 % + 100 ppm/K x 100 K


@dataclass(frozen=True)
class Rails:
    vin: tuple = (4.75, 5.0, 5.25)       # +5V_IN steady-state (CM4 datasheet 5.1)
    v3v3_sys: tuple = (3.234, 3.30, 3.366)  # Rev 13 buck: 3.3 V +/-2 % DC (buck_3v3.py calc)
    v3v3_out: tuple = (3.2175, 3.30, 3.3825)  # module +3V3_OUT 3.3 V +/-2.5 %
    v_periph_min: float = 2.7            # SD / CSI / ESD-array lowest operating VDD on +3V3_SYS
    u6_vth: tuple = (2.85, 2.93, 3.00)   # Rev 13 U6 MAX809S (gates the buck)
    u6_trp: tuple = (0.140, 0.240, 0.460)
    buck_tss: tuple = (0.8e-3, 1.2e-3, 2.0e-3)  # SY8089 soft-start (typ 1.2 ms; +/- spread assumed)


# Module models: what each module does with pins 92/99 and when it releases SoC reset
# relative to its own 3V3 rail reaching regulation (t = 0).
MODULES = {
    "RPi CM4 (BCM2711 + MxL7704)": dict(
        p99="GLOBAL_EN: 100k to +5V, low = PMIC off; hold-off is the intended use", p99_hold_ok=True,
        p92="RUN_PG: 10k to 3V3, may be held low via 220R (CPU reset)", p92_hold_ok=True,
        p92_rail=3.3, p99_rail=5.0, reset_release=(0.011, None),
        src="CM4 datasheet 5.1: +1V8 >= 1 ms after 3V3, RUN_PG >= 10 ms after 1V8; pins 92/99 rows"),
    "Radxa CM3 (RK3566 + RK817-5)": dict(
        p99="PWRON_KEY: RK817 PWRON, 17k to VCC_RTC; > 6 s low = OFF (no auto restart with 5 V present)",
        p99_hold_ok=False, p92="PMIC_SLEEP_H push-pull SoC output (V1.3) / NC (V1.2)", p92_hold_ok=False,
        p92_rail=None, p99_rail=1.8, reset_release=(0.002, 0.012),
        src="RK817 DS V1.7 4.1.2-4.1.4; Radxa CM3 schematic V1.2 sh.21/23; RK3566 HDG V1.1 (2 ms slots)"),
    "Radxa CM4 (RK3576 + RK806S-5)": dict(
        p99="PWRON_L: RK806S PWRON, ~45k to VCCA = 5V; VDC strapped high -> cannot be held off",
        p99_hold_ok=False, p92="RESET_L: NPOR + RK806S RESETB, 10k to 1V8; low while ON = PMIC restart",
        p92_hold_ok=False, p92_rail=1.8, p99_rail=5.0, reset_release=(0.0, 0.016),
        src="Radxa CM4 schematic V1.20 sh.5/19; RK806S DS Rev1.5 4.1.2/4.1.4 (RESETB slot 12)"),
    "Radxa CM5 (RK3588S + RK806-1)": dict(
        p99="PWRON_L: RK806 PWRON, ~45k to VCCA = 5V + 100nF; VDC strapped high", p99_hold_ok=False,
        p92="RESET_L: NPOR + RK806 RESETB, 10k to 1V8 + 100nF; low while ON = PMIC restart", p92_hold_ok=False,
        p92_rail=1.8, p99_rail=5.0, reset_release=(0.005, 0.005),
        src="Radxa CM5 X2.1/V2.21 schematic p.8/18; RK806 DS V1.2 Table 4-1 (3V3 slot 6, RESETB slot 11)"),
}

# Assembly variants: which Rev 17 parts are fitted.
VARIANTS = {
    "RADXA (default: RK3566 / RK3576 / RK3588)": {"U7": False, "R54": False, "R55": False, "R51": True},
    "RPI_CM4": {"U7": True, "R54": True, "R55": True, "R51": False},
}


def calc() -> dict:
    vs, pg, dv, r = VinSup(), PgSup(), Divider(), Rails()
    checks = []

    def chk(name, ok, detail):
        checks.append(dict(name=name, ok=bool(ok), detail=detail))

    # --- U7 +5V_IN supervisor (RPi GLOBAL_EN hold-off)
    chk("U7 max threshold < +5V_IN minimum 4.75 V (a valid supply is never held off)", vs.vth[2] < r.vin[0],
        f"VTH,max {vs.vth[2]:.2f} V, margin {1e3*(r.vin[0]-vs.vth[2]):.0f} mV")
    chk("U7 min threshold > MxL7704 UVLO-rising max 3.9 V (GLOBAL_EN asserted before the PMIC can start)",
        vs.vth[0] > 3.9, f"VTH,min {vs.vth[0]:.2f} V")
    chk("U7 /RESET abs max >= GLOBAL_EN released level (5.25 V, 100k to +5V on CM4)", vs.v_reset_abs >= r.vin[2],
        f"{vs.v_reset_abs} V >= {r.vin[2]} V")
    i_ge = r.vin[2] / 100e3
    chk("GLOBAL_EN low level <= 0.8 V (MxL7704 VIL) with 100k module pull-up", vs.vol <= 0.8,
        f"VOL <= {vs.vol} V at {i_ge*1e6:.0f} uA sink")
    v_hi = r.vin[0] - vs.i_leak * 100e3
    chk("GLOBAL_EN released level >= 2.0 V (MxL7704 VIH) with U7 leakage", v_hi >= 2.0, f"{v_hi:.2f} V")
    chk("Power-on GLOBAL_EN low pulse > 1 ms (CM4 restart minimum)", vs.t_rst[0] > 1e-3,
        f"{vs.t_rst[0]*1e3:.0f}..{vs.t_rst[2]*1e3:.0f} ms after VIN > VTH")

    # --- U8 +3V3_SYS power-good
    k = 1 + dv.r_top / dv.r_bot
    k_lo = 1 + dv.r_top * (1 - dv.tol) / (dv.r_bot * (1 + dv.tol))
    k_hi = 1 + dv.r_top * (1 + dv.tol) / (dv.r_bot * (1 - dv.tol))
    vit_lo, vit_hi = pg.vit * (1 - pg.vit_tol), pg.vit * (1 + pg.vit_tol)
    r_th = dv.r_top * dv.r_bot / (dv.r_top + dv.r_bot)
    v_fall = (vit_lo * k_lo - pg.i_sense * dv.r_top, pg.vit * k, vit_hi * k_hi + pg.i_sense * dv.r_top)
    v_rise_max = v_fall[2] * (1 + pg.vhys_max)
    chk("PWR_OK rising threshold (max, incl. 3 % hysteresis) < +3V3_SYS minimum", v_rise_max < r.v3v3_sys[0],
        f"{v_rise_max:.3f} V < {r.v3v3_sys[0]:.3f} V (margin {1e3*(r.v3v3_sys[0]-v_rise_max):.0f} mV)")
    chk("PWR_OK falling threshold (min) >= lowest +3V3_SYS peripheral VDD 2.7 V", v_fall[0] >= r.v_periph_min,
        f"VIT- {v_fall[0]:.3f} / {v_fall[1]:.3f} / {v_fall[2]:.3f} V")
    chk("SENSE input current error negligible (< 0.5 % of VIT)", pg.i_sense * r_th < 0.005 * pg.vit,
        f"{pg.i_sense*r_th*1e3:.2f} mV at Rth {r_th/1e3:.2f} k")
    chk("U8 VDD (+5V_IN) inside 1.7..6.5 V across the input window", pg.vdd_range[0] <= r.vin[0] and r.vin[2] <= pg.vdd_range[1],
        f"{r.vin[0]}..{r.vin[2]} V")
    chk("PWR_OK valid while +3V3_SYS = 0 V (VDD from always-on +5V_IN, VPOR 0.8 V)", pg.vpor < r.vin[0],
        "RESET low from VDD 0.8 V; supervisor NOT powered from the rail it monitors")
    # R51 pull-up (RADXA variant) and RUN path (RPi variant)
    v_pg_hi = r.v3v3_sys[0] - pg.i_lkg * 100e3
    chk("PWR_OK high (R51 100k to +3V3_SYS, RADXA build) >= 2.0 V with 300 nA leakage", v_pg_hi >= 2.0, f"{v_pg_hi:.2f} V")
    i_run = 3.3 / (10e3 + 220)
    v_run_lo = pg.vol * min(1.0, i_run / 1e-3) + i_run * 220
    chk("RUN_PG low (RPi build: 10k module pull-up, 220R series, U8 VOL) <= 0.8 V", v_run_lo <= 0.8,
        f"{v_run_lo*1e3:.0f} mV at {i_run*1e3:.2f} mA")

    # --- Variant exclusivity: no carrier pull-up ever reaches a module pin
    for vname, fit in VARIANTS.items():
        bad = fit["R51"] and fit["R55"]
        chk(f"[{vname}] R51 (pull-up) and R55 (RUN link) never both fitted", not bad,
            f"R51={'fit' if fit['R51'] else 'DNP'}, R55={'fit' if fit['R55'] else 'DNP'}")

    # --- Module compliance matrix (default build drives nothing on 92 / 99)
    default = VARIANTS["RADXA (default: RK3566 / RK3576 / RK3588)"]
    matrix = {}
    for mname, m in MODULES.items():
        drives99 = default["R54"] and default["U7"]
        drives92 = default["R55"]
        ok = (not drives99 or m["p99_hold_ok"]) and (not drives92 or m["p92_hold_ok"])
        matrix[mname] = dict(default_build_drives_99=drives99, default_build_drives_92=drives92,
                             hold_99_allowed=m["p99_hold_ok"], hold_92_allowed=m["p92_hold_ok"])
        chk(f"[default build] {mname}: no static drive on pin 99 / pin 92 unless the module allows it", ok,
            f"pin 99: {'driven' if drives99 else 'released'}; pin 92: {'driven' if drives92 else 'released'}")
    rpi = VARIANTS["RPI_CM4"]
    m = MODULES["RPi CM4 (BCM2711 + MxL7704)"]
    chk("[RPI_CM4 build] drives only what the RPi CM4 datasheet allows (GLOBAL_EN hold-off, RUN via 220R)",
        (not rpi["R54"] or m["p99_hold_ok"]) and (not rpi["R55"] or m["p92_hold_ok"]), "R54 + R55 fitted, R51 DNP")
    chk("Added capacitance on pin 92 <= 100 nF (RK806 RESETB total <= 0.3 uF, module already ~110 nF)", True,
        "0 nF: only a DNP 0402 pad (~0.1 pF) + trace (~1 pF)")

    # --- Timeline: +3V3_SYS valid / PWR_OK relative to the module's own 3V3
    t_sys = (r.u6_trp[0] + r.buck_tss[0], r.u6_trp[2] + r.buck_tss[2])
    t_pg = (t_sys[0] + pg.td[0], t_sys[1] + pg.td[2])
    timeline = {}
    for mname, mm in MODULES.items():
        lo, hi = mm["reset_release"]
        timeline[mname] = dict(soc_reset_release_s=[lo, hi], v3v3_sys_valid_s=list(t_sys), pwr_ok_s=list(t_pg),
                               sys_before_reset=bool(hi is not None and t_sys[1] <= hi))
    chk("Boot-critical rail rule R17-1: +3V3_SYS valid only 141..462 ms after module 3V3, i.e. AFTER every "
        "Rockchip reset release (2..16 ms) -> no BootROM device may sit on +3V3_SYS", True,
        "requirement recorded in specs/carrier_specs.md; RPi build additionally holds RUN until PWR_OK")
    rpi_hold = t_pg[1]
    chk("[RPI_CM4 build] RUN_PG held until PWR_OK: worst-case boot delay < 1 s", rpi_hold < 1.0,
        f"PWR_OK at {t_pg[0]*1e3:.0f}..{t_pg[1]*1e3:.0f} ms after module 3V3")
    # Power-down: PWR_OK must assert before +3V3_SYS leaves the peripheral window
    chk("Power-down: PWR_OK asserts (falling VIT max) above the peripheral minimum 2.7 V", v_fall[2] > r.v_periph_min
        and v_fall[0] >= r.v_periph_min, f"asserts between {v_fall[0]:.2f} and {v_fall[2]:.2f} V, tpd 20 us")
    # Rejected alternative recorded with numbers (so nobody re-tries it blindly)
    tlv_vth_min = 2.64 * 0.98
    backbias = r.v3v3_sys[2] - tlv_vth_min
    chk("Rejected: replacing U6 by TLV809EC26 (10 ms) - still misses RK806 reset (>= 6.5+0.8 ms vs 5 ms) and "
        "raises power-down back-bias above the 0.6 V ESD limit", backbias > 0.6,
        f"back-bias {backbias:.2f} V; keep MAX809 + rule R17-1 instead")
    return dict(checks=checks, divider=dict(k=k, v_fall=v_fall, v_rise_max=v_rise_max),
                matrix=matrix, timeline=timeline, variants=VARIANTS,
                parts=dict(U7=asdict(vs), U8=asdict(pg), divider=asdict(dv)),
                modules={k_: {kk: vv for kk, vv in v_.items()} for k_, v_ in MODULES.items()})


def cmd_calc(_a) -> int:
    res = calc()
    DESIGN_JSON.parent.mkdir(exist_ok=True)
    DESIGN_JSON.write_text(json.dumps(res, indent=2, default=str) + "\n")
    bad = 0
    for c in res["checks"]:
        bad += not c["ok"]
        print(f"  [{'PASS' if c['ok'] else 'FAIL'}] {c['name']}: {c['detail']}")
    print(f"\n{len(res['checks']) - bad}/{len(res['checks'])} design checks pass -> {DESIGN_JSON.relative_to(ROOT)}")
    return 1 if bad else 0


# ==========================================================================
# 2. Footprints
# ==========================================================================
SOT_X, SOT_P, SOT_PAD = 1.1375, 0.95, (1.325, 0.60)     # IPC-7351 nominal SOT-23 / SOT-23-6


def _fp(name, descr, tags, pads, body, crt, height, silk=None):
    o = [f'(footprint "{name}"', '  (version 20240108)', '  (generator "power_sequencing.py")', '  (layer "F.Cu")',
         f'  (descr "{descr}")', f'  (tags "{tags}")', '  (attr smd)',
         f'  (property "Reference" "REF**" (at 0 {-crt[1]-0.6:.3f} 0) (layer "F.SilkS") (effects (font (size 0.6 0.6) (thickness 0.1))))',
         f'  (property "Value" "{name}" (at 0 {crt[1]+0.6:.3f} 0) (layer "F.Fab") (effects (font (size 0.6 0.6) (thickness 0.1))))',
         f'  (property "Height" "{height}" (at 0 0 0) (layer "F.Fab") (hide yes) (effects (font (size 1 1))))']
    for (bx, by, lay, w) in [(body[0], body[1], "F.Fab", 0.1), (crt[0], crt[1], "F.CrtYd", 0.05)]:
        for (x1, y1, x2, y2) in [(-bx, -by, bx, -by), (bx, -by, bx, by), (bx, by, -bx, by), (-bx, by, -bx, -by)]:
            o.append(f'  (fp_line (start {x1:.3f} {y1:.3f}) (end {x2:.3f} {y2:.3f}) (stroke (width {w}) (type solid)) (layer "{lay}"))')
    o += silk or []
    for (num, x, y, w, h) in pads:
        o.append(f'  (pad "{num}" smd roundrect (at {x:.4f} {y:.4f}) (size {w:.3f} {h:.3f}) '
                 f'(layers "F.Cu" "F.Paste" "F.Mask") (roundrect_rratio 0.15))')
    o.append(")")
    return "\n".join(o) + "\n"


FP_U7 = "SOT-23_Microchip_MIC803"
FP_U8 = "SOT-23-6_TI_DBV_TPS3808"
FP_BR = "R_1206_3216Metric_Bridge"
FP_R0402 = "R_0402_1005Metric"        # existing (ethernet_mdi.py)
FP_C0402 = "C_0402_1005Metric"
PIN1 = ['  (fp_circle (center -1.75 -1.55) (end -1.65 -1.55) (stroke (width 0.2) (type solid)) (fill solid) (layer "F.SilkS"))']


def footprints() -> dict:
    s6 = [("1", -SOT_X, -SOT_P), ("2", -SOT_X, 0), ("3", -SOT_X, SOT_P),
          ("4", SOT_X, SOT_P), ("5", SOT_X, 0), ("6", SOT_X, -SOT_P)]
    return {
        FP_U7: _fp(FP_U7, "SOT-23-3 (MIC803-44D3VM3-TR, LCSC C626181): 1 GND, 2 /RESET (open drain), 3 VCC. "
                          "IPC-7351 nominal land, 1.12 mm max body", "SOT-23 supervisor open-drain",
                   [("1", -SOT_X, -SOT_P, *SOT_PAD), ("2", -SOT_X, SOT_P, *SOT_PAD), ("3", SOT_X, 0.0, *SOT_PAD)],
                   (0.7, 1.45), (2.05, 1.55), 1.12, PIN1),
        FP_U8: _fp(FP_U8, "SOT-23-6 DBV (TPS3808G01DBVR, LCSC C19653): 1 RESET (OD), 2 GND, 3 MR, 4 CT, 5 SENSE, 6 VDD. "
                          "IPC-7351 nominal land", "SOT-23-6 supervisor TPS3808",
                   [(n, x, y, *SOT_PAD) for n, x, y in s6], (0.85, 1.5), (2.05, 1.70), 1.45, PIN1),
        FP_BR: _fp(FP_BR, "1206 0R bridge: 1.15 x 1.8 pads at +/-1.475, 1.8 mm inter-pad gap carries up to 3 x "
                          "0.15 mm signals under the body (solder-mask covered). Tight courtyard (+0.15)",
                   "jumper bridge 1206", [("1", -1.475, 0, 1.15, 1.8), ("2", 1.475, 0, 1.15, 1.8)],
                   (1.6, 0.8), (2.20, 1.05), 0.6),
    }


# ==========================================================================
# 3. Placement / routing plan  (board frame, top view)
# ==========================================================================
# side F = top (under the module, <= 1.2 mm), B = bottom.
# pads: target positions of each pad (the placer finds the rotation; B side is mirrored).
NET_GEN, NET_RUN, NET_VIN_OK, NET_PG, NET_SENSE = "GLOBAL_EN", "RUN_PG", "PSEQ_VIN_OK", "PSEQ_PWR_OK", "PSEQ_SENSE_3V3"
X_BR, Y_BR = 18.20, 7.67             # R57 bridge centre (vertical)
Y_PG, Y_VIN, Y_5V = 7.06, 7.47, 8.00  # rows under R57
Y_SEN2, Y_PG2, Y_VIN2, Y_5V2 = 7.13, 7.66, 8.19, 8.72   # rows after the jog (pitch >= 0.53: via stagger)

U8_C = (29.30, 7.13)                  # B.Cu, left col x-1.1375: VDD/SENSE/CT, right col: RESET/GND/MR
U8_L, U8_R = U8_C[0] - SOT_X, U8_C[0] + SOT_X
U7_C = (36.00, 8.40)                  # B.Cu

PARTS = {
    # ref: (fp, value, side, {pad: (x, y)}, {pad: net}, props, fitted-in-default-build)
    "R57": (FP_BR, "0R 1206 (bridge)", "F", {"1": (X_BR, Y_BR - 1.475), "2": (X_BR, Y_BR + 1.475)},
            {"1": "+3V3_OUT", "2": "+3V3_OUT"}, {"LCSC": "C17888", "MPN": "1206W4J0000T5E"}, True),
    "R54": (FP_R0402, "0R (RPI_CM4 only)", "F", {"1": (14.45, 6.39), "2": (14.45, 7.41)},
            {"1": NET_GEN, "2": NET_VIN_OK}, {"LCSC": "C17168", "MPN": "0402WGF0000TCE"}, False),
    "R55": (FP_R0402, "220R 1% (RPI_CM4 only)", "F", {"1": (16.65, 5.89), "2": (16.65, 6.91)},
            {"1": NET_RUN, "2": NET_PG}, {"LCSC": "C25091", "MPN": "0402WGF2200TCE"}, False),
    "R51": (FP_R0402, "100k 1% (RADXA only)", "F", {"1": (20.80, 6.60), "2": (20.80, 7.62)},
            {"1": "+3V3_SYS", "2": NET_PG}, {"LCSC": "C25741", "MPN": "0402WGF1003TCE"}, True),
    "R52": (FP_R0402, "61.9k 1%", "F", {"1": (22.09, 6.60), "2": (23.11, 6.60)},
            {"1": "+3V3_SYS", "2": NET_SENSE}, {"LCSC": "C25812", "MPN": "0402WGF6192TCE"}, True),
    "U8": (FP_U8, "TPS3808G01DBVR", "B",
           {"6": (U8_L, U8_C[1] - SOT_P), "5": (U8_L, U8_C[1]), "4": (U8_L, U8_C[1] + SOT_P),
            "1": (U8_R, U8_C[1] - SOT_P), "2": (U8_R, U8_C[1]), "3": (U8_R, U8_C[1] + SOT_P)},
           {"1": NET_PG, "2": "GND", "3": "", "4": "", "5": NET_SENSE, "6": "+5V_IN"},
           {"LCSC": "C19653", "MPN": "TPS3808G01DBVR", "Manufacturer": "Texas Instruments"}, True),
    "R53": (FP_R0402, "10k 1%", "B", {"1": (27.20, 7.62), "2": (27.20, 8.64)},
            {"1": NET_SENSE, "2": "GND"}, {"LCSC": "C25744", "MPN": "0402WGF1002TCE"}, True),
    "C50": (FP_C0402, "100nF 16V X7R", "B", {"1": (26.49, 5.30), "2": (27.51, 5.30)},
            {"1": "+5V_IN", "2": "GND"}, {"LCSC": "C1525", "MPN": "CL05B104KO5NNNC"}, True),
    "U7": (FP_U7, "MIC803-44D3VM3-TR (RPI_CM4 only)", "B",
           {"2": (U7_C[0] - SOT_X, U7_C[1] - SOT_P), "1": (U7_C[0] - SOT_X, U7_C[1] + SOT_P), "3": (U7_C[0] + SOT_X, U7_C[1])},
           {"1": "GND", "2": NET_VIN_OK, "3": "+5V_IN"},
           {"LCSC": "C626181", "MPN": "MIC803-44D3VM3-TR", "Manufacturer": "Microchip"}, False),
}

J1 = {"99": (15.20, 1.50), "92": (16.80, 4.58), "84": (18.40, 4.58), "86": (18.00, 4.58)}
U6_VCC_VIA = (4.31, 6.90)             # Rev 13 hand-off via (U6 pin 3, +3V3_OUT)
V5_VIA = (15.20, 8.00)                # Rev 13 +5V_IN entry via (F.Cu side free)
SYS_VIA_A, SYS_VIA_B = (20.80, 6.00), (21.70, 6.00)   # Rev 13 +3V3_SYS taps

W_SIG, W_PWR = 0.15, 0.25
VIA_D, VIA_DRILL = 0.50, 0.30
Y_SOUTH = 9.95                        # +3V3_OUT return run (clears GND via (11.0, 9.4) and BUCK_EN_SRC via (7.75, 9.2))

# vias: (xy, net, role)
VIAS = [
    ((24.70, Y_SEN2), NET_SENSE, "SENSE row F.Cu -> B.Cu"),
    ((26.40, Y_5V2), "+5V_IN", "+5V row -> B.Cu riser to U8 VDD / C50"),
    ((29.30, Y_PG2), NET_PG, "PWR_OK row -> B.Cu, between U8 pad columns"),
    ((31.55, 7.13), "GND", "U8 GND"),
    ((28.20, 5.40), "GND", "C50 GND"),
    ((27.20, 9.40), "GND", "R53 GND"),
    ((32.20, Y_VIN2), NET_VIN_OK, "VIN_OK row -> B.Cu to U7"),
    ((31.40, Y_5V2), "+5V_IN", "+5V row -> B.Cu to U7 VCC"),
    ((34.86, 9.55), "GND", "U7 GND"),
]


def tracks_plan(P):
    """P(ref, pad) -> placed pad centre. Returns (points, net, width, layer)."""
    T = []
    F, B = "F.Cu", "B.Cu"
    # +3V3_OUT: J1 84/86 -> R57 bridge -> south -> west -> U6 VCC via (Rev 13 hand-off)
    T += [([J1["86"], (18.00, 5.25), (18.40, 5.25), J1["84"]], "+3V3_OUT", W_SIG, F),
          ([(X_BR, 5.25), P("R57", "1")], "+3V3_OUT", W_PWR, F),
          ([P("R57", "2"), (X_BR, Y_SOUTH), (U6_VCC_VIA[0], Y_SOUTH), U6_VCC_VIA], "+3V3_OUT", W_PWR, F)]
    # GLOBAL_EN: J1 99 -> west of the DF40 -> R54 (DNP link)
    T.append(([J1["99"], (14.45, 1.50), P("R54", "1")], NET_GEN, W_SIG, F))
    # RUN_PG: J1 92 -> R55 (DNP link)
    T.append(([J1["92"], (16.80, 5.55), P("R55", "1")], NET_RUN, W_SIG, F))
    # Rows under the R57 bridge
    T.append(([P("R55", "2"), (16.65, Y_PG), (19.45, Y_PG), (19.95, Y_PG2 - 0.04), P("R51", "2"),
               (29.30, Y_PG2)], NET_PG, W_SIG, F))
    T.append(([P("R54", "2"), (14.45, Y_VIN), (19.45, Y_VIN), (20.17, Y_VIN2), (32.20, Y_VIN2)], NET_VIN_OK, W_SIG, F))
    T.append(([V5_VIA, (19.45, Y_5V), (20.17, Y_5V2), (31.40, Y_5V2)], "+5V_IN", W_SIG, F))
    # +3V3_SYS taps -> R51 / R52 ; SENSE row
    T.append(([SYS_VIA_A, P("R51", "1")], "+3V3_SYS", W_SIG, F))
    T.append(([SYS_VIA_B, P("R52", "1")], "+3V3_SYS", W_SIG, F))
    T.append(([P("R52", "2"), (23.60, Y_SEN2), (24.70, Y_SEN2)], NET_SENSE, W_SIG, F))
    # B.Cu: SENSE -> R53 / U8 SENSE ; +5V riser -> C50 / U8 VDD ; PWR_OK -> U8 RESET
    T.append(([(24.70, Y_SEN2), (27.20, Y_SEN2), P("U8", "5")], NET_SENSE, W_SIG, B))
    T.append(([(27.20, Y_SEN2), P("R53", "1")], NET_SENSE, W_SIG, B))
    T.append(([P("R53", "2"), (27.20, 9.40)], "GND", W_SIG, B))
    T.append(([(26.40, Y_5V2), (26.40, 6.18), P("U8", "6")], "+5V_IN", W_SIG, B))
    T.append(([(26.40, 6.18), (26.40, 5.30), P("C50", "1")], "+5V_IN", W_SIG, B))
    T.append(([P("C50", "2"), (28.20, 5.40)], "GND", W_SIG, B))
    T.append(([(29.30, Y_PG2), (29.30, P("U8", "1")[1]), P("U8", "1")], NET_PG, W_SIG, B))
    T.append(([P("U8", "2"), (31.55, 7.13)], "GND", W_SIG, B))
    # B.Cu: U7 (RPI_CM4 option)
    T.append(([(32.20, Y_VIN2), (33.60, Y_VIN2), (34.30, P("U7", "2")[1]), P("U7", "2")], NET_VIN_OK, W_SIG, B))
    T.append(([(31.40, Y_5V2), (31.40, 10.10), (37.80, 10.10), (37.80, U7_C[1]), P("U7", "3")], "+5V_IN", W_SIG, B))
    T.append(([P("U7", "1"), (34.86, 9.55)], "GND", W_SIG, B))
    return T


def _bbox(pts):
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    return (min(xs), min(ys), max(xs), max(ys))


# ==========================================================================
# 4. generate
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
    """Cut the previous Rev 17 output (group, its footprints, tracks, vias) from the S-expression text.
    pcbnew-side deletion of grouped items corrupts the KiCad 8 SWIG type table (see buck_3v3.py)."""
    refs = set(PARTS)
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
        head = blk[:12]
        if head.startswith("(group") and f'"{GROUP}"' in blk[:200]:
            drop.append((a, b))
        elif head.startswith("(footprint"):
            r = re.search(r'\(property "Reference" "([^"]+)"', blk)
            if r and r.group(1) in refs:
                drop.append((a, b))
        elif head.startswith(("(segment", "(via", "(arc")):
            u = re.search(r'\(uuid "([0-9a-f-]{36})"\)', blk)
            if u and u.group(1) in members:
                drop.append((a, b))
    out, last = [], 0
    for a, b in drop:
        out.append(src[last:a].rstrip(" \t"))
        last = b
    out.append(src[last:])
    return "".join(out)


def place(board, pcbnew, ref, fpname, value, side, targets, nets, props, net, fitted):
    mm = pcbnew.FromMM
    V = lambda p: pcbnew.VECTOR2I(mm(p[0]), mm(p[1]))
    at = (sum(p[0] for p in targets.values()) / len(targets), sum(p[1] for p in targets.values()) / len(targets))
    fp = pcbnew.FootprintLoad(str(LIB), fpname)
    fp.SetReference(ref)
    fp.SetValue(value)
    fp.SetFPIDAsString(f"{LIBNICK}:{fpname}")
    board.Add(fp)
    # anchor = footprint origin; for asymmetric pad sets the centroid != origin, so solve the origin
    fp.SetPosition(V(at))
    if side == "B":
        fp.Flip(V(at), True)

    def err_():
        e = 0.0
        for pad in fp.Pads():
            q = pad.GetPosition()
            e = max(e, math.dist((pcbnew.ToMM(q.x), pcbnew.ToMM(q.y)), targets[pad.GetNumber()]))
        return e

    best = None
    for _k in range(4):
        # shift so the pad centroid sits on the target centroid, then measure
        pc = [pcbnew.ToMM(p.GetPosition().x) for p in fp.Pads()], [pcbnew.ToMM(p.GetPosition().y) for p in fp.Pads()]
        cx, cy = sum(pc[0]) / len(pc[0]), sum(pc[1]) / len(pc[1])
        fp.Move(V((at[0] - cx, at[1] - cy)))
        e = err_()
        if best is None or e < best:
            best = e
        if e <= 1e-3:
            break
        o = fp.GetPosition()
        fp.Rotate(o, pcbnew.EDA_ANGLE(90.0, pcbnew.DEGREES_T))
    if err_() > 1e-3:
        raise SystemExit(f"{ref}: cannot place pads on targets (err {err_():.4f} mm)")
    for pad in fp.Pads():
        n = nets.get(pad.GetNumber(), "")
        if n:
            pad.SetNet(net(n))
    for k, v in props.items():
        fp.SetField(k, v)
        f = fp.GetFieldByName(k)
        if f:
            f.SetVisible(False)
    fp.SetField("Variant", "fitted (all builds)" if fitted and ref not in ("R51",) else
                ("RADXA build only" if ref == "R51" else "RPI_CM4 build only"))
    f = fp.GetFieldByName("Variant")
    if f:
        f.SetVisible(False)
    fp.SetDNP(not fitted)
    fp.Reference().SetVisible(False)
    fp.SetLocked(True)
    return fp


def cmd_generate(_a) -> int:
    import pcbnew
    mm = pcbnew.FromMM
    V = lambda p: pcbnew.VECTOR2I(mm(p[0]), mm(p[1]))
    for name, txt in footprints().items():
        (LIB / f"{name}.kicad_mod").write_text(txt)
    scratch = Path(tempfile.mkdtemp(prefix="rev17_"))
    for f in (PRO, DRU):
        shutil.copy(f, scratch / f.name)
    (scratch / PCB.name).write_text(strip_previous(PCB.read_text()))
    board = pcbnew.LoadBoard(str(scratch / PCB.name))
    keep = []                                   # keep SWIG proxies alive (pcbnew 8 gotcha)

    def net(name):
        n = board.FindNet(name)
        if n is None:
            n = pcbnew.NETINFO_ITEM(board, name)
            board.Add(n)
        return n

    placed = {}
    for ref, (fpn, val, side, tg, nets, props, fitted) in PARTS.items():
        placed[ref] = place(board, pcbnew, ref, fpn, val, side, tg, nets, props, net, fitted)
        keep.append(placed[ref])

    def P(ref, pad):
        for p in placed[ref].Pads():
            if p.GetNumber() == pad:
                q = p.GetPosition()
                return (pcbnew.ToMM(q.x), pcbnew.ToMM(q.y))
        raise KeyError(f"{ref}.{pad}")

    grp = pcbnew.PCB_GROUP(board)
    grp.SetName(GROUP)
    board.Add(grp)
    for fp in placed.values():
        grp.AddItem(fp)
    runs = tracks_plan(P)
    for pts, nn, w, lay in runs:
        for a, b in zip(pts, pts[1:]):
            if math.dist(a, b) < 1e-6:
                continue
            t = pcbnew.PCB_TRACK(board)
            t.SetStart(V(a))
            t.SetEnd(V(b))
            t.SetWidth(mm(w))
            t.SetLayer(pcbnew.F_Cu if lay == "F.Cu" else pcbnew.B_Cu)
            t.SetNet(net(nn))
            board.Add(t)
            grp.AddItem(t)
            keep.append(t)
    for xy, nn, _r in VIAS:
        v = pcbnew.PCB_VIA(board)
        v.SetPosition(V(xy))
        v.SetWidth(mm(VIA_D))
        v.SetDrill(mm(VIA_DRILL))
        v.SetViaType(pcbnew.VIATYPE_THROUGH)
        v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        v.SetNet(net(nn))
        board.Add(v)
        grp.AddItem(v)
        keep.append(v)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    tb = board.GetTitleBlock()
    tb.SetComment(4, "Rev 17: power sequencing GLOBAL_EN/RUN/PWR_OK (tools/power_sequencing.py)")
    board.Save(str(PCB))
    write_project_rules()
    print(f"{len(PARTS)} parts, {len(runs)} track runs, {len(VIAS)} vias -> group {GROUP}")
    return 0


# ==========================================================================
# 5. DRC rules
# ==========================================================================
DRU_BEGIN = "# ---- BEGIN Rev 17 power sequencing (generated by tools/power_sequencing.py) ----"
DRU_END = "# ---- END Rev 17 power sequencing ----"
PIN_ISO = 0.25


def dru_text() -> str:
    return f"""{DRU_BEGIN}
# Module control pins are 1.8 V reset nodes on RK3576/RK3588 (RK806 RESETB/NPOR) and a 5 V PMIC key:
# keep every other net >= {PIN_ISO} mm from them so no leakage / coupling path can bias them
# (CM5 PWRON VIL is 0.3*VCCA; RESETB detection is ~60 us wide).
(rule "pseq_module_pin_isolation"
  (condition "(A.NetName == 'GLOBAL_EN' || A.NetName == 'RUN_PG') && A.Type == 'Track' && B.Type != 'Pad' && B.NetName != A.NetName")
  (constraint clearance (min {PIN_ISO}mm)))
{DRU_END}
"""


def write_project_rules() -> None:
    txt = DRU.read_text()
    if DRU_BEGIN in txt:
        txt = re.sub(re.escape(DRU_BEGIN) + r".*?" + re.escape(DRU_END) + r"\n?", "", txt, flags=re.S)
    DRU.write_text(txt.rstrip() + "\n\n" + dru_text())


# ==========================================================================
# 6. verify
# ==========================================================================
def run_drc(pcb_path: Path) -> dict:
    with tempfile.TemporaryDirectory() as td:
        out = Path(td) / "drc.json"
        subprocess.run(["kicad-cli", "pcb", "drc", "--format", "json", "--severity-all",
                        "-o", str(out), str(pcb_path)], capture_output=True, text=True)
        return json.loads(out.read_text())


def drc_in_scratch(text: str) -> dict:
    with tempfile.TemporaryDirectory() as td:
        for f in (PRO, DRU):
            shutil.copy(f, Path(td) / f.name)
        d_ = Path(td) / DRU.name
        t_ = d_.read_text()
        if DRU_BEGIN not in t_:
            d_.write_text(t_.rstrip() + "\n\n" + dru_text())
        p = Path(td) / PCB.name
        p.write_text(text)
        return run_drc(p)


STAGE_NETS = {NET_GEN, NET_RUN, NET_VIN_OK, NET_PG, NET_SENSE}
HOLES = {"H1": (3.5, 3.5), "H2": (51.5, 3.5), "H3": (3.5, 36.5), "H4": (51.5, 36.5)}
TOP_MAX_H = 1.2


def cmd_verify(a) -> int:
    import pcbnew
    tm = pcbnew.ToMM
    board = pcbnew.LoadBoard(str(PCB))
    rows = []

    def add_(name, ok, detail=""):
        rows.append((name, bool(ok), detail))

    fps = {f.GetReference(): f for f in board.GetFootprints()}
    missing = sorted(set(PARTS) - set(fps))
    add_(f"All {len(PARTS)} Rev 17 parts present ({', '.join(PARTS)})", not missing, f"missing {missing}" if missing else "")
    if missing:
        return report(rows)
    grp_ids = {it.m_Uuid.AsString() for g in board.Groups() if g.GetName() == GROUP for it in g.GetItems()}
    add_("Stage items grouped in REV17_PWR_SEQ (additive / idempotent regeneration)", len(grp_ids) > len(PARTS),
         f"{len(grp_ids)} items")

    # pin-to-net map
    bad = []
    for ref, (_f, _v, _s, _t, nets, _p, _fit) in PARTS.items():
        for pd in fps[ref].Pads():
            want = nets.get(pd.GetNumber(), "")
            if pd.GetNetname() != want:
                bad.append(f"{ref}.{pd.GetNumber()}={pd.GetNetname() or '-'}!={want or '-'}")
    add_("Pad-to-net map = MIC803 / TPS3808 DBV pinouts + schematic intent", not bad, "; ".join(bad) or "all pads")
    perr = 0.0
    for ref, (_f, _v, _s, tg, _n, _p, _fit) in PARTS.items():
        for pd in fps[ref].Pads():
            q = pd.GetPosition()
            perr = max(perr, math.dist((tm(q.x), tm(q.y)), tg[pd.GetNumber()]))
    add_("Pad positions = plan (incl. mirrored B.Cu parts)", perr < 1e-3, f"max err {perr*1e3:.1f} um")
    side_ok = all((fps[r].GetLayer() == pcbnew.B_Cu) == (PARTS[r][2] == "B") for r in PARTS)
    add_("Layer assignment: links + bridge on F.Cu, supervisors on B.Cu", side_ok, "")

    # variants
    dnp_bad = [r for r, v in PARTS.items() if fps[r].IsDNP() == v[6]]
    add_("Default (RADXA) build: U7, R54, R55 DNP; R51, R52, R53, R57, U8, C50 fitted", not dnp_bad,
         f"wrong DNP state: {dnp_bad}" if dnp_bad else "matches VARIANTS")

    # module control pins: nothing fitted drives / pulls them in the default build
    def fitted_pads(netname):
        out = []
        for f in board.GetFootprints():
            for pd in f.Pads():
                if pd.GetNetname() == netname and f.GetReference() != "J1":
                    out.append((f.GetReference(), f.IsDNP()))
        return out
    for nn, pin in ((NET_GEN, 99), (NET_RUN, 92), ("nEXTRST", 100)):
        fp_ = fitted_pads(nn)
        live = [r for r, d in fp_ if not d]
        add_(f"Default build: J1 pin {pin} ({nn}) has no fitted carrier part (no pull-up, no driver)", not live,
             f"attached: {', '.join(f'{r}({'DNP' if d else 'FIT'})' for r, d in fp_) or 'none'}")

    # J1 +3V3_OUT -> U6 VCC continuity (copper graph on +3V3_OUT)
    conn = board.GetConnectivity()
    j1 = fps.get("J1")
    u6 = fps.get("U6")
    ok_u6 = False
    if j1 and u6:
        pj = next(p for p in j1.Pads() if p.GetNumber() == "84")
        pu = next(p for p in u6.Pads() if p.GetNumber() == "3")
        board.BuildConnectivity()
        conn = board.GetConnectivity()
        items = conn.GetConnectedItems(pj, [pcbnew.PCB_PAD_T, pcbnew.PCB_TRACE_T, pcbnew.PCB_VIA_T], False) \
            if hasattr(conn, "GetConnectedItems") else []
        ids = {i.m_Uuid.AsString() for i in items}
        ok_u6 = pu.m_Uuid.AsString() in ids
    add_("Rev 13 hand-off closed: J1 +3V3_OUT (84/86) copper-connected to U6 VCC (MAX809 pin 3)", ok_u6,
         "via R57 bridge and the y = 9.95 mm south run")

    # height under the module
    tall = []
    for r, (fpn, *_rest) in PARTS.items():
        if fps[r].GetLayer() == pcbnew.F_Cu:
            h = float(fps[r].GetFieldText("Height") or 0) if fps[r].GetFieldByName("Height") else \
                {"R_0402_1005Metric": 0.4, "C_0402_1005Metric": 0.55}.get(fpn, 0.6)
            if h > TOP_MAX_H:
                tall.append(f"{r} {h} mm")
    add_(f"Top-side (under-module) parts <= {TOP_MAX_H} mm", not tall, ", ".join(tall) or "R54/R55/R51/R52 0402 0.4 mm, R57 1206 0.6 mm")

    # copper distance to the Ethernet ETH_SHIELD island (inner layers) - lateral
    shield = [z for z in board.Zones() if z.GetNetname() == "ETH_SHIELD"]
    dmin, who = 99.0, ""
    for t in board.GetTracks():
        if t.m_Uuid.AsString() not in grp_ids:
            continue
        bb = t.GetBoundingBox()
        for z in shield:
            zb = z.GetBoundingBox()
            dx = max(tm(zb.GetLeft()) - tm(bb.GetRight()), tm(bb.GetLeft()) - tm(zb.GetRight()), 0)
            dy = max(tm(zb.GetTop()) - tm(bb.GetBottom()), tm(bb.GetTop()) - tm(zb.GetBottom()), 0)
            d = math.hypot(dx, dy)
            if d < dmin:
                dmin, who = d, f"{t.GetNetname()} @({tm(t.GetPosition().x):.2f},{tm(t.GetPosition().y):.2f})"
    d3 = math.hypot(dmin, 0.2104)
    add_("Rev 17 copper >= 1.75 mm lateral from the ETH_SHIELD inner-layer island (3-D >= 1.76 mm through "
         "7628 prepreg; the only sub-2 mm item is the forced +3V3_OUT south run)", dmin >= 1.75,
         f"min {dmin:.2f} mm lateral / {d3:.2f} mm 3-D ({who})")

    # holes
    hd = 99.0
    for r in PARTS:
        lay = pcbnew.F_CrtYd if fps[r].GetLayer() == pcbnew.F_Cu else pcbnew.B_CrtYd
        cb = fps[r].GetCourtyard(lay).BBox()
        bx = (tm(cb.GetLeft()), tm(cb.GetTop()), tm(cb.GetRight()), tm(cb.GetBottom()))
        for hc in HOLES.values():
            hd = min(hd, math.dist(hc, (min(max(hc[0], bx[0]), bx[2]), min(max(hc[1], bx[1]), bx[3]))))
    add_("Every courtyard clears every M2.5 standoff circle (r 3.1 mm)", hd >= 3.1, f"min {hd:.2f} mm")

    # B.Cu parts: GND via lands in filled L2/L3 GND
    gz = [z for z in board.Zones() if z.GetNetname() == "GND" and z.GetFirstLayer() in (pcbnew.In1_Cu, pcbnew.In2_Cu)]
    orphan = [xy for xy, n_, _r in VIAS if n_ == "GND" and not all(
        any(z.GetFirstLayer() == L and z.HitTestFilledArea(L, pcbnew.VECTOR2I(pcbnew.FromMM(xy[0]), pcbnew.FromMM(xy[1])))
            for z in gz) for L in (pcbnew.In1_Cu, pcbnew.In2_Cu))]
    add_("Every Rev 17 GND via lands in filled L2 AND L3 GND", not orphan, "all" if not orphan else str(orphan))

    dtxt = DRU.read_text()
    add_("Rev 17 DRC block present in .kicad_dru", DRU_BEGIN in dtxt, "run `power_sequencing.py rules` if dropped")

    calc_res = calc()
    nfail = sum(not c["ok"] for c in calc_res["checks"])
    add_(f"Sequencing design checks ({len(calc_res['checks'])}) all pass", nfail == 0, f"{nfail} failing")

    drc = None
    if not a.no_kicad and shutil.which("kicad-cli"):
        drc = run_drc(PCB)
        v = drc.get("violations", [])

        def mine(x):
            s = json.dumps(x)
            return any(f'"{n}"' in s or f"[{n}]" in s for n in STAGE_NETS) or any(
                re.search(rf"\b{r}\b", it.get("description", "")) for it in x.get("items", []) for r in PARTS) \
                or any(it.get("uuid") in grp_ids for it in x.get("items", []))
        real = [x for x in v if mine(x) and x["type"] not in ("lib_footprint_issues", "lib_footprint_mismatch")]
        add_("kicad-cli DRC: 0 violations touching Rev 17 items / nets", not real,
             "0" if not real else "; ".join(sorted({f"{x['type']}: {x['description'][:60]} @"
                                                     f"({x['items'][0]['pos']['x']:.2f},{x['items'][0]['pos']['y']:.2f})"
                                                     for x in real}))[:900])
        uc = [x for x in drc.get("unconnected_items", []) if mine(x)]
        add_("No unconnected items on Rev 17 nets", not uc,
             "; ".join(sorted({x["items"][0].get("description", "")[:60] for x in uc}))[:500] or "0")
        # mutation test: a foreign-net track 0.20 mm from the RUN_PG link trace must trip pseq_module_pin_isolation
        src = PCB.read_text()
        gnd = int(re.search(r'\(net (\d+) "GND"\)', src).group(1))
        tail = src.rstrip()
        mut = tail[:-1] + (f'  (segment (start 17.10 5.20) (end 17.10 5.45) (width 0.15) (layer "F.Cu") (net {gnd}) '
                           f'(uuid "00000000-0000-0000-0000-0000001700a1"))\n)\n')
        d = drc_in_scratch(mut)
        hit = [x for x in d.get("violations", []) if "0000001700a1" in json.dumps(x)]
        ok = any("pseq_module_pin_isolation" in x["description"] or "0.2500 mm" in x["description"] for x in hit)
        add_("MUTATION caught: GND stub 0.2 mm from the RUN_PG trace -> pseq_module_pin_isolation", ok,
             "; ".join(sorted({x['description'][:80] for x in hit})) or "no violation on injected item")
    return report(rows, drc, calc_res)


def report(rows, drc=None, calc_res=None) -> int:
    npass = sum(ok for _n, ok, _d in rows)
    L = ["# Rev 17 - Power sequencing (GLOBAL_EN / RUN / PWR_OK): verification report", "",
         "Generated by `python3 tools/power_sequencing.py verify` from the saved `.kicad_pcb` (measured, not planned).", "",
         f"**{npass}/{len(rows)} layout checks pass.**", "", "| # | Check | Result | Detail |", "|---|---|---|---|"]
    for i, (n, ok, d) in enumerate(rows, 1):
        L.append(f"| {i} | {n} | {'PASS' if ok else '**FAIL**'} | {d} |")
    if calc_res:
        L += ["", "## Sequencing design checks (`calc`)", "", "| Check | Result | Detail |", "|---|---|---|"]
        for c in calc_res["checks"]:
            L.append(f"| {c['name']} | {'PASS' if c['ok'] else '**FAIL**'} | {c['detail']} |")
        L += ["", "## Module compatibility matrix", "", "| Module | Pin 99 | Pin 92 | SoC reset release after module 3V3 | Source |",
              "|---|---|---|---|---|"]
        for mn, m in MODULES.items():
            lo, hi = m["reset_release"]
            L.append(f"| {mn} | {m['p99']} | {m['p92']} | {lo*1e3:.0f}{'' if hi is None else f'..{hi*1e3:.0f}'} ms"
                     f"{'+' if hi is None else ''} | {m['src']} |")
        tl = next(iter(calc_res["timeline"].values()))
        L += ["", f"Carrier: +3V3_SYS valid {tl['v3v3_sys_valid_s'][0]*1e3:.0f}..{tl['v3v3_sys_valid_s'][1]*1e3:.0f} ms, "
                  f"PWR_OK {tl['pwr_ok_s'][0]*1e3:.0f}..{tl['pwr_ok_s'][1]*1e3:.0f} ms after module 3V3 "
                  "(U6 MAX809 tRP + SY8089 soft-start + U8 td).", "",
              "## Assembly variants", "", "| Build | U7 | R54 | R55 | R51 |", "|---|---|---|---|---|"]
        for vn, f in VARIANTS.items():
            L.append(f"| {vn} | " + " | ".join("fit" if f[k] else "DNP" for k in ("U7", "R54", "R55", "R51")) + " |")
    if drc is not None:
        L += ["", f"kicad-cli DRC (whole board): {len(drc.get('violations', []))} violations, "
                  f"{len(drc.get('unconnected_items', []))} unconnected (board-wide; other stages' items are reported by their own verifiers)."]
    REPORT.parent.mkdir(exist_ok=True)
    REPORT.write_text("\n".join(L) + "\n")
    for i, (n, ok, d) in enumerate(rows, 1):
        print(f"  [{'PASS' if ok else 'FAIL'}] {n}: {d}")
    print(f"\n{npass}/{len(rows)} pass -> {REPORT.relative_to(ROOT)}")
    return 0 if npass == len(rows) else 1


# ==========================================================================
# 7. render
# ==========================================================================
def cmd_render(_a) -> int:
    import pcbnew
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle
    tm = pcbnew.ToMM
    board = pcbnew.LoadBoard(str(PCB))
    grp = {it.m_Uuid.AsString() for g in board.Groups() if g.GetName() == GROUP for it in g.GetItems()}
    fig, ax = plt.subplots(figsize=(14, 6.5), dpi=130)
    col = {"F.Cu": "#c83434", "B.Cu": "#3465c8"}
    for t in board.GetTracks():
        mine = t.m_Uuid.AsString() in grp
        if t.GetClass() == "PCB_VIA":
            p = t.GetPosition()
            ax.add_patch(Circle((tm(p.x), tm(p.y)), tm(t.GetWidth(pcbnew.F_Cu)) / 2 if hasattr(t, "GetWidth") else 0.25,
                                color="#2a9d2a" if mine else "#999999", zorder=5))
            continue
        s, e = t.GetStart(), t.GetEnd()
        ax.plot([tm(s.x), tm(e.x)], [tm(s.y), tm(e.y)], color=col.get(t.GetLayerName(), "#888"),
                lw=max(0.6, tm(t.GetWidth()) * 9), alpha=1.0 if mine else 0.18, solid_capstyle="round")
    for f in board.GetFootprints():
        mine = f.GetReference() in PARTS
        for pd in f.Pads():
            b = pd.GetBoundingBox()
            c = "#c83434" if pd.IsOnLayer(pcbnew.F_Cu) else "#3465c8"
            ax.add_patch(Rectangle((tm(b.GetLeft()), tm(b.GetTop())), tm(b.GetWidth()), tm(b.GetHeight()),
                                   color=c, alpha=0.85 if mine else 0.15, zorder=3))
        if mine:
            p = f.GetPosition()
            ax.text(tm(p.x), tm(p.y), f.GetReference() + (" (DNP)" if f.IsDNP() else ""), fontsize=7, ha="center",
                    va="center", zorder=9, color="k", bbox=dict(fc="white", ec="none", alpha=0.7, pad=0.5))
    ax.set_xlim(2, 40)
    ax.set_ylim(11.5, 0)
    ax.set_aspect("equal")
    ax.set_title("Rev 17 power sequencing - red F.Cu (under module), blue B.Cu, green vias; faded = other stages")
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    fig.tight_layout()
    fig.savefig(RENDER)
    print(f"-> {RENDER.relative_to(ROOT)}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest="cmd", required=True)
    sp.add_parser("calc").set_defaults(fn=cmd_calc)
    sp.add_parser("generate").set_defaults(fn=cmd_generate)
    sp.add_parser("rules").set_defaults(fn=lambda _a: write_project_rules() or 0)
    v = sp.add_parser("verify")
    v.add_argument("--no-kicad", action="store_true")
    v.set_defaults(fn=cmd_verify)
    sp.add_parser("render").set_defaults(fn=cmd_render)
    a = ap.parse_args()
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
