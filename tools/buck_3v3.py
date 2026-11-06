#!/usr/bin/env python3
"""
Radxa CM4 Nano Carrier - Rev 13: +3V3_SYS synchronous buck (Silergy SY8089AAAC)
================================================================================

Carrier-side 3.3 V system rail (SD card, CSI camera, PCIe/FPC aux, LEDs, ESD
arrays) generated from +5V_IN by a Silergy SY8089AAAC (LCSC C78988, SOT23-5,
2 A continuous / 3 A peak, internal compensation, hic-cup OCP).

Sub-commands
------------
  calc      Design equations only (no KiCad needed): inductor selection and
            ripple across the full input / tolerance corner set, Cin RMS and
            input ripple, Cout ripple, load-step deviation, feedback divider
            tolerance stack, feed-forward zero, loss + junction-temperature
            budget and the thermal derating table. Writes
            artifacts/rev13_buck_3v3_design.json. Exit 1 if any check fails.
  generate  Write the footprints, place U5/U6/L1/C40..C44/R40..R43 on B.Cu under the
            west half of J1 (board x 3.3-25, y 0.3-10 mm), route the power stage
            (input hot loop, LX, VOUT, Kelvin sense, FB, EN), pour the local
            B.Cu GND island and (if no stage has done so yet) the L2/L3 GND
            planes, write the BUCK_POWER net class + Rev 13 DRC block. Idempotent:
            deletes the previous REV13_BUCK_3V3 group first.
  rules     Re-apply only the BUCK_POWER net class (.kicad_pro) and the Rev 13
            block in .kicad_dru. Other stages rewrite both files; verify fails
            (mutation tests) if the block has been dropped.
  verify    Reload the saved board and measure what the fab gets: pin-to-net
            map, cap-to-pin distances, hot-loop polygon area and estimated
            loop inductance, LX copper area, FB/LX separation, courtyard and
            keep-out envelope vs. the Ethernet (T1) / HDMI (J4) / H2 reservations,
            then kicad-cli DRC with mutation tests. Writes
            artifacts/rev13_buck_3v3_report.md. Exit 1 on failure.

Pipeline:  cm4_mechanical.py generate -> ethernet_mdi.py generate ->
           hdmi_tmds.py generate -> buck_3v3.py generate  (this stage is additive).

Primary sources
---------------
[D1] Silergy AN_SY8089/SY8089A Rev 0.9A (LCSC C78988 datasheet):
     fsw 1 MHz (typ, IOUT = 500 mA) - NOT 1.5 MHz as written in specs/carrier_specs.md;
     RDS(on) P/N 110/80 mOhm typ; ILIM(P) >= 3.5 A; VREF 0.588/0.600/0.612 V;
     ton,min 75 ns; Dmax 100 %; tSS 1.2 ms; VUVLO <= 2.5 V; EN VIH 1.5 V / VIL 0.4 V;
     EN, FB abs-max VIN + 0.6 V; thetaJA 170 C/W (2"x2" 2 oz FR-4); TJ(op) <= 125 C;
     pinout 1 EN, 2 GND, 3 LX, 4 IN, 5 FB; Vout = 0.6 (1 + R1/R2), R 10k..1M;
     L design rule: ripple ~40 % of IOUT,max; Isat > IOUT + dI/2; DCR < 50 mOhm;
     typical app: CIN 22 uF, L 2.2 uH, COUT 2 x 22 uF, 22 pF across R1 (1.8 V, R1 200k).
[D2] cjiang FNR4030S2R2MT (LCSC C167869): 2.2 uH +/-20 %, Isat 5.8 A, Irms 3.2 A,
     DCR 39 mOhm, 4.0 x 4.0 x 3.0 mm, land 1.6 x 3.9 mm at +/-1.5 mm (EasyEDA).
[D3] Samsung CL21A226MAQNNNE (LCSC C45783, JLC basic): 22 uF 25 V X5R 0805.
[D4] onsemi MAX809STRG (LCSC C9965): SOT-23 1 GND / 2 RESET# / 3 VCC, push-pull active-low,
     Vth 2.93 V (2.85..3.00 V over temperature), tRP >= 140 ms, RESET# valid down to VCC 1.0 V.
[S]  Stackup from tools/mdi_impedance.py: JLC04161H-2313 (Rev 19), L4-L3 prepreg 0.100 mm.

Enable / sequencing: EN = U6 RESET# (monitors the module's +3V3_OUT) -> R42 10k -> EN,
R43 1M to GND. A plain 10k/1M from +3V3_OUT was rejected in review: SY8089 EN is only
guaranteed OFF below 0.4 V, so +3V3_SYS could stay up while the module rail collapses
and back-power the SoC IO through its ESD diodes.

Coordinate frame: carrier board, mm, KiCad Y-down, viewed from the TOP. All
power-stage parts are on B.Cu (the top side is covered by the CM4 module).
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
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))
from mdi_impedance import STACKUP  # noqa: E402  (single stackup source of truth)

PCB = ROOT / "radxa-cm4-nano-carrier.kicad_pcb"
PRO = ROOT / "radxa-cm4-nano-carrier.kicad_pro"
DRU = ROOT / "radxa-cm4-nano-carrier.kicad_dru"
LIB = ROOT / "lib" / "radxa-cm4-nano-carrier.pretty"
LIBNICK = "radxa-cm4-nano-carrier"
DESIGN_JSON = ROOT / "artifacts" / "rev13_buck_3v3_design.json"
REPORT = ROOT / "artifacts" / "rev13_buck_3v3_report.md"
GROUP = "REV13_BUCK_3V3"
MU0 = 4e-7 * math.pi


# ==========================================================================
# 1. Electrical design (pure math, no KiCad)
# ==========================================================================
@dataclass(frozen=True)
class Spec:
    # Input: +5V_IN from USB-C (USB 2.0/Type-C 5 V: 4.75..5.25 V at the receptacle;
    # 5.5 V = SY8089 recommended max, used as the abs. upper corner).
    vin_nom: float = 5.0
    vin_min: float = 4.75
    vin_max: float = 5.25
    vin_abs: float = 5.5
    vout_target: float = 3.3
    vout_tol: float = 0.05            # +/-5 % TOTAL window (DC stack + IFB + ripple/2 + 1 A step), checked combined
    iout_max: float = 2.0             # continuous design load (datasheet rating)
    istep: float = 1.0                # load-step for transient estimate (A)
    vout_ripple_max: float = 0.033    # 1 % of 3.3 V, p-p
    vin_ripple_max: float = 0.100     # p-p at the IC IN pin (conducted-EMI budget input)
    ripple_ratio_lo: float = 0.20     # dI/IOUT acceptance band at nominal L, fsw
    ripple_ratio_hi: float = 0.40


@dataclass(frozen=True)
class IC:                              # [D1]
    mpn: str = "SY8089AAAC"
    lcsc: str = "C78988"
    fsw: float = 1.0e6
    fsw_tol: float = 0.20             # NOT in [D1]; +/-20 % assumed (typ. for this class) -> corners
    rds_p: float = 0.110
    rds_n: float = 0.080
    rds_tc_per_k: float = 0.004       # RDS(on) tempco (assumed, typical CMOS +0.4 %/K); tc(TJ) = 1 + k (TJ - 25)
    ilim_min: float = 3.5
    vref: tuple = (0.588, 0.600, 0.612)
    ifb: float = 50e-9                # FB input current, |max| (EC table)
    ton_min: float = 75e-9
    tss: float = 1.2e-3
    theta_ja_ds: float = 170.0        # datasheet board
    theta_ja_est: float = 115.0       # ESTIMATE for this 4-layer board: L2/L3 planes + under-body via
    tj_max_op: float = 125.0
    t_sw: float = 8e-9                # tr + tf at LX (assumed; not specified) -> switching loss
    p_fixed: float = 0.012            # gate drive + controller (assumed, 1 MHz, 5 V)
    en_vih: float = 1.5
    en_vil: float = 0.4

    def tc(self, tj: float) -> float:
        return 1 + self.rds_tc_per_k * (tj - 25.0)


@dataclass(frozen=True)
class Supervisor:                      # [D4] onsemi MAX809STRG (LCSC C9965), SOT-23: 1 GND, 2 RESET#, 3 VCC
    mpn: str = "MAX809STRG"
    lcsc: str = "C9965"
    vth_typ: float = 2.93
    vth_min: float = 2.85             # over -40..+85 C (MAX809S grade)
    vth_max: float = 3.00
    t_rp: float = 0.140               # reset timeout (min), release delay after VCC > Vth
    vcc_valid: float = 1.0            # RESET# guaranteed low down to VCC = 1.0 V
    vf_esd: float = 0.6               # IO ESD-diode conduction onset used as back-power limit


@dataclass(frozen=True)
class Inductor:                        # [D2]
    mpn: str = "FNR4030S2R2MT"
    lcsc: str = "C167869"
    L: float = 2.2e-6
    tol: float = 0.20
    isat: float = 5.8
    irms: float = 3.2
    dcr: float = 0.039
    size: str = "4.0 x 4.0 x 3.0 mm"


@dataclass(frozen=True)
class Caps:
    # 22 uF 25 V X5R 0805 (C45783). Effective capacitance under DC bias from
    # Samsung's published CL21A226MAQNNNE curve (approx.; verify with KSIM):
    c22_nom: float = 22e-6
    c22_eff_3v3: float = 11.5e-6
    c22_eff_5v: float = 9.0e-6
    esr_22: float = 0.004             # ~4 mOhm at 1 MHz (MLCC, typical)
    n_out: int = 2
    n_in: int = 1
    c_hf: float = 100e-9              # 0402 X7R HF bypass at IN
    tol: float = 0.20


@dataclass(frozen=True)
class Divider:
    r1: float = 100e3                 # VOUT_SENSE -> FB   YAGEO RT0402BRD07100KL 0.1 % 25 ppm (C852472)
    r2: float = 22.1e3                # FB -> GND          RESI PTFR0402B22K1P9 0.1 % 25 ppm (C2692911)
    tol: float = 0.001 + 25e-6 * 100  # 0.1 % initial + 25 ppm/K x 100 K drift (no ratio tracking assumed) = 0.35 %
    cff: float = 47e-12               # across R1 (see calc: zero placement)
    r_en: float = 10e3                # U6 RESET# (push-pull, VOH = +3V3_OUT) -> EN
    r_en_pd: float = 1e6              # EN -> GND (datasheet layout note 5)
    v_en_src: float = 3.3


SPEC, U, LL, CC, FB, SUP = Spec(), IC(), Inductor(), Caps(), Divider(), Supervisor()


def ripple(vin, vout, L, f):
    return vout * (1 - vout / vin) / (f * L)


def calc() -> dict:
    s, u, l, c, d = SPEC, U, LL, CC, FB
    checks: list[tuple[str, bool, str]] = []

    def chk(name, ok, detail):
        checks.append((name, bool(ok), detail))

    # --- feedback divider + Vout tolerance stack
    vout_nom = d.r1 / d.r2 + 1
    vout_nom *= u.vref[1]
    vout_hi = u.vref[2] * (1 + d.r1 * (1 + d.tol) / (d.r2 * (1 - d.tol)))
    vout_lo = u.vref[0] * (1 + d.r1 * (1 - d.tol) / (d.r2 * (1 + d.tol)))
    vo = vout_nom
    chk("Vout setpoint within 1 % of 3.300 V", abs(vo - s.vout_target) / s.vout_target < 0.01,
        f"0.6 x (1 + {d.r1/1e3:.0f}k/{d.r2/1e3:.1f}k) = {vo:.4f} V")
    chk("Vout DC worst case (VREF +/-2 %, R +/-0.35 %) inside 3.3 V +/-5 %",
        vout_lo >= s.vout_target * (1 - s.vout_tol) and vout_hi <= s.vout_target * (1 + s.vout_tol),
        f"{vout_lo:.3f} .. {vout_hi:.3f} V (window {s.vout_target*(1-s.vout_tol):.3f} .. {s.vout_target*(1+s.vout_tol):.3f})")
    chk("Divider resistors inside datasheet 10k..1M range", 10e3 <= d.r2 <= 1e6 and 10e3 <= d.r1 <= 1e6,
        f"R1 {d.r1/1e3:.0f}k, R2 {d.r2/1e3:.1f}k, I_div {vo/(d.r1+d.r2)*1e6:.1f} uA")

    # --- inductor: datasheet sizing rule at VIN,max, 40 % ripple
    l_rule = vo * (1 - vo / s.vin_max) / (u.fsw * s.iout_max * 0.40)
    di_nom = ripple(s.vin_max, vo, l.L, u.fsw)
    corners = []
    for vin in (s.vin_min, s.vin_nom, s.vin_max, s.vin_abs):
        for fk in (1 - u.fsw_tol, 1.0, 1 + u.fsw_tol):
            for lk in (1 - l.tol, 1.0, 1 + l.tol):
                corners.append(dict(vin=vin, f=u.fsw * fk, L=l.L * lk, di=ripple(vin, vo, l.L * lk, u.fsw * fk)))
    di_wc = max(cn["di"] for cn in corners)
    di_min = min(cn["di"] for cn in corners)
    ipk_wc = s.iout_max + di_wc / 2
    ripple_ratio = di_nom / s.iout_max
    chk("Datasheet L rule: L >= Vout(1-Vout/Vin,max)/(fsw*0.4*Iout)", l.L >= l_rule,
        f"L_rule {l_rule*1e6:.2f} uH -> {l.L*1e6:.1f} uH ({LL.mpn}, also the datasheet's own typical value)")
    chk(f"Ripple ratio at nominal L/fsw, VIN,max in {s.ripple_ratio_lo:.0%}..{s.ripple_ratio_hi:.0%}",
        s.ripple_ratio_lo <= ripple_ratio <= s.ripple_ratio_hi,
        f"dI = {di_nom*1e3:.0f} mA p-p = {ripple_ratio*100:.1f} % of {s.iout_max} A")
    chk("Worst-corner peak current < Isat (L -20 %, fsw -20 %, VIN 5.5 V)", ipk_wc < l.isat,
        f"Ipk {ipk_wc:.2f} A vs Isat {l.isat} A (dI_wc {di_wc*1e3:.0f} mA)")
    chk("Isat > IC current limit (no saturation during OCP / hic-cup)", l.isat > u.ilim_min,
        f"Isat {l.isat} A > ILIM,min {u.ilim_min} A (ILIM max not specified -> 66 % headroom)")
    irms_l = math.sqrt(s.iout_max ** 2 + di_wc ** 2 / 12)
    chk("Inductor RMS current < Irms rating", irms_l < l.irms, f"{irms_l:.3f} A vs {l.irms} A")
    chk("Inductor DCR < 50 mOhm (datasheet efficiency guideline)", l.dcr < 0.050, f"{l.dcr*1e3:.0f} mOhm")
    chk("Ripple stays >= 10 % at the low corner (current-mode ramp / light-load PFM entry)", di_min / s.iout_max >= 0.10,
        f"dI_min {di_min*1e3:.0f} mA ({di_min/s.iout_max*100:.1f} %) at VIN {s.vin_min} V, L +20 %, fsw +20 %")

    # --- duty, minimum on-time, dropout headroom
    def duty(vin, i):
        # CCM duty incl. conduction drops: D = (Vo + I(Rn + DCR)) / (Vin - I(Rp - Rn))
        rn, rp = u.rds_n * u.tc(u.tj_max_op), u.rds_p * u.tc(u.tj_max_op)
        return (vo + i * (rn + l.dcr)) / (vin - i * (rp - rn))
    d_max = duty(s.vin_min, s.iout_max)
    d_min = duty(s.vin_abs, 0.0)
    ton_min = d_min / (u.fsw * (1 + u.fsw_tol))
    chk("Min on-time margin (D_min / fsw,max >> 75 ns)", ton_min > 3 * u.ton_min,
        f"ton = {ton_min*1e9:.0f} ns at VIN {s.vin_abs} V, fsw +20 % (limit {u.ton_min*1e9:.0f} ns)")
    vin_drop = vo + s.iout_max * (u.rds_p * u.tc(u.tj_max_op) + l.dcr + 0.010)
    chk("Regulation headroom at 2 A (100 % duty dropout) below VIN,min", vin_drop < s.vin_min,
        f"VIN needed {vin_drop:.2f} V vs USB VIN,min {s.vin_min} V (D_max {d_max*100:.1f} %)")

    # --- input capacitor
    D = vo / s.vin_nom
    i_cin_rms = s.iout_max * math.sqrt(D * (1 - D))
    i_cin_rms_wc = s.iout_max / 2
    cin_eff = c.n_in * c.c22_eff_5v * (1 - c.tol) + c.c_hf
    # sweep VIN: D(1-D) rises toward VIN = 2 Vout, ESR term at the SAME corner (L -20 %, fsw -20 %)
    dv_c = []
    for vin in (s.vin_min, s.vin_nom, s.vin_max, s.vin_abs):
        Dv = vo / vin
        ce = c.n_in * c.c22_eff_5v * (1 - c.tol) * (1 - 0.15 * max(0.0, vin - 5.0) / 0.5) + c.c_hf   # extra DC-bias loss > 5 V
        di_v = ripple(vin, vo, l.L * (1 - l.tol), u.fsw * (1 - u.fsw_tol))
        dv_c.append((s.iout_max * Dv * (1 - Dv) / (u.fsw * (1 - u.fsw_tol) * ce) + (s.iout_max + di_v / 2) * c.esr_22, vin, ce))
    dvin, dvin_vin, cin_eff = max(x for x in dv_c if x[1] <= s.vin_max)   # budget: USB operating range
    dvin_abs = max(dv_c)[0]                                                  # 5.5 V abs corner: reported
    chk("Cin RMS ripple current handled by 22 uF X5R (self-heating < 10 mW)",
        i_cin_rms_wc ** 2 * c.esr_22 < 0.010, f"I_rms {i_cin_rms:.3f} A (worst VIN=2Vout bound {i_cin_rms_wc:.2f} A), "
        f"P_ESR {i_cin_rms_wc**2*c.esr_22*1e3:.1f} mW")
    chk("Input ripple at IN pin, worst USB VIN corner 4.75..5.25 V (C -20 %, fsw -20 %, L -20 %, DC-bias derated)",
        dvin < s.vin_ripple_max,
        f"dVin {dvin*1e3:.1f} mV p-p at VIN {dvin_vin} V (C_eff {cin_eff*1e6:.1f} uF) vs {s.vin_ripple_max*1e3:.0f} mV; "
        f"{dvin_abs*1e3:.0f} mV at the 5.5 V IC limit (outside USB VBUS range)")

    # --- output capacitor: ripple + load step
    cout_eff = c.n_out * c.c22_eff_3v3 * (1 - c.tol)
    dvo = di_wc / (8 * u.fsw * (1 - u.fsw_tol) * cout_eff) + di_wc * c.esr_22 / c.n_out
    chk("Output ripple (worst corner) < 1 % Vout", dvo < s.vout_ripple_max,
        f"dVout {dvo*1e3:.2f} mV p-p (C_eff {cout_eff*1e6:.1f} uF)")
    L_hi = l.L * (1 + l.tol)
    dv_up = L_hi * s.istep ** 2 / (2 * cout_eff * (s.vin_min - vo))      # load application (slew-limited)
    dv_dn = L_hi * s.istep ** 2 / (2 * cout_eff * vo)                     # load release
    chk("1 A load-step LC energy deviation (inductor slew bound) < 3 % Vout",
        max(dv_up, dv_dn) < 0.03 * vo, f"undershoot {dv_up*1e3:.1f} mV, overshoot {dv_dn*1e3:.1f} mV "
        "(lower bound; loop-delay term measured on the bench)")
    # combined +/-5 % window: DC stack + FB bias current through R1 + half ripple + 1 A step
    v_ifb = u.ifb * d.r1
    win_hi = vout_hi + v_ifb + dvo / 2 + dv_dn
    win_lo = vout_lo - v_ifb - dvo / 2 - dv_up
    chk("Vout TOTAL window (DC + IFB*R1 + ripple/2 + 1 A step) inside 3.3 V +/-5 %",
        win_lo >= s.vout_target * (1 - s.vout_tol) and win_hi <= s.vout_target * (1 + s.vout_tol),
        f"{win_lo:.3f} .. {win_hi:.3f} V (margins {(win_lo - s.vout_target*(1-s.vout_tol))*1e3:.0f} / "
        f"{(s.vout_target*(1+s.vout_tol) - win_hi)*1e3:.0f} mV)")
    f_lc = 1 / (2 * math.pi * math.sqrt(l.L * c.n_out * c.c22_eff_3v3))
    f_z = 1 / (2 * math.pi * d.r1 * d.cff)
    f_p = 1 / (2 * math.pi * (d.r1 * d.r2 / (d.r1 + d.r2)) * d.cff)
    f_boost = math.sqrt(f_z * f_p)
    phi = math.degrees(math.asin((f_p - f_z) / (f_p + f_z)))
    # Datasheet-validated reference: 1.8 V, R1 200k / R2 100k / 22 pF
    ds_z = 1 / (2 * math.pi * 200e3 * 22e-12)
    ds_boost = ds_z * math.sqrt(3.0)
    chk("Cff phase-boost centre within 0.5x..2x of the datasheet-validated network",
        0.5 <= f_boost / ds_boost <= 2.0,
        f"zero {f_z/1e3:.1f} kHz, pole {f_p/1e3:.0f} kHz, max boost {phi:.0f} deg at {f_boost/1e3:.0f} kHz "
        f"(datasheet 1.8 V example: {ds_boost/1e3:.0f} kHz); LC pole {f_lc/1e3:.1f} kHz")
    chk("Same L / Cout as datasheet typical application (internal compensation validated region)",
        abs(l.L - 2.2e-6) < 1e-9 and c.n_out == 2, "2.2 uH, 2 x 22 uF")

    # --- soft-start inrush
    i_inrush = c.n_out * c.c22_nom * vo / u.tss
    chk("Soft-start Cout charge current << ILIM", i_inrush < 0.2 * u.ilim_min, f"{i_inrush*1e3:.0f} mA over {u.tss*1e3:.1f} ms")

    # --- EN network: U6 supervisor on the module's +3V3_OUT -> R42 -> EN, R43 pull-down
    sp = SUP
    v_en = d.v_en_src * d.r_en_pd / (d.r_en + d.r_en_pd)
    chk("EN high level (U6 VOH = +3V3_OUT) >= VIH 1.5 V and <= VIN + 0.6 V", u.en_vih <= v_en <= s.vin_min + 0.6,
        f"V_EN {v_en:.2f} V")
    # Power-down: buck must stop while the module rail is still high enough that +3V3_SYS cannot
    # forward-bias the SoC IO ESD diodes into a collapsing +3V3_OUT. Plain RC/divider from +3V3_OUT
    # only guarantees EN-off below VENL 0.4 V (-> buck alive down to +3V3_OUT ~0.40 V): rejected.
    v_off_divider = u.en_vil * (d.r_en + d.r_en_pd) / d.r_en_pd
    backpower = win_hi - sp.vth_min
    chk("Power-down: +3V3_SYS shut off while +3V3_OUT >= Vth,min, back-bias < ESD onset",
        backpower < sp.vf_esd, f"U6 trips at +3V3_OUT {sp.vth_min:.2f}..{sp.vth_max:.2f} V -> max +3V3_SYS - +3V3_OUT "
        f"{backpower:.2f} V < {sp.vf_esd} V (plain 10k/1M from +3V3_OUT: EN-off only below {v_off_divider:.2f} V)")
    chk("Power-up: buck enabled >= 140 ms after +3V3_OUT valid (module rail first)", sp.t_rp >= 0.1,
        f"t_RP {sp.t_rp*1e3:.0f} ms min; RESET# low for VCC 1.0 V..Vth; R43 1M holds EN low with module absent")
    i_en_inj = max(0.0, (d.v_en_src * 1.05 - 0.0 - 0.6)) / d.r_en
    chk("EN clamp current limited if +5V_IN collapses first (10k series)", i_en_inj < 1e-3,
        f"<= {i_en_inj*1e3:.2f} mA into EN ESD diode with VIN = 0")

    # --- losses / efficiency / thermal
    def losses(i, vin=s.vin_nom, tc=u.tc(u.tj_max_op), f=u.fsw):
        dd = ripple(vin, vo, l.L, f)
        i2 = i ** 2 + dd ** 2 / 12
        dty = vo / vin
        p_cond = i2 * (dty * u.rds_p + (1 - dty) * u.rds_n) * tc
        p_sw = 0.5 * vin * i * u.t_sw * f
        p_ic = p_cond + p_sw + u.p_fixed
        p_l = i2 * l.dcr
        p_div = vo ** 2 / (d.r1 + d.r2)
        return p_ic, p_l + p_div
    def tj_solve(ta, theta, i=s.iout_max):
        """Self-consistent TJ: RDS(on) evaluated at the junction temperature it produces."""
        t = ta
        for _ in range(50):
            t = ta + losses(i, tc=u.tc(t))[0] * theta
        return t
    tj = {ta: (tj_solve(ta, u.theta_ja_ds), tj_solve(ta, u.theta_ja_est)) for ta in (25, 50, 70, 85)}
    p_ic, p_ext = losses(s.iout_max, tc=u.tc(tj[50][1]))       # reported at TA 50 C, board thetaJA
    eff = vo * s.iout_max / (vo * s.iout_max + p_ic + p_ext)
    eff_1a = vo * 1.0 / (vo * 1.0 + sum(losses(1.0, tc=u.tc(tj_solve(50, u.theta_ja_est, 1.0)))))

    def imax(ta, theta):          # losses() default tc = tc(TJ = 125 C): the bisection asks "TJ <= 125 C?"
        lo, hi = 0.0, s.iout_max
        if ta + losses(hi)[0] * theta <= u.tj_max_op:
            return hi
        for _ in range(60):
            m = (lo + hi) / 2
            lo, hi = (m, hi) if ta + losses(m)[0] * theta <= u.tj_max_op else (lo, m)
        return lo
    derate = {ta: (imax(ta, u.theta_ja_ds), imax(ta, u.theta_ja_est)) for ta in (25, 40, 50, 60, 70, 85)}
    chk("Efficiency at 2 A / 5 V >= 88 %", eff >= 0.88, f"{eff*100:.1f} % (1 A: {eff_1a*100:.1f} %), "
        f"P_IC {p_ic*1e3:.0f} mW, P_L+div {p_ext*1e3:.0f} mW")
    chk("TJ <= 125 C at 2 A, TA 50 C (board-estimated thetaJA, self-consistent RDS(on)(TJ))", tj[50][1] <= u.tj_max_op,
        f"TJ {tj[50][1]:.0f} C @ {u.theta_ja_est:.0f} C/W ({tj[50][0]:.0f} C @ datasheet {u.theta_ja_ds:.0f} C/W)")
    budget = 1.2
    chk(f"Carrier 3V3 load budget {budget} A <= derated limit at TA 70 C (datasheet thetaJA, conservative)",
        derate[70][0] >= budget, f"I_max(70 C) = {derate[70][0]:.2f} A @170 C/W, {derate[70][1]:.2f} A @115 C/W")

    res = dict(
        vout=dict(nom=vo, lo=vout_lo, hi=vout_hi, window_lo=win_lo, window_hi=win_hi),
        inductor=dict(L_rule_uH=l_rule * 1e6, L_uH=l.L * 1e6, di_nom_mA=di_nom * 1e3, di_wc_mA=di_wc * 1e3,
                      di_min_mA=di_min * 1e3, ripple_ratio=ripple_ratio, ipk_wc=ipk_wc, irms=irms_l),
        duty=dict(d_max=d_max, ton_min_ns=ton_min * 1e9, vin_dropout=vin_drop),
        cin=dict(i_rms=i_cin_rms, i_rms_wc=i_cin_rms_wc, c_eff_uF=cin_eff * 1e6, dvin_mV=dvin * 1e3, dvin_at_vin=dvin_vin,
                 dvin_5v5_mV=dvin_abs * 1e3),
        cout=dict(c_eff_uF=cout_eff * 1e6, dvo_mV=dvo * 1e3, step_under_mV=dv_up * 1e3, step_over_mV=dv_dn * 1e3,
                  f_lc_kHz=f_lc / 1e3),
        ff=dict(fz_kHz=f_z / 1e3, fp_kHz=f_p / 1e3, boost_kHz=f_boost / 1e3, phase_deg=phi, ds_boost_kHz=ds_boost / 1e3),
        en=dict(v_en=v_en, v_off_divider_only=v_off_divider, backpower_V=backpower), inrush_mA=i_inrush * 1e3,
        thermal=dict(p_ic_W=p_ic, p_ext_W=p_ext, eff=eff, eff_1a=eff_1a,
                     tj={str(k): v for k, v in tj.items()}, derate={str(k): v for k, v in derate.items()}),
        corners=len(corners),
        checks=[dict(name=n, ok=o, detail=t) for n, o, t in checks],
    )
    return res


def cmd_calc(_a) -> int:
    r = calc()
    DESIGN_JSON.parent.mkdir(exist_ok=True)
    DESIGN_JSON.write_text(json.dumps(r, indent=2))
    bad = 0
    for c_ in r["checks"]:
        print(f"  [{'PASS' if c_['ok'] else 'FAIL'}] {c_['name']}: {c_['detail']}")
        bad += not c_["ok"]
    print("\n  Thermal derating (TJ <= 125 C): TA -> I_max @170 C/W (datasheet) | @115 C/W (board est.)")
    for ta, (a_, b_) in r["thermal"]["derate"].items():
        print(f"    {ta:>3} C  {a_:.2f} A | {b_:.2f} A")
    print(f"\n{len(r['checks']) - bad}/{len(r['checks'])} design checks pass -> {DESIGN_JSON.relative_to(ROOT)}")
    return 1 if bad else 0


# ==========================================================================
# 2. Footprints (written into lib/radxa-cm4-nano-carrier.pretty)
# ==========================================================================
def _fp(name, descr, tags, pads, body, crt, height, attr="smd", silk=None):
    o = [f'(footprint "{name}"', '  (version 20240108)', '  (generator "buck_3v3.py")', '  (layer "F.Cu")',
         f'  (descr "{descr}")', f'  (tags "{tags}")', f'  (attr {attr})',
         f'  (property "Reference" "REF**" (at 0 {-crt[1]-0.6:.3f} 0) (layer "F.SilkS") (effects (font (size 0.6 0.6) (thickness 0.1))))',
         f'  (property "Value" "{name}" (at 0 {crt[1]+0.6:.3f} 0) (layer "F.Fab") (effects (font (size 0.6 0.6) (thickness 0.1))))',
         f'  (property "Height" "{height}" (at 0 0 0) (layer "F.Fab") (hide yes) (effects (font (size 1 1))))']
    bx, by = body
    for (x1, y1, x2, y2) in [(-bx, -by, bx, -by), (bx, -by, bx, by), (bx, by, -bx, by), (-bx, by, -bx, -by)]:
        o.append(f'  (fp_line (start {x1:.3f} {y1:.3f}) (end {x2:.3f} {y2:.3f}) (stroke (width 0.1) (type solid)) (layer "F.Fab"))')
    cx, cy = crt
    for (x1, y1, x2, y2) in [(-cx, -cy, cx, -cy), (cx, -cy, cx, cy), (cx, cy, -cx, cy), (-cx, cy, -cx, -cy)]:
        o.append(f'  (fp_line (start {x1:.3f} {y1:.3f}) (end {x2:.3f} {y2:.3f}) (stroke (width 0.05) (type solid)) (layer "F.CrtYd"))')
    for s_ in silk or []:
        o.append(s_)
    for (num, x, y, w, h, shape) in pads:
        extra = " (roundrect_rratio 0.15)" if shape == "roundrect" else ""
        o.append(f'  (pad "{num}" smd {shape} (at {x:.4f} {y:.4f}) (size {w:.3f} {h:.3f}) '
                 f'(layers "F.Cu" "F.Paste" "F.Mask"){extra})')
    o.append(")")
    return "\n".join(o) + "\n"


FP_U = "SOT-23-5_Silergy_SY8089"
FP_L = "L_4.0x4.0mm_cjiang_FNR4030"
FP_C0805 = "C_0805_2012Metric"
FP_U6 = "SOT-23_onsemi_MAX809"
FP_C0402 = "C_0402_1005Metric"      # existing (ethernet_mdi.py)
FP_R0402 = "R_0402_1005Metric"      # existing (ethernet_mdi.py)

SOT_X, SOT_P, SOT_PAD = 1.1375, 0.95, (1.325, 0.60)     # IPC-7351 nominal SOT-23-5 (JEDEC MO-178)


def footprints() -> dict:
    sot = [(str(n), -SOT_X, (n - 2) * SOT_P, *SOT_PAD, "roundrect") for n in (1, 2, 3)] + \
          [("4", SOT_X, SOT_P, *SOT_PAD, "roundrect"), ("5", SOT_X, -SOT_P, *SOT_PAD, "roundrect")]
    pin1 = ['  (fp_line (start -0.9 -1.6) (end 0.7 -1.6) (stroke (width 0.12) (type solid)) (layer "F.SilkS"))',
            '  (fp_circle (center -1.75 -1.75) (end -1.65 -1.75) (stroke (width 0.2) (type solid)) (fill solid) (layer "F.SilkS"))']
    return {
        FP_U: _fp(FP_U, "SOT-23-5 (SY8089AAAC, LCSC C78988): 1 EN, 2 GND, 3 LX, 4 IN, 5 FB. IPC-7351 nominal land",
                  "SOT-23-5 buck Silergy", sot, (0.8, 1.45), (2.05, 1.75), 1.45, silk=pin1),
        FP_L: _fp(FP_L, "cjiang FNR4030S2R2MT 4.0x4.0x3.0 mm shielded power inductor, land 1.6x3.9 at +/-1.5 (LCSC C167869)",
                  "inductor power shielded 4030", [("1", -1.5, 0, 1.6, 3.9, "roundrect"), ("2", 1.5, 0, 1.6, 3.9, "roundrect")],
                  (2.0, 2.0), (2.55, 2.25), 3.0),     # courtyard encloses the 1.6 x 3.9 pads (+/-2.30) + 0.25
        FP_U6: _fp(FP_U6, "SOT-23 (MAX809STRG, LCSC C9965): 1 GND, 2 RESET#, 3 VCC. IPC-7351 nominal land",
                   "SOT-23 supervisor reset", [("1", -SOT_X, -SOT_P, *SOT_PAD, "roundrect"),
                                               ("2", -SOT_X, SOT_P, *SOT_PAD, "roundrect"),
                                               ("3", SOT_X, 0.0, *SOT_PAD, "roundrect")],
                   (0.7, 1.45), (2.05, 1.50), 1.1, silk=pin1),
        FP_C0805: _fp(FP_C0805, "0805 chip capacitor, IPC-7351 nominal (1.0 x 1.45 pads, 1.9 mm pitch)",
                      "capacitor 0805", [("1", -0.95, 0, 1.0, 1.45, "roundrect"), ("2", 0.95, 0, 1.0, 1.45, "roundrect")],
                      (1.0, 0.625), (1.68, 0.95), 1.35),
    }


# ==========================================================================
# 3. Placement / routing plan
# ==========================================================================
# The power stage is drawn once in a "cell frame" (design coordinates below,
# origin CELL_O = U5 centre, power flow IN(west) -> LX -> L1 -> COUT (south),
# FB/EN north, Kelvin sense corridor east) and then mapped onto the board by a
# PROPER rigid rotation (det = +1, no mirror) + translation. Every distance,
# loop area and clearance is therefore invariant under re-placement.
#
# Rev 13 placement: B.Cu under the west half of J1, next to the module's
# +5V_IN (J1 pins 77-87, x 17.6-19.6) and +3V3_OUT (J1 84/86) pins. The east
# strip used in the first pass was taken by the Rev 9 MDI pairs (x 45.5-50.5)
# and the Rev 11 HDMI J4 (51.0, 29.2) while this stage was being built.
CELL_O = (50.0, 12.0)            # cell-frame origin (design coordinates)
U_AT_BOARD = (13.0, 4.9)         # where CELL_O lands on the board (all copper y <= 9.65: Ethernet GND notch starts 9.8)
CELL_ROT = 1                     # quarter turns: cell south (+y) -> board east (+x)


def xf(p):
    """cell frame -> board frame (proper rotation, k x 90 deg, then translate)."""
    dx, dy = p[0] - CELL_O[0], p[1] - CELL_O[1]
    for _ in range(CELL_ROT % 4):
        dx, dy = dy, -dx          # (0,+1) -> (+1,0): south -> east
    return (round(U_AT_BOARD[0] + dx, 6), round(U_AT_BOARD[1] + dy, 6))


U_AT = CELL_O
# pad target positions (cell frame). U5 is flipped to B.Cu: local x mirrors.
U_PIN = {"1": (U_AT[0] + SOT_X, U_AT[1] - SOT_P), "2": (U_AT[0] + SOT_X, U_AT[1]),
         "3": (U_AT[0] + SOT_X, U_AT[1] + SOT_P), "4": (U_AT[0] - SOT_X, U_AT[1] + SOT_P),
         "5": (U_AT[0] - SOT_X, U_AT[1] - SOT_P)}
NETS_U = {"1": "BUCK_EN", "2": "GND", "3": "BUCK_LX", "4": "+5V_IN", "5": "BUCK_FB"}

# 2-pin parts: ref -> (footprint, value, LCSC, MPN, pad1 xy, pad1 net, pad2 xy, pad2 net)
R04, C08 = 0.51, 0.95
X_C41, X_C40 = 47.40, 45.90
PARTS = {
    # Input: C41 = HF 100 nF right at pin 4, C40 = 22 uF bulk
    "C41": (FP_C0402, "100nF 16V X7R", "C1525", "CL05B104KO5NNNC",
            (X_C41, U_PIN["4"][1]), "+5V_IN", (X_C41, U_PIN["4"][1] - 2 * R04), "GND"),
    "C40": (FP_C0805, "22uF 25V X5R", "C45783", "CL21A226MAQNNNE",
            (X_C40, U_PIN["4"][1]), "+5V_IN", (X_C40, U_PIN["4"][1] - 2 * C08), "GND"),
    # Power inductor, pad 1 = LX (north), pad 2 = VOUT (south)
    "L1": (FP_L, "2.2uH 5.8A 39mR", "C167869", "FNR4030S2R2MT", (51.60, 15.30), "BUCK_LX", (51.60, 18.30), "+3V3_SYS"),
    # Output caps
    "C42": (FP_C0805, "22uF 25V X5R", "C45783", "CL21A226MAQNNNE", (50.50, 20.25), "+3V3_SYS", (50.50, 22.15), "GND"),
    "C43": (FP_C0805, "22uF 25V X5R", "C45783", "CL21A226MAQNNNE", (52.70, 20.25), "+3V3_SYS", (52.70, 22.15), "GND"),
    # Feedback: R40 top (sense -> FB), R41 bottom (FB -> GND), C44 feed-forward across R40
    "R41": (FP_R0402, "22.1k 0.1% 25ppm", "C2692911", "PTFR0402B22K1P9",
            (U_PIN["5"][0], 9.40), "BUCK_FB", (U_PIN["5"][0] - 2 * R04, 9.40), "GND"),
    "R40": (FP_R0402, "100k 0.1% 25ppm", "C852472", "RT0402BRD07100KL",
            (U_PIN["5"][0], 8.40), "BUCK_FB", (U_PIN["5"][0] + 2 * R04, 8.40), "+3V3_SYS"),
    "C44": (FP_C0402, "47pF 50V C0G", "C1567", "CL05C470JB5NNNC",
            (U_PIN["5"][0], 7.40), "BUCK_FB", (U_PIN["5"][0] + 2 * R04, 7.40), "+3V3_SYS"),
    # EN: R42 series from module +3V3_OUT, R43 1M pull-down
    "R42": (FP_R0402, "10k 1%", "C25744", "0402WGF1002TCE", (52.20, 9.70), "BUCK_EN", (53.22, 9.70), "BUCK_EN_SRC"),
    "R43": (FP_R0402, "1M 1%", "C26083", "0402WGF1004TCE", (52.20, 8.70), "BUCK_EN", (53.22, 8.70), "GND"),
}
U_REF = "U5"

# U6 MAX809 supervisor, BOARD frame (not cell frame): west of the cell between the H1
# standoff circle (r 3.1) and the Ethernet inner-layer GND notch (copper y <= 9.8).
# On B.Cu the flipped SOT-23 has pins 1/2 in the east column with pin 1 north.
U6_REF = "U6"
U6_AT = (5.45, 8.25)
U6_PIN = {"1": (U6_AT[0] + SOT_X, U6_AT[1] - SOT_P), "2": (U6_AT[0] + SOT_X, U6_AT[1] + SOT_P),
          "3": (U6_AT[0] - SOT_X, U6_AT[1])}
NETS_U6 = {"1": "GND", "2": "BUCK_EN_SRC", "3": "+3V3_OUT"}
U6_VIA_GND, U6_VIA_OUT, U6_VIA_VCC = (6.59, 6.35), (7.75, 9.20), (4.31, 6.90)   # GND via N of pin 1: off the F.Cu jumper path
# board-frame extras: (pts, net, width, layer)
U6_TRACKS = [
    ([U6_PIN["1"], U6_VIA_GND], "GND", 0.30, "B.Cu"),
    ([U6_PIN["2"], U6_VIA_OUT], "BUCK_EN_SRC", 0.30, "B.Cu"),
    ([U6_PIN["3"], U6_VIA_VCC], "+3V3_OUT", 0.30, "B.Cu"),
    # F.Cu jumper over the Kelvin-sense loop (which encloses U5 EN on B.Cu) to the R42 landing via
    ([U6_VIA_OUT, (7.75, 3.60), (9.67, 1.68), xf((53.22, 10.70))], "BUCK_EN_SRC", 0.20, "F.Cu"),
]
U6_VIAS = [(U6_VIA_GND, "GND", "U6 GND"), (U6_VIA_OUT, "BUCK_EN_SRC", "U6 RESET# -> F.Cu jumper"),
           (U6_VIA_VCC, "+3V3_OUT", "U6 VCC = module +3V3_OUT hand-off (J1 pins 84/86)")]

W_PWR, W_PIN, W_LX, W_SIG = 1.0, 0.6, 1.2, 0.2
VIA_D, VIA_DRILL = 0.50, 0.30          # board-setup minimums (0.5 / 0.3)
SENSE_X, SENSE_Y = 54.45, 7.40

# Vias: (xy, net, role)
VIAS = [
    ((U_AT[0], U_AT[1]), "GND", "U5 under-body GND (hot-loop return + thermal)"),
    ((52.40, 11.60), "GND", "U5 pin 2 east GND"),
    ((52.40, 12.40), "GND", "U5 pin 2 east GND"),
    ((X_C41, 11.00), "GND", "C41 GND"),
    ((45.50, 10.00), "GND", "C40 GND"),
    ((46.50, 10.00), "GND", "C40 GND"),
    ((46.90, 9.40), "GND", "R41 GND (quiet FB reference)"),
    # Cout GND vias sit WEST of C42 in the cell frame: at the Rev 13 placement the
    # cell's x>50 / y>22 corner lies under J1's F.Cu pad rows, and a through via
    # there lands in the 0.4 mm DF40 pad field (first-pass DRC: GPIO3/GPIO4).
    ((49.20, 23.10), "GND", "Cout GND"),
    ((48.30, 23.10), "GND", "Cout GND"),
    ((46.00, 14.20), "+5V_IN", "VIN entry (PDN from USB-C / J1, next stage)"),
    ((46.90, 14.20), "+5V_IN", "VIN entry"),
    ((48.90, 19.80), "+3V3_SYS", "VOUT distribution tap"),
    ((48.90, 20.70), "+3V3_SYS", "VOUT distribution tap"),
    ((53.22, 10.70), "BUCK_EN_SRC", "EN source: F.Cu jumper landing from U6 RESET#"),
]


def tracks_plan():
    p, c = U_PIN, {k: (v[4], v[6]) for k, v in PARTS.items()}
    T = []   # (pts, net, width)
    # input: C40 VIN -> C41 VIN -> pin 4 (straight 0.6 mm bus, y = pin-4 row)
    T.append(([c["C40"][0], c["C41"][0], p["4"]], "+5V_IN", W_PIN))
    T.append(([c["C40"][0], (c["C40"][0][0], 14.20)], "+5V_IN", W_PWR))
    T.append(([(46.00, 14.20), (46.90, 14.20)], "+5V_IN", W_PWR))
    # GND: pin 2 -> under-body via, pin 2 -> east vias
    T.append(([p["2"], U_AT], "GND", 0.30))
    T.append(([p["2"], (52.40, 12.0)], "GND", 0.50))
    T.append(([(52.40, 11.60), (52.40, 12.40)], "GND", 0.40))
    T.append(([c["C41"][1], (X_C41, 11.00)], "GND", 0.40))
    T.append(([c["C40"][1], (46.00, 10.00)], "GND", W_PWR))
    T.append(([c["C40"][1], (X_C41, 11.00)], "GND", 0.40))      # bulk return -> C41 GND via (shared L3 entry)
    T.append(([(45.50, 10.00), (46.50, 10.00)], "GND", 0.40))
    T.append(([c["R41"][1], (46.90, 9.40)], "GND", 0.30))
    # LX: pin 3 straight south into L1 pad 1 (0.6 at the pin, 1.2 once clear of pin 2)
    T.append(([p["3"], (p["3"][0], 13.90)], "BUCK_LX", W_PIN))
    T.append(([(p["3"][0], 13.90), (p["3"][0], 15.30)], "BUCK_LX", W_LX))
    # VOUT: L1 pad 2 -> C42 / C43, C42 <-> C43, C42 -> distribution vias
    T.append(([(50.50, 18.30), c["C42"][0]], "+3V3_SYS", W_LX))
    T.append(([(52.70, 18.30), c["C43"][0]], "+3V3_SYS", W_LX))
    T.append(([c["C42"][0], c["C43"][0]], "+3V3_SYS", W_PWR))
    T.append(([c["C42"][0], (48.90, 20.25)], "+3V3_SYS", W_PWR))
    T.append(([(48.90, 19.80), (48.90, 20.70)], "+3V3_SYS", 0.40))
    # Cout GND -> vias
    T.append(([c["C42"][1], (50.50, 23.10)], "GND", W_PWR))
    T.append(([c["C43"][1], (52.70, 23.10)], "GND", W_PWR))
    T.append(([(48.30, 23.10), (52.70, 23.10)], "GND", W_PWR))
    # Kelvin sense: C43 VOUT pad (east side) -> east edge corridor -> R40 / C44 top
    T.append(([c["C43"][0], (SENSE_X, c["C43"][0][1]), (SENSE_X, SENSE_Y), c["C44"][1]], "+3V3_SYS", W_SIG))
    T.append(([c["C44"][1], c["R40"][1]], "+3V3_SYS", W_SIG))
    # FB spine: pin 5 north through R41 / R40 / C44 FB pads (away from LX)
    T.append(([p["5"], c["R41"][0], c["R40"][0], c["C44"][0]], "BUCK_FB", W_SIG))
    # EN: pin 1 north -> R42 / R43; R42 -> +3V3_OUT via
    T.append(([p["1"], (p["1"][0], 9.70), c["R42"][0]], "BUCK_EN", W_SIG))
    T.append(([c["R42"][0], c["R43"][0]], "BUCK_EN", W_SIG))
    T.append(([c["R42"][1], (53.22, 10.70)], "BUCK_EN_SRC", 0.30))
    return T


GND_ISLAND = [(45.20, 8.00), (54.40, 8.00), (54.40, 24.00), (45.20, 24.00)]   # B.Cu local GND pour (cell frame)


def _bbox(pts):
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    return (min(xs), min(ys), max(xs), max(ys))


CELL_BBOX = _bbox([xf(p) for p in [(44.85, 6.6), (54.6, 6.6), (54.6, 24.2), (44.85, 24.2)]]
                  + [(3.30, 6.65), (7.60, 9.85)])   # board frame: rotated cell + U6 pocket west of it
HOLES = {"H1": (3.5, 3.5), "H2": (51.5, 3.5), "H3": (3.5, 36.5), "H4": (51.5, 36.5)}       # 3.1 mm keep-out


# ==========================================================================
# 4. generate
# ==========================================================================
def _top_level(src: str):
    """Yield (start, end) spans of the top-level children of (kicad_pcb ...)."""
    i, depth, start, n = 0, 0, None, len(src)
    while i < n:
        ch = src[i]
        if ch == '"':                              # skip strings (may contain parens)
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
    """Remove the previous Rev 13 output at S-expression level (idempotent re-run).

    pcbnew-side removal of grouped items corrupts the KiCad 8 SWIG type table
    (later Footprints()/FootprintLoad() return raw SwigPyObjects), so the old
    U5/L1/C4x/R4x footprints, the REV13 group, its member tracks/vias and the
    BUCK_GND_BCU zone are cut from the text before the board is loaded."""
    refs = set(PARTS) | {U_REF, U6_REF}
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
        elif head.startswith("(zone") and '(name "BUCK_GND_BCU")' in blk:
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


def place(board, pcbnew, ref, fpname, value, at, pad_targets, nets, props, net):
    mm = pcbnew.FromMM
    V = lambda p: pcbnew.VECTOR2I(mm(p[0]), mm(p[1]))
    fp = pcbnew.FootprintLoad(str(LIB), fpname)
    fp.SetReference(ref)
    fp.SetValue(value)
    fp.SetFPIDAsString(f"{LIBNICK}:{fpname}")
    board.Add(fp)                          # Flip() needs the board (layer stack); unattached -> segfault
    fp.SetPosition(V(at))
    fp.Flip(V(at), True)                   # to B.Cu, mirrored left/right

    def err_():
        e = 0.0
        for pad in fp.Pads():
            q = pad.GetPosition()
            e = max(e, math.dist((pcbnew.ToMM(q.x), pcbnew.ToMM(q.y)), pad_targets[pad.GetNumber()]))
        return e
    err = err_()
    for _ in range(3):                     # rotate in 90 deg steps about the origin (board frame)
        if err <= 1e-3:
            break
        fp.Rotate(V(at), pcbnew.EDA_ANGLE(90.0, pcbnew.DEGREES_T))
        err = err_()
    if err > 1e-3:
        raise SystemExit(f"{ref}: cannot place pads on targets (err {err:.4f} mm)")
    for pad in fp.Pads():
        pad.SetNet(net(nets[pad.GetNumber()]))
    for k, v in props.items():
        fp.SetField(k, v)
        f = fp.GetFieldByName(k)
        if f:
            f.SetVisible(False)
    fp.Reference().SetVisible(False)   # B.Cu under a 55x40 module: refs on fab only
    fp.SetLocked(True)
    return fp


def cmd_generate(_a) -> int:
    import pcbnew
    mm = pcbnew.FromMM
    V = lambda p: pcbnew.VECTOR2I(mm(p[0]), mm(p[1]))
    for name, txt in footprints().items():
        (LIB / f"{name}.kicad_mod").write_text(txt)

    scratch = Path(tempfile.mkdtemp(prefix="rev13_"))
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

    place(board, pcbnew, U_REF, FP_U, "SY8089AAAC", xf(U_AT), {k: xf(v) for k, v in U_PIN.items()}, NETS_U,
          {"LCSC": U.lcsc, "MPN": U.mpn, "Manufacturer": "Silergy"}, net)
    for ref, (fpn, val, lcsc, mpn, p1, n1, p2, n2) in PARTS.items():
        p1, p2 = xf(p1), xf(p2)
        at = ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)
        place(board, pcbnew, ref, fpn, val, at, {"1": p1, "2": p2}, {"1": n1, "2": n2},
              {"LCSC": lcsc, "MPN": mpn}, net)

    place(board, pcbnew, U6_REF, FP_U6, "MAX809STRG 2.93V", U6_AT, U6_PIN, NETS_U6,
          {"LCSC": SUP.lcsc, "MPN": SUP.mpn, "Manufacturer": "onsemi"}, net)

    grp = pcbnew.PCB_GROUP(board)
    grp.SetName(GROUP)
    board.Add(grp)
    runs = [([xf(q) for q in pts], nn, w, "B.Cu") for pts, nn, w in tracks_plan()] + U6_TRACKS
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
    for xy, nn, _role in [(xf(q), n_, r_) for q, n_, r_ in VIAS] + U6_VIAS:
        v = pcbnew.PCB_VIA(board)
        v.SetPosition(V(xy))
        v.SetWidth(mm(VIA_D))
        v.SetDrill(mm(VIA_DRILL))
        v.SetViaType(pcbnew.VIATYPE_THROUGH)
        v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        v.SetNet(net(nn))
        board.Add(v)
        grp.AddItem(v)

    gnd = net("GND")
    z = pcbnew.ZONE(board)
    z.SetZoneName("BUCK_GND_BCU")
    z.SetLayer(pcbnew.B_Cu)
    z.SetNet(gnd)
    z.SetLocalClearance(mm(0.25))
    z.SetMinThickness(mm(0.20))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    z.SetAssignedPriority(5)
    ol = z.Outline()
    ol.NewOutline()
    for p in GND_ISLAND:
        q = xf(p)
        ol.Append(mm(q[0]), mm(q[1]))
    board.Add(z)
    # L2/L3 GND planes: the architecture puts GND on both inner layers. Pour them
    # under the HDMI stage's names only if no stage has poured them yet, so a later
    # hdmi_tmds.py / ethernet_mdi.py run replaces (not duplicates) them.
    have = {zz.GetLayer() for zz in board.Zones() if zz.GetNetname() == "GND" and zz.GetZoneName() != "BUCK_GND_BCU"}
    for lname, lid in (("GND_L2_REF", pcbnew.In1_Cu), ("GND_L3_REF", pcbnew.In2_Cu)):
        if lid in have:
            continue
        zz = pcbnew.ZONE(board)
        zz.SetZoneName(lname)
        zz.SetLayer(lid)
        zz.SetNet(gnd)
        zz.SetLocalClearance(mm(0.2))
        zz.SetMinThickness(mm(0.2))
        zz.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
        zz.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
        o2 = zz.Outline()
        o2.NewOutline()
        for p in [(0.3, 0.3), (54.7, 0.3), (54.7, 39.7), (0.3, 39.7)]:
            o2.Append(mm(p[0]), mm(p[1]))
        board.Add(zz)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    tb = board.GetTitleBlock()
    tb.SetComment(3, "Rev 13: U5 SY8089AAAC +3V3_SYS buck, 2.2uH/2x22uF, B.Cu east strip (tools/buck_3v3.py)")
    board.Save(str(PCB))
    write_project_rules()
    print(f"{U_REF}, {U6_REF} + {len(PARTS)} passives placed on B.Cu, {len(runs)} track runs, {len(VIAS) + len(U6_VIAS)} vias.")
    return 0


# ==========================================================================
# 5. Net class + DRC rules
# ==========================================================================
DRU_BEGIN = "# ---- BEGIN Rev 13 SY8089 buck (generated by tools/buck_3v3.py) ----"
DRU_END = "# ---- END Rev 13 SY8089 buck ----"
FB_LX_MIN = 1.0
VIN_LX_W_MIN = 0.55


def dru_text() -> str:
    return f"""{DRU_BEGIN}
# FB is a 18 kOhm node: keep it >= {FB_LX_MIN} mm from the 1 MHz / 5 V LX node (datasheet layout rule 4).
(rule "buck_fb_lx_isolation"
  (condition "A.NetName == 'BUCK_FB' && B.NetName == 'BUCK_LX'")
  (constraint clearance (min {FB_LX_MIN}mm)))

# High-current conductors of the power stage: no neck-downs below {VIN_LX_W_MIN} mm.
# Scoped to this stage's group so the J1 DF40 0.2 mm +5V_IN pad fan-out (other stages) is not caught.
(rule "buck_power_width"
  (condition "A.Type == 'Track' && A.memberOfGroup('{GROUP}') && (A.NetName == 'BUCK_LX' || A.NetName == '+5V_IN')")
  (constraint track_width (min {VIN_LX_W_MIN}mm)))
{DRU_END}
"""


def write_project_rules() -> None:
    txt = DRU.read_text()
    if DRU_BEGIN in txt:
        txt = re.sub(re.escape(DRU_BEGIN) + r".*?" + re.escape(DRU_END) + r"\n?", "", txt, flags=re.S)
    DRU.write_text(txt.rstrip() + "\n\n" + dru_text())
    pro = json.loads(PRO.read_text())
    ns = pro.setdefault("net_settings", {})
    classes = [c for c in ns.get("classes", []) if c.get("name") != "BUCK_POWER"]
    default = next((c for c in classes if c.get("name") == "Default"), {})
    nc = dict(default)
    nc.update(name="BUCK_POWER", clearance=0.2, track_width=W_PWR, via_diameter=VIA_D, via_drill=VIA_DRILL)
    classes.append(nc)
    ns["classes"] = classes
    pats = [p for p in (ns.get("netclass_patterns") or []) if p.get("netclass") != "BUCK_POWER"]
    pats += [{"netclass": "BUCK_POWER", "pattern": n} for n in ("+5V_IN", "+3V3_SYS", "BUCK_LX")]
    ns["netclass_patterns"] = pats
    PRO.write_text(json.dumps(pro, indent=2) + "\n")


# ==========================================================================
# 6. verify
# ==========================================================================
def poly_area(pts):
    return abs(sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(pts, pts[1:] + pts[:1]))) / 2


def seg_dist(p, a, b):
    ax, ay = b[0] - a[0], b[1] - a[1]
    L2 = ax * ax + ay * ay
    t = 0 if L2 == 0 else max(0, min(1, ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / L2))
    return math.dist(p, (a[0] + t * ax, a[1] + t * ay))


def seg_seg(a, b, c, d):
    return min(seg_dist(a, c, d), seg_dist(b, c, d), seg_dist(c, a, b), seg_dist(d, a, b))


def run_drc(pcb_path: Path) -> dict:
    with tempfile.TemporaryDirectory() as td:
        out = Path(td) / "drc.json"
        subprocess.run(["kicad-cli", "pcb", "drc", "--format", "json", "--severity-all", "--schematic-parity",
                        "-o", str(out), str(pcb_path)], capture_output=True, text=True)
        if not out.exists():
            subprocess.run(["kicad-cli", "pcb", "drc", "--format", "json", "--severity-all",
                            "-o", str(out), str(pcb_path)], capture_output=True, text=True)
        return json.loads(out.read_text())


def drc_in_scratch(text: str) -> dict:
    """DRC a mutated board next to copies of the project files. The Rev 13 block is
    (re-)inserted into the scratch .kicad_dru so the mutation tests exercise THIS
    stage's rule text even if a concurrent stage has dropped it from the shared file
    (that case is reported separately by the presence check)."""
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


BUCK_NETS = {"BUCK_LX", "BUCK_FB", "BUCK_EN", "BUCK_EN_SRC", "+3V3_SYS"}
BUCK_REFS = set(PARTS) | {U_REF, U6_REF}


def buck_item(v: dict) -> bool:
    s = json.dumps(v)
    return any(n in s for n in BUCK_NETS) or any(re.search(rf"\b{r}\b", s) for r in BUCK_REFS)


def cmd_verify(a) -> int:
    import pcbnew
    tm = pcbnew.ToMM
    board = pcbnew.LoadBoard(str(PCB))
    rows = []

    def add_(name, ok, detail=""):
        rows.append((name, bool(ok), detail))

    fps = {f.GetReference(): f for f in board.GetFootprints()}
    missing = sorted(BUCK_REFS - set(fps))
    add_(f"All {len(BUCK_REFS)} parts present (U5, U6, L1, C40-C44, R40-R43)", not missing,
         f"missing {missing}" if missing else f"{len(BUCK_REFS)}/{len(BUCK_REFS)}")
    if missing:
        return report(rows, a)
    P = lambda pad: (round(tm(pad.GetPosition().x), 4), round(tm(pad.GetPosition().y), 4))
    pads = {(r, p.GetNumber()): (P(p), p.GetNetname(), p) for r in BUCK_REFS for p in fps[r].Pads()}

    # pinout / nets
    want = {(U_REF, k): v for k, v in NETS_U.items()}
    want.update({(U6_REF, k): v for k, v in NETS_U6.items()})
    for ref, (_f, _v, _l, _m, _p1, n1, _p2, n2) in PARTS.items():
        want[(ref, "1")], want[(ref, "2")] = n1, n2
    bad = [f"{r}.{n}={pads[(r, n)][1]}!={w}" for (r, n), w in want.items() if pads[(r, n)][1] != w]
    add_(f"Pad-to-net map matches SY8089 / MAX809 pinouts + schematic intent ({len(want)} pads)", not bad,
         "; ".join(bad) or f"{len(want)}/{len(want)}")
    add_("All power-stage parts on B.Cu (top side is under the CM4 module)",
         all(fps[r].GetLayer() == pcbnew.B_Cu for r in BUCK_REFS), "")
    pos_err = max([math.dist(pads[(U_REF, k)][0], xf(v)) for k, v in U_PIN.items()]
                  + [math.dist(pads[(U6_REF, k)][0], v) for k, v in U6_PIN.items()])
    add_("U5 / U6 pin positions = plan (mirror-correct flip)", pos_err < 1e-3, f"max err {pos_err*1e3:.1f} um")
    # courtyard must enclose the footprint's own copper (KiCad has no DRC for this)
    leak = []
    for r in BUCK_REFS:
        cb = fps[r].GetCourtyard(pcbnew.B_CrtYd).BBox()
        for pd in fps[r].Pads():
            pb = pd.GetBoundingBox()
            if pb.GetLeft() < cb.GetLeft() or pb.GetRight() > cb.GetRight() or pb.GetTop() < cb.GetTop() \
                    or pb.GetBottom() > cb.GetBottom():
                leak.append(f"{r}.{pd.GetNumber()}")
    add_("Every courtyard encloses its own pads (KiCad does not check this)", not leak, ", ".join(leak) or "all")

    # --- hot loop geometry
    def edge_gap(r1, n1, r2, n2):
        b1 = pads[(r1, n1)][2].GetBoundingBox()
        b2 = pads[(r2, n2)][2].GetBoundingBox()
        dx = max(0, max(tm(b1.GetLeft()), tm(b2.GetLeft())) - min(tm(b1.GetRight()), tm(b2.GetRight())))
        dy = max(0, max(tm(b1.GetTop()), tm(b2.GetTop())) - min(tm(b1.GetBottom()), tm(b2.GetBottom())))
        return math.hypot(dx, dy)
    g41 = edge_gap("C41", "1", U_REF, "4")
    g40 = edge_gap("C40", "1", U_REF, "4")
    add_("C41 (100 nF HF) pad edge <= 0.5 mm from U5 IN (pin 4)", g41 <= 0.5, f"{g41:.3f} mm")
    add_("C40 (22 uF bulk) pad edge <= 2.5 mm from U5 IN (pin 4)", g40 <= 2.5, f"{g40:.3f} mm")
    vias = [(round(tm(v.GetPosition().x), 4), round(tm(v.GetPosition().y), 4), v.GetNetname())
            for v in board.GetTracks() if v.GetClass() == "PCB_VIA"]
    has_via = lambda xy, n: any(math.dist((x, y), xy) < 1e-3 and nn == n for x, y, nn in vias)
    add_("U5 GND has an under-body via to L3 (hot-loop return directly beneath the IC)", has_via(xf(U_AT), "GND"),
         str(xf(U_AT)))
    # HF loop: C41+ -> pin4 -> (die) -> pin2 -> under-body via -> L3 -> C41 GND via -> C41-
    c41v, c41g = pads[("C41", "1")][0], pads[("C41", "2")][0]
    hf = [c41v, pads[(U_REF, "4")][0], pads[(U_REF, "2")][0], xf(U_AT), xf((X_C41, 11.00)), c41g]
    bulk = [pads[("C40", "1")][0], pads[(U_REF, "4")][0], pads[(U_REF, "2")][0], xf(U_AT), xf((X_C41, 11.00)),
            pads[("C40", "2")][0]]
    a_hf, a_bulk = poly_area(hf), poly_area(bulk)
    h = STACKUP.h_pp + STACKUP.t_outer
    per = lambda pts: sum(math.dist(p, q) for p, q in zip(pts, pts[1:] + pts[:1]))
    # Loop inductance estimate: plan-view flux (mu0 * A / w_eff) is an UPPER bound for
    # the outer-layer part; return in the L3 GND partition at h = h_pp + t_cu (0.135 mm on 2313) adds mu0*h*len/w.
    w_eff = 0.6
    l_hf = MU0 * (a_hf * 1e-6) / (w_eff * 1e-3) * 0.5 + MU0 * h * 1e-3 * per(hf) / 2 * 1e-3 / (w_eff * 1e-3) \
        + 2 * MU0 / (2 * math.pi) * h * 1e-3 * (math.log(4 * h / VIA_D) + 1 if 4 * h > VIA_D else 0.3)
    add_("HF input loop plan-view area <= 4.0 mm^2 (C41 / IN / GND / under-body via)", a_hf <= 4.0,
         f"{a_hf:.2f} mm^2, perimeter {per(hf):.2f} mm, est. L_loop ~{l_hf*1e9:.2f} nH (excl. package)")
    add_("Bulk input loop plan-view area <= 8.0 mm^2 (C40)", a_bulk <= 8.0, f"{a_bulk:.2f} mm^2")

    # --- per-net copper: LX area, FB / LX separation, widths
    segs = {}
    for t in board.GetTracks():
        if t.GetClass() == "PCB_TRACK" and t.GetNetname() in BUCK_NETS | {"+5V_IN", "GND", "+3V3_OUT"}:
            s_ = (tm(t.GetStart().x), tm(t.GetStart().y))
            e_ = (tm(t.GetEnd().x), tm(t.GetEnd().y))
            segs.setdefault(t.GetNetname(), []).append((s_, e_, tm(t.GetWidth()), t.GetLayerName()))
    lx_area = sum(math.dist(s_, e_) * w for s_, e_, w, _ in segs.get("BUCK_LX", []))
    lx_area += sum(tm(p.GetSize().x) * tm(p.GetSize().y) for (r, n), (_xy, nn, p) in pads.items() if nn == "BUCK_LX")
    add_("LX node copper area <= 12 mm^2 (pin 3 + trace + L1 pad; datasheet rule 3)", lx_area <= 12.0, f"{lx_area:.2f} mm^2")
    lx_len = sum(math.dist(s_, e_) for s_, e_, _w, _l in segs.get("BUCK_LX", []))
    add_("LX trace length <= 3.0 mm", lx_len <= 3.0, f"{lx_len:.2f} mm")
    def box_of(pd):
        b_ = pd.GetBoundingBox()
        return (tm(b_.GetLeft()), tm(b_.GetTop()), tm(b_.GetRight()), tm(b_.GetBottom()))

    def seg_box(a_, b_, bx):
        L_, T_, R_, B_ = bx
        ins = lambda q: L_ <= q[0] <= R_ and T_ <= q[1] <= B_
        if ins(a_) or ins(b_):
            return 0.0
        cs = [(L_, T_), (R_, T_), (R_, B_), (L_, B_)]
        ed = list(zip(cs, cs[1:] + cs[:1]))
        cr = lambda p_, q_, r_: (r_[1] - p_[1]) * (q_[0] - p_[0]) - (q_[1] - p_[1]) * (r_[0] - p_[0])
        for c_, d_ in ed:
            if cr(a_, b_, c_) * cr(a_, b_, d_) < 0 and cr(c_, d_, a_) * cr(c_, d_, b_) < 0:
                return 0.0
        return min(seg_seg(a_, b_, c_, d_) for c_, d_ in ed)

    def box_box(b1, b2):
        dx = max(0.0, max(b1[0], b2[0]) - min(b1[2], b2[2]))
        dy = max(0.0, max(b1[1], b2[1]) - min(b1[3], b2[3]))
        return math.hypot(dx, dy)

    def min_gap(net_a, net_b, a_filter=lambda w: True):
        """True minimum copper gap between two nets on B.Cu: track-track, track-pad, pad-pad."""
        ta = [x for x in segs.get(net_a, []) if x[3] == "B.Cu" and a_filter(x[2])]
        tb = [x for x in segs.get(net_b, []) if x[3] == "B.Cu"]
        pa = [box_of(p) for (_r, _n), (_xy, nn, p) in pads.items() if nn == net_a] if a_filter(0.0) else []
        pb = [box_of(p) for (_r, _n), (_xy, nn, p) in pads.items() if nn == net_b]
        g = 99.0
        for s1, e1, w1, _ in ta:
            g = min([g] + [seg_seg(s1, e1, s2, e2) - w1 / 2 - w2 / 2 for s2, e2, w2, _l in tb]
                    + [seg_box(s1, e1, bx) - w1 / 2 for bx in pb])
        for bx in pa:
            g = min([g] + [seg_box(s2, e2, bx) - w2 / 2 for s2, e2, w2, _l in tb] + [box_box(bx, by) for by in pb])
        return g
    fb_lx = min_gap("BUCK_FB", "BUCK_LX")
    add_(f"FB copper (tracks AND pads) >= {FB_LX_MIN} mm from any LX copper (true segment/box distance)",
         fb_lx >= FB_LX_MIN, f"{fb_lx:.2f} mm")
    fb_len = sum(math.dist(s_, e_) for s_, e_, _w, _l in segs.get("BUCK_FB", []))
    add_("FB node trace length <= 5 mm (pin 5 -> divider)", fb_len <= 5.0, f"{fb_len:.2f} mm")
    sense = segs.get("+3V3_SYS", [])
    c43 = pads[("C43", "1")][0]
    add_("VOUT sensed at C43 pad (Kelvin, after the output caps), not at L1",
         any(math.dist(s_, c43) < 1e-3 and w <= 0.25 for s_, e_, w, _ in sense), "")
    sense_lx = min_gap("+3V3_SYS", "BUCK_LX", a_filter=lambda w: 0 < w <= 0.25)   # 0.2 mm sense tracks only
    add_("Sense trace >= 0.7 mm from LX copper", sense_lx >= 0.7, f"{sense_lx:.2f} mm")
    w_min = min((w for n in ("+5V_IN", "BUCK_LX") for _s, _e, w, _l in segs.get(n, [])), default=0)
    add_("+5V_IN / LX tracks >= 0.6 mm (2.4 A peak, 1 oz outer: < 10 C rise)", w_min >= 0.6 - 1e-6, f"min {w_min:.2f} mm")
    # vias by NET inside the stage envelope (not by matching the plan's own coordinates)
    inbox = lambda x, y: CELL_BBOX[0] - 0.5 <= x <= CELL_BBOX[2] + 0.5 and CELL_BBOX[1] - 0.5 <= y <= CELL_BBOX[3] + 0.5
    vdrill = {(round(tm(v.GetPosition().x), 4), round(tm(v.GetPosition().y), 4)): tm(v.GetDrillValue())
              for v in board.GetTracks() if v.GetClass() == "PCB_VIA"}
    nvia = lambda n: sum(1 for x, y, nn in vias if nn == n and inbox(x, y) and vdrill[(x, y)] >= 0.3 - 1e-6)
    add_("Via count by net in the stage envelope (0.3 mm drill): +5V_IN >= 2, +3V3_SYS >= 2 hand-off, GND >= 8",
         nvia("+5V_IN") >= 2 and nvia("+3V3_SYS") >= 2 and nvia("GND") >= 8,
         f"+5V_IN {nvia('+5V_IN')}, +3V3_SYS {nvia('+3V3_SYS')}, GND {nvia('GND')} "
         "(current capacity applies once the PDN stage connects the hand-offs)")

    # --- envelope / reservations
    bb = None
    for r in BUCK_REFS:
        cb = fps[r].GetCourtyard(pcbnew.B_CrtYd).BBox()
        box = (tm(cb.GetLeft()), tm(cb.GetTop()), tm(cb.GetRight()), tm(cb.GetBottom()))
        bb = box if bb is None else (min(bb[0], box[0]), min(bb[1], box[1]), max(bb[2], box[2]), max(bb[3], box[3]))
    inside = CELL_BBOX[0] <= bb[0] and CELL_BBOX[1] <= bb[1] and bb[2] <= CELL_BBOX[2] and bb[3] <= CELL_BBOX[3]
    add_("All courtyards inside the cell envelope", inside,
         f"courtyard bbox {tuple(round(x, 2) for x in bb)} in {tuple(round(x, 2) for x in CELL_BBOX)}")
    on_board = bb[0] >= 0.3 and bb[1] >= 0.3 and bb[2] <= 54.7 and bb[3] <= 39.7
    add_("Courtyards >= 0.3 mm inside the 55 x 40 outline", on_board, "")
    # Live check against every OTHER stage: foreign B.Cu footprints / copper / vias inside our envelope (+0.3 mm)
    grp_ids = {it.m_Uuid.AsString() for g in board.Groups() if g.GetName() == GROUP for it in g.GetItems()}
    ex = (bb[0] - 0.3, bb[1] - 0.3, bb[2] + 0.3, bb[3] + 0.3)
    hit = lambda b_: not (tm(b_.GetRight()) <= ex[0] or ex[2] <= tm(b_.GetLeft()) or tm(b_.GetBottom()) <= ex[1]
                          or ex[3] <= tm(b_.GetTop()))
    foreign = []
    for f in board.GetFootprints():
        if f.GetReference() in BUCK_REFS:
            continue
        if f.GetLayer() == pcbnew.B_Cu and hit(f.GetCourtyard(pcbnew.B_CrtYd).BBox()):
            foreign.append(f"{f.GetReference()} courtyard")
        foreign += [f"{f.GetReference()}.{pd.GetNumber()}" for pd in f.Pads()
                    if pd.IsOnLayer(pcbnew.B_Cu) and pd.GetAttribute() != pcbnew.PAD_ATTRIB_NPTH and hit(pd.GetBoundingBox())]
    for t in board.GetTracks():
        if t.m_Uuid.AsString() in grp_ids:
            continue
        if (t.GetClass() == "PCB_VIA" or t.GetLayer() == pcbnew.B_Cu) and hit(t.GetBoundingBox()):
            foreign.append(f"{t.GetClass()}[{t.GetNetname()}]@({tm(t.GetPosition().x):.2f},{tm(t.GetPosition().y):.2f})")
    add_("No other stage's B.Cu courtyard / pad / track / via inside the cell envelope (+0.3 mm)", not foreign,
         "clear" if not foreign else "; ".join(sorted(set(foreign)))[:400])
    def cy_box(r):
        c_ = fps[r].GetCourtyard(pcbnew.B_CrtYd).BBox()
        return (tm(c_.GetLeft()), tm(c_.GetTop()), tm(c_.GetRight()), tm(c_.GetBottom()))
    hd, hwho = 99.0, ""
    for r in BUCK_REFS:
        bx = cy_box(r)
        for hn, hc in HOLES.items():
            dd = math.dist(hc, (min(max(hc[0], bx[0]), bx[2]), min(max(hc[1], bx[1]), bx[3])))
            if dd < hd:
                hd, hwho = dd, f"{r} to {hn}"
    add_("Every courtyard clears every M2.5 standoff circle (r 3.1 mm)", hd >= 3.1, f"min {hd:.2f} mm ({hwho})")
    zn = {(z.GetZoneName(), board.GetLayerName(z.GetFirstLayer())) for z in board.Zones() if z.GetNetname() == "GND"}
    l3_ok = any(z.GetNetname() == "GND" and z.GetFirstLayer() == pcbnew.In2_Cu and z.HitTestFilledArea(
        pcbnew.In2_Cu, pcbnew.VECTOR2I(pcbnew.FromMM(xf(U_AT)[0] + 1.2), pcbnew.FromMM(xf(U_AT)[1]))) for z in board.Zones())
    add_("GND planes on L2 + L3 and local B.Cu GND island present", any(l_ == "In1.Cu" for _n, l_ in zn)
         and any(l_ == "In2.Cu" for _n, l_ in zn) and ("BUCK_GND_BCU", "B.Cu") in zn, ", ".join(sorted(f"{n}@{l_}" for n, l_ in zn)))
    add_("L3 GND copper is actually filled beneath U5 (hot-loop return plane present)", l3_ok, "")
    l3z = [z for z in board.Zones() if z.GetNetname() == "GND" and z.GetFirstLayer() == pcbnew.In2_Cu]
    orphan = [xf(xy) for xy, n_, _r in VIAS if n_ == "GND" and not any(
        z.HitTestFilledArea(pcbnew.In2_Cu, pcbnew.VECTOR2I(pcbnew.FromMM(xf(xy)[0]), pcbnew.FromMM(xf(xy)[1])))
        for z in l3z)]
    add_("Every buck GND via lands in filled L3 GND (no via inside another stage's plane notch)", not orphan,
         "all" if not orphan else f"no L3 under {orphan}")
    shield = [z for z in board.Zones() if "SHIELD" in z.GetNetname().upper()]
    if shield:
        # copper extent of this stage: pads + tracks + vias + the B.Cu GND island outline
        cu = [xf(q) for q in GND_ISLAND]
        for (_r, _n), (_xy, _nn, pd) in pads.items():
            b_ = pd.GetBoundingBox()
            cu += [(tm(b_.GetLeft()), tm(b_.GetTop())), (tm(b_.GetRight()), tm(b_.GetBottom()))]
        for t in board.GetTracks():
            if t.m_Uuid.AsString() in grp_ids:
                b_ = t.GetBoundingBox()
                cu += [(tm(b_.GetLeft()), tm(b_.GetTop())), (tm(b_.GetRight()), tm(b_.GetBottom()))]
        cb = _bbox(cu)
        sb = [z.GetBoundingBox() for z in shield]
        gap = min(max(tm(b_.GetLeft()) - cb[2], cb[0] - tm(b_.GetRight()), tm(b_.GetTop()) - cb[3],
                      cb[1] - tm(b_.GetBottom())) for b_ in sb)
        add_("Buck COPPER >= 2.0 mm from any *SHIELD* zone (Ethernet chassis isolation gap)", gap >= 2.0,
             f"{gap:.2f} mm to {', '.join(sorted({z.GetNetname() for z in shield}))}")

    dtxt, ptxt = DRU.read_text(), PRO.read_text()
    add_("Rev 13 DRC block + BUCK_POWER net class present in .kicad_dru / .kicad_pro",
         DRU_BEGIN in dtxt and '"BUCK_POWER"' in ptxt, "run `buck_3v3.py rules` if another stage dropped them")

    # --- design calc must also pass
    calc_res = calc()
    nfail = sum(not c_["ok"] for c_ in calc_res["checks"])
    add_(f"Electrical design checks ({len(calc_res['checks'])}) all pass", nfail == 0, f"{nfail} failing")

    # --- kicad-cli DRC + mutation tests
    drc = None
    if not a.no_kicad and shutil.which("kicad-cli"):
        drc = run_drc(PCB)
        v = drc.get("violations", [])
        # Hand-offs = vias deliberately left for the next (PDN / USB-C) stage. Keyed by name so each is reported.
        handoff = {(round(xy[0], 2), round(xy[1], 2)): (n_, why)
                   for xy, n_, why in [(xf(q), n_, r_) for q, n_, r_ in VIAS] + U6_VIAS
                   if n_ in ("+5V_IN", "+3V3_SYS", "+3V3_OUT")}
        pos_of = lambda x: (round(x["items"][0]["pos"]["x"], 2), round(x["items"][0]["pos"]["y"], 2))
        is_handoff = lambda x: x["type"] == "via_dangling" and len(x["items"]) == 1 and pos_of(x) in handoff
        bv = [x for x in v if buck_item(x) or any(
            CELL_BBOX[0] <= it["pos"]["x"] <= CELL_BBOX[2] and CELL_BBOX[1] <= it["pos"]["y"] <= CELL_BBOX[3]
            for it in x["items"])]
        real = [x for x in bv if not is_handoff(x)]
        add_("kicad-cli DRC: 0 violations touching the buck stage / cell (excl. documented hand-off vias)", not real,
             "0" if not real else "; ".join(sorted({f"{x['type']}: {x['description']}" for x in real}))[:700])
        open_ho = sorted({handoff[pos_of(x)] for x in bv if is_handoff(x)})
        add_("Open hand-offs (informational): " + ", ".join(f"{n}" for n, _w in open_ho), True,
             "; ".join(f"{n}: {w}" for n, w in open_ho) or "none")
        # Fails once another stage starts routing a hand-off net but leaves OUR via unconnected
        # (e.g. J1 +3V3_OUT routed but U6 VCC open -> R43 holds EN low -> +3V3_SYS never starts).
        foreign_nets = {t.GetNetname() for t in board.GetTracks()
                        if t.GetClass() == "PCB_TRACK" and t.m_Uuid.AsString() not in grp_ids}
        stale = [n for n, _w in open_ho if n in foreign_nets]
        add_("No hand-off left open on a net another stage has already routed", not stale,
             "; ".join(f"{n} routed elsewhere but buck via still dangling" for n in stale) or "consistent")
        uc = drc.get("unconnected_items", [])
        internal = []
        for x in uc:
            its = x.get("items", [])
            refs_in = [it for it in its if any(re.search(rf"\b{r}\b", it.get("description", "")) for r in BUCK_REFS)
                       or "REV13" in it.get("description", "") or "Via" in it.get("description", "") and any(
                n in it.get("description", "") for n in BUCK_NETS)]
            if len(its) >= 2 and len(refs_in) == len(its) and any(n in json.dumps(x) for n in BUCK_NETS | {"GND"}):
                internal.append(x)
        add_("No unconnected items internal to the power stage (only the 2 documented inter-stage links remain)",
             not internal, f"{len(internal)} internal; {len([x for x in uc if buck_item(x) or '+5V_IN' in json.dumps(x)])} "
             "touching buck nets (J1 +5V_IN / +3V3_OUT / GND fan-out = next PDN stage)")
        src = PCB.read_text()
        # net codes from the FILE text (pcbnew renumbers nets on load, so its codes can differ)
        fbcode = int(re.search(r'\(net (\d+) "BUCK_FB"\)', src).group(1))
        vincode = int(re.search(r'\(net (\d+) "\+5V_IN"\)', src).group(1))
        tail = src.rstrip()
        assert tail.endswith(")")
        lx_x = U_PIN["3"][0]
        seg_ = lambda a_, b_, w, code, k: (f'  (segment (start {xf(a_)[0]:.4f} {xf(a_)[1]:.4f}) (end {xf(b_)[0]:.4f} '
                                          f'{xf(b_)[1]:.4f}) (width {w}) (layer "B.Cu") (net {code}) '
                                          f'(uuid "00000000-0000-0000-0000-0000000d000{k}"))\n)\n')
        # Mutant stubs START on copper of their own net: KiCad re-nets a floating track
        # that lies inside another net's zone fill on load (it would silently become GND).
        p5 = U_PIN["5"]

        def in_group(text, k):
            """Add the injected item's uuid to the REV13 group's member list (the width rule is group-scoped)."""
            m = re.search(r'\(group "' + GROUP + r'".*?\(members', text, re.S)
            return text[:m.end()] + f' "00000000-0000-0000-0000-0000000d000{k}"' + text[m.end():]
        muts = {
            "FB stub from U5 pin 5 to 0.7 mm of LX copper -> rule buck_fb_lx_isolation": (
                ("buck_fb_lx_isolation", "clearance", f"{FB_LX_MIN:.4f} mm"), 1, tail[:-1] + seg_(p5, (49.70, 12.40), 0.2, fbcode, 1)),
            "+5V_IN 0.3 mm neck-down (member of REV13 group) from the VIN entry via -> rule buck_power_width": (
                ("buck_power_width", "track_width", f"{VIN_LX_W_MIN:.4f} mm"), 2,
                in_group(tail[:-1] + seg_((46.00, 14.20), (46.00, 15.30), 0.3, vincode, 2), 2)),
        }
        # Negative control: an UNgrouped 0.2 mm +5V_IN stub on a J1 DF40 pad must NOT trip buck_power_width.
        j1p = re.search(r'\(pad "77" smd \w+\s*\(at ([\d.\-]+) ([\d.\-]+)', src)
        jpad = next((pd for pd in fps["J1"].Pads() if pd.GetNumber() == "77"), None) if "J1" in fps else None
        if jpad is not None:
            jx, jy = tm(jpad.GetPosition().x), tm(jpad.GetPosition().y)
            neg = tail[:-1] + (f'  (segment (start {jx:.4f} {jy:.4f}) (end {jx:.4f} {jy - 0.5:.4f}) (width 0.2) '
                               f'(layer "F.Cu") (net {vincode}) (uuid "00000000-0000-0000-0000-0000000d0003"))\n)\n')
            dn = drc_in_scratch(neg)
            hn = [x for x in dn.get("violations", []) if "0000000d0003" in json.dumps(x) and x["type"] == "track_width"
                  and f"{VIN_LX_W_MIN:.4f} mm" in x["description"]]
            add_("NEGATIVE CONTROL: ungrouped 0.2 mm +5V_IN J1 fan-out NOT caught by buck_power_width", not hn,
                 f"{len(hn)} width violations on the control stub")
        for name, (want_rule, k, text) in muts.items():
            d = drc_in_scratch(text)
            tag = f"0000000d000{k}"
            hits_ = [x for x in d.get("violations", []) if tag in json.dumps(x)]
            # KiCad names custom clearance rules in the text, but reports custom track_width
            # limits as "board setup constraints min width <rule value>": accept the rule's value.
            rule_n, typ_, val_ = want_rule
            ok = any(rule_n in x["description"] or (x["type"] == typ_ and val_ in x["description"]) for x in hits_)
            add_(f"MUTATION caught: {name}", ok,
                 "; ".join(sorted({f"{x['type']}: {x['description'][:70]}" for x in hits_})) or "no violation on injected item")
    return report(rows, a, drc, calc_res, dict(a_hf=a_hf, a_bulk=a_bulk, l_hf=l_hf, lx_area=lx_area, g41=g41, g40=g40))


def report(rows, a, drc=None, calc_res=None, geo=None) -> int:
    npass = sum(ok for _n, ok, _d in rows)
    lines = ["# Rev 13 - SY8089AAAC +3V3_SYS buck: layout verification report", "",
             "Generated by `python3 tools/buck_3v3.py verify` from the saved `.kicad_pcb` (measured, not planned).", "",
             f"**{npass}/{len(rows)} checks pass.**", "", "| # | Check | Result | Detail |", "|---|---|---|---|"]
    for i, (n, ok, d) in enumerate(rows, 1):
        lines.append(f"| {i} | {n} | {'PASS' if ok else '**FAIL**'} | {d} |")
    if calc_res:
        lines += ["", "## Design calculation checks", "", "| Check | Result | Detail |", "|---|---|---|"]
        for c_ in calc_res["checks"]:
            lines.append(f"| {c_['name']} | {'PASS' if c_['ok'] else '**FAIL**'} | {c_['detail']} |")
        lines += ["", "## Thermal derating (TJ <= 125 C)", "", "| TA (C) | I_max @ 170 C/W (datasheet) | I_max @ 115 C/W (board est.) |",
                  "|---|---|---|"]
        for ta, (x, y) in calc_res["thermal"]["derate"].items():
            lines.append(f"| {ta} | {x:.2f} A | {y:.2f} A |")
    if drc is not None:
        lines += ["", f"kicad-cli DRC (whole board): {len(drc.get('violations', []))} violations, "
                      f"{len(drc.get('unconnected_items', []))} unconnected items (board-wide; most are the J1/J2 fan-out "
                      "owned by later stages)."]
    REPORT.parent.mkdir(exist_ok=True)
    REPORT.write_text("\n".join(lines) + "\n")
    for i, (n, ok, d) in enumerate(rows, 1):
        print(f"  [{'PASS' if ok else 'FAIL'}] {n}: {d}")
    print(f"\n{npass}/{len(rows)} pass -> {REPORT.relative_to(ROOT)}")
    return 0 if npass == len(rows) else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest="cmd", required=True)
    sp.add_parser("calc").set_defaults(fn=cmd_calc)
    sp.add_parser("generate").set_defaults(fn=cmd_generate)
    sp.add_parser("rules", help="re-apply only the BUCK_POWER net class + Rev 13 DRC block "
                  "(other stages rewrite .kicad_pro/.kicad_dru)").set_defaults(fn=lambda _a: write_project_rules() or 0)
    v = sp.add_parser("verify")
    v.add_argument("--no-kicad", action="store_true", help="skip kicad-cli DRC + mutation tests")
    v.set_defaults(fn=cmd_verify)
    a = ap.parse_args()
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
