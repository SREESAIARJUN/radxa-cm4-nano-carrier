#!/usr/bin/env python3
"""
Radxa CM4 Nano Carrier - Rev 15: 15 W continuous-load thermal architecture study
==================================================================================

Heat-pipe vs. solid thermal-pedestal dissipation modelling for the CM4-form-factor
compute module (Radxa CM4 / RK3576) running a *continuous* 15 W compute load on
this 55 x 40 mm carrier.

Sub-commands
------------
  selftest  Validate the numerical machinery against independent references
            (exit 1 on failure): 3D finite-difference conduction solver vs.
            exact 1D slab; closed-form rectangular flux-channel spreading
            series vs. the 3D FD solver; Bar-Cohen/Rohsenow optimum fin
            spacing vs. brute-force scan; heat-pipe capillary limit vs.
            published 6 mm sintered-pipe capacity band; network energy balance.
  calc      Full study (no KiCad needed). Evaluates four concepts at three
            ambient corners and two load distributions, solves the coupled
            junction / module / carrier / sink network with temperature-
            dependent convection + radiation, runs transients, heat-pipe
            operating-limit envelope (capillary incl. adverse tilt, viscous,
            sonic, entrainment, boiling) at cold start and at operating
            vapour temperature, TIM tolerance stack, sizing sweeps, then
            checks DF40 receptacle temperature and the Rev 13 SY8089 buck
            derating against the carrier temperature each concept produces.
            Writes artifacts/rev15_thermal_design.json and
            artifacts/rev15_thermal_report.md. Exit 1 if a gating check fails.
  cad       Write cad/rev15_thermal_concepts.scad (parametric, values from the
            calc) and, if `openscad` is installed, render
            stl/rev15_pedestal_sink.stl and stl/rev15_heatpipe_assembly.stl.

Concepts (all evaluated, recommendation is computed - not hard-coded)
--------------------------------------------------------------------
  A1  Rigid integral Al pedestal (gap pad) + 55 x 40 footprint sink, natural
  A2  Rigid integral Al pedestal (gap pad) + 130 x 95 overhanging sink, natural
      (same dissipator as B1 -> isolates the pedestal-vs-pipe geometry)
  A3  Rigid integral Al pedestal (gap pad) + 55 x 40 footprint sink + 30 mm fan
  A4  Spring-loaded (floating) Al pedestal sink (phase-change TIM, same TIM
      as B1) + 130 x 95 overhanging sink, natural -> isolates the TIM effect
  A5  Spring-loaded Al pedestal sink (phase-change) + 55 x 40 sink + 30 mm fan
  B1  Cu evaporator block + 6 mm sintered Cu/water heat pipe (flattened 2.5 mm)
      -> 130 x 95 remote/enclosure-wall fin sink, natural convection

Sources [D] and engineering assumptions [A] (assumptions are swept / flagged)
----------------------------------------------------------------------------
[D1] Hirose DF40 catalog: operating temperature -35..+85 C INCLUDING
     temperature rise from current; contact Cu alloy.
[D2] Rockchip RK3576 Linux DT (rk3576.dtsi soc-thermal): passive trip 85 C,
     critical 115 C.  -> continuous-load design target TJ <= 85 C (no throttle).
[D3] Bar-Cohen & Rohsenow, J. Heat Transfer 106 (1984): isothermal symmetric
     vertical parallel-plate channel correlation + optimum spacing 2.714 P^-1/4.
[D4] Teertstra, Yovanovich & Culham, IEEE CPT 23(2000): forced convection
     parallel-plate composite (developing + fully developed) Nusselt model.
[D5] Churchill & Chu (1975) vertical-plate laminar natural convection.
[D6] Muzychka, Culham & Yovanovich, J. Electronic Packaging 125 (2003):
     rectangular flux channel spreading - re-derived here from the Fourier
     cosine series (see spreading_rect()), validated against the FD solver.
[D7] Faghri, Heat Pipe Science & Technology (2nd ed.); Chi (1976):
     sintered wick r_eff = 0.21 d_p, Blake-Kozeny K, Busse viscous limit,
     Levy sonic limit, entrainment limit; saturated-water property table
     (IAPWS-IF97 values rounded).
[D8] Rev 13 tools/buck_3v3.py calc(): SY8089 losses + derating vs. TA.
[A1] RK3576 lid-less (exposed-die) FCBGA: die heat footprint 8.0 x 8.0 mm,
     package 15 x 15 mm, thetaJC 0.25 K/W, thetaJB 6.0 K/W. NOT published by
     Rockchip/Radxa -> swept 0.5x..2x in the sensitivity table. Verify on the
     Radxa CM4 module drawing / thermal test die before tooling.
[A2] SoC centre on the module = (27.5, 20.0) in carrier coordinates (module
     centre). Tallest module top-side part 3.0 mm above module PCB top.
[A3] 6 mm sintered Cu/water pipe: wall 0.30 mm, wick 0.50 mm, powder d_p 50 um,
     porosity 0.50; flattening factor 0.75, 0.95 per 90 deg bend, x1.5 safety.
     Evaporator / condenser area-specific film resistance 0.20 / 0.10 K cm^2/W.
[A4] 30 x 30 x 10 mm 5 V axial fan class: free-air 1.5e-3 m^3/s (3.2 CFM),
     shut-off 20 Pa, linear P-Q; fully ducted over the fins.
[A5] TIMs: gap pad k 6.0 W/mK, 1.0 mm free height, usable compression
     10..50 %, ~0.14 MPa at 50 %; phase-change (spring-loaded) 0.03 mm BLT,
     0.08 K cm^2/W.
[A7] DRAM + PMIC carry the non-SoC module heat; each gets a gap pad to the
     dissipator (theta junction->top 6 K/W, 10 x 14 mm pad). Positions are
     module-specific -> confirm on the Radxa CM4 module drawing.
[A6] Mass the compute module may carry through its 4 M2.5 holes without
     overloading the DF40 pair / module PCB in a 50 g, 11 ms shock: 60 g.
"""
from __future__ import annotations

import argparse
import json
import math
import shutil
import subprocess
import sys
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path

import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))
from mdi_impedance import STACKUP  # noqa: E402  single stackup source of truth
from cm4_mechanical import BOARD_W, BOARD_H, HOLES, STACK, SELECTED_RECEPTACLE  # noqa: E402

DESIGN_JSON = ROOT / "artifacts" / "rev15_thermal_design.json"
REPORT = ROOT / "artifacts" / "rev15_thermal_report.md"
SCAD = ROOT / "cad" / "rev15_thermal_concepts.scad"
STL_DIR = ROOT / "stl"

SIGMA = 5.670374419e-8
G = 9.80665
K0 = 273.15

# ==========================================================================
# 1. Inputs
# ==========================================================================
P_LOAD = 15.0                       # W, continuous compute load (task requirement)
TJ_THROTTLE = 85.0                  # [D2] passive trip
TJ_CRIT = 115.0                     # [D2] critical trip
TJ_MARGIN = 5.0                     # K guard band below the trip for the design gate
TA_CORNERS = (25.0, 40.0, 50.0)     # lab / typical enclosure / worst-case enclosure
TA_GATE = 40.0                      # design gate: no throttling at 40 C ambient
DF40_TMAX = 85.0                    # [D1]
CARRIER_3V3_BUDGET_A = 1.2          # Rev 13 carrier 3V3 load budget


@dataclass(frozen=True)
class Soc:                          # [A1]
    die: tuple = (8.0e-3, 8.0e-3)
    pkg: tuple = (15.0e-3, 15.0e-3)
    theta_jc: float = 0.25
    theta_jb: float = 6.0
    centre: tuple = (27.5, 20.0)    # [A2] carrier coordinates (mm)
    top_above_pcb: float = 1.10e-3  # ball collapse 0.20 + substrate/die 0.90 (nominal)


@dataclass(frozen=True)
class LoadCase:
    name: str
    p_soc: float                    # into the junction
    p_module_pcb: float             # DRAM / PMIC / eMMC / WiFi into the module PCB


LOADS = (LoadCase("concentrated (15 W at SoC junction)", P_LOAD, 0.0),
         LoadCase("distributed (11.5 W SoC + 3.5 W module)", 11.5, 3.5))


@dataclass(frozen=True)
class Mat:
    k: float
    rho: float
    cp: float


AL6063 = Mat(209.0, 2700.0, 900.0)
CU = Mat(391.0, 8930.0, 385.0)
FR4_INPLANE_K = 0.0                 # computed from STACKUP copper (see board_k_inplane)


@dataclass(frozen=True)
class FinSink:
    name: str
    W: float                        # base width, across fins (m)
    L: float                        # base length = channel length, flow direction (m)
    tb: float                       # base thickness (m)
    H: float                        # fin height (m)
    tf: float = 1.0e-3              # fin thickness (m)
    mat: Mat = AL6063
    eps: float = 0.85               # black anodise
    n_fins: int = 0                 # 0 -> optimise (natural) / given (forced)


SINK_FOOT = FinSink("55 x 40 x 15 footprint sink", 55e-3, 40e-3, 3.0e-3, 15e-3)
# Remote sink: STARTING POINT only. calc() re-sizes it with size_remote_sink() for B1 at the gate and
# rounds W/L up to the next 5 mm extrusion step; the same dissipator is then given to A2/A4 so the
# comparison is apples-to-apples (only the heat-acquisition hardware differs).
SINK_REMOTE = FinSink("remote / enclosure-wall sink", 130e-3, 95e-3, 4.0e-3, 20e-3)
# Secondary path for the module-PCB heat (DRAM + PMIC, [A7]): 1.0 mm gap pads (k 6, 30 % compressed) from
# the top of both packages to the dissipator (pedestal sink underside, or a 40 x 30 x 1 mm Cu saddle
# soldered to the heat-pipe evaporator block). Package junction->top 6 K/W each (LPDDR/PMIC class).
SEC_PADS = dict(n=2, theta_top=6.0, area=140e-6, t=0.7e-3, k=6.0, saddle=(40e-3, 30e-3, 1.0e-3))


def g_secondary() -> float:
    r_pad = SEC_PADS["t"] / (SEC_PADS["k"] * SEC_PADS["area"])
    return SEC_PADS["n"] / (SEC_PADS["theta_top"] + r_pad)


MASS_ON_MODULE_MAX_G = 60.0         # [A6] carried by 4x M2.5 + the DF40 pair (50 g / 11 ms shock -> ~30 N)


@dataclass(frozen=True)
class Pedestal:
    tip: tuple = (12e-3, 12e-3)     # covers die + 2 mm each side for placement tolerance
    mat: Mat = AL6063               # integral with the sink (one-piece CNC), no joint
    base_clear: float = 3.5e-3      # sink-base underside above module PCB top (> 3.0 [A2] + 0.5)


@dataclass(frozen=True)
class HeatPipe:                     # [A3]
    d: float = 6.0e-3
    wall: float = 0.30e-3
    wick: float = 0.50e-3
    t_flat: float = 2.5e-3
    d_p: float = 50e-6
    poros: float = 0.50
    k_solid: float = 391.0
    L_e: float = 20e-3              # evaporator = block length
    L_a: float = 45e-3              # SoC centre -> board edge (27.5) + 17.5 mm routing to sink
    L_c: float = 60e-3              # condenser soldered along the remote sink base
    bends_90: int = 1
    flat_factor: float = 0.75
    bend_factor: float = 0.95
    safety: float = 1.5
    r_evap: float = 0.20e-4         # K m^2/W (0.20 K cm^2/W)
    r_cond: float = 0.10e-4
    solder_t: float = 0.05e-3
    solder_k: float = 50.0
    block: tuple = (20e-3, 20e-3, 2.0e-3)    # Cu evaporator block (x, y, z)
    spring_force_N: float = 30.0    # 2x M2.5 shoulder screws + leaf spring

    @property
    def L_total(self) -> float:
        return self.L_e + self.L_a + self.L_c

    @property
    def L_eff(self) -> float:
        return self.L_a + (self.L_e + self.L_c) / 2

    @property
    def flat_width(self) -> float:  # perimeter preserved on flattening
        return (math.pi * self.d - math.pi * self.t_flat) / 2 + self.t_flat

    @property
    def vapour(self) -> tuple:
        a = self.flat_width - 2 * (self.wall + self.wick)
        b = self.t_flat - 2 * (self.wall + self.wick)
        return a, b

    @property
    def A_v(self) -> float:
        a, b = self.vapour
        return a * b - (4 - math.pi) * (b / 2) ** 2      # stadium cross-section

    @property
    def A_w(self) -> float:         # wick annulus of the round pipe (flattening keeps the sintered mass)
        ri = self.d / 2 - self.wall
        return math.pi * (ri ** 2 - (ri - self.wick) ** 2)


@dataclass(frozen=True)
class Tim:                          # [A5]
    gap_k: float = 6.0
    gap_t0: float = 1.0e-3
    gap_comp: tuple = (0.10, 0.50)
    gap_stress_50: float = 0.14e6
    pcm_r: float = 0.08e-4          # K m^2/W
    pcm_blt: float = 0.03e-3


@dataclass(frozen=True)
class Fan:                          # [A4]
    q_max: float = 1.5e-3
    p0: float = 20.0
    k_minor: float = 1.5            # inlet + exit loss coefficients


# Tolerance contributors for the rigid pedestal gap (mm, +/-; one-sided entries flagged)
GAP_STACK = [
    ("standoff length (module-top seat -> sink base underside)", 0.05),
    ("pedestal machined height", 0.03),
    ("sink base flatness under pedestal", 0.03),
    ("BGA ball collapse", 0.05),
    ("package substrate + die height", 0.07),
    ("module PCB local bow at SoC (IPC-6012 0.75 % over 15 mm, half)", 0.06),
    ("die tilt across 8 mm (coplanarity)", 0.03),
]


# ==========================================================================
# 2. Property tables
# ==========================================================================
#            T C    rho_l   mu_l      k_l    sigma    h_fg     P_sat     rho_v    mu_v
WATER = np.array([
    [0.0,   999.8, 1.792e-3, 0.561, 0.0757, 2501e3,    611.2, 0.00485, 9.22e-6],
    [20.0,  998.2, 1.002e-3, 0.598, 0.0728, 2454e3,   2339.0, 0.01731, 9.73e-6],
    [40.0,  992.2, 0.653e-3, 0.631, 0.0696, 2407e3,   7384.0, 0.05121, 10.4e-6],
    [60.0,  983.2, 0.467e-3, 0.654, 0.0662, 2358e3,  19940.0, 0.1302, 11.1e-6],
    [80.0,  971.8, 0.355e-3, 0.670, 0.0626, 2309e3,  47390.0, 0.2935, 11.8e-6],
    [100.0, 958.4, 0.282e-3, 0.679, 0.0589, 2257e3, 101325.0, 0.5974, 12.3e-6],
    [120.0, 943.1, 0.232e-3, 0.683, 0.0550, 2203e3, 198500.0, 1.1220, 12.9e-6],
])
WKEYS = ("T", "rho_l", "mu_l", "k_l", "sigma", "h_fg", "P", "rho_v", "mu_v")


def water(t_c: float) -> dict:
    t_c = min(max(t_c, WATER[0, 0]), WATER[-1, 0])
    out = {}
    for j, k in enumerate(WKEYS):
        if k in ("mu_l", "P", "rho_v"):           # log-interpolate the exponential ones
            out[k] = float(np.exp(np.interp(t_c, WATER[:, 0], np.log(WATER[:, j]))))
        else:
            out[k] = float(np.interp(t_c, WATER[:, 0], WATER[:, j]))
    return out


AIR = np.array([  # T K, k, nu, Pr, rho, cp
    [250.0, 0.0223, 11.44e-6, 0.720, 1.395, 1006.0],
    [300.0, 0.0263, 15.89e-6, 0.707, 1.161, 1007.0],
    [350.0, 0.0300, 20.92e-6, 0.700, 0.995, 1009.0],
    [400.0, 0.0338, 26.41e-6, 0.690, 0.871, 1014.0],
])


def air(t_c: float) -> dict:
    T = t_c + K0
    v = {k: float(np.interp(T, AIR[:, 0], AIR[:, j])) for j, k in enumerate(("T", "k", "nu", "Pr", "rho", "cp"))}
    v["beta"] = 1.0 / T
    v["alpha"] = v["nu"] / v["Pr"]
    return v


def h_rad(ts: float, ta: float, eps: float) -> float:
    a, b = ts + K0, ta + K0
    return eps * SIGMA * (a * a + b * b) * (a + b)


def h_vplate(ts: float, ta: float, L: float) -> float:
    """[D5] Churchill-Chu laminar vertical plate, film properties."""
    dT = max(ts - ta, 0.05)
    p = air((ts + ta) / 2)
    ra = G * p["beta"] * dT * L ** 3 / (p["nu"] * p["alpha"])
    nu = 0.68 + 0.670 * ra ** 0.25 / (1 + (0.492 / p["Pr"]) ** (9 / 16)) ** (4 / 9)
    return nu * p["k"] / L


# ==========================================================================
# 3. Conduction solvers
# ==========================================================================
def spreading_rect(src: tuple, plate: tuple, t: float, k: float, h: float, n_terms: int = 120) -> tuple[float, float]:
    """Mean-source-temperature resistance of a centred rectangular source (a x b)
    on a rectangular plate (c x d x t), adiabatic edges, uniform h on the far face.
    Fourier cosine series re-derived from Laplace's equation (cf. [D6]).
    Returns (R_total, R_spreading) in K/W, with R_total = R_1D + R_sp."""
    a, b = src
    c, d = plate
    if a >= c - 1e-12 and b >= d - 1e-12:
        r1 = t / (k * c * d) + 1 / (h * c * d)
        return r1, 0.0

    def phi(beta):
        th = math.tanh(beta * t)
        return (k * beta + h * th) / (k * beta * th + h)

    def avg(lam, w, span):           # mean of cos(lam x) over the source, centre span/2
        return 2 * math.cos(lam * span / 2) * math.sin(lam * w / 2) / (lam * w)

    m = np.arange(1, n_terms + 1)
    lam = m * math.pi / c
    dl = m * math.pi / d
    ax = np.array([avg(x, a, c) for x in lam])
    ay = np.array([avg(y, b, d) for y in dl])
    phx = np.array([phi(x) for x in lam])
    phy = np.array([phi(y) for y in dl])
    s1 = np.sum(2 * ax ** 2 * phx / lam)
    s2 = np.sum(2 * ay ** 2 * phy / dl)
    LL, DD = np.meshgrid(lam, dl, indexing="ij")
    BB = np.sqrt(LL ** 2 + DD ** 2)
    PH = (k * BB + h * np.tanh(BB * t)) / (k * BB * np.tanh(BB * t) + h)
    s3 = np.sum(4 * np.outer(ax ** 2, ay ** 2) * PH / BB)
    r_sp = (s1 + s2 + s3) / (c * d * k)
    r1 = t / (k * c * d) + 1 / (h * c * d)
    return r1 + r_sp, r_sp


def fd3d(plate: tuple, t: float, k: float, src: tuple, sink_h: float, sink_patch: tuple | None = None,
         n: tuple = (40, 40, 8)) -> float:
    """Independent finite-volume 3D steady solver. Uniform flux Q=1 W into a
    centred rectangular patch `src` on face z=0; convective h (to 0 K) over a
    centred patch `sink_patch` (default: the whole face) on z=t; all other
    faces adiabatic. Returns mean source temperature = R (K/W)."""
    c, d = plate
    nx, ny, nz = n
    dx, dy, dz = c / nx, d / ny, t / nz
    N = nx * ny * nz
    idx = lambda i, j, l: (l * ny + j) * nx + i  # noqa: E731
    xc = (np.arange(nx) + 0.5) * dx
    yc = (np.arange(ny) + 0.5) * dy

    def frac(cent, lo, hi, dd):      # cell-overlap fraction of [lo, hi]
        return np.clip((np.minimum(cent + dd / 2, hi) - np.maximum(cent - dd / 2, lo)) / dd, 0, 1)

    fx = frac(xc, (c - src[0]) / 2, (c + src[0]) / 2, dx)
    fy = frac(yc, (d - src[1]) / 2, (d + src[1]) / 2, dy)
    sp_ = sink_patch or (c, d)
    gx = frac(xc, (c - sp_[0]) / 2, (c + sp_[0]) / 2, dx)
    gy = frac(yc, (d - sp_[1]) / 2, (d + sp_[1]) / 2, dy)
    rows, cols, vals = [], [], []
    rhs = np.zeros(N)
    diag = np.zeros(N)
    gxx, gyy, gzz = k * dy * dz / dx, k * dx * dz / dy, k * dx * dy / dz
    for l in range(nz):
        for j in range(ny):
            for i in range(nx):
                p = idx(i, j, l)
                for (ii, jj, ll, g) in ((i + 1, j, l, gxx), (i - 1, j, l, gxx), (i, j + 1, l, gyy),
                                        (i, j - 1, l, gyy), (i, j, l + 1, gzz), (i, j, l - 1, gzz)):
                    if 0 <= ii < nx and 0 <= jj < ny and 0 <= ll < nz:
                        rows.append(p); cols.append(idx(ii, jj, ll)); vals.append(-g)
                        diag[p] += g
                if l == nz - 1:
                    a_h = gx[i] * gy[j] * dx * dy
                    if a_h > 0:
                        gb = 1.0 / (dz / 2 / (k * dx * dy) + 1.0 / (sink_h * a_h))
                        diag[p] += gb
                if l == 0:
                    rhs[p] += fx[i] * fy[j] * dx * dy / (src[0] * src[1])
    rows += list(range(N)); cols += list(range(N)); vals += list(diag)
    A = sp.csr_matrix((vals, (rows, cols)), shape=(N, N))
    T = spla.spsolve(A, rhs)
    w = np.outer(fy, fx).ravel() * dx * dy        # weights on layer 0 (j, i)
    T0 = T[:nx * ny]
    return float(np.sum(w * T0) / np.sum(w))          # cell-centre value; see fd3d_surface()


def fd3d_surface(*args, **kw) -> float:
    """fd3d + analytic half-cell correction to the source *surface* (q dz/2k)."""
    plate, t, k, src = args[0], args[1], args[2], args[3]
    n = kw.get("n", (40, 40, 8))
    dz = t / n[2]
    return fd3d(*args, **kw) + (dz / 2) / (k * src[0] * src[1])


# ==========================================================================
# 4. Convection models for fin sinks
# ==========================================================================
def fin_eff(h: float, s: FinSink) -> float:
    m = math.sqrt(2 * h / (s.mat.k * s.tf))
    hc = s.H + s.tf / 2
    return math.tanh(m * hc) / (m * hc)


def natural_sink(s: FinSink, ts: float, ta: float, n: int | None = None) -> dict:
    """[D3] vertical channels (board mounted vertically, fins vertical) + envelope radiation."""
    dT = max(ts - ta, 0.1)
    p = air((ts + ta) / 2)
    n = n or s.n_fins
    S = (s.W - n * s.tf) / (n - 1)
    ra_s = G * p["beta"] * dT * S ** 3 / (p["nu"] * p["alpha"])
    el = ra_s * S / s.L
    nu = (576 / el ** 2 + 2.873 / el ** 0.5) ** -0.5
    h = nu * p["k"] / S
    eta = fin_eff(h, s)
    a_fin = 2 * n * s.H * s.L
    a_base = (n - 1) * S * s.L
    g_conv = h * (eta * a_fin + a_base)
    a_env = s.W * s.L + 2 * s.H * s.L + 2 * s.W * s.H * 0.5          # tips + outer fins + half of open ends
    g_rad = h_rad(ts, ta, s.eps) * a_env
    return dict(n=n, S=S, h=h, eta=eta, g_conv=g_conv, g_rad=g_rad, G=g_conv + g_rad, El=el)


def best_natural(s: FinSink, ts: float, ta: float) -> dict:
    best = None
    for n in range(3, int(s.W / (s.tf + 1.5e-3)) + 1):     # S >= 1.5 mm (extrusion/fin-gap limit)
        r = natural_sink(s, ts, ta, n)
        if r["S"] < 1.5e-3:
            continue
        if best is None or r["G"] > best["G"]:
            best = r
    return best


def fan_operating_point(s: FinSink, n: int, ta: float, fan: Fan = Fan()) -> dict:
    p = air(ta)
    S = (s.W - n * s.tf) / (n - 1)
    a_flow = (n - 1) * S * s.H
    dh = 2 * S

    def dp_sys(q):
        v = q / a_flow
        re = v * dh / p["nu"]
        f = 96.0 / max(re, 1e-9)            # fully-developed parallel plates (fRe = 96)
        return (f * s.L / dh + fan.k_minor) * 0.5 * p["rho"] * v * v

    lo, hi = 0.0, fan.q_max
    for _ in range(80):
        m = (lo + hi) / 2
        if fan.p0 * (1 - m / fan.q_max) > dp_sys(m):
            lo = m
        else:
            hi = m
    q = lo
    return dict(q=q, v=q / a_flow, dp=dp_sys(q), S=S, a_flow=a_flow)


def forced_sink(s: FinSink, ts: float, ta: float, n: int) -> dict:
    """[D4] + epsilon-NTU air heating along the channel."""
    op = fan_operating_point(s, n, ta)
    p = air((ts + ta) / 2)
    S = op["S"]
    re_b = op["v"] * S / p["nu"]
    re_s = re_b * S / s.L
    nu = ((re_s * p["Pr"] / 2) ** -3 + (0.664 * re_s ** 0.5 * p["Pr"] ** (1 / 3)
                                        * (1 + 3.65 / re_s ** 0.5) ** 0.5) ** -3) ** (-1 / 3)
    h = nu * p["k"] / S
    eta = fin_eff(h, s)
    hA = h * (eta * 2 * n * s.H * s.L + (n - 1) * S * s.L)
    mcp = air(ta)["rho"] * op["q"] * p["cp"]
    eff = 1 - math.exp(-hA / mcp)
    g_conv = mcp * eff
    g_rad = h_rad(ts, ta, s.eps) * (s.W * s.L + 2 * s.H * s.L)
    return dict(n=n, S=S, h=h, eta=eta, g_conv=g_conv, g_rad=g_rad, G=g_conv + g_rad, fan=op, NTU=hA / mcp)


def best_forced(s: FinSink, ts: float, ta: float) -> dict:
    best = None
    for n in range(4, int(s.W / (s.tf + 1.0e-3)) + 1):
        r = forced_sink(s, ts, ta, n)
        if r["S"] < 1.0e-3:
            continue
        if best is None or r["G"] > best["G"]:
            best = r
    return best


# ==========================================================================
# 5. Heat-pipe operating limits [D7]
# ==========================================================================
def hp_limits(hp: HeatPipe, t_v: float, tilt_deg: float = 0.0) -> dict:
    """tilt_deg > 0 : evaporator ABOVE condenser (adverse, gravity opposes return)."""
    w = water(t_v)
    r_eff = 0.21 * hp.d_p
    K = hp.d_p ** 2 * hp.poros ** 3 / (150 * (1 - hp.poros) ** 2)
    dp_cap = 2 * w["sigma"] / r_eff
    dp_g = w["rho_l"] * G * hp.L_total * math.sin(math.radians(tilt_deg))
    f_l = w["mu_l"] * hp.L_eff / (w["rho_l"] * w["h_fg"] * K * hp.A_w)         # Pa/W
    a, b = hp.vapour
    dh_v = 4 * hp.A_v / (2 * (a - b) + math.pi * b)
    r_hv = dh_v / 2
    f_v = 8 * w["mu_v"] * hp.L_eff / (w["rho_v"] * w["h_fg"] * hp.A_v * r_hv ** 2)   # laminar Pa/W
    q_cap_ideal = max(dp_cap - dp_g, 0.0) / (f_l + f_v)
    derate = hp.flat_factor * hp.bend_factor ** hp.bends_90
    q_cap = q_cap_ideal * derate
    q_vis = hp.A_v * r_hv ** 2 * w["h_fg"] * w["rho_v"] * w["P"] / (16 * w["mu_v"] * hp.L_eff)
    q_son = 0.474 * hp.A_v * w["h_fg"] * math.sqrt(w["rho_v"] * w["P"])
    r_hw = 0.5 * hp.d_p * 0.41                                   # surface pore hydraulic radius
    q_ent = hp.A_v * w["h_fg"] * math.sqrt(w["sigma"] * w["rho_v"] / (2 * r_hw))
    q_boil_flux = 50e4 * 0.6                                     # sintered CHF ~50 W/cm^2, 60 % usable
    a_e = hp.flat_width * hp.L_e
    q_boil = q_boil_flux * a_e
    lims = dict(capillary=q_cap, viscous=q_vis, sonic=q_son, entrainment=q_ent, boiling=q_boil)
    gov = min(lims, key=lims.get)
    return dict(T_v=t_v, tilt=tilt_deg, dp_cap=dp_cap, dp_g=dp_g, f_l=f_l, f_v=f_v, q_cap_ideal=q_cap_ideal,
                derate=derate, K=K, limits=lims, governing=gov, q_max=lims[gov], q_flux_evap_W_cm2=None)


def hp_max_adverse_tilt(hp: HeatPipe, t_v: float, q_req: float) -> float:
    lo, hi = -90.0, 90.0
    if hp_limits(hp, t_v, 90.0)["q_max"] >= q_req:
        return 90.0
    if hp_limits(hp, t_v, -90.0)["q_max"] < q_req:
        return float("nan")
    for _ in range(60):
        m = (lo + hi) / 2
        lo, hi = (m, hi) if hp_limits(hp, t_v, m)["q_max"] >= q_req else (lo, m)
    return lo


def hp_film_resistance(hp: HeatPipe, t_v: float) -> dict:
    """Wall + solder + evaporator/condenser film; vapour drop from Clausius-Clapeyron."""
    a_e = hp.flat_width * hp.L_e
    a_c = hp.flat_width * hp.L_c
    r_wall = hp.wall / CU.k
    r_sol = hp.solder_t / hp.solder_k
    w = water(t_v)
    a, b = hp.vapour
    dh_v = 4 * hp.A_v / (2 * (a - b) + math.pi * b)
    dp_v_perW = 8 * w["mu_v"] * hp.L_eff / (w["rho_v"] * w["h_fg"] * hp.A_v * (dh_v / 2) ** 2)
    T = t_v + K0
    dTdP = T * (1 / w["rho_v"] - 1 / w["rho_l"]) / w["h_fg"]
    return dict(h_evap=1 / (r_wall + r_sol + hp.r_evap), r_cond=(r_wall + r_sol + hp.r_cond) / a_c,
                r_vap=dp_v_perW * dTdP, a_e=a_e, a_c=a_c)


# ==========================================================================
# 6. Module / carrier stack network
# ==========================================================================
def board_k_inplane(t_cu_total: float, t_board: float) -> float:
    return (t_cu_total * CU.k + (t_board - t_cu_total) * 0.30) / t_board


MOD_T = 1.2e-3                       # module PCB thickness (CM4 class), 6-8 layer, ~4 oz total Cu (assumed)
MOD_K_IN = board_k_inplane(4 * 0.035e-3, MOD_T)
CAR_T = STACKUP.total * 1e-3          # m.  Rev 19 fix: STACKUP is in mm (was used as metres -> 1000x)
CAR_K_IN = board_k_inplane((2 * STACKUP.t_outer + 2 * STACKUP.t_inner) * 1e-3, CAR_T)
A_BOARD = BOARD_W * BOARD_H * 1e-6
STACK_H = STACK[SELECTED_RECEPTACLE]["stack"] * 1e-3


def g_module_to_carrier(tm: float, tk: float) -> dict:
    """200 DF40 contacts (Cu alloy, ~3 mm path, 0.20 x 0.10 mm) + 3 mm air gap + gap radiation.
    The M2.5 standoffs land on carrier NPTH holes on bare laminate (no Cu ring, planes pulled
    back 3.1 mm) -> conduction through 1.6 mm FR-4 under a 5 mm A/F hex; counted per hole."""
    g_pins = 200 * (70.0 * 0.20e-3 * 0.10e-3 / STACK_H)
    p = air((tm + tk) / 2)
    g_air = p["k"] * (A_BOARD - 2 * 22.6e-3 * 3.0e-3) / STACK_H
    e = 1 / (1 / 0.9 + 1 / 0.9 - 1)
    g_rad = h_rad(tm, tk, e) * A_BOARD
    a_hex = (3 ** 0.5 / 2) * 5.0e-3 ** 2 - math.pi * (1.35e-3) ** 2
    g_so = 4 * 1 / (STACK_H / (110.0 * a_hex) + CAR_T / (0.30 * a_hex) + 2.0)      # brass, FR-4, 2 K/W contacts
    return dict(pins=g_pins, air=g_air, rad=g_rad, standoffs=g_so, total=g_pins + g_air + g_rad + g_so)


def g_surface_to_amb(ts: float, ta: float, area: float, L: float, eps: float = 0.9) -> float:
    return (h_vplate(ts, ta, L) + h_rad(ts, ta, eps)) * area


@dataclass
class Concept:
    key: str
    title: str
    kind: str                         # pedestal | heatpipe
    sink: FinSink
    forced: bool = False
    tim: str = "gap"                  # gap | pcm
    notes: list = field(default_factory=list)


CONCEPTS = [
    Concept("A1", "Integral Al pedestal + 55x40x15 footprint sink (natural)", "pedestal", SINK_FOOT),
    Concept("A2", "Integral Al pedestal + 130x95x20 overhanging sink (natural)", "pedestal", SINK_REMOTE),
    Concept("A3", "Integral Al pedestal + 55x40x15 footprint sink + ducted 30 mm fan", "pedestal", SINK_FOOT, forced=True),
    Concept("A4", "Spring-loaded Al pedestal sink (phase-change TIM) + 130x95x20 overhanging sink (natural)", "pedestal",
            SINK_REMOTE, tim="pcm"),
    Concept("A5", "Spring-loaded Al pedestal sink (phase-change TIM) + 55x40x15 sink + ducted 30 mm fan", "pedestal",
            SINK_FOOT, forced=True, tim="pcm"),
    Concept("B1", "Cu block + 6 mm flattened sintered heat pipe -> 130x95x20 remote sink (natural)", "heatpipe",
            SINK_REMOTE, tim="pcm"),
]


def gap_stack(tim: Tim = Tim()) -> dict:
    wc = sum(v for _n, v in GAP_STACK)
    rss = math.sqrt(sum(v * v for _n, v in GAP_STACK))
    gmin = tim.gap_t0 * (1 - tim.gap_comp[1])
    gmax = tim.gap_t0 * (1 - tim.gap_comp[0])
    nom = (gmin + gmax) / 2
    ok_wc = nom - wc * 1e-3 >= gmin - 1e-12 and nom + wc * 1e-3 <= gmax + 1e-12
    ok_rss = nom - rss * 1e-3 >= gmin - 1e-12 and nom + rss * 1e-3 <= gmax + 1e-12
    die_force = tim.gap_stress_50 * Soc().die[0] * Soc().die[1]
    pkg_force = tim.gap_stress_50 * Soc().pkg[0] * Soc().pkg[1]
    return dict(wc_mm=wc, rss_mm=rss, gap_min_mm=gmin * 1e3, gap_max_mm=gmax * 1e3, gap_nom_mm=nom * 1e3,
                fits_wc=ok_wc, fits_rss=ok_rss, worst_gap_mm=(nom + rss * 1e-3) * 1e3,
                wc_gap_mm=(nom + wc * 1e-3) * 1e3, pkg_force_N_at_50pct=pkg_force,
                die_force_N_at_50pct=die_force,
                pedestal_h_mm=(Pedestal().base_clear - Soc().top_above_pcb - nom) * 1e3)


def r_tim(concept: Concept, worst: bool, soc: Soc, tim: Tim = Tim()) -> float:
    a = soc.die[0] * soc.die[1]
    if concept.tim == "pcm":
        return tim.pcm_r / a
    gs = gap_stack(tim)
    t = (gs["worst_gap_mm"] if worst else gs["gap_nom_mm"]) * 1e-3
    return t / (tim.gap_k * a)


def solve_concept(c: Concept, ta: float, load: LoadCase, soc: Soc = Soc(), hp: HeatPipe = HeatPipe(),
                  worst_tim: bool = False, p_scale: float = 1.0, carrier_q: float = 0.0,
                  sink_override: FinSink | None = None) -> dict:
    s = sink_override or c.sink
    q_j, q_m = load.p_soc * p_scale, load.p_module_pcb * p_scale
    T = dict(J=ta + 40, C=ta + 38, X=ta + 30, S=ta + 30, M=ta + 20, K=ta + 10)
    ped = Pedestal()
    sink_r = None
    a_mod_open = A_BOARD - (0 if c.kind == "pedestal" else hp.block[0] * hp.block[1])
    for it in range(200):
        # --- sink to ambient
        if c.forced:
            sink_r = best_forced(s, T["S"], ta)
        else:
            sink_r = best_natural(s, T["S"], ta)
        g_sa = sink_r["G"]
        h_eff = g_sa / (s.W * s.L)
        if c.kind == "pedestal":
            # die -> pedestal (plate = pedestal tip, far-face h = conductance of the base below it)
            r_base, r_base_sp = spreading_rect(ped.tip, (s.W, s.L), s.tb, s.mat.k, h_eff)
            r_base_cond = r_base - 1 / h_eff / (s.W * s.L)          # base conduction + spreading only
            h_ped = 1 / ((r_base_cond + 1 / g_sa) * ped.tip[0] * ped.tip[1])
            ped_h = gap_stack()["pedestal_h_mm"] * 1e-3
            r_ped_tot, r_ped_sp = spreading_rect(soc.die, ped.tip, ped_h, ped.mat.k, h_ped)
            r_ped = r_ped_tot - 1 / (h_ped * ped.tip[0] * ped.tip[1])
            r_cx = r_tim(c, worst_tim, soc) + r_ped + r_base_cond     # case -> sink-fin node
            g_xs = None
            parts = dict(tim=r_tim(c, worst_tim, soc), pedestal=r_ped, pedestal_sp=r_ped_sp,
                         base=r_base_cond, base_sp=r_base_sp)
        else:
            film = hp_film_resistance(hp, T["X"])
            r_blk = _hp_block_r(hp, soc, film["h_evap"])
            r_cnd_tot, r_cnd_sp = spreading_rect((hp.flat_width, hp.L_c), (s.W, s.L), s.tb, s.mat.k, h_eff)
            r_cnd = r_cnd_tot - 1 / h_eff / (s.W * s.L)
            r_cx = r_tim(c, worst_tim, soc) + r_blk          # case -> vapour (X)
            g_xs = 1 / (film["r_vap"] + film["r_cond"] + r_cnd)
            parts = dict(tim=r_tim(c, worst_tim, soc), block_and_evap=r_blk, vapour=film["r_vap"],
                         condenser=film["r_cond"], base=r_cnd, base_sp=r_cnd_sp)
        g_mc = g_module_to_carrier(T["M"], T["K"])["total"]
        if c.kind == "pedestal":
            # module top couples to the sink base across ~2.4 mm air + radiation (sink covers the module)
            gap = ped.base_clear - 1.0e-3            # mean component height ~1 mm
            p = air((T["M"] + T["S"]) / 2)
            a_cov = min(s.W * s.L, A_BOARD) - ped.tip[0] * ped.tip[1]
            g_ms = p["k"] * a_cov / gap + h_rad(T["M"], T["S"], 1 / (1 / 0.9 + 1 / 0.85 - 1)) * a_cov + g_secondary()
            g_mx = 0.0
            g_ma = g_surface_to_amb(T["M"], ta, max(A_BOARD - a_cov, 0) + 2 * (BOARD_W + BOARD_H) * 1e-3 * MOD_T,
                                    BOARD_H * 1e-3)
        else:
            g_ms = 0.0
            g_mx = g_secondary()
            g_ma = g_surface_to_amb(T["M"], ta, a_mod_open - SEC_PADS["saddle"][0] * SEC_PADS["saddle"][1]
                                    + hp.block[0] * hp.block[1], BOARD_H * 1e-3)
        g_ka = g_surface_to_amb(T["K"], ta, A_BOARD, BOARD_H * 1e-3)
        # Linear network: nodes J, C, X, S, M, K ; ambient fixed.
        names = ["J", "C", "X", "S", "M", "K"]
        ix = {n: i for i, n in enumerate(names)}
        A = np.zeros((6, 6))
        b = np.zeros(6)

        def link(u, v, g):
            if g <= 0:
                return
            iu = ix[u]
            A[iu, iu] += g
            if v == "A":
                b[iu] += g * ta
            else:
                iv = ix[v]
                A[iu, iv] -= g
                A[iv, iv] += g
                A[iv, iu] -= g

        link("J", "C", 1 / soc.theta_jc)
        link("J", "M", 1 / soc.theta_jb)
        link("C", "X", 1 / r_cx)
        if c.kind == "pedestal":
            link("X", "S", 1e6)               # X == S (fin-root node) for the pedestal
        else:
            link("X", "S", g_xs)
        link("S", "A", g_sa)
        link("M", "S", g_ms)
        link("M", "X", g_mx)
        link("M", "A", g_ma)
        link("M", "K", g_mc)
        link("K", "A", g_ka)
        b[ix["J"]] += q_j
        b[ix["M"]] += q_m
        b[ix["K"]] += carrier_q
        x = np.linalg.solve(A, b)
        new = {n: float(x[ix[n]]) for n in names}
        diff = max(abs(new[n] - T[n]) for n in names)
        T = {n: 0.5 * T[n] + 0.5 * new[n] for n in names}
        if diff < 1e-4 and it > 3:
            break
    # energy balance (to ambient)
    q_out = (g_sa * (T["S"] - ta) + g_ma * (T["M"] - ta) + g_ka * (T["K"] - ta))
    q_in = q_j + q_m + carrier_q
    flows = dict(sink=g_sa * (T["S"] - ta), module_surface=g_ma * (T["M"] - ta), carrier=g_ka * (T["K"] - ta),
                 jb=(T["J"] - T["M"]) / soc.theta_jb, module_to_carrier=g_mc * (T["M"] - T["K"]),
                 module_to_dissipator=g_ms * (T["M"] - T["S"]) + g_mx * (T["M"] - T["X"]))
    return dict(T=T, ta=ta, sink=sink_r, g_sa=g_sa, r_sa=1 / g_sa, parts=parts, q_in=q_in, q_out=q_out,
                flows=flows, iters=it, g_mc=g_mc, g_ma=g_ma, g_ka=g_ka, g_ms=g_ms, g_mx=g_mx,
                r_ja_eff=(T["J"] - ta) / max(q_in, 1e-9))


_BLOCK_CACHE: dict = {}


def _hp_block_r(hp: HeatPipe, soc: Soc, h_evap: float) -> float:
    """Die -> vapour through the Cu evaporator block (3D FD; heat pipe soldered in a strip
    `flat_width` wide along the block's full length on the far face)."""
    key = (hp.block, soc.die, round(h_evap, 3), hp.flat_width)
    if key not in _BLOCK_CACHE:
        bx, by, bz = hp.block
        _BLOCK_CACHE[key] = fd3d_surface((bx, by), bz, CU.k, soc.die, h_evap,
                                         sink_patch=(hp.flat_width, by), n=(40, 40, 8))
    return _BLOCK_CACHE[key]


# ==========================================================================
# 7. Transient (backward-Euler on the linearised steady network)
# ==========================================================================
def transient(c: Concept, ta: float, load: LoadCase, steady: dict, hp: HeatPipe = HeatPipe(),
              t_end: float = 4 * 3600.0, dt: float = 2.0, soc: Soc = Soc(), carrier_q: float = 0.0) -> dict:
    s = c.sink
    n_f = steady["sink"]["n"]
    v_sink = s.W * s.L * s.tb + n_f * s.tf * s.H * s.L
    ped = Pedestal()
    if c.kind == "pedestal":
        v_sink += ped.tip[0] * ped.tip[1] * gap_stack()["pedestal_h_mm"] * 1e-3
        c_x = 0.0
    else:
        bx, by, bz = hp.block
        sx_, sy_, sz_ = SEC_PADS["saddle"]
        c_x = (bx * by * bz + sx_ * sy_ * sz_ - bx * by * sz_) * CU.rho * CU.cp
        v_sink += 0.0
    hp_mass = 0.0
    if c.kind == "heatpipe":
        hp_mass = (math.pi * ((hp.d / 2) ** 2 - (hp.d / 2 - hp.wall) ** 2) * hp.L_total * CU.rho
                   + hp.A_w * hp.L_total * (1 - hp.poros) * CU.rho)
    C = dict(J=2.0, C=0.0, X=c_x, S=v_sink * s.mat.rho * s.mat.cp + hp_mass * CU.cp,
             M=A_BOARD * MOD_T * 1.9e6 + 8.0, K=A_BOARD * CAR_T * 1.9e6 + 4.0)
    T_s = steady["T"]
    # Rebuild the linear conductance matrix with steady-state conductances
    names = ["J", "C", "X", "S", "M", "K"]
    ix = {n: i for i, n in enumerate(names)}
    Gm = np.zeros((6, 6))
    ga = np.zeros(6)
    pr = steady["parts"]

    def link(u, v, g):
        iu = ix[u]
        Gm[iu, iu] += g
        if v == "A":
            ga[iu] += g
        else:
            iv = ix[v]
            Gm[iu, iv] -= g; Gm[iv, iv] += g; Gm[iv, iu] -= g

    link("J", "C", 1 / soc.theta_jc)
    link("J", "M", 1 / soc.theta_jb)
    if c.kind == "pedestal":
        link("C", "X", 1 / (pr["tim"] + pr["pedestal"] + pr["base"]))
        link("X", "S", 1e3)
    else:
        link("C", "X", 1 / (pr["tim"] + pr["block_and_evap"]))
        link("X", "S", 1 / (pr["vapour"] + pr["condenser"] + pr["base"]))
    link("S", "A", steady["g_sa"])
    link("M", "S", steady["g_ms"]) if steady["g_ms"] > 0 else None
    link("M", "X", steady["g_mx"]) if steady["g_mx"] > 0 else None
    link("M", "A", steady["g_ma"])
    link("M", "K", steady["g_mc"])
    link("K", "A", steady["g_ka"])
    cap = np.array([max(C[n], 1e-3) for n in names])
    q = np.zeros(6)
    q[ix["J"]] = load.p_soc
    q[ix["M"]] = load.p_module_pcb
    q[ix["K"]] = carrier_q
    x = np.full(6, ta)
    M_ = np.diag(cap / dt) + Gm
    lu = np.linalg.inv(M_)
    t = 0.0
    t_trip = None
    tj_ss = T_s["J"]
    t63 = None
    tr = []
    while t < t_end:
        x = lu @ (cap / dt * x + q + ga * ta)
        t += dt
        tj = x[ix["J"]]
        if t_trip is None and tj >= TJ_THROTTLE:
            t_trip = t
        if t63 is None and tj - ta >= 0.632 * (tj_ss - ta):
            t63 = t
        if int(t) % 60 == 0:
            tr.append((t, float(tj)))
    m_block = C["X"] / CU.cp if c.kind == "heatpipe" else 0.0
    mass = v_sink * s.mat.rho + hp_mass + m_block
    if c.kind == "heatpipe":   # module carries block + evaporator/adiabatic pipe + clip; sink is on the enclosure
        m_mod = m_block + hp_mass * (hp.L_e + hp.L_a) / hp.L_total + 3.0e-3
    else:                      # pedestal sinks are bolted to the module holes (+ 4 standoffs ~1 g each)
        m_mod = v_sink * s.mat.rho + 4.0e-3
    return dict(t_trip_s=t_trip, t63_s=t63, tj_end=float(x[ix["J"]]), tj_ss=float(tj_ss), C_sink_J_K=C["S"],
                mass_g=mass * 1e3, mass_on_module_g=m_mod * 1e3, trace=tr[::5])


# ==========================================================================
# 8. Study
# ==========================================================================
def p_sustain(c: Concept, ta: float, load: LoadCase, target: float = TJ_THROTTLE, node: str = "J", **kw) -> float:
    lo, hi = 0.0, 3.0
    for _ in range(18):
        m = (lo + hi) / 2
        if solve_concept(c, ta, load, p_scale=m, **kw)["T"][node] <= target:
            lo = m
        else:
            hi = m
    return lo * (load.p_soc + load.p_module_pcb)


def size_remote_sink(c: Concept, ta: float, load: LoadCase, tj_target: float, carrier_q: float) -> dict:
    """Smallest remote sink length L (W = 1.375 L, H fixed) meeting tj_target."""
    base = c.sink
    lo, hi = 40e-3, 300e-3
    for _ in range(16):
        m = (lo + hi) / 2
        s = replace(base, W=1.375 * m, L=m)
        if solve_concept(c, ta, load, sink_override=s, carrier_q=carrier_q)["T"]["J"] <= tj_target:
            hi = m
        else:
            lo = m
    s = replace(base, W=1.375 * hi, L=hi)
    return dict(W_mm=s.W * 1e3, L_mm=s.L * 1e3, H_mm=s.H * 1e3, volume_cm3=s.W * s.L * (s.H + s.tb) * 1e6,
                feasible=hi < 299e-3)


def buck_derate_at(t_local: float) -> tuple[float, dict]:
    import buck_3v3
    r = buck_3v3.calc()["thermal"]["derate"]
    xs = sorted((float(k), v[1]) for k, v in r.items())
    tt = [x for x, _ in xs]
    ii = [y for _, y in xs]
    if t_local > tt[-1]:
        # extrapolate linearly from the last two points (conservative: slope is steepening)
        slope = (ii[-1] - ii[-2]) / (tt[-1] - tt[-2])
        return max(0.0, ii[-1] + slope * (t_local - tt[-1])), r
    return float(np.interp(t_local, tt, ii)), r


def carrier_heat() -> float:
    import buck_3v3
    th = buck_3v3.calc()["thermal"]
    # conservative: full 2 A buck losses + 0.25 W (LEDs, ESD leakage, HDMI/ETH magnetics bias)
    return th["p_ic_W"] + th["p_ext_W"] + 0.25


def calc() -> dict:
    checks = []

    def chk(name, ok, detail, gate=True):
        checks.append(dict(name=name, ok=bool(ok), detail=detail, gate=gate))

    q_k = carrier_heat()
    hp = HeatPipe()
    out = dict(inputs=dict(P_load=P_LOAD, TJ_throttle=TJ_THROTTLE, TJ_crit=TJ_CRIT, TA_corners=TA_CORNERS,
                           TA_gate=TA_GATE, carrier_heat_W=q_k, soc=asdict(Soc()), heatpipe=asdict(hp),
                           tim=asdict(Tim()), fan=asdict(Fan()),
                           sinks={s.name: asdict(s) for s in (SINK_FOOT, SINK_REMOTE)},
                           module_k_inplane=MOD_K_IN, carrier_k_inplane=CAR_K_IN),
               concepts={})
    gs = gap_stack()
    out["gap_stack"] = gs
    chk("Rigid-pedestal TIM gap: RSS stack fits the gap-pad 10..50 % compression window",
        gs["fits_rss"], f"stack RSS +/-{gs['rss_mm']:.3f} mm in window {gs['gap_min_mm']:.2f}..{gs['gap_max_mm']:.2f} mm "
        f"(nominal {gs['gap_nom_mm']:.2f}); 'worst TIM' columns use nominal + RSS = {gs['worst_gap_mm']:.2f} mm")
    chk("Rigid-pedestal TIM gap: arithmetic worst-case stack fits the window (info: RSS is the release criterion)",
        gs["fits_wc"], gate=False, detail=f"stack WC +/-{gs['wc_mm']:.2f} mm (RSS {gs['rss_mm']:.3f}) vs window "
        f"{gs['gap_min_mm']:.2f}..{gs['gap_max_mm']:.2f} mm, nominal {gs['gap_nom_mm']:.2f} mm, pedestal "
        f"{gs['pedestal_h_mm']:.2f} mm")
    chk("Gap-pad clamp load on the SoC package at 50 % compression <= 40 N", gs["pkg_force_N_at_50pct"] <= 40.0,
        f"{gs['pkg_force_N_at_50pct']:.1f} N over 15 x 15 mm ({Tim().gap_stress_50/1e6:.2f} MPa)")
    chk("Heat-pipe evaporator spring preload <= 40 N on the package (phase-change TIM needs >= 0.1 MPa)",
        hp.spring_force_N <= 40.0 and hp.spring_force_N / (Soc().pkg[0] * Soc().pkg[1]) >= 0.1e6,
        f"{hp.spring_force_N:.0f} N -> {hp.spring_force_N / (Soc().pkg[0] * Soc().pkg[1]) / 1e6:.2f} MPa")

    # ---- size the shared remote sink on B1 (passive heat pipe) and round up to 5 mm steps
    b1 = next(x for x in CONCEPTS if x.key == "B1")
    sz = size_remote_sink(b1, TA_GATE, LOADS[0], TJ_THROTTLE - TJ_MARGIN, q_k)
    L_r = math.ceil(sz["L_mm"] / 5 - 1e-9) * 5e-3
    W_r = math.ceil(sz["W_mm"] / 5 - 1e-9) * 5e-3
    remote = replace(SINK_REMOTE, name=f"{W_r*1e3:.0f} x {L_r*1e3:.0f} x {SINK_REMOTE.H*1e3:.0f} remote / enclosure-wall sink",
                     W=W_r, L=L_r)
    out["remote_sink"] = dict(asdict(remote), sized_min=sz)
    concepts = [replace(c, sink=remote, title=c.title.replace("130x95x20", f"{W_r*1e3:.0f}x{L_r*1e3:.0f}x20"))
                if c.sink is SINK_REMOTE else c for c in CONCEPTS]
    out["inputs"]["sinks"][remote.name] = asdict(remote)

    for c in concepts:
        res = dict(title=c.title, kind=c.kind, sink=c.sink.name, forced=c.forced, tim=c.tim, corners={})
        for load in LOADS:
            for ta in TA_CORNERS:
                r = solve_concept(c, ta, load, carrier_q=q_k)
                rw = solve_concept(c, ta, load, carrier_q=q_k, worst_tim=True)
                res["corners"][f"{load.name}|{ta:.0f}"] = dict(
                    TJ=r["T"]["J"], TJ_worst_tim=rw["T"]["J"], T_case=r["T"]["C"], T_sink=r["T"]["S"],
                    T_vap_or_root=r["T"]["X"], T_module=r["T"]["M"], T_carrier=r["T"]["K"], R_sa=r["r_sa"],
                    R_ja_eff=r["r_ja_eff"], parts=r["parts"], flows=r["flows"], q_in=r["q_in"], q_out=r["q_out"],
                    n_fins=r["sink"]["n"], fin_gap_mm=r["sink"]["S"] * 1e3, h=r["sink"]["h"],
                    fan=r["sink"].get("fan"))
        L0 = LOADS[0]
        res["P_sustain_W"] = {f"{ta:.0f}": p_sustain(c, ta, L0, carrier_q=q_k) for ta in TA_CORNERS}
        res["P_sustain_worst_tim_W"] = {f"{ta:.0f}": p_sustain(c, ta, L0, carrier_q=q_k, worst_tim=True)
                                        for ta in TA_CORNERS}
        st = solve_concept(c, TA_GATE, L0, carrier_q=q_k)
        res["transient_gate"] = transient(c, TA_GATE, L0, st, carrier_q=q_k)
        # Thermal-governor view: above the trip, the OS caps power at P_sustain -> evaluate the
        # module/carrier temperatures the hardware actually sees at every corner.
        gov = {}
        for load in LOADS:
            for ta in TA_CORNERS:
                key = f"{load.name}|{ta:.0f}"
                if res["corners"][key]["TJ"] <= TJ_THROTTLE:
                    gov[key] = dict(P=load.p_soc + load.p_module_pcb, T_module=res["corners"][key]["T_module"],
                                    T_carrier=res["corners"][key]["T_carrier"], TJ=res["corners"][key]["TJ"])
                else:
                    pg = p_sustain(c, ta, load, carrier_q=q_k)
                    rg = solve_concept(c, ta, load, carrier_q=q_k, p_scale=pg / (load.p_soc + load.p_module_pcb))
                    gov[key] = dict(P=pg, T_module=rg["T"]["M"], T_carrier=rg["T"]["K"], TJ=rg["T"]["J"])
                if gov[key]["T_module"] > DF40_TMAX:     # mitigation: power cap that keeps the DF40 in rating
                    gov[key]["P_cap_df40"] = p_sustain(c, ta, load, target=DF40_TMAX, node="M", carrier_q=q_k)
        res["governed"] = gov
        if c.kind == "heatpipe":
            tv = st["T"]["X"]
            res["hp"] = dict(
                op=hp_limits(hp, tv, 0.0), op_adverse_90=hp_limits(hp, tv, 90.0),
                cold_start_0C=hp_limits(hp, 0.0, 0.0), cold_start_20C=hp_limits(hp, 20.0, 0.0),
                max_adverse_tilt_deg=hp_max_adverse_tilt(hp, tv, hp.safety * P_LOAD),
                max_adverse_tilt_cold20_deg=hp_max_adverse_tilt(hp, 20.0, P_LOAD),
                evap_flux_W_cm2=P_LOAD / (hp.flat_width * hp.L_e * 1e4),
                cold_start=_hp_cold_start(hp),
                geometry=dict(flat_width_mm=hp.flat_width * 1e3, vapour_mm=[v * 1e3 for v in hp.vapour],
                              A_v_mm2=hp.A_v * 1e6, A_w_mm2=hp.A_w * 1e6, L_total_mm=hp.L_total * 1e3,
                              L_eff_mm=hp.L_eff * 1e3))
        if not c.forced:
            res["sizing_gate"] = (sz if c.key == "B1" else size_remote_sink(c, TA_GATE, L0, TJ_THROTTLE - TJ_MARGIN, q_k)) \
                if c.sink is remote else None
        out["concepts"][c.key] = res

    # --- sensitivity (B1 and A3 at the gate)
    sens = {}
    for key in ("A3", "B1"):
        c = next(x for x in concepts if x.key == key)
        base_tj = solve_concept(c, TA_GATE, LOADS[0], carrier_q=q_k)["T"]["J"]
        rows = {}
        for lab, soc in (("thetaJC x0.5", replace(Soc(), theta_jc=Soc().theta_jc * 0.5)),
                         ("thetaJC x2", replace(Soc(), theta_jc=Soc().theta_jc * 2)),
                         ("thetaJB x0.5", replace(Soc(), theta_jb=Soc().theta_jb * 0.5)),
                         ("thetaJB x2", replace(Soc(), theta_jb=Soc().theta_jb * 2)),
                         ("die 6x6 mm", replace(Soc(), die=(6e-3, 6e-3))),
                         ("die 10x10 mm", replace(Soc(), die=(10e-3, 10e-3)))):
            rows[lab] = solve_concept(c, TA_GATE, LOADS[0], soc=soc, carrier_q=q_k)["T"]["J"] - base_tj
        if key == "B1":
            for lab, h2 in (("R_evap x2 (0.40 K cm2/W)", replace(HeatPipe(), r_evap=0.40e-4)),
                            ("R_cond x2", replace(HeatPipe(), r_cond=0.20e-4))):
                _BLOCK_CACHE.clear()
                rows[lab] = solve_concept(c, TA_GATE, LOADS[0], hp=h2, carrier_q=q_k)["T"]["J"] - base_tj
        sens[key] = dict(base_TJ=base_tj, delta=rows)
    out["sensitivity"] = sens

    # --- decision
    gate_key = f"{LOADS[0].name}|{TA_GATE:.0f}"
    feas_th = {k: v["corners"][gate_key]["TJ_worst_tim"] <= TJ_THROTTLE - TJ_MARGIN for k, v in out["concepts"].items()}
    feas_m = {k: v["transient_gate"]["mass_on_module_g"] <= MASS_ON_MODULE_MAX_G for k, v in out["concepts"].items()}
    feas = {k: feas_th[k] and feas_m[k] for k in feas_th}
    ranked = sorted(out["concepts"], key=lambda k: out["concepts"][k]["corners"][gate_key]["TJ_worst_tim"])
    passive = [k for k in ranked if feas[k] and not out["concepts"][k]["forced"]]
    active = [k for k in ranked if feas[k] and out["concepts"][k]["forced"]]
    rec = passive[0] if passive else (active[0] if active else ranked[0])
    out["decision"] = dict(feasible=feas, feasible_thermal=feas_th, feasible_mass=feas_m, ranked_by_TJ=ranked,
                           recommended=rec,
                           rule=f"meet TJ <= {TJ_THROTTLE - TJ_MARGIN:.0f} C at TA {TA_GATE:.0f} C with worst-case TIM AND "
                                f"mass on the module <= {MASS_ON_MODULE_MAX_G:.0f} g; passive preferred over fan (no "
                                "wear-out item, no acoustic noise); then lowest TJ")
    chk(f"Recommended concept {rec}: mass carried by the compute module <= {MASS_ON_MODULE_MAX_G:.0f} g",
        feas_m[rec], f"{out['concepts'][rec]['transient_gate']['mass_on_module_g']:.0f} g")

    # --- gating checks
    for k, v in out["concepts"].items():
        cg = v["corners"][gate_key]
        chk(f"{k}: energy balance closes (|Qin - Qout| < 1 mW) at the gate corner",
            abs(cg["q_in"] - cg["q_out"]) < 1e-3, f"Qin {cg['q_in']:.3f} W, Qout {cg['q_out']:.3f} W")
    r = out["concepts"][rec]
    cg = r["corners"][gate_key]
    chk(f"Recommended concept {rec}: TJ <= {TJ_THROTTLE - TJ_MARGIN:.0f} C (trip {TJ_THROTTLE:.0f} C - {TJ_MARGIN:.0f} K) "
        f"at TA {TA_GATE:.0f} C, 15 W continuous, worst-case TIM", cg["TJ_worst_tim"] <= TJ_THROTTLE - TJ_MARGIN,
        f"TJ {cg['TJ']:.1f} C nominal / {cg['TJ_worst_tim']:.1f} C worst TIM")
    w50 = r["corners"][f"{LOADS[0].name}|50"]
    chk(f"Recommended concept {rec}: TJ < critical trip {TJ_CRIT:.0f} C at TA 50 C (no shutdown)",
        w50["TJ_worst_tim"] < TJ_CRIT, f"TJ {w50['TJ_worst_tim']:.1f} C; sustained P at 50 C = "
        f"{r['P_sustain_W']['50']:.1f} W before throttling")
    for k, v in out["concepts"].items():
        kmax = max(v["governed"], key=lambda q: v["governed"][q]["T_module"])
        g_ = v["governed"][kmax]
        chk(f"{k}: DF40 receptacles (module PCB node) <= {DF40_TMAX:.0f} C at every corner and load split "
            "(power capped by the 85 C governor)", g_["T_module"] <= DF40_TMAX,
            f"max T_module {g_['T_module']:.1f} C at {kmax.split('|')[0].split(' (')[0]}, TA {kmax.split('|')[1]} C, "
            f"P {g_['P']:.1f} W" + (f"; mitigation: firmware cap {g_['P_cap_df40']:.1f} W at this corner"
                                    if "P_cap_df40" in g_ else ""), gate=(k == rec))
    for k, v in out["concepts"].items():
        tk = max(x["T_carrier"] for x in v["governed"].values())
        i_lim, _ = buck_derate_at(tk)
        chk(f"{k}: SY8089 buck derated I_max at the carrier temperature >= {CARRIER_3V3_BUDGET_A} A budget",
            i_lim >= CARRIER_3V3_BUDGET_A, f"T_carrier,max {tk:.1f} C (governed) -> I_max {i_lim:.2f} A "
            "(Rev 13 115 C/W model)", gate=(k == rec))
    if "hp" in out["concepts"]["B1"]:
        h = out["concepts"]["B1"]["hp"]
        chk("B1 heat pipe: derated operating limit >= 1.5 x 15 W, horizontal, at operating vapour T",
            h["op"]["q_max"] >= hp.safety * P_LOAD, f"{h['op']['q_max']:.1f} W ({h['op']['governing']}) at "
            f"T_v {h['op']['T_v']:.1f} C", gate=(rec == "B1"))
        chk("B1 heat pipe: starts and carries 15 W from a 20 C cold soak (horizontal)",
            h["cold_start_20C"]["q_max"] >= P_LOAD, f"{h['cold_start_20C']['q_max']:.1f} W "
            f"({h['cold_start_20C']['governing']})", gate=(rec == "B1"))
        chk("B1 heat pipe: evaporator heat flux < 60 % of sintered-wick CHF (30 W/cm^2)",
            h["evap_flux_W_cm2"] < 30.0, f"{h['evap_flux_W_cm2']:.1f} W/cm^2", gate=(rec == "B1"))
        chk("B1 heat pipe: >= 1.5x margin up to 30 deg adverse tilt (evaporator above condenser)",
            h["max_adverse_tilt_deg"] >= 30.0, f"max adverse tilt {h['max_adverse_tilt_deg']:.0f} deg at 22.5 W",
            gate=(rec == "B1"))
        g = h["geometry"]
        bend_r = 3 * hp.d
        chk("B1 heat pipe routing: bend radius >= 3 D and pipe clears the M2.5 standoff circles",
            _hp_route_clear(hp) and bend_r >= 3 * hp.d, f"R_bend {bend_r*1e3:.0f} mm, min standoff clearance "
            f"{_hp_route_clear(hp, report=True):.2f} mm, L_total {g['L_total_mm']:.0f} mm", gate=(rec == "B1"))
    out["checks"] = checks
    return out


def _hp_cold_start(hp: HeatPipe) -> dict:
    """Below ~20 C the viscous/capillary limits are < 15 W. The SoC + block self-heat (pipe stalled,
    only junction + Cu block heat capacity) until T_v reaches the temperature where Q_max >= 15 W."""
    t_star = None
    for tv in np.arange(0.0, 60.01, 0.5):
        if hp_limits(hp, float(tv), 0.0)["q_max"] >= P_LOAD:
            t_star = float(tv)
            break
    c_blk = hp.block[0] * hp.block[1] * hp.block[2] * CU.rho * CU.cp + 2.0
    return dict(T_v_15W=t_star, self_heat_s_from_0C=None if t_star is None else t_star * c_blk / P_LOAD,
                C_J_K=c_blk, note="water pipe: storage/operation below 0 C freezes the working fluid (sintered "
                                  "wicks tolerate freeze-thaw; cold start then follows this self-heating path)")


def _hp_route_clear(hp: HeatPipe, report: bool = False):
    """Pipe leaves the SoC along +X at y = SoC centre, flattened width centred; standoff keep-out r 3.1 mm."""
    y0 = Soc().centre[1]
    half = hp.flat_width * 1e3 / 2
    dmin = min(abs(hy - y0) - half - 3.1 for hx, hy in HOLES if hx > Soc().centre[0])
    return dmin if report else dmin >= 0.5


# ==========================================================================
# 9. Self-test
# ==========================================================================
def selftest() -> list[tuple[str, bool, str]]:
    rows = []
    # 1) FD vs exact 1D slab (source = full face)
    k, t, h = 200.0, 3e-3, 1000.0
    r_fd = fd3d_surface((20e-3, 20e-3), t, k, (20e-3, 20e-3), h, n=(10, 10, 12))
    r_ex = t / (k * 4e-4) + 1 / (h * 4e-4)
    rows.append(("FD solver == exact 1D slab", abs(r_fd / r_ex - 1) < 1e-3, f"{r_fd:.5f} vs {r_ex:.5f} K/W"))
    # 2) spreading series vs FD (two regimes)
    for src, plate, t, h in (((10e-3, 10e-3), (40e-3, 40e-3), 3e-3, 2000.0),
                             ((8e-3, 30e-3), (60e-3, 50e-3), 4e-3, 300.0)):
        r_an, r_sp = spreading_rect(src, plate, t, 209.0, h)
        r_n = fd3d_surface(plate, t, 209.0, src, h, n=(60, 60, 10))
        rows.append((f"Spreading series vs 3D FD, src {src[0]*1e3:.0f}x{src[1]*1e3:.0f} on "
                     f"{plate[0]*1e3:.0f}x{plate[1]*1e3:.0f}x{t*1e3:.0f}, h {h:.0f}",
                     abs(r_an / r_n - 1) < 0.03, f"series {r_an:.4f} (R_sp {r_sp:.4f}) vs FD {r_n:.4f} K/W "
                     f"({(r_an / r_n - 1) * 100:+.2f} %)"))
    # 3) Bar-Cohen optimum spacing vs scan
    s = SINK_REMOTE
    ts, ta = 75.0, 40.0
    p = air((ts + ta) / 2)
    P = G * p["beta"] * (ts - ta) / (p["nu"] * p["alpha"] * s.L)
    s_opt = 2.714 / P ** 0.25
    best = best_natural(s, ts, ta)
    rows.append(("Bar-Cohen S_opt = 2.714 P^-1/4 agrees with the brute-force fin-count optimum (+/-1.5 mm)",
                 abs(best["S"] - s_opt) < 1.5e-3, f"S_opt {s_opt*1e3:.2f} mm vs scan {best['S']*1e3:.2f} mm "
                 f"({best['n']} fins, h {best['h']:.2f} W/m2K)"))
    # 4) heat-pipe capillary limit in the published band for 6 mm sintered Cu/water round pipes
    rnd = replace(HeatPipe(), flat_factor=1.0, bends_90=0, L_e=50e-3, L_a=50e-3, L_c=50e-3)
    q = hp_limits(rnd, 60.0, 0.0)["limits"]["capillary"]
    rows.append(("Capillary limit, round 6 mm sintered, 150 mm, horizontal, 60 C in 25..80 W vendor band",
                 25.0 <= q <= 80.0, f"{q:.1f} W"))
    # 5) h sanity for a vertical plate
    hv = h_vplate(60.0, 25.0, 0.04)
    rows.append(("Churchill-Chu h(40 mm plate, dT 35 K) in 5..9 W/m2K", 5.0 <= hv <= 9.0, f"{hv:.2f} W/m2K"))
    # 6) network energy balance
    r = solve_concept(CONCEPTS[0], 40.0, LOADS[1], carrier_q=0.75)
    rows.append(("Network energy balance (A1, distributed load)", abs(r["q_in"] - r["q_out"]) < 1e-3,
                 f"in {r['q_in']:.4f} W, out {r['q_out']:.4f} W"))
    return rows


def cmd_selftest(_a) -> int:
    rows = selftest()
    for n, ok, d in rows:
        print(f"  [{'PASS' if ok else 'FAIL'}] {n}: {d}")
    bad = sum(not ok for _n, ok, _d in rows)
    print(f"\n{len(rows) - bad}/{len(rows)} self-tests pass")
    return 1 if bad else 0


# ==========================================================================
# 10. Report
# ==========================================================================
def _f(x, nd=1):
    return "-" if x is None or (isinstance(x, float) and math.isnan(x)) else f"{x:.{nd}f}"


def write_report(r: dict, st: list) -> None:
    L0 = LOADS[0].name
    lines = ["# Rev 15 - 15 W continuous-load thermal study: heat pipe vs. solid thermal pedestal", "",
             "Generated by `python3 tools/thermal_15w.py calc` (do not hand-edit). Inputs, sources and "
             "assumptions are listed in the tool header; every [A] assumption is swept below.", ""]
    n_gate = [c for c in r["checks"] if c["gate"]]
    npass = sum(c["ok"] for c in n_gate)
    lines += [f"**Recommended concept: {r['decision']['recommended']}** - "
              f"{r['concepts'][r['decision']['recommended']]['title']}.  "
              f"**{npass}/{len(n_gate)} gating checks pass**; "
              f"{sum(st_[1] for st_ in st)}/{len(st)} solver self-tests pass.", ""]
    d = r["decision"]
    lines += ["## 0. Decision matrix (gate: TA 40 C, 15 W at the SoC, worst-case TIM)", "",
              f"Rule: {d['rule']}.", "",
              "| Concept | TJ worst TIM | thermal gate | mass on module | mass gate | feasible |", "|---|---|---|---|---|---|"]
    for k in d["ranked_by_TJ"]:
        v = r["concepts"][k]
        c = v["corners"][f"{L0}|{TA_GATE:.0f}"]
        lines.append(f"| {k}{' **(recommended)**' if k == d['recommended'] else ''} | {c['TJ_worst_tim']:.1f} C | "
                     f"{'PASS' if d['feasible_thermal'][k] else 'fail'} | {v['transient_gate']['mass_on_module_g']:.0f} g | "
                     f"{'PASS' if d['feasible_mass'][k] else 'fail'} | {'**yes**' if d['feasible'][k] else 'no'} |")
    lines += [""]
    lines += ["## 1. Junction temperature, 15 W concentrated at the SoC", "",
              "| Concept | TA | TJ nom | TJ worst TIM | T_case | T_sink root | T_module (DF40) | T_carrier | R_sa K/W | "
              "R_ja,eff K/W | fins / gap |", "|---|---|---|---|---|---|---|---|---|---|---|"]
    for k, v in r["concepts"].items():
        for ta in TA_CORNERS:
            c = v["corners"][f"{L0}|{ta:.0f}"]
            lines.append(f"| {k} | {ta:.0f} | {c['TJ']:.1f} | {c['TJ_worst_tim']:.1f} | {c['T_case']:.1f} | "
                         f"{c['T_sink']:.1f} | {c['T_module']:.1f} | {c['T_carrier']:.1f} | {c['R_sa']:.2f} | "
                         f"{c['R_ja_eff']:.2f} | {c['n_fins']} / {c['fin_gap_mm']:.1f} mm |")
    lines += ["", "## 2. Distributed load (11.5 W SoC + 3.5 W DRAM/PMIC/eMMC into the module PCB)", "",
              "| Concept | TA | TJ nom | T_module | T_carrier |", "|---|---|---|---|---|"]
    L1 = LOADS[1].name
    for k, v in r["concepts"].items():
        for ta in TA_CORNERS:
            c = v["corners"][f"{L1}|{ta:.0f}"]
            lines.append(f"| {k} | {ta:.0f} | {c['TJ']:.1f} | {c['T_module']:.1f} | {c['T_carrier']:.1f} |")
    lines += ["", "## 2b. Governed operation (OS throttles at 85 C): power and board temperatures actually reached", "",
              "| Concept | load | TA | P W | TJ | T_module (DF40) | T_carrier (buck ambient) |", "|---|---|---|---|---|---|---|"]
    for k, v in r["concepts"].items():
        for key, g_ in v["governed"].items():
            ld, ta = key.split("|")
            lines.append(f"| {k} | {ld.split(' (')[0]} | {ta} | {g_['P']:.1f} | {g_['TJ']:.1f} | {g_['T_module']:.1f} | "
                         f"{g_['T_carrier']:.1f} |")
    lines += ["", "## 3. Thermal resistance breakdown at the gate corner (TA 40 C, 15 W, nominal TIM), K/W", ""]
    for k, v in r["concepts"].items():
        c = v["corners"][f"{L0}|{TA_GATE:.0f}"]
        parts = ", ".join(f"{n} {x:.3f}" for n, x in c["parts"].items())
        fl = ", ".join(f"{n} {x:.2f} W" for n, x in c["flows"].items())
        fan = c.get("fan")
        fan_s = (f"; fan operating point {fan['q']*1e3*2.119:.2f} CFM @ {fan['dp']:.1f} Pa, channel "
                 f"{fan['v']:.2f} m/s") if fan else ""
        lines.append(f"- **{k}** - {parts}; R_sa {c['R_sa']:.2f}; h {c['h']:.1f} W/m2K{fan_s}.  Heat flows: {fl}.")
    lines += ["", "## 4. Sustained power before the 85 C trip, transient and mass", "",
              "| Concept | P_sustain 25 C | 40 C | 50 C | 40 C worst TIM | t to 85 C trip @40 C | tau63 | TJ after 4 h "
              "(steady) | dissipator mass | on module |", "|---|---|---|---|---|---|---|---|---|---|"]
    for k, v in r["concepts"].items():
        p = v["P_sustain_W"]
        tr = v["transient_gate"]
        trip = "never" if tr["t_trip_s"] is None else f"{tr['t_trip_s']/60:.1f} min"
        t63 = "-" if tr["t63_s"] is None else f"{tr['t63_s']/60:.1f} min"
        lines.append(f"| {k} | {p['25']:.1f} W | {p['40']:.1f} W | {p['50']:.1f} W | "
                     f"{v['P_sustain_worst_tim_W']['40']:.1f} W | {trip} | {t63} | "
                     f"{tr['tj_end']:.1f} C ({tr['tj_ss']:.1f}) | {tr['mass_g']:.0f} g | {tr['mass_on_module_g']:.0f} g |")
    lines += ["", "Sizing (natural convection, TJ <= 80 C at TA 40 C, worst-case carrier heat): "]
    for k, v in r["concepts"].items():
        sz = v.get("sizing_gate")
        if sz:
            lines.append(f"- {k}: minimum remote/overhanging sink {sz['W_mm']:.0f} x {sz['L_mm']:.0f} x "
                         f"{sz['H_mm']:.0f} mm fins ({sz['volume_cm3']:.0f} cm^3)"
                         f"{'' if sz['feasible'] else ' - NOT FEASIBLE (exceeds the 412 x 300 mm search bound: TIM/pedestal resistance alone breaks the budget)'}")
    if "hp" in r["concepts"]["B1"]:
        h = r["concepts"]["B1"]["hp"]
        g = h["geometry"]
        lines += ["", "## 5. Heat-pipe operating envelope (B1)", "",
                  f"Flattened 6 mm pipe: width {g['flat_width_mm']:.2f} mm, vapour core "
                  f"{g['vapour_mm'][0]:.2f} x {g['vapour_mm'][1]:.2f} mm (A_v {g['A_v_mm2']:.2f} mm^2), wick "
                  f"{g['A_w_mm2']:.2f} mm^2, L_total {g['L_total_mm']:.0f} mm, L_eff {g['L_eff_mm']:.0f} mm. "
                  f"Evaporator flux {h['evap_flux_W_cm2']:.1f} W/cm^2.", "",
                  "| Condition | T_v C | capillary | viscous | sonic | entrainment | boiling | governing Q_max |",
                  "|---|---|---|---|---|---|---|---|"]
        for lab, key in (("operating, horizontal", "op"), ("operating, 90 deg adverse", "op_adverse_90"),
                         ("cold start 0 C", "cold_start_0C"), ("cold start 20 C", "cold_start_20C")):
            x = h[key]
            lm = x["limits"]
            lines.append(f"| {lab} | {x['T_v']:.1f} | {lm['capillary']:.1f} | {lm['viscous']:.3g} | "
                         f"{lm['sonic']:.3g} | {lm['entrainment']:.3g} | {lm['boiling']:.1f} | "
                         f"**{x['q_max']:.1f} W ({x['governing']})** |")
        cs = h["cold_start"]
        lines += ["", f"Cold start: Q_max reaches 15 W at T_v = {_f(cs['T_v_15W'])} C; from a 0 C soak the stalled "
                      f"evaporator (C = {cs['C_J_K']:.1f} J/K) self-heats there in {_f(cs['self_heat_s_from_0C'])} s at 15 W "
                      f"- far below the throttle time constant. {cs['note']}."]
        lines += ["", f"Maximum adverse tilt (evaporator above condenser) keeping 1.5 x 15 W: "
                      f"**{_f(h['max_adverse_tilt_deg'], 0)} deg** at operating T_v; carrying 15 W from a 20 C "
                      f"soak: {_f(h['max_adverse_tilt_cold20_deg'], 0)} deg."]
    lines += ["", "## 6. Rigid-pedestal TIM tolerance stack", "", "| Contributor | +/- mm |", "|---|---|"]
    for n, v in GAP_STACK:
        lines.append(f"| {n} | {v:.2f} |")
    gs = r["gap_stack"]
    lines += [f"| **Worst case / RSS** | **{gs['wc_mm']:.2f} / {gs['rss_mm']:.3f}** |", "",
              f"Gap-pad window {gs['gap_min_mm']:.2f}..{gs['gap_max_mm']:.2f} mm (1.0 mm pad, 10..50 % compression), "
              f"nominal gap {gs['gap_nom_mm']:.2f} mm, worst-case thick gap {gs['worst_gap_mm']:.2f} mm, "
              f"pedestal height {gs['pedestal_h_mm']:.2f} mm. The heat-pipe block is spring-loaded, so its "
              "phase-change BLT is independent of this stack."]
    lines += ["", "## 7. Sensitivity of TJ to the unpublished SoC/heat-pipe parameters (TA 40 C, 15 W)", "",
              "| Concept | base TJ | " + " | ".join(next(iter(r["sensitivity"].values()))["delta"].keys()) + " |"]
    keys = list(next(iter(r["sensitivity"].values()))["delta"].keys())
    lines.append("|---|---|" + "---|" * len(keys))
    for k, v in r["sensitivity"].items():
        lines.append(f"| {k} | {v['base_TJ']:.1f} | " + " | ".join(f"{v['delta'][x]:+.1f} K" for x in keys) + " |")
    for k, v in r["sensitivity"].items():
        extra = [x for x in v["delta"] if x not in keys]
        if extra:
            lines.append("")
            lines.append(f"{k} additional: " + ", ".join(f"{x} {v['delta'][x]:+.1f} K" for x in extra))
    lines += ["", "## 8. Checks", "", "| Check | Gate | Result | Detail |", "|---|---|---|---|"]
    for c in r["checks"]:
        lines.append(f"| {c['name']} | {'yes' if c['gate'] else 'info'} | "
                     f"{'PASS' if c['ok'] else '**FAIL**'} | {c['detail']} |")
    lines += ["", "## 9. Solver self-tests", "", "| Test | Result | Detail |", "|---|---|---|"]
    for n, ok, d in st:
        lines.append(f"| {n} | {'PASS' if ok else '**FAIL**'} | {d} |")
    REPORT.parent.mkdir(exist_ok=True)
    REPORT.write_text("\n".join(lines) + "\n")


def _jsonable(o):
    if isinstance(o, dict):
        return {str(k): _jsonable(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_jsonable(v) for v in o]
    if isinstance(o, float) and math.isnan(o):
        return None
    if isinstance(o, (np.floating,)):
        return float(o)
    return o


def cmd_calc(_a) -> int:
    st = selftest()
    r = calc()
    DESIGN_JSON.parent.mkdir(exist_ok=True)
    DESIGN_JSON.write_text(json.dumps(_jsonable(r), indent=1))
    write_report(r, st)
    L0 = LOADS[0].name
    print(f"  {'':4}{'TA':>4} {'TJ':>7} {'TJwc':>7} {'Tmod':>7} {'Tcar':>7} {'Rsa':>6}")
    for k, v in r["concepts"].items():
        for ta in TA_CORNERS:
            c = v["corners"][f"{L0}|{ta:.0f}"]
            print(f"  {k:4}{ta:4.0f} {c['TJ']:7.1f} {c['TJ_worst_tim']:7.1f} {c['T_module']:7.1f} "
                  f"{c['T_carrier']:7.1f} {c['R_sa']:6.2f}")
    print()
    bad = 0
    for c in r["checks"]:
        print(f"  [{'PASS' if c['ok'] else ('FAIL' if c['gate'] else 'info')}] {c['name']}: {c['detail']}")
        bad += c["gate"] and not c["ok"]
    bad += sum(not ok for _n, ok, _d in st)
    print(f"\nrecommended: {r['decision']['recommended']}  ->  {REPORT.relative_to(ROOT)}, "
          f"{DESIGN_JSON.relative_to(ROOT)}  ({bad} gating failures)")
    return 1 if bad else 0


# ==========================================================================
# 11. CAD
# ==========================================================================
def scad_text(r: dict | None = None) -> str:
    gs = gap_stack()
    hp = HeatPipe()
    n_foot = n_rem = None
    if r:
        g = f"{LOADS[0].name}|{TA_GATE:.0f}"
        n_foot = r["concepts"]["A1"]["corners"][g]["n_fins"]
        n_rem = r["concepts"]["B1"]["corners"][g]["n_fins"]
    rw, rl = (r["remote_sink"]["W"], r["remote_sink"]["L"]) if r else (SINK_REMOTE.W, SINK_REMOTE.L)
    n_foot = n_foot or 9
    n_rem = n_rem or 14
    sx, sy = Soc().centre
    holes = ", ".join(f"[{x:.2f},{y:.2f}]" for x, y in HOLES)
    return f"""// Rev 15 thermal concepts - GENERATED by tools/thermal_15w.py cad (do not hand-edit)
// Units mm. Carrier frame (X 0..55, Y 0..40, Y as KiCad but Z up from carrier top).
// Render:  openscad -D 'part="pedestal"' -o stl/rev15_pedestal_sink.stl cad/rev15_thermal_concepts.scad
//          openscad -D 'part="heatpipe"' -o stl/rev15_heatpipe_assembly.stl cad/rev15_thermal_concepts.scad
part = "both";                      // pedestal | heatpipe | stack | both
$fn = 40;
BOARD = [{BOARD_W}, {BOARD_H}];
HOLES = [{holes}];
CAR_T = {CAR_T*1e3:.3f};
STACK_H = {STACK_H*1e3:.2f};
MOD_T = {MOD_T*1e3:.2f};
SOC_C = [{sx}, {sy}];
SOC_PKG = [{Soc().pkg[0]*1e3:.1f}, {Soc().pkg[1]*1e3:.1f}];
SOC_TOP = {Soc().top_above_pcb*1e3:.2f};
DIE = [{Soc().die[0]*1e3:.1f}, {Soc().die[1]*1e3:.1f}];
// --- concept A (rigid integral pedestal)
BASE_CLR = {Pedestal().base_clear*1e3:.2f};       // sink-base underside above module PCB top
PED_TIP = [{Pedestal().tip[0]*1e3:.1f}, {Pedestal().tip[1]*1e3:.1f}];
PED_H = {gs['pedestal_h_mm']:.3f};                // = BASE_CLR - SOC_TOP - nominal gap-pad gap
GAP_NOM = {gs['gap_nom_mm']:.3f};
A_TB = {SINK_FOOT.tb*1e3:.1f}; A_H = {SINK_FOOT.H*1e3:.1f}; A_TF = {SINK_FOOT.tf*1e3:.1f}; A_N = {n_foot};
// --- concept B (heat pipe)
BLK = [{hp.block[0]*1e3:.1f}, {hp.block[1]*1e3:.1f}, {hp.block[2]*1e3:.1f}];
HP_W = {hp.flat_width*1e3:.3f}; HP_T = {hp.t_flat*1e3:.2f};
HP_LA = {hp.L_a*1e3:.1f}; HP_LC = {hp.L_c*1e3:.1f}; HP_RB = {3*hp.d*1e3:.1f};
R_W = {rw*1e3:.1f}; R_L = {rl*1e3:.1f}; R_TB = {SINK_REMOTE.tb*1e3:.1f};
R_H = {SINK_REMOTE.H*1e3:.1f}; R_TF = {SINK_REMOTE.tf*1e3:.1f}; R_N = {n_rem};

module_top = CAR_T + STACK_H + MOD_T;

module stack() {{
    color("green") difference() {{ cube([BOARD[0], BOARD[1], CAR_T]);
        for (h = HOLES) translate([h[0], h[1], -1]) cylinder(d = 2.7, h = CAR_T + 2); }}
    color("darkgreen") translate([0, 0, CAR_T + STACK_H]) difference() {{ cube([BOARD[0], BOARD[1], MOD_T]);
        for (h = HOLES) translate([h[0], h[1], -1]) cylinder(d = 2.6, h = MOD_T + 2); }}
    color("black") translate([SOC_C[0] - SOC_PKG[0]/2, SOC_C[1] - SOC_PKG[1]/2, module_top])
        cube([SOC_PKG[0], SOC_PKG[1], SOC_TOP]);
}}

module fins(w, l, tb, h, tf, n) {{
    s = (w - n * tf) / (n - 1);
    cube([w, l, tb]);
    for (i = [0 : n - 1]) translate([i * (tf + s), 0, tb]) cube([tf, l, h]);
}}

module pedestal_sink() {{
    z0 = module_top + BASE_CLR;
    difference() {{
        union() {{
            translate([0, 0, z0]) fins(BOARD[0], BOARD[1], A_TB, A_H, A_TF, A_N);
            translate([SOC_C[0] - PED_TIP[0]/2, SOC_C[1] - PED_TIP[1]/2, z0 - PED_H]) cube([PED_TIP[0], PED_TIP[1], PED_H + 0.01]);
        }}
        for (h = HOLES) translate([h[0], h[1], z0 - 1]) cylinder(d = 2.7, h = A_TB + A_H + 2);   // M2.5 clearance
        for (h = HOLES) translate([h[0], h[1], z0 + A_TB]) cylinder(d = 6.4, h = A_H + 1);      // driver access
    }}
    color("grey") for (h = HOLES) translate([h[0], h[1], module_top]) cylinder(d = 5.0, h = BASE_CLR, $fn = 6);
}}

module heatpipe_assembly() {{
    zb = module_top + SOC_TOP + 0.03;
    color("orange") translate([SOC_C[0] - BLK[0]/2, SOC_C[1] - BLK[1]/2, zb]) cube(BLK);
    // flattened pipe: evaporator over the block, +X run to the board edge, 90 deg bend, condenser along the sink
    zp = zb + BLK[2];
    color("peru") {{
        translate([SOC_C[0] - BLK[0]/2, SOC_C[1] - HP_W/2, zp]) cube([BLK[0] + HP_LA - HP_RB, HP_W, HP_T]);
        translate([SOC_C[0] + BLK[0]/2 + HP_LA - HP_RB, SOC_C[1] + HP_RB - HP_W/2, zp])
            rotate([0, 0, -90]) rotate_extrude(angle = 90) translate([HP_RB - HP_W/2, 0]) square([HP_W, HP_T]);
        translate([SOC_C[0] + BLK[0]/2 + HP_LA - HP_W/2, SOC_C[1] + HP_RB, zp]) cube([HP_W, HP_LC, HP_T]);
    }}
    // remote sink: base under the condenser run, fins up
    translate([SOC_C[0] + BLK[0]/2 + HP_LA - R_W/2, SOC_C[1] + HP_RB - 4, zp - R_TB]) {{
        fins(R_W, R_L, R_TB, R_H + HP_T, R_TF, R_N);
    }}
    // spring clip bar across the block, anchored on the two +X standoffs
    color("silver") translate([SOC_C[0] - 1.5, 3.5 - 2, zp + HP_T]) cube([3, BOARD[1] - 3, 0.6]);
}}

if (part == "pedestal") pedestal_sink();
else if (part == "heatpipe") heatpipe_assembly();
else if (part == "stack") stack();
else {{ stack(); pedestal_sink(); translate([0, 70, 0]) {{ stack(); heatpipe_assembly(); }} }}
"""


def cmd_cad(_a) -> int:
    r = json.loads(DESIGN_JSON.read_text()) if DESIGN_JSON.exists() else None
    SCAD.parent.mkdir(exist_ok=True)
    SCAD.write_text(scad_text(r))
    print(f"wrote {SCAD.relative_to(ROOT)}")
    exe = shutil.which("openscad")
    if not exe:
        print("openscad not found - STL render skipped")
        return 0
    STL_DIR.mkdir(exist_ok=True)
    rc = 0
    for part, name in (("pedestal", "rev15_pedestal_sink.stl"), ("heatpipe", "rev15_heatpipe_assembly.stl")):
        out = STL_DIR / name
        p = subprocess.run([exe, "-D", f'part="{part}"', "-o", str(out), str(SCAD)], capture_output=True, text=True)
        ok = p.returncode == 0 and out.exists() and out.stat().st_size > 1000
        print(f"  [{'PASS' if ok else 'FAIL'}] render {name} ({out.stat().st_size if out.exists() else 0} B)")
        if not ok:
            print(p.stderr[-2000:])
            rc = 1
    return rc


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp_ = ap.add_subparsers(dest="cmd", required=True)
    sp_.add_parser("selftest").set_defaults(fn=cmd_selftest)
    sp_.add_parser("calc").set_defaults(fn=cmd_calc)
    sp_.add_parser("cad").set_defaults(fn=cmd_cad)
    a = ap.parse_args()
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
