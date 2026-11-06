#!/usr/bin/env python3
"""
Radxa CM4 Nano Carrier - controlled-impedance model for the 1000BASE-T MDI
(TRD0..TRD3) pairs.  Rev 9.0.

Single source of truth for the stackup and the 100-ohm differential geometry
that tools/ethernet_mdi.py writes into the board (netclass, DRU, tracks).

Two INDEPENDENT impedance models are used and must agree:

  1. Closed form - Hammerstad & Jensen single microstrip (with Wheeler /
     Hammerstad strip-thickness correction) + Kirschning & Jansen (1984)
     coupled-microstrip even/odd mode equations, frequency-independent
     (quasi-static) form.  Uncoated (no solder mask).
  2. 2-D finite-difference Laplace solver (quasi-static, 5-point stencil
     with face-averaged permittivity, discrete Gauss-law charge).  Models the
     real cross-section: finite copper thickness, solder mask conformal over
     the traces and substrate.  Z = 1 / (c0 * sqrt(C * C_air)).

The FD solver is the sign-off number; the closed form is a sanity cross-check
(it ignores the solder mask, which typically lowers Zdiff by 2-4 ohm).

Stackup: JLCPCB JLC04161H-7628 (1.6 mm, 4 layer) - the board's 1.6 mm FR4
4-layer build in specs/carrier_specs.md.

    L1 F.Cu    35 um  (1 oz finished)       signal  -> ref L2
    PP 7628   210.4 um  er 4.4
    L2 In1.Cu  15.2 um (0.5 oz)            GND plane (solid)
    Core     1065 um   er 4.6
    L3 In2.Cu  15.2 um                      GND plane (solid under MDI)
    PP 7628   210.4 um  er 4.4
    L4 B.Cu    35 um                        signal (MDI) -> ref L3
    Solder mask 10 um er 3.8 both sides

Usage:
    python3 tools/mdi_impedance.py            # print design + tolerance table
    python3 tools/mdi_impedance.py --solve    # re-solve W for 100 ohm at chosen S
    python3 tools/mdi_impedance.py --check    # exit 1 if design out of +/-10 %
"""
from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass

C0 = 299_792_458.0          # m/s
EPS0 = 8.8541878128e-12     # F/m
ETA0 = 376.730313668        # ohm


# ---------------------------------------------------------------------------
# Stackup (mm)
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Stackup:
    name: str = "JLC04161H-7628"
    t_outer: float = 0.035
    t_inner: float = 0.0152
    h_pp: float = 0.2104            # prepreg L1-L2 and L3-L4
    er_pp: float = 4.4
    h_core: float = 1.065
    er_core: float = 4.6
    t_mask: float = 0.010
    er_mask: float = 3.8
    tan_d: float = 0.02

    @property
    def total(self) -> float:
        return 2 * self.t_outer + 2 * self.t_inner + 2 * self.h_pp + self.h_core


STACKUP = Stackup()

# ---------------------------------------------------------------------------
# MDI design targets and the chosen geometry (written by --solve, then frozen)
# ---------------------------------------------------------------------------
Z_DIFF_TARGET = 100.0             # IEEE 802.3 clause 40 MDI, 100 ohm +/-15 % cable
Z_DIFF_TOL = 0.10                 # board budget +/-10 %
MDI_W = 0.150                     # trace width (mm)       } FD-solved for 100 ohm
MDI_S = 0.150                     # edge-to-edge gap (mm)  } on L4 over L3 GND
MDI_PAIR_PITCH = MDI_W + MDI_S    # P/N centreline spacing
MDI_CLEARANCE = 0.30              # to any other net (>= 2*W, ~1.4 h) - DRU enforced
MDI_INTERPAIR = 0.80              # pair-to-pair edge spacing (>= 0.76 mm / 30 mil, ~3.8 h)
MDI_MAX_UNCOUPLED = 2.0           # DF40 / transformer fan-out stubs (mm)
MDI_INTRA_SKEW_MAX = 0.15         # mm  (~1 ps on FR4; 802.3 budget is far looser)
MDI_MAX_LEN = 50.0                # mm  PHY-module pin to RJ45 contact, per side


# ---------------------------------------------------------------------------
# 1. Closed form (Hammerstad-Jensen + Kirschning-Jansen), quasi-static
# ---------------------------------------------------------------------------
def _hj_z0_eeff(u: float, er: float) -> tuple[float, float]:
    """Hammerstad & Jensen zero-thickness microstrip; u = W/h."""
    a = 1 + math.log((u ** 4 + (u / 52) ** 2) / (u ** 4 + 0.432)) / 49 \
        + math.log(1 + (u / 18.1) ** 3) / 18.7
    b = 0.564 * ((er - 0.9) / (er + 3)) ** 0.053
    eeff = (er + 1) / 2 + (er - 1) / 2 * (1 + 10 / u) ** (-a * b)
    f = 6 + (2 * math.pi - 6) * math.exp(-((30.666 / u) ** 0.7528))
    z01 = ETA0 / (2 * math.pi) * math.log(f / u + math.sqrt(1 + (2 / u) ** 2))
    return z01 / math.sqrt(eeff), eeff


def _thickness_du(u: float, t_h: float, er: float) -> float:
    """Hammerstad-Jensen width correction for strip thickness (dielectric)."""
    if t_h <= 0:
        return 0.0
    du1 = t_h / math.pi * math.log(1 + 4 * math.e / (t_h * (1 / math.tanh(math.sqrt(6.517 * u))) ** 2))
    return 0.5 * (1 + 1 / math.cosh(math.sqrt(er - 1))) * du1


def microstrip_closed(w: float, h: float, t: float, er: float) -> tuple[float, float]:
    u = w / h
    ur = u + _thickness_du(u, t / h, er)
    z, eeff = _hj_z0_eeff(ur, er)
    return z, eeff


def coupled_closed(w: float, s: float, h: float, t: float, er: float) -> dict:
    """Kirschning & Jansen quasi-static coupled microstrip, with the
    Hammerstad-Jensen thickness-corrected width.  Returns Zodd, Zeven, Zdiff,
    Zcomm and effective permittivities."""
    u0 = w / h
    g = s / h
    # Strip-thickness correction for COUPLED lines (Kirschning/Jansen, as used
    # in the Qucs technical documentation): the even mode sees a reduced
    # widening, the odd mode additionally sees the side-wall capacitance
    # across the gap (dt = t / (s * er)).
    du = _thickness_du(u0, t / h, er)
    dt = (t / h) / (g * er) if t > 0 else 0.0
    du_e = du * (1 - 0.5 * math.exp(-0.69 * du / dt)) if dt > 0 else 0.0
    du_o = du_e + dt
    ze_, ee_ = _kj_modes(u0 + du_e, g, er)
    zo_, eo_ = _kj_modes(u0 + du_o, g, er)
    z0, ee0 = _hj_z0_eeff(u0 + du, er)
    return dict(z0=z0, zodd=zo_["zodd"], zeven=ze_["zeven"], zdiff=2 * zo_["zodd"],
                zcomm=ze_["zeven"] / 2, eeff_odd=eo_["eeff_odd"], eeff_even=ee_["eeff_even"])


def _kj_modes(u: float, g: float, er: float) -> tuple[dict, dict]:
    """Kirschning-Jansen even/odd impedance + eeff at normalised width u."""
    z0, ee0 = _hj_z0_eeff(u, er)
    # even/odd effective permittivity (K&J eq. 3-6)
    v = u * (20 + g * g) / (10 + g * g) + g * math.exp(-g)
    ae = 1 + math.log((v ** 4 + (v / 52) ** 2) / (v ** 4 + 0.432)) / 49 + math.log(1 + (v / 18.1) ** 3) / 18.7
    be = 0.564 * ((er - 0.9) / (er + 3)) ** 0.053
    eee = (er + 1) / 2 + (er - 1) / 2 * (1 + 10 / v) ** (-ae * be)
    a0 = 0.7287 * (ee0 - (er + 1) / 2) * (1 - math.exp(-0.179 * u))
    b0 = 0.747 * er / (0.15 + er)
    c0 = b0 - (b0 - 0.207) * math.exp(-0.414 * u)
    d0 = 0.593 + 0.694 * math.exp(-0.562 * u)
    eeo = ((er + 1) / 2 + a0 - ee0) * math.exp(-c0 * g ** d0) + ee0
    # even/odd impedance (K&J eq. 8-14)
    q1 = 0.8695 * u ** 0.194
    q2 = 1 + 0.7519 * g + 0.189 * g ** 2.31
    q3 = 0.1975 + (16.6 + (8.4 / g) ** 6) ** -0.387 + math.log(g ** 10 / (1 + (g / 3.4) ** 10)) / 241
    q4 = 2 * q1 / q2 / (math.exp(-g) * u ** q3 + (2 - math.exp(-g)) * u ** -q3)
    q5 = 1.794 + 1.14 * math.log(1 + 0.638 / (g + 0.517 * g ** 2.43))
    q6 = 0.2305 + math.log(g ** 10 / (1 + (g / 5.8) ** 10)) / 281.3 + math.log(1 + 0.598 * g ** 1.154) / 5.1
    q7 = (10 + 190 * g * g) / (1 + 82.3 * g ** 3)
    q8 = math.exp(-6.5 - 0.95 * math.log(g) - (g / 0.15) ** 5)
    q9 = math.log(q7) * (q8 + 1 / 16.5)
    q10 = (q2 * q4 - q5 * math.exp(math.log(u) * q6 * u ** -q9)) / q2
    ze = math.sqrt(ee0 / eee) * z0 / (1 - z0 / ETA0 * math.sqrt(ee0) * q4)
    zo = math.sqrt(ee0 / eeo) * z0 / (1 - z0 / ETA0 * math.sqrt(ee0) * q10)
    r = dict(zodd=zo, zeven=ze, eeff_odd=eeo, eeff_even=eee)
    return r, r


# ---------------------------------------------------------------------------
# 2. 2-D finite-difference field solver (quasi-static)
# ---------------------------------------------------------------------------
def fd_coupled(w: float, s: float, h: float, t: float, er: float,
               t_mask: float = 0.0, er_mask: float = 1.0, dx: float = 0.005,
               margin: float = 1.6, top: float = 1.6) -> dict:
    """Coupled microstrip over an infinite ground plane in a grounded box.

    Grid is uniform (dx mm).  Ground plane at row 0.  Box side/top walls at
    V=0 placed `margin` / `top` mm away (>= 7 h - box error < 0.3 %).
    Returns Zodd, Zeven, Zdiff, Zcomm, eeff_odd/even."""
    import numpy as np
    import scipy.sparse as sp
    import scipy.sparse.linalg as spla

    def n(x):
        return int(round(x / dx))

    nw, ns, nh, nt, nm = n(w), n(s), n(h), max(1, n(t)), n(t_mask)
    nmar, ntop = n(margin), n(top)
    nx = 2 * nmar + 2 * nw + ns + 1             # nodes
    ny = nh + nt + ntop + 1
    # conductors: x node range [x0, x1], y node range [nh, nh+nt]
    xa0 = nmar
    xa1 = xa0 + nw
    xb0 = xa1 + ns
    xb1 = xb0 + nw

    # cell permittivity (cells indexed [j, i] for j in 0..ny-2, i in 0..nx-2)
    def eps_map(with_diel: bool):
        e = np.ones((ny - 1, nx - 1))
        if not with_diel:
            return e
        e[:nh, :] = er
        if nm > 0:
            # conformal mask: nm cells over the substrate between/beside traces
            e[nh:nh + nm, :] = er_mask
            # over the traces: mask on top and on the side walls
            for (c0, c1) in ((xa0, xa1), (xb0, xb1)):
                e[nh + nt:nh + nt + nm, max(0, c0 - nm):min(nx - 1, c1 + nm)] = er_mask
                e[nh:nh + nt, max(0, c0 - nm):c0] = er_mask
                e[nh:nh + nt, c1:min(nx - 1, c1 + nm)] = er_mask
        return e

    cond = np.zeros((ny, nx), dtype=np.int8)       # 0 free, 1 = A, 2 = B, 3 = ground
    cond[0, :] = 3
    cond[-1, :] = 3
    cond[:, 0] = 3
    cond[:, -1] = 3
    cond[nh:nh + nt + 1, xa0:xa1 + 1] = 1
    cond[nh:nh + nt + 1, xb0:xb1 + 1] = 2

    idx = -np.ones((ny, nx), dtype=np.int64)
    free = cond == 0
    idx[free] = np.arange(free.sum())
    N = int(free.sum())

    def solve(eps):
        """Return (C_odd, C_even) per unit length for one permittivity map.
        Odd and even modes share the system matrix -> one LU factorisation."""
        epad = np.pad(eps, ((1, 1), (1, 1)), mode="edge")
        # horizontal edge (j,i)-(j,i+1) borders cells (j-1,i) and (j,i)
        ex = 0.5 * (epad[0:ny, 1:nx] + epad[1:ny + 1, 1:nx])        # (ny, nx-1)
        # vertical edge (j,i)-(j+1,i) borders cells (j,i-1) and (j,i)
        ey = 0.5 * (epad[1:ny, 0:nx] + epad[1:ny, 1:nx + 1])        # (ny-1, nx)
        jj, ii = np.nonzero(free)
        k = idx[jj, ii]
        rows, cols, vals = [], [], []
        diag = np.zeros(N)
        fixed_terms = []
        for (dj, di, coef) in ((0, 1, ex[jj, ii]), (0, -1, ex[jj, ii - 1]),
                               (1, 0, ey[jj, ii]), (-1, 0, ey[jj - 1, ii])):
            nj, ni = jj + dj, ii + di
            diag += coef
            nb_free = free[nj, ni]
            rows.append(k[nb_free]); cols.append(idx[nj, ni][nb_free]); vals.append(-coef[nb_free])
            fx = ~nb_free
            fixed_terms.append((k[fx], coef[fx], nj[fx], ni[fx]))
        rows.append(k); cols.append(k); vals.append(diag)
        A = sp.csc_matrix((np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))), shape=(N, N))
        lu = spla.splu(A)
        caps = []
        for vb in (-1.0, 1.0):                      # odd, even (conductor A always +1)
            V = np.zeros((ny, nx))
            V[cond == 1] = 1.0
            V[cond == 2] = vb
            rhs = np.zeros(N)
            for kk, cc, nj, ni in fixed_terms:
                np.add.at(rhs, kk, cc * V[nj, ni])
            V[free] = lu.solve(rhs)
            # discrete Gauss law: net flux out of conductor A (consistent with
            # the stencil, so the charge is exact for the discrete problem)
            a = cond == 1
            q = 0.0
            m = a[:, :-1] & ~a[:, 1:]
            q += (ex[m] * (V[:, :-1][m] - V[:, 1:][m])).sum()
            m = a[:, 1:] & ~a[:, :-1]
            q += (ex[:, :][m] * (V[:, 1:][m] - V[:, :-1][m])).sum()
            m = a[:-1, :] & ~a[1:, :]
            q += (ey[m] * (V[:-1, :][m] - V[1:, :][m])).sum()
            m = a[1:, :] & ~a[:-1, :]
            q += (ey[m] * (V[1:, :][m] - V[:-1, :][m])).sum()
            caps.append(q * EPS0)                   # F/m; dx cancels on a square grid
        return caps

    (c_odd, c_even), (c_odd0, c_even0) = solve(eps_map(True)), solve(eps_map(False))
    zo = 1 / (C0 * math.sqrt(c_odd * c_odd0))
    ze = 1 / (C0 * math.sqrt(c_even * c_even0))
    return dict(zodd=zo, zeven=ze, zdiff=2 * zo, zcomm=ze / 2,
                eeff_odd=c_odd / c_odd0, eeff_even=c_even / c_even0,
                grid=f"{nx}x{ny} @ {dx*1000:.1f} um")


def fd_single(w, h, t, er, t_mask=0.0, er_mask=1.0, dx=0.005):
    """Single-ended Z0 via the even/odd solver at very wide spacing."""
    r = fd_coupled(w, 2.0, h, t, er, t_mask, er_mask, dx)
    return r["zeven"]


def design_point(w=MDI_W, s=MDI_S, su=STACKUP, dx=0.005) -> dict:
    cf = coupled_closed(w, s, su.h_pp, su.t_outer, su.er_pp)
    fd = fd_coupled(w, s, su.h_pp, su.t_outer, su.er_pp, su.t_mask, su.er_mask, dx=dx)
    fd_nomask = fd_coupled(w, s, su.h_pp, su.t_outer, su.er_pp, 0.0, 1.0, dx=dx)
    return dict(closed=cf, fd=fd, fd_nomask=fd_nomask)


def solve_w(s: float, target=Z_DIFF_TARGET, su=STACKUP, lo=0.08, hi=0.30, dx=0.005) -> float:
    f = lambda w: fd_coupled(w, s, su.h_pp, su.t_outer, su.er_pp, su.t_mask, su.er_mask, dx=dx)["zdiff"] - target
    for _ in range(14):
        mid = round((lo + hi) / 2 / dx) * dx
        if f(mid) > 0:
            lo = mid
        else:
            hi = mid
        if hi - lo <= dx + 1e-9:
            break
    return lo if abs(f(lo)) < abs(f(hi)) else hi


def tolerance_corners(w=MDI_W, s=MDI_S, su=STACKUP) -> list[dict]:
    """Fab corners: etch +/-0.02 mm on W (gap moves opposite), prepreg +/-10 %,
    er +/-0.2.  Evaluated with the FD solver (mask on)."""
    out = []
    for dw in (-0.02, 0.0, 0.02):
        for kh in (0.9, 1.0, 1.1):
            for der in (-0.2, 0.0, 0.2):
                if (dw, kh, der) != (0.0, 1.0, 0.0) and sum(x != y for x, y in zip((dw, kh, der), (0.0, 1.0, 0.0))) < 3:
                    continue
                r = fd_coupled(w + dw, s - dw, su.h_pp * kh, su.t_outer, su.er_pp + der,
                               su.t_mask, su.er_mask, dx=0.005)
                out.append(dict(dw=dw, kh=kh, der=der, zdiff=r["zdiff"]))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--solve", action="store_true", help="solve W for 100 ohm at S = MDI_S and several S")
    ap.add_argument("--check", action="store_true", help="exit 1 unless design within +/-10 %% at all corners")
    ap.add_argument("--corners", action="store_true", help="print fab tolerance corners")
    a = ap.parse_args()
    su = STACKUP
    print(f"Stackup {su.name}: total {su.total:.4f} mm, PP {su.h_pp} mm er {su.er_pp}, "
          f"Cu {su.t_outer} mm, mask {su.t_mask} mm er {su.er_mask}")
    if a.solve:
        for s in (0.127, 0.150, 0.175, 0.200):
            w = solve_w(s)
            r = fd_coupled(w, s, su.h_pp, su.t_outer, su.er_pp, su.t_mask, su.er_mask)
            print(f"  S={s:.3f}  ->  W={w:.3f}  Zdiff(FD)={r['zdiff']:.2f}  Zodd={r['zodd']:.2f}  Zcomm={r['zcomm']:.2f}")
    d = design_point()
    cf, fd, fn = d["closed"], d["fd"], d["fd_nomask"]
    print(f"Design W={MDI_W} S={MDI_S} mm on L4 (B.Cu) over L3 GND (and L1 over L2):")
    print(f"  FD solver, masked   : Zdiff {fd['zdiff']:.2f}  Zodd {fd['zodd']:.2f}  Zcomm {fd['zcomm']:.2f}  "
          f"eeff_odd {fd['eeff_odd']:.3f}  [{fd['grid']}]")
    print(f"  FD solver, bare     : Zdiff {fn['zdiff']:.2f}  Zodd {fn['zodd']:.2f}  Zcomm {fn['zcomm']:.2f}")
    print(f"  Kirschning-Jansen   : Zdiff {cf['zdiff']:.2f}  Zodd {cf['zodd']:.2f}  Zcomm {cf['zcomm']:.2f}  (bare)")
    agree = abs(cf["zdiff"] - fn["zdiff"]) / fn["zdiff"]
    print(f"  closed-form vs FD (bare) disagreement: {agree*100:.2f} %")
    ok = abs(fd["zdiff"] - Z_DIFF_TARGET) <= Z_DIFF_TARGET * 0.05 and agree < 0.05
    if a.corners or a.check:
        cs = tolerance_corners()
        zs = [c["zdiff"] for c in cs]
        for c in cs:
            print(f"  corner dW={c['dw']:+.2f} h x{c['kh']:.1f} der={c['der']:+.1f}: Zdiff {c['zdiff']:.2f}")
        print(f"  corner range {min(zs):.2f} .. {max(zs):.2f} ohm (limit {Z_DIFF_TARGET*(1-Z_DIFF_TOL):.0f}"
              f"..{Z_DIFF_TARGET*(1+Z_DIFF_TOL):.0f})")
        ok = ok and min(zs) >= Z_DIFF_TARGET * (1 - Z_DIFF_TOL) and max(zs) <= Z_DIFF_TARGET * (1 + Z_DIFF_TOL)
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
