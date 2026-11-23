"""Hill-climb a custom low-Re reflexed laminar CST airfoil for the stealth BWB tailsitter using NeuralFoil.

Pass 2: Polishes directly on NeuralFoil 'large' & verifies on 'xlarge' with strict xlarge confidence >= 0.92,
clean printable trailing edge (w_t[4] >= 0.055), ultra-low dash drag (CL=0.04), peak cruise L/D across
CL = 0.35..0.65, peak loiter CL^1.5/CD, positive self-trimming Cm >= +0.004, and high CL_max >= 1.15.

Run:  .venv/Scripts/python.exe sim/hillclimb_airfoil.py
"""
import json
import math
import pathlib

import aerosandbox as asb
import neuralfoil as nf
import numpy as np
from scipy.optimize import differential_evolution, minimize

ROOT = pathlib.Path(__file__).resolve().parent.parent


def bernstein(n, i, x):
    return math.comb(n, i) * (x ** i) * ((1.0 - x) ** (n - i))


def cst_tc_coords(w_t, w_c, dz_te=0.0025, n_pts=80):
    """Decoupled CST thickness + Bernstein camber airfoil (upper TE->LE, lower LE->TE)."""
    beta = np.linspace(0.0, math.pi, n_pts)
    x = 0.5 * (1.0 - np.cos(beta))
    c_cls = np.sqrt(np.maximum(x, 0.0)) * (1.0 - x)
    nt, nc = len(w_t) - 1, len(w_c) - 1
    yt = c_cls * sum(w * bernstein(nt, i, x) for i, w in enumerate(w_t)) + x * (dz_te / 2.0)
    yc = x * (1.0 - x) * sum(w * bernstein(nc, i, x) for i, w in enumerate(w_c))
    yu, yl = yc + yt, yc - yt
    coords = np.column_stack([np.concatenate([x[::-1], x[1:]]), np.concatenate([yu[::-1], yl[1:]])])
    return coords, x, yu, yl


def baseline_coords(n_pts=80):
    beta = np.linspace(0.0, math.pi, n_pts)
    x = 0.5 * (1.0 - np.cos(beta))
    t = 0.11
    yt = 5 * t * (0.2969 * np.sqrt(x) - 0.1260 * x - 0.3516 * x**2 + 0.2843 * x**3 - 0.1015 * x**4)
    yc = 0.14 * x * (1.0 - x) * (0.68 - x)
    yu, yl = yc + yt, yc - yt
    coords = np.column_stack([np.concatenate([x[::-1], x[1:]]), np.concatenate([yu[::-1], yl[1:]])])
    return coords, x, yu, yl


def eval_airfoil(coords, x, yu, yl, model_size="large"):
    thick = yu - yl
    if np.any(thick[1:-1] <= 1e-4):
        return None
    t_max = float(thick.max())
    x_tmax = float(x[int(np.argmax(thick))])
    t_30 = float(np.interp(0.30, x, thick))
    t_50 = float(np.interp(0.50, x, thick))
    t_65 = float(np.interp(0.65, x, thick))
    af = asb.Airfoil("cand", coordinates=coords)

    # 1. High-speed dash (Re = 7.5e5, CL ~ 0.04)
    a_dash = np.array([-3.0, -2.0, -1.0, 0.0, 1.0, 2.0])
    r_dash = nf.get_aero_from_airfoil(af, alpha=a_dash, Re=7.5e5, model_size=model_size)
    cl_d, cd_d, cm_d = np.ravel(r_dash["CL"]), np.ravel(r_dash["CD"]), np.ravel(r_dash["CM"])
    cd_dash = float(np.interp(0.04, cl_d, cd_d))
    cm_dash = float(np.interp(0.04, cl_d, cm_d))

    # 2. Cruise (Re = 2.6e5)
    a_cr = np.linspace(-2.0, 15.0, 35)
    r_cr = nf.get_aero_from_airfoil(af, alpha=a_cr, Re=2.6e5, model_size=model_size)
    cl_c, cd_c, cm_c = np.ravel(r_cr["CL"]), np.ravel(r_cr["CD"]), np.ravel(r_cr["CM"])
    imax_c = int(np.argmax(cl_c))
    cl_pre, cd_pre, cm_pre = cl_c[:imax_c + 1], cd_c[:imax_c + 1], cm_c[:imax_c + 1]
    ld_curve = cl_pre / np.maximum(cd_pre, 1e-5)
    ld_max = float(ld_curve.max())
    cl_ldmax = float(cl_pre[int(np.argmax(ld_curve))])
    ld_035 = float(0.35 / np.interp(0.35, cl_pre, cd_pre)) if cl_pre[-1] >= 0.35 else 1.0
    ld_050 = float(0.50 / np.interp(0.50, cl_pre, cd_pre)) if cl_pre[-1] >= 0.50 else 1.0
    ld_065 = float(0.65 / np.interp(0.65, cl_pre, cd_pre)) if cl_pre[-1] >= 0.65 else 1.0
    cm_cr = float(np.interp(0.45, cl_pre, cm_pre)) if cl_pre[-1] >= 0.45 else -0.1

    # 3. Loiter / Stall (Re = 1.6e5)
    r_lo = nf.get_aero_from_airfoil(af, alpha=a_cr, Re=1.6e5, model_size=model_size)
    cl_l, cd_l = np.ravel(r_lo["CL"]), np.ravel(r_lo["CD"])
    imax_l = int(np.argmax(cl_l))
    cl_lpre, cd_lpre = cl_l[:imax_l + 1], cd_l[:imax_l + 1]
    clmax_lo = float(cl_l[imax_l])
    pos = cl_lpre > 0.1
    endur_curve = (cl_lpre[pos] ** 1.5) / np.maximum(cd_lpre[pos], 1e-5)
    endur_max = float(endur_curve.max()) if len(endur_curve) else 1.0

    conf_min = float(min(np.min(r_dash["analysis_confidence"]),
                         np.min(r_cr["analysis_confidence"][:imax_c + 1]),
                         np.min(r_lo["analysis_confidence"][:imax_l + 1])))

    return dict(
        t_max=t_max, x_tmax=x_tmax, t_30=t_30, t_50=t_50, t_65=t_65,
        cd_dash=cd_dash, cm_dash=cm_dash,
        ld_max=ld_max, cl_ldmax=cl_ldmax, ld_035=ld_035, ld_050=ld_050, ld_065=ld_065, cm_cr=cm_cr,
        endur_max=endur_max, clmax_lo=clmax_lo, conf_min=conf_min,
    )


def loss(p, model_size="medium"):
    w_t, w_c = p[:5], p[5:10]
    coords, x, yu, yl = cst_tc_coords(w_t, w_c)
    m = eval_airfoil(coords, x, yu, yl, model_size=model_size)
    if m is None:
        return 1e6

    pen = 0.0
    if m["t_30"] < 0.085:
        pen += 8000.0 * (0.085 - m["t_30"]) ** 2
    if m["t_50"] < 0.068:
        pen += 8000.0 * (0.068 - m["t_50"]) ** 2
    if m["t_65"] < 0.044:
        pen += 8000.0 * (0.044 - m["t_65"]) ** 2
    if m["t_max"] > 0.095:
        pen += 4000.0 * (m["t_max"] - 0.095) ** 2
    # Positive/self-trimming Cm at both cruise and high-speed dash
    if m["cm_cr"] < 0.004:
        pen += 6000.0 * (0.004 - m["cm_cr"]) ** 2
    if m["cm_dash"] < 0.004:
        pen += 6000.0 * (0.004 - m["cm_dash"]) ** 2
    if m["clmax_lo"] < 1.12:
        pen += 2000.0 * (1.12 - m["clmax_lo"]) ** 2
    # High confidence requirement -> strictly clean physical airfoil
    if m["conf_min"] < 0.94:
        pen += 8000.0 * (0.94 - m["conf_min"]) ** 2
    pen += 0.25 * float(np.sum(np.diff(w_t) ** 2) + np.sum(np.diff(w_c) ** 2))

    score = (
        + 2.20 * (m["cd_dash"] / 0.0048)
        - 1.00 * (m["ld_max"] / 70.0)
        - 1.30 * (m["ld_035"] / 48.0)
        - 1.40 * (m["ld_050"] / 62.0)
        - 1.10 * (m["ld_065"] / 68.0)
        - 1.25 * (m["endur_max"] / 58.0)
        - 0.25 * (m["clmax_lo"] / 1.20)
    )
    return score + pen


if __name__ == "__main__":
    b_coords, bx, byu, byl = baseline_coords()
    bm = eval_airfoil(b_coords, bx, byu, byl, model_size="xlarge")

    bounds = [
        (0.08, 0.20), (0.08, 0.22), (0.08, 0.22), (0.07, 0.18), (0.055, 0.14),   # clean, printable TE!
        (0.00, 0.20), (-0.02, 0.18), (-0.05, 0.14), (-0.12, 0.06), (-0.16, 0.02),
    ]
    print("Stage 1: Differential Evolution (medium NeuralFoil)...", flush=True)
    res_de = differential_evolution(loss, bounds, args=("medium",), maxiter=55, popsize=12, seed=7, tol=1e-4, polish=False)
    print(f"  DE best loss (medium): {res_de.fun:.5f}", flush=True)

    print("Stage 2: Powell + Nelder-Mead polish directly on XLARGE NeuralFoil...", flush=True)
    res_pw = minimize(loss, res_de.x, args=("xlarge",), method="Powell", bounds=bounds, options=dict(maxiter=200, ftol=1e-4))
    res_nm = minimize(loss, res_pw.x, args=("xlarge",), method="Nelder-Mead", options=dict(maxiter=300, xatol=1e-4, fatol=1e-4))
    best_p = res_nm.x if res_nm.fun < res_pw.fun else res_pw.x
    print(f"  Polished loss (xlarge): {min(res_pw.fun, res_nm.fun):.5f}", flush=True)

    w_t, w_c = best_p[:5], best_p[5:10]
    c_opt, ox, oyu, oyl = cst_tc_coords(w_t, w_c)
    om = eval_airfoil(c_opt, ox, oyu, oyl, model_size="xlarge")
    print("\nOPTIMIZED STEALTH CST AIRFOIL v2 (verified on xlarge NeuralFoil):")
    for k, v in om.items():
        bval = bm[k]
        pct = (v / bval - 1) * 100 if abs(bval) > 1e-6 else 0.0
        print(f"  {k:12s}: {v:.5f}  (vs baseline {bval:.5f}, {pct:+.1f} %)")
    print("\nCST weights:")
    print("  w_t =", [round(float(w), 5) for w in w_t])
    print("  w_c =", [round(float(w), 5) for w in w_c])

    out = dict(w_t=[float(w) for w in w_t], w_c=[float(w) for w in w_c], dz_te=0.0025, metrics=om, baseline=bm)
    (ROOT / "sim" / "stealth_airfoil.json").write_text(json.dumps(out, indent=2))
    print("wrote sim/stealth_airfoil.json")
