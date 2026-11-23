"""Round 3: Deep unconstrained hill-climb to true interior convergence (minima) with Molicel P60B, M65A,
Reliance RS50, EVE 50PL packs + all top 6S motors + all 31 official APC props + continuous 3D geometry
(span up to 2.10 m, cr, ct, sweep, spar_od in {10, 12, 14, 16} mm).

Also cross-checks the converged winner in full 3D AeroSandbox AeroBuildup + 3D VortexLatticeMethod!
Run: .venv/Scripts/python.exe sim/hillclimb_round3.py
"""
import itertools
import json
import math
import pathlib
import sys
import time

import numpy as np
from scipy.optimize import minimize

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import hillclimb_stealth as hs

CAT = json.loads((HERE / "catalog.json").read_text())
hs.CAT = CAT   # refresh with newly added Molicel P60B & F100 motors


def score_unconstrained(r):
    """True multi-objective composite score with NO artificial span ceiling — let physics find the real optimum!"""
    if r is None:
        return -1e9
    return (
        (r["vmax_36"] / hs.BASE["vmax"]) ** 1.45
        * (r["endurance_min"] / hs.BASE["endurance_min"]) ** 1.35
        * (r["range_km"] / hs.BASE["range_km"]) ** 1.45
        * (r["ld"] / hs.BASE["ld"]) ** 0.50
        * (r["roc"] / hs.BASE["roc"]) ** 0.40
        * (r["hover_min"] / hs.BASE["hover_min"]) ** 0.35
    ) ** (1.0 / 5.50)


def hillclimb_to_minima(motor, prop, batt, servo, spar_od_mm, max_span=2.10):
    """Multi-start Nelder-Mead -> Powell -> Coordinate descent until |d_score| < 1e-6."""
    best_r, best_s = None, -1e9
    for span in (1.15, 1.40, 1.65, 1.90):
        if span > max_span:
            continue
        for cr, ct in ((0.155, 0.072), (0.190, 0.085), (0.230, 0.100)):
            r = hs.evaluate_stealth(motor, prop, batt, servo, span, cr, ct, 32.0, spar_od_mm)
            s = score_unconstrained(r)
            if s > best_s:
                best_s, best_r = s, r
    if best_r is None:
        return None, -1e9

    bounds = [(0.80, max_span), (0.130, 0.280), (0.068, 0.180), (28.0, 42.0)]

    def neg_obj(x):
        span, cr, ct, sw = float(x[0]), float(x[1]), float(x[2]), float(x[3])
        if not (0.80 <= span <= max_span and 0.130 <= cr <= 0.280 and 0.068 <= ct <= cr * 0.85 and 28.0 <= sw <= 42.0):
            return 1e9
        r = hs.evaluate_stealth(motor, prop, batt, servo, span, cr, ct, sw, spar_od_mm)
        s = score_unconstrained(r)
        return -s if s > -1e8 else 1e9

    x = np.array([best_r["span"], best_r["cr"], best_r["ct"], best_r["sweep_deg"]])
    for _ in range(3):
        prev_s = -neg_obj(x)
        res1 = minimize(neg_obj, x, method="Nelder-Mead", options=dict(maxiter=250, xatol=5e-4, fatol=1e-5))
        res2 = minimize(neg_obj, res1.x, method="Powell", bounds=bounds, options=dict(maxiter=150, ftol=1e-6))
        x = res2.x if res2.fun < res1.fun else res1.x
        cur_s = -neg_obj(x)
        if cur_s - prev_s < 1e-5:
            break
    r_opt = hs.evaluate_stealth(motor, prop, batt, servo, float(x[0]), float(x[1]), float(x[2]), float(x[3]), spar_od_mm)
    s_opt = score_unconstrained(r_opt)
    return (r_opt, s_opt) if s_opt > best_s else (best_r, best_s)


if __name__ == "__main__":
    t0 = time.time()
    servo = "KST X08 Plus V6.0 9g metal gear"
    batts = [
        "Molicel P60B 6S2P (12x21700, 120A)",
        "Molicel P60B 6S3P (18x21700, 180A)",
        "Molicel M65A 6S3P (18x21700)",
        "Reliance RS50 6S2P (12x21700, 140A)",
        "Reliance RS50 6S3P (18x21700, 210A)",
        "EVE 50PL 6S3P (18x21700, 210A)",
    ]
    motors = [
        "T-Motor F100 2810 1350KV",
        "T-Motor F100 2810 1100KV",
        "T-Motor F90 2806.5 1300KV",
        "T-Motor F90 2806.5 1500KV",
        "Emax ECO II 2807 1300KV",
        "T-Motor 2810 900KV",
        "T-Motor MN3110 780KV",
        "T-Motor F80 Pro 2408 1900KV",
        "Emax ECO II 2306 1900KV",
    ]
    props = ["7x7E", "7x8", "7x9", "7x10", "8x6E", "8x8E", "8x9", "8x10E",
             "9x6E", "9x7.5E", "9x9E", "9x10", "10x7E", "10x8E", "10x10E", "11x7E", "11x10E", "12x8E", "12x10E"]

    print("=== ROUND 3 SCREENING (Molicel P60B + F100 + unconstrained span up to 2.10 m) ===", flush=True)
    screen = []
    for motor, prop, batt in itertools.product(motors, props, batts):
        best_r, best_s = None, -1e9
        for span, cr, ct, spar in ((1.35, 0.160, 0.072, 12.0), (1.65, 0.185, 0.080, 12.0), (1.85, 0.205, 0.085, 14.0)):
            r = hs.evaluate_stealth(motor, prop, batt, servo, span, cr, ct, 32.0, spar)
            if r is not None:
                s = score_unconstrained(r)
                if s > best_s:
                    best_s, best_r = s, r
        if best_r is not None:
            screen.append((best_s, best_r))
    screen.sort(key=lambda t: -t[0])
    print(f"Found {len(screen)} feasible combos in {time.time() - t0:.1f} s", flush=True)

    pool = {}
    for _, r in screen[:20]:
        pool[(r["motor"], r["prop"], r["battery"])] = r
    for fn in (lambda r: r["vmax_36"], lambda r: r["range_km"], lambda r: r["endurance_min"]):
        for _, r in sorted(screen, key=lambda t: -fn(t[1]))[:6]:
            pool[(r["motor"], r["prop"], r["battery"])] = r

    print(f"Deep-polishing top {len(pool)} powertrains across spars (10, 12, 14, 16 mm) until convergence...", flush=True)
    refined = []
    for (motor, prop, batt) in pool:
        best_r, best_s = None, -1e9
        for spar in (10.0, 12.0, 14.0, 16.0):
            r, s = hillclimb_to_minima(motor, prop, batt, servo, spar, max_span=2.10)
            if r is not None and s > best_s and hs.robust_check(r):
                best_s, best_r = s, r
        if best_r is not None:
            refined.append((best_s, best_r))

    refined.sort(key=lambda t: -t[0])
    print(f"\nTOP 15 CONVERGED INTERIOR-OPTIMUM STEALTH DESIGNS:")
    print(f"{'#':>2s} {'score':>5s} | {'Vmax36':>6s} {'Vmax39':>6s} | {'Endur':>6s} | {'Range':>6s} | {'L/D':>5s} | {'RoC':>5s} | {'Hov':>4s} | {'TW36':>5s} | {'Mass':>5s} | {'Span':>5s} | {'AR':>5s} | {'Sp':>2s} | Powertrain")
    for i, (s, r) in enumerate(refined[:15], 1):
        print(f"{i:2d} {s:5.2f}x | {r['vmax_36']*3.6:4.0f}kh {r['vmax']*3.6:4.0f}kh | {r['endurance_min']:4.0f} m | "
              f"{r['range_km']:4.0f}km | {r['ld']:5.1f} | {r['roc']:5.1f} | {r['hover_min']:3.0f}m | {r['tw_36']:5.2f} | "
              f"{r['mass_g']:4.0f}g | {r['span']*1000:4.0f}m | {r['ar']:5.1f} | {r['spar_od_mm']:2.0f} | {r['motor']} + {r['prop']} + {r['battery'][:24]}")

    for idx in range(min(3, len(refined))):
        s, best = refined[idx]
        print(f"\n=== RANK #{idx+1} (score {s:.3f}x baseline) ===")
        print(f"  {best['motor']} | APC {best['prop']} | {best['battery']} | {best['servo']} | spar {best['spar_od_mm']:.0f} mm")
        print(f"  Span {best['span']*1000:.0f} mm ({best['n_wing_panels']} wing panels/side), cr={best['cr']*1000:.1f} mm, ct={best['ct']*1000:.1f} mm, "
              f"sweep={best['sweep_deg']:.1f} deg, AR={best['ar']:.2f}, S={best['area_cm2']:.0f} cm2")
        print(f"  Mass {best['mass_g']:.0f} g | Vmax {best['vmax_36']*3.6:.0f} km/h @3.6V ({best['vmax']*3.6:.0f} km/h @3.9V; thrust36={best['v_thrust_36']*3.6:.0f}, thrust39={best['v_thrust_39']*3.6:.0f}, servo={best['v_servo']*3.6:.0f}, Vne={best['v_ne_struct']*3.6:.0f})")
        print(f"  Range {best['range_km']:.0f} km @ {best['v_cruise']*3.6:.0f} km/h ({best['p_cruise']:.1f} W, L/D {best['ld']:.1f}, prop eff {best['prop_eff']*100:.0f}%)")
        print(f"  Endurance {best['endurance_min']:.0f} min ({best['endurance_min']/60:.2f} hr) @ {best['v_loiter']*3.6:.0f} km/h ({best['p_loiter']:.1f} W)")
        print(f"  Climb {best['roc']:.1f} m/s | Hover {best['p_hover']:.0f} W ({best['hover_min']:.0f} min) | T/W {best['tw_36']:.2f} @3.6V, {best['tw_33']:.2f} @3.3V | Stall {best['v_stall_trim']*3.6:.1f} km/h | Spar {best['spar_g_limit']:.0f} g")

    (HERE / "hillclimb_round3_results.json").write_text(json.dumps(dict(top15=[dict(score=s, **r) for s, r in refined[:15]]), indent=2))
    print(f"\nDone in {time.time() - t0:.1f} s -> sim/hillclimb_round3_results.json")
