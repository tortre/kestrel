"""Simulations of a chosen design: uncertainty (Monte Carlo), wind, speed sweep, hover control.

Run:  .venv/Scripts/python sim/mission.py            (the current build from sim/aircraft.py, harness model)
      .venv/Scripts/python sim/mission.py budget     (a LEGACY 610 mm design on the old optimize.py model)

The default "current" mode uses harness.py on exactly aircraft.CURRENT_BUILD. The named DESIGNS below are
the legacy 610 mm-span studies (SunnySky X2206 + 7x6E) kept for comparison; they are not the current aircraft.
"""
import math
import sys

import numpy as np

import aircraft as A
import harness as h
import optimize as opt

# LEGACY 610 mm-span designs on the old optimize.py model (not the current build)
DESIGNS = {
    "optimised": dict(motor="SunnySky X2206 1500KV", prop="7x6E", cell="Molicel M65A", pack=(3, 1), cr=0.160, ct=0.100),
    # same motors and prop size (Gemfan 7x6E treated as the APC 7x6E), genuine P50B cells bought from a battery specialist
    "budget": dict(motor="SunnySky X2206 1500KV", prop="7x6E", cell="Molicel P50B", pack=(3, 1), cr=0.160, ct=0.100),
    "printable": dict(motor="SunnySky X2206 1500KV", prop="7x6E", cell="Molicel P50B", pack=(3, 1), cr=0.160, ct=0.100),
    "budget-pla": dict(motor="SunnySky X2206 1500KV", prop="7x6E", cell="Molicel P50B", pack=(3, 1), cr=0.160, ct=0.100),
}


def monte_carlo(design, n=400, seed=0):
    """Vary the things I estimated rather than measured, and see how range and endurance spread."""
    rng = np.random.default_rng(seed)
    base = (opt.CP_DERATE, opt.USABLE, opt.E_OSW, opt.AVIONICS_W)
    out = []
    for _ in range(n):
        opt.CP_DERATE = rng.uniform(1.00, 1.20)        # prop worse than APC's computed file
        opt.USABLE = rng.uniform(0.78, 0.90)           # cell capacity actually delivered
        opt.E_OSW = rng.uniform(0.70, 0.88)            # span efficiency
        opt.AVIONICS_W = rng.uniform(3.0, 8.0)
        r = opt.evaluate(**design, esc_loss=rng.uniform(0.4, 1.6), drag_scale=rng.uniform(0.9, 1.35), full=True)
        if r["feasible"]:
            out.append((r["range_km"], r["endurance_min"], r["p_cruise"]))
    opt.CP_DERATE, opt.USABLE, opt.E_OSW, opt.AVIONICS_W = base
    return np.array(out), n


def wind_table(design):
    """Out-and-back radius: fly out against the wind and home with it, at the best airspeed for each wind."""
    r = opt.evaluate(**design, full=True)
    rows = []
    for wind in (0, 3, 5, 8, 10):
        best = (0, 0)
        for v in np.arange(max(1.2 * r["v_stall"], wind + 2), 30, 0.5):
            p = power_at(design, v)
            if p is None:
                continue
            energy = r["wh"] * opt.USABLE - r["p_hover"] * opt.HOVER_S / 3600
            hours = energy / p
            # distance d each way: d/(v-w) + d/(v+w) = hours
            d = hours / (1 / (v - wind) + 1 / (v + wind)) * 3.6
            best = max(best, (d, v))
        rows.append((wind, best[0], best[1]))
    return rows


def power_at(design, v, esc_loss=1.0, drag_scale=1.0):
    """Electrical power in level flight at airspeed v, from the same model as optimize.evaluate
    (same drag_scale on the parasite drag and the same opt.esc_loss_w ESC model)."""
    kv, rm, i0, m_motor, _, _ = opt.MOTORS[design["motor"]]
    d, J, CT, CP = opt.prop_arrays(design["prop"])
    s, ar, mac = opt.wing(design["cr"], design["ct"])
    full = opt.evaluate(**design, full=True)
    w = full["mass_g"] / 1000 * opt.G
    q = 0.5 * opt.RHO * v * v
    cl = w / (q * s)
    cf = lambda length: 0.074 / (v * length / opt.NU)**0.2
    other = (cf(0.34) * opt.HULL_FF * 0.060 + cf(0.12) * 1.05 * 0.056 + opt.POD_CD * 2 * math.pi * 0.014**2 + opt.EXTRAS_DQ)
    cd = (opt.section_cd(np.array([cl]), np.array([v * mac / opt.NU]))[0] + other / s) * 1.15 * drag_scale + cl**2 / (math.pi * ar * opt.E_OSW)
    n = opt.thrust_solve(np.array([q * s * cd / 2]), np.array([v]), d, J, CT)[0]
    shaft = np.interp(v / (n * d), J, CP) * opt.RHO * n**3 * d**5
    e = n * 60 / kv
    im = shaft / e + i0
    vbat = design["pack"][0] * 3.6
    if e + im * rm > vbat:
        return None
    return 2 * ((e + im * rm) * im + opt.esc_loss_w(vbat, im, esc_loss)) + opt.AVIONICS_W


def hover_wind_limit(design):
    """Rough static estimate of the steady wind the elevons can hold against in hover.

    Elevon force comes from the prop slipstream (momentum theory); the disturbance is the wing acting
    as a flat plate broadside to the wind. Ignores gusts, dynamics and the controller: an order of magnitude only.
    """
    full = opt.evaluate(**design, full=True)
    w = full["mass_g"] / 1000 * opt.G
    d = opt.APC[design["prop"]]["d_in"] * 0.0254
    disk = math.pi * d**2 / 4
    v_slip = 2 * math.sqrt(w / 2 / (2 * opt.RHO * disk))
    chord = design["cr"] + (design["ct"] - design["cr"]) * 0.150 / opt.HALF          # chord at the motor station
    washed = 2 * chord * d / math.sqrt(2)                                              # wing area inside both slipstreams
    force = 0.5 * opt.RHO * v_slip**2 * washed * 0.9                                   # 0.9 = lift change at ~40 deg elevon
    arm_ctrl = 0.45 * chord - 0.25 * chord + 0.10 * 0.135                              # elevon lift acts ~45 % chord; CG 10 % MAC ahead of 25 %
    m_ctrl = 0.6 * force * arm_ctrl                                                    # keep 40 % of throw for stabilising
    s, _, mac = opt.wing(design["cr"], design["ct"])
    arm_dist = 0.50 * mac - 0.15 * mac                                                 # plate centre of pressure ~50 % chord, CG ~15 % MAC
    k = 0.5 * opt.RHO * (s + 0.012) * 1.2 * arm_dist                                   # moment per (m/s)^2
    return v_slip, m_ctrl, math.sqrt(m_ctrl / k)


def run(name, design):
    print(f"===== {name} =====")
    r = opt.describe(design)
    mc, n = monte_carlo(design)
    print(f"\n  Monte Carlo, {n} runs ({len(mc)} feasible), varying drag, ESC loss, prop efficiency, usable capacity, span efficiency, avionics power:")
    for label, col, unit in (("range", 0, "km"), ("endurance", 1, "min"), ("cruise power", 2, "W")):
        p10, p50, p90 = np.percentile(mc[:, col], [10, 50, 90])
        print(f"    {label:13s} 10th pct {p10:5.0f}   median {p50:5.0f}   90th pct {p90:5.0f} {unit}")
    print("\n  Out-and-back radius in steady wind (out against it, home with it):")
    for wind, radius, v in wind_table(design):
        print(f"    wind {wind:2d} m/s ({wind * 3.6:2.0f} km/h): radius {radius:4.0f} km at {v:.1f} m/s airspeed")
    print("\n  Speed sweep:   m/s  km/h    W   min    km")
    energy = r["wh"] * opt.USABLE - r["p_hover"] * opt.HOVER_S / 3600
    for v in (12, 14, 16, 18, 20, 22, 25, 28):
        if v < A.MIN_SPEED_FACTOR * r["v_stall"]:
            continue
        p = power_at(design, float(v))
        if p:
            print(f"                {v:4d} {v * 3.6:5.0f} {p:4.0f} {energy / p * 60:5.0f} {energy / p * v * 3.6:5.0f}")
    v_slip, m_ctrl, v_wind = hover_wind_limit(design)
    print(f"\n  Hover control (rough): slipstream {v_slip:.1f} m/s, usable elevon moment {m_ctrl:.3f} N.m,"
          f" steady-wind limit about {v_wind:.1f} m/s ({v_wind * 3.6:.0f} km/h)")
    print(f"  Hover budget: 60 s uses {r['p_hover'] / 60:.1f} Wh ({r['p_hover'] / 60 / r['wh'] * 100:.1f} % of the pack);"
          f" each extra minute of hover costs about {r['p_hover'] / 60 / (r['p_cruise'] / r['v_cruise'] / 3.6):.1f} km of range")
    return r


def run_current(build=None):
    """Monte Carlo, wind radius and speed sweep for an aircraft.py build on the harness model."""
    build = build or A.CURRENT_BUILD
    design = A.harness_design(build)
    r = h.evaluate(design, full=True)
    ac = h.make_aircraft(design)
    print(f"===== {build['name']} ({ac.mass * 1000:.0f} g, harness model) =====")
    print(f"  trimmed stall {ac.v_stall_trim:.1f} m/s, slowest allowed cruise/loiter {ac.v_min_ops:.1f} m/s "
          f"({A.MIN_SPEED_FACTOR} x trimmed stall); ESC {ac.esc_name}")
    mc = h.monte_carlo(design)
    ok = mc[mc[:, 3] > 0]
    print(f"\n  Monte Carlo, {len(mc)} runs ({len(ok)} feasible), varying drag, ESC loss, prop, motor, capacity, span efficiency, mass:")
    for label, col, unit, k in (("top speed", 0, "km/h", 3.6), ("endurance", 1, "min", 1), ("range", 2, "km", 1)):
        p10, p50, p90 = np.percentile(ok[:, col] * k, [10, 50, 90])
        print(f"    {label:13s} 10th pct {p10:5.0f}   median {p50:5.0f}   90th pct {p90:5.0f} {unit}")
    energy = ac.wh * ac.a["usable"] - r["p_hover"] * A.HOVER_S / 3600
    print("\n  Out-and-back radius in steady wind (out against it, home with it):")
    for wind in (0, 3, 5, 8, 10):
        vs = np.arange(max(ac.v_min_ops, wind + 2), 30, 0.25)
        p = ac.level_power(vs)[0]
        d = energy / p / (1 / (vs - wind) + 1 / (vs + wind)) * 3.6
        i = int(np.argmax(d))
        print(f"    wind {wind:2d} m/s ({wind * 3.6:2.0f} km/h): radius {d[i]:4.0f} km at {vs[i]:.1f} m/s airspeed")
    print("\n  Speed sweep:   m/s  km/h    W   min    km")
    for v in (14, 15, 16, 17, 18, 20, 22, 25, 28):
        if v < ac.v_min_ops:
            continue
        p = float(ac.level_power(float(v))[0][0])
        if math.isfinite(p):
            print(f"                {v:4d} {v * 3.6:5.0f} {p:4.0f} {energy / p * 60:5.0f} {energy / p * v * 3.6:5.0f}")
    print(f"  Hover budget: {A.HOVER_S} s uses {r['p_hover'] * A.HOVER_S / 3600:.1f} Wh ({r['p_hover'] * A.HOVER_S / 3600 / ac.wh * 100:.1f} % of the pack);"
          f" each extra minute of hover costs about {r['p_hover'] / 60 / (r['p_cruise'] / r['v_cruise'] / 3.6):.1f} km of range")
    return r


if __name__ == "__main__":
    which = sys.argv[1] if len(sys.argv) > 1 else "current"
    if which == "current":
        run_current()
        sys.exit(0)
    print("LEGACY 610 mm design on the old optimize.py model (not the current build)")
    if which == "printable":
        # smooth build: rounded hull, faired pods (my drag estimates), no sensor ball or antenna
        opt.HULL_FF, opt.POD_CD, opt.EXTRAS_DQ = 1.25, 0.15, 0.0
        opt.EXTRA_MASS_G = 161.0 + 22.0 - (70 + 1100 * 0.0785)     # printed parts + carbon tubes, against the model's airframe
    if which == "budget-pla":
        opt.EXTRA_MASS_G = 90.0          # ordinary PLA airframe instead of LW-PLA (my estimate)
        opt.STALL_MAX = 13.0
    run(which, DESIGNS[which])
