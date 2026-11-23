"""Hill-climb the Kestrel-600 design for still-air range (the older 610 mm design-space model).

The current build and every published number live in sim/aircraft.py; this file keeps its own motor and
cell tables for the design search only. Air density is aircraft.RHO.

Run:  .venv/Scripts/python sim/optimize.py
Discrete choices are real parts (motor, APC prop, Molicel cell, pack layout); continuous ones are
root chord, tip chord and cruise speed. Span is fixed at 610 mm.

Measured / published inputs: APC performance files (10 000 rpm block), Molicel cell pages,
SunnySky and T-Motor spec listings, MiniQuadTestBench for the Emax Eco 2306.
Estimates: drag of everything but the wing section, airframe mass, ESC part-throttle loss
(8 W per ESC at 16 V, scaled with pack voltage; taken from the Eco 2306 bench fit).
"""
import itertools
import json
import math
import sys

import numpy as np

from aircraft import G, NU, RHO      # one air density for every script (ISA sea level)
SPAN, HALF = 0.610, 0.302

# name: (Kv, ohm, no-load A, grams, max A, max cells in series)
# X2206 rows are the manufacturer's table (13 A / 30 s, 150 W, rated 2-3S); X2204 and AT2303 are retailer listings.
MOTORS = {
    "Emax Eco 2306 2400KV (bench)": (2400, 0.103, 0.95, 29.7, 40, 6),   # rated Kv (the bench fit read ~2300 loaded)
    "Emax ECO II 2306 1900KV": (1900, 0.062, 0.85, 30.2, 46, 6),
    "Emax ECO II 2306 1700KV": (1700, 0.074, 0.75, 30.4, 36, 6),
    "SunnySky X2204 1800KV": (1800, 0.156, 0.20, 19.6, 12, 3),
    "SunnySky X2206 1500KV": (1500, 0.147, 0.30, 25.2, 13, 3),
    "SunnySky X2206 1900KV": (1900, 0.112, 0.60, 25.0, 13, 3),
    "T-Motor AT2306 1900KV": (1900, 0.090, 0.70, 30.0, 22, 3),     # resistance and no-load current are estimates
    "T-Motor AT2303 1500KV": (1500, 0.251, 0.30, 18.0, 10, 3),
    "T-Motor AT2303 1800KV": (1800, 0.240, 0.50, 18.0, 10, 3),
    "T-Motor AT2303 2300KV": (2300, 0.236, 0.75, 18.0, 10, 3),
}
# name: (Ah, Wh, max A, grams, ohm)   -- M65A resistance is an assumption (not on its page)
CELLS = {
    "Reliance RS50": (5.0, 18.0, 70, 66.5, 0.0110),
    "Molicel P45B": (4.5, 16.2, 45, 70.0, 0.0138),
    "Molicel P50B": (5.0, 18.0, 60, 68.0, 0.0140),
    "Molicel M65A": (6.5, 23.4, 26, 72.7, 0.0250),
}
PACKS = [(2, 1), (3, 1), (4, 1), (2, 2), (3, 2)]
# APC "E" props: diameter in, [J, Ct, Cp] at 10 000 rpm
APC = json.loads(open(__file__.replace("optimize.py", "apc_props.json")).read())
MAX_PROP_IN = 7.0
CP_DERATE = 1.08            # APC files are computed, not measured
USABLE, AVIONICS_W, HOVER_S = 0.85, 5.0, 60.0
TW_MIN, STALL_MAX, E_OSW = 1.8, 12.0, 0.80
HULL_FF, POD_CD, EXTRAS_DQ = 1.40, 0.30, 0.40 * 3.0e-4 + 1.10 * 1.8e-4   # faceted hull, blunt pods, sensor ball + antenna
EXTRA_MASS_G = 0.0              # e.g. printing in ordinary PLA instead of foaming LW-PLA
CR_MIN, CT_MIN = 0.160, 0.100   # judgment: keep wing area in the prop wash for hover control


# ---- wing section polars from NeuralFoil (MH45 reflexed airfoil) ---------------------------
def build_polars():
    import aerosandbox as asb
    import neuralfoil as nf
    af = asb.Airfoil("mh45")
    res = np.array([6e4, 1e5, 1.5e5, 2e5, 3e5, 4.5e5])
    alpha = np.linspace(-3, 13, 33)
    cl_grid = np.linspace(0.0, 1.2, 61)
    cd, clmax = [], []
    for re in res:
        a = nf.get_aero_from_airfoil(af, alpha=alpha, Re=re, model_size="large")
        cl, cdv = np.asarray(a["CL"]), np.asarray(a["CD"])
        i = int(np.argmax(cl))
        clmax.append(cl[i])
        cd.append(np.interp(cl_grid, cl[:i + 1], cdv[:i + 1], right=0.2))
    return res, cl_grid, np.array(cd), np.array(clmax)


RES, CL_GRID, CD_TAB, CLMAX_TAB = build_polars()


def section_cd(cl, re):
    lre = np.log(np.clip(re, RES[0], RES[-1]))
    j = np.clip(np.searchsorted(np.log(RES), lre) - 1, 0, len(RES) - 2)
    t = (lre - np.log(RES[j])) / (np.log(RES[j + 1]) - np.log(RES[j]))
    lo = np.array([np.interp(c, CL_GRID, CD_TAB[k]) for c, k in zip(np.atleast_1d(cl), np.atleast_1d(j))])
    hi = np.array([np.interp(c, CL_GRID, CD_TAB[k + 1]) for c, k in zip(np.atleast_1d(cl), np.atleast_1d(j))])
    return lo + t * (hi - lo)


def wing(cr, ct):
    s = 2 * HALF * (cr + ct) / 2
    lam = ct / cr
    mac = 2 / 3 * cr * (1 + lam + lam**2) / (1 + lam)
    return s, SPAN**2 / s, mac


def prop_arrays(name):
    t = np.array(APC[name]["table"])
    return APC[name]["d_in"] * 0.0254, t[:, 0], t[:, 1], t[:, 2] * CP_DERATE


def thrust_solve(thrust, v, d, J, CT):
    """rev/s giving `thrust` (N) per prop at airspeed v (arrays)."""
    lo = np.maximum(v / (J[-1] * d), 5.0) * np.ones_like(thrust)
    hi = np.full_like(thrust, 900.0)
    for _ in range(45):
        mid = (lo + hi) / 2
        t = np.interp(v / (mid * d), J, CT) * RHO * mid**2 * d**4
        lo, hi = np.where(t < thrust, mid, lo), np.where(t < thrust, hi, mid)
    return hi


def esc_loss_w(vbat, im, esc_loss=1.0):
    """ESC electrical loss (W): quiescent MCU/driver + MOSFET conduction (2*Rds_on*I^2) + PWM switching & ripple loss.
    Matches the 8 W @ 16 V / 10 A miniquad bench fit and ~5.5 W in hover while scaling physically to ~1.1 W in cruise."""
    return esc_loss * (0.30 * vbat / 11.1 + 0.012 * im**2 + 0.045 * vbat * im)


def evaluate(motor, prop, cell, pack, cr, ct, esc_loss=1.0, drag_scale=1.0, full=False):
    kv, rm, i0, m_motor, i_motor_max, s_max = MOTORS[motor]
    ah, wh, i_cell_max, m_cell, r_cell = CELLS[cell]
    ns, npar = pack
    d, J, CT, CP = prop_arrays(prop)
    s, ar, mac = wing(cr, ct)
    n_cells = ns * npar
    mass = (70 + EXTRA_MASS_G + 1100 * s + 2 * m_motor + 2 * 6 * (d / 0.1524)**2.2 + 16 + 23 + 20 + 8 + 2 + 24 + 20
            + n_cells * m_cell + 15 + (10 if npar > 1 else 0)) / 1000
    w = mass * G
    vbat = ns * 3.6
    r_pack = ns * r_cell / npar + 0.012

    def electrical(n, v):
        shaft = np.interp(v / (n * d), J, CP) * RHO * n**3 * d**5
        e = n * 60 / kv
        im = shaft / e + i0
        return (e + im * rm) * im + esc_loss_w(vbat, im, esc_loss), e + im * rm, im

    # hover and the thrust margin a tailsitter needs
    n_h = thrust_solve(np.array([w / 2]), np.zeros(1), d, J, CT)
    p_h, _, _ = electrical(n_h, np.zeros(1))
    p_hover = 2 * p_h[0] + AVIONICS_W
    n_m = thrust_solve(np.array([TW_MIN * w / 2]), np.zeros(1), d, J, CT)
    p_m, v_m, i_m = electrical(n_m, np.zeros(1))
    i_pack = 2 * p_m[0] / vbat
    ok_thrust = (v_m[0] <= vbat - i_pack * r_pack) and (i_m[0] <= i_motor_max) and (i_pack <= i_cell_max * npar)

    clmax = 0.9 * np.interp(math.log(11 * mac / NU), np.log(RES), CLMAX_TAB)
    v_stall = math.sqrt(2 * w / (RHO * s * clmax))
    v = np.arange(max(1.2 * v_stall, 9.0), 30.0, 0.5)
    q = 0.5 * RHO * v**2
    cl = w / (q * s)
    cf = lambda length: 0.074 / (v * length / NU)**0.2
    other = (cf(0.34) * HULL_FF * 0.060 + cf(0.12) * 1.05 * 0.056 + POD_CD * 2 * math.pi * 0.014**2
             + EXTRAS_DQ)
    cd = (section_cd(cl, v * mac / NU) + other / s) * 1.15 * drag_scale + cl**2 / (math.pi * ar * E_OSW)
    drag = q * s * cd
    n = thrust_solve(drag / 2, v, d, J, CT)
    p_one, v_mot, _ = electrical(n, v)
    p = 2 * p_one + AVIONICS_W
    p = np.where(v_mot <= vbat, p, np.inf)
    energy = n_cells * wh * USABLE - p_hover * HOVER_S / 3600
    rng = energy / p * v * 3.6
    k = int(np.argmax(rng))
    feasible = ok_thrust and ns <= s_max and v_stall <= STALL_MAX and np.isfinite(p[k])
    out = dict(range_km=float(rng[k]) if feasible else 0.0, feasible=bool(feasible))
    if full:
        ke = int(np.argmin(p))
        out.update(mass_g=mass * 1000, area_cm2=s * 1e4, AR=ar, v_stall=v_stall, v_cruise=float(v[k]),
                   p_cruise=float(p[k]), LD=float(cl[k] / cd[k]), rpm=float(n[k] * 60), v_loiter=float(v[ke]),
                   p_loiter=float(p[ke]), endurance_min=float(energy / p[ke] * 60), p_hover=float(p_hover),
                   hover_min=float(n_cells * wh * USABLE / p_hover * 60), i_pack_18=float(i_pack),
                   i_motor_18=float(i_m[0]), wh=n_cells * wh, prop_eff=float(drag[k] / 2 * v[k] /
                   (np.interp(v[k] / (n[k] * d), J, CP) * RHO * n[k]**3 * d**5)))
    return out


def climb_wing(design, **kw):
    """Coordinate hill-climb on root and tip chord with shrinking steps."""
    cr, ct = design["cr"], design["ct"]
    best = evaluate(design["motor"], design["prop"], design["cell"], design["pack"], cr, ct, **kw)["range_km"]
    step = 0.020
    while step >= 0.0025:
        moved = False
        for dcr, dct in ((step, 0), (-step, 0), (0, step), (0, -step), (step, step), (-step, -step)):
            a, b = cr + dcr, ct + dct
            if not (CR_MIN <= a <= 0.280 and CT_MIN <= b <= a):
                continue
            r = evaluate(design["motor"], design["prop"], design["cell"], design["pack"], a, b, **kw)["range_km"]
            if r > best + 1e-6:
                cr, ct, best, moved = a, b, r, True
        if not moved:
            step /= 2
    return dict(design, cr=cr, ct=ct), best


def hill_climb(start, log=print, **kw):
    """Greedy: at each step take the single change (one part swapped, or the wing re-climbed) that helps most."""
    cur = dict(start)
    best = evaluate(cur["motor"], cur["prop"], cur["cell"], cur["pack"], cur["cr"], cur["ct"], **kw)["range_km"]
    log(f"  start: {best:5.1f} km")
    props = [p for p in APC if APC[p]["d_in"] <= MAX_PROP_IN]
    while True:
        moves = []
        for key, options in (("motor", MOTORS), ("prop", props), ("cell", CELLS), ("pack", PACKS)):
            for opt in options:
                if opt == cur[key]:
                    continue
                cand = dict(cur, **{key: opt})
                r = evaluate(cand["motor"], cand["prop"], cand["cell"], cand["pack"], cand["cr"], cand["ct"], **kw)
                moves.append((r["range_km"], f"{key} -> {opt}", cand))
        # paired moves: motor and prop, or pack and motor, usually have to change together
        for (k1, o1), (k2, o2) in itertools.chain(
                itertools.product([("motor", m) for m in MOTORS], [("prop", p) for p in props]),
                itertools.product([("pack", p) for p in PACKS], [("motor", m) for m in MOTORS])):
            cand = dict(cur, **{k1: o1, k2: o2})
            r = evaluate(cand["motor"], cand["prop"], cand["cell"], cand["pack"], cand["cr"], cand["ct"], **kw)
            moves.append((r["range_km"] - 0.01, f"{k1} -> {o1} and {k2} -> {o2}", cand))
        w_design, w_range = climb_wing(cur, **kw)
        moves.append((w_range, f"wing -> root {w_design['cr'] * 1000:.0f} mm, tip {w_design['ct'] * 1000:.0f} mm", w_design))
        r, what, cand = max(moves, key=lambda m: m[0])
        if r <= best + 0.05:
            return cur, best
        cur, best = cand, r
        log(f"  {best:5.1f} km  <- {what}")


def describe(design, **kw):
    r = evaluate(design["motor"], design["prop"], design["cell"], design["pack"], design["cr"], design["ct"],
                 full=True, **kw)
    ns, npar = design["pack"]
    print(f"    {design['motor']} | APC {design['prop']} | {ns}S{npar}P {design['cell']} ({r['wh']:.0f} Wh)")
    print(f"    wing root {design['cr'] * 1000:.0f} / tip {design['ct'] * 1000:.0f} mm, {r['area_cm2']:.0f} cm2, AR {r['AR']:.1f}"
          f" | mass {r['mass_g']:.0f} g | stall {r['v_stall']:.1f} m/s")
    print(f"    cruise {r['v_cruise']:.1f} m/s ({r['v_cruise'] * 3.6:.0f} km/h) {r['p_cruise']:.0f} W, L/D {r['LD']:.1f}, "
          f"prop eff {r['prop_eff'] * 100:.0f} %, {r['rpm']:.0f} rpm -> RANGE {r['range_km']:.0f} km")
    print(f"    loiter {r['v_loiter']:.1f} m/s {r['p_loiter']:.0f} W -> ENDURANCE {r['endurance_min']:.0f} min"
          f" | hover {r['p_hover']:.0f} W ({r['hover_min']:.0f} min pure hover)")
    print(f"    at thrust/weight {TW_MIN}: {r['i_motor_18']:.1f} A per motor, {r['i_pack_18']:.1f} A pack")
    return r


if __name__ == "__main__":
    base = dict(motor="Emax Eco 2306 2400KV (bench)", prop="6x4E", cell="Molicel P45B", pack=(3, 1), cr=0.240, ct=0.120)
    print("BASELINE (previous design, same model)")
    describe(base)
    results = {}
    for label, kw in (("nominal", {}), ("pessimistic: ESC loss x1.5, drag +25 %", dict(esc_loss=1.5, drag_scale=1.25)),
                      ("optimistic: ESC loss x0.5", dict(esc_loss=0.5))):
        print(f"\nHILL CLIMB, {label}")
        best, _ = hill_climb(base, **kw)
        results[label] = (best, describe(best, **kw))
    print("\nNOMINAL WINNER re-scored under the pessimistic assumptions")
    describe(results["nominal"][0], esc_loss=1.5, drag_scale=1.25)
    print("\nEXHAUSTIVE CHECK (nominal): every motor x prop x cell x pack, wing hill-climbed for each")
    props = [p for p in APC if APC[p]["d_in"] <= MAX_PROP_IN]
    table = []
    for motor, prop, cell, pack in itertools.product(MOTORS, props, CELLS, PACKS):
        if pack[0] > MOTORS[motor][5]:
            continue
        d0 = dict(motor=motor, prop=prop, cell=cell, pack=pack, cr=0.200, ct=0.120)
        d1, r = climb_wing(d0)
        if r == 0:                      # infeasible at the start wing: try a big wing before giving up
            d1, r = climb_wing(dict(d0, cr=0.280, ct=0.200))
        if r > 0:
            table.append((r, d1))
    table.sort(key=lambda t: -t[0])
    for n_cells in (2, 3, 4, 6):
        rows = [t for t in table if t[1]["pack"][0] * t[1]["pack"][1] == n_cells]
        print(f"\n  best with {n_cells} cells ({len(rows)} feasible combinations):")
        for r, d in rows[:3]:
            describe(d)
            print()
        if rows:
            results[f"best_{n_cells}_cells"] = (rows[0][1], evaluate(**{k: rows[0][1][k] for k in ("motor", "prop", "cell", "pack", "cr", "ct")}, full=True))
    print("\n  best 3-cell design re-scored pessimistic (ESC loss x1.5, drag +25 %) and optimistic (ESC loss x0.5):")
    b3 = results["best_3_cells"][0]
    describe(b3, esc_loss=1.5, drag_scale=1.25)
    describe(b3, esc_loss=0.5)
    json.dump({k: dict(v[0], **v[1]) for k, v in results.items()}, open(__file__.replace("optimize.py", "best.json"), "w"), indent=1)
