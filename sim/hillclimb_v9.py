"""Kestrel-X Stealth v9: hill climb over every part slot at a fixed parts budget, then over the design variables.

What is searched
  parts    motor, prop pair, ESC, battery pack, elevon servo, GPS, flight controller: every catalogue entry that fits
           the airframe (sim/catalog.json; motors are the 28xx class the nacelle is drawn for). The Walksnail HD camera + VTX kit, the composite airframe and the
           tilt-nacelles are fixed.
  design   prop diameter and pitch up to 10 in (the CAD clears the nose by 27.8 mm at 10 in), pack cell count and
           layout (6S1P, 4S2P, 5S2P, 6S2P bricks in the same bay), system voltage with the matching motor KV, and,
           as a sensitivity only, the wing planform (span and chords on harness.py's parametric airframe).
           Not searched: battery station / CG (the harness has no trim-drag model) and anything that changes the CAD
           exterior, because the flutter, spar and CFD results are tied to it.

Score    weighted geometric mean of the gains over the v9.0 build (BASE below) in endurance, range, top speed,
         static thrust/weight (the lower of the limiter and hard-ceiling figures) and trimmed stall, for builds
         that pass every gate and cost no more than BUDGET.
Gates    harness feasibility with its robust-margin check; thrust/weight >= 1.95 at 3.6 V/cell with the pack held to
         its continuous rating; ESC continuous rating >= fresh-pack full-throttle current; ESC 10 s burst rating >=
         the current with NO firmware limit at all; pack current within the flight controller's 80 A sensor; and
         thrust/weight >= 1.8 (harness.TW_MIN) under a hard output ceiling low enough that a fresh pack with NO
         firmware limiter stays inside the ESC and cell burst ratings, so the hardware never depends on the limiter.

The climb is greedy (best single-slot or two-slot swap) from the v9.0 build and from random starts, and is checked
against a full enumeration of every parts set that fits the budget.

Parts whose catalogue entry says in_stock: false are left out (the 30 A LittleBee ESC of v9.0 is one: sold out and
discontinued on 22 Nov 2026); --allow-unverified puts them back.

Run:  .venv/Scripts/python sim/hillclimb_v9.py                (about 5 min; writes sim/hillclimb_v9_results.json)
      .venv/Scripts/python sim/hillclimb_v9.py --budget 306   (what a few dollars more would buy; writes nothing)
"""
import itertools
import json
import math
import pathlib
import random
import sys

import numpy as np

import aircraft as A
import harness as h

HERE = pathlib.Path(__file__).resolve().parent
ALLOW_UNVERIFIED = "--allow-unverified" in sys.argv     # also use parts whose catalogue entry says in_stock: false
BUDGET = float(sys.argv[sys.argv.index("--budget") + 1]) if "--budget" in sys.argv else 300.0
TW_GATE = 1.95
MAX_PROP_IN = 10.0            # CAD nose clearance: 47.8 mm at 8 in, 37.8 at 9, 27.8 at 10, 17.8 at 11
POD_MOTOR_D = 34.5            # the nacelle takes 28xx motors: 19 mm bolt circle, 34.4 mm envelope (src/printable.py)
POCKET_MAX_T = 9.0            # deepest elevon servo pocket that still leaves 0.8 mm of skin (src/printable.py)
FC_SENSOR_A = 80.0            # SpeedyBee F405 Wing Mini current sensor, continuous
SLOTS = ("motor", "prop", "esc", "battery", "servo", "gps", "receiver", "fc")
PAIRS = (("motor", "prop"), ("motor", "battery"), ("battery", "prop"), ("motor", "esc"), ("prop", "esc"), ("battery", "esc"))
WEIGHTS = dict(endurance_min=0.30, range_km=0.30, vmax=0.15, tw=0.15, stall=0.10)

# v9.0: the first under-$300 build, the reference every gain is measured against
BASE = dict(A.V8_BUDGET_BUILD, name="v9.0", motor="Glee Hobby 2812 900KV",
            prop="APC 8x6E / 8x6EP CW+CCW pair (LP08060E + LP08060EP)", battery="Molicel P45B 6S1P (6x21700, 97Wh, 45A)",
            esc="LittleBee 30A BLHeli_S", servo="Emax ES9051 4.3g", gps="GPS + compass (Quescan 18 mm M10 + QMC5883L)",
            hardware=A.V8_BUDGET_BUILD["hardware"][:3] + (
                "6S XT60 harness on the FC PDB (16AWG to the pods + 2x 470uF), pin hinges, elevon mass balance, fasteners",))
CAT = A.CAT


def _cheapest_of_identical(group, keys):
    """Drop an option when another one in the slot has the same mass and power draw and costs no more."""
    keep = []
    for k in keys:
        e = group[k]
        twin = [o for o in keys if o != k and (group[o]["g"], group[o].get("w")) == (e["g"], e.get("w"))
                and (group[o]["price"], o) < (e["price"], k)]
        if not twin:
            keep.append(k)
    return keep


def options():
    live = lambda v: ALLOW_UNVERIFIED or v.get("in_stock", True)
    props = [k for k, v in CAT["props"]["models"].items() if "CW+CCW pair" in k and v["d_in"] <= MAX_PROP_IN]
    servos = [k for k, v in CAT["servos"].items() if "dims" in v and v["dims"][1] <= POCKET_MAX_T]
    packs = [k for k, v in CAT["batteries"].items() if v["ns"] <= 6 and all(x <= y for x, y in zip(v["dims"], CAT["bay_mm"]))]
    fcs = [k for k, v in CAT["fcs"].items() if not v.get("risky") and not v.get("esc_included")]
    motors = [k for k, v in CAT["motors"].items() if 32.5 <= v.get("d_mm", 0.0) <= POD_MOTOR_D and v.get("shaft") == "M5"]
    return dict(motor=motors, prop=props, esc=[k for k, v in CAT["escs"].items() if live(v)], battery=packs, servo=servos,
                gps=_cheapest_of_identical(CAT["gps"], list(CAT["gps"])),
                receiver=_cheapest_of_identical(CAT["receivers"], list(CAT["receivers"])), fc=fcs)


OPT = options()


def price(slot, key):
    e = CAT["props"]["models"][key] if slot == "prop" else A.part(slot, key)
    if slot in ("motor", "esc"):
        return 2 * (e["price"] + e.get("ship", 0))
    if slot in ("prop", "servo"):
        return e["price_pair"]
    return e["price"] + e.get("ship", 0)


_FIXED_COST = sum(c for _, c in A.bill_of_materials(BASE).values()) - sum(price(s, BASE[s]) for s in SLOTS)


def cost_of(state):
    return _FIXED_COST + sum(price(s, state[s]) for s in SLOTS)


_CACHE = {}


def hard_ceiling(build, ac=None, wot=None):
    """(ceiling, thrust/weight under it). The ceiling is the highest motor output at which a FRESH pack, with NO firmware
    current limiter at all, stays inside the ESC's 10 s burst rating and the pack's burst rating (Q_M_SPIN_MAX in the
    param file). The thrust/weight is at 3.6 V/cell with that ceiling and the normal limits."""
    esc, batt = A.part("esc", build["esc"]), A.part("battery", build["battery"])
    if ac is None:
        ac = h.Aircraft(A.harness_design(build))
        ac.esc_amps, ac.batt, ac.motor = math.inf, dict(ac.batt, imax=1e9), dict(ac.motor, imax=1e9)
        wot = ac.full_throttle_state(0.0, A.V_CELL_FULL)
    esc_burst, pack_burst = esc.get("burst", 1.15 * esc["amps"]), batt.get("imax_burst", batt["imax"])
    ceiling = 1.0
    if wot["motor_a"] > esc_burst or wot["pack_a"] > pack_burst:
        lo, hi = 0.3, 1.0
        for _ in range(12):
            mid = 0.5 * (lo + hi)
            st = ac.full_throttle_state(0.0, A.V_CELL_FULL * mid)      # an ESC at duty mid puts mid x the pack voltage on the motor
            lo, hi = (mid, hi) if st["motor_a"] <= esc_burst and st["pack_a"] * mid <= pack_burst else (lo, mid)
        ceiling = lo
    acl = h.make_aircraft(A.harness_design(build))
    return ceiling, acl.full_throttle_state(0.0, acl.v_cell_nom * ceiling)["thrust"] / acl.w


def evaluate(state):
    """Every stat and every gate for one parts set (state: slot -> catalogue key)."""
    key = tuple(state[s] for s in SLOTS)
    if key in _CACHE:
        return _CACHE[key]
    build = dict(BASE, **{s: state[s] for s in SLOTS}, name="candidate")
    out = dict(state=dict(zip(SLOTS, key)), cost=cost_of(state), why=[])
    try:
        A.validate(build)
        hd = h.headline(build)
    except (ValueError, KeyError) as err:
        out.update(ok=False, why=[str(err).splitlines()[-1].strip()], score=-math.inf, pen=-1e3)
        _CACHE[key] = out
        return out
    esc, s36 = A.part("esc", build["esc"]), hd["static"]["3.6"]
    ac = h.Aircraft(A.harness_design(build))              # same aircraft with every current limit removed
    ac.esc_amps, ac.batt, ac.motor = math.inf, dict(ac.batt, imax=1e9), dict(ac.motor, imax=1e9)
    wot = ac.full_throttle_state(0.0, A.V_CELL_FULL)
    out.update(mass_g=hd["mass_g"], endurance_min=hd["loiter_min"], range_km=hd["best_range_km"], vmax=hd["vmax"],
               tw=s36["tw"], tw_low=hd["static"]["3.3"]["tw"], stall=hd["v_stall_trimmed"], hover_w=hd["hover_w"],
               i_fresh=hd["i_motor_full_fresh"], i_unlimited=wot["motor_a"], pack_a=s36["pack_a"],
               pack_a_unlimited=wot["pack_a"], rpm_unlimited=wot["rpm"])
    why = list(hd["reasons"])
    if out["tw"] < TW_GATE:
        why.append(f"thrust/weight {out['tw']:.2f} < {TW_GATE}")
    if esc["amps"] < out["i_fresh"] - 1e-9:
        why.append(f"ESC {esc['amps']} A under the {out['i_fresh']:.1f} A full-throttle current")
    if esc.get("burst", 1.15 * esc["amps"]) < out["i_unlimited"]:
        why.append(f"ESC burst rating under the {out['i_unlimited']:.1f} A unlimited current")
    if out["pack_a"] > FC_SENSOR_A + 1e-6:
        why.append(f"pack current {out['pack_a']:.0f} A over the FC's {FC_SENSOR_A:.0f} A sensor")
    if out["cost"] > BUDGET:
        why.append(f"${out['cost']:.2f} over the ${BUDGET:.0f} budget")
    if not ALLOW_UNVERIFIED and not esc.get("in_stock", True):
        why.append("ESC is not in stock anywhere checked")
    out["throttle_ceiling"], out["tw_ceiling"] = (hard_ceiling(build, ac, wot) if not why else (float("nan"), float("nan")))
    if not why and out["tw_ceiling"] < h.TW_MIN:
        why.append(f"thrust/weight {out['tw_ceiling']:.2f} < {h.TW_MIN} under the {out['throttle_ceiling'] * 100:.0f} % hard output ceiling")
    out.update(ok=not why, why=why)
    _CACHE[key] = out
    return out


BASE_R = evaluate(BASE)
BASE_R["throttle_ceiling"], BASE_R["tw_ceiling"] = hard_ceiling(dict(BASE, name="v9.0"))


def tw_eff(r):
    """Thrust/weight the shipped firmware gives at 3.6 V/cell: the current limiter AND the hard output ceiling."""
    c = r.get("tw_ceiling", float("nan"))
    return min(r["tw"], c) if c == c else r["tw"]


def gains(r):
    return dict(endurance_min=r["endurance_min"] / BASE_R["endurance_min"], range_km=r["range_km"] / BASE_R["range_km"],
                vmax=r["vmax"] / BASE_R["vmax"], tw=tw_eff(r) / tw_eff(BASE_R), stall=BASE_R["stall"] / r["stall"])


def score(r):
    if "endurance_min" not in r or r["endurance_min"] <= 0 or r["range_km"] <= 0 or r["vmax"] <= 0:
        return 0.0
    return math.prod(g ** WEIGHTS[k] for k, g in gains(r).items())


def climb_value(r):
    """Score for passing builds; a graded penalty otherwise, so the climb can walk out of an infeasible start."""
    if r["ok"]:
        return score(r)
    if "tw" not in r:
        return -10.0
    pen = max(0.0, r["cost"] / BUDGET - 1) * 5 + max(0.0, TW_GATE - r["tw"]) + 0.2 * len(r["why"])
    return min(score(r), 1.0) - 1.0 - pen


def hill_climb(start, log=None):
    cur, best = dict(start), climb_value(evaluate(start))
    while True:
        moves = []
        for slot in SLOTS:
            for o in OPT[slot]:
                if o != cur[slot]:
                    c = dict(cur, **{slot: o})
                    moves.append((climb_value(evaluate(c)), f"{slot} -> {o}", c))
        for s1, s2 in PAIRS:
            for o1, o2 in itertools.product(OPT[s1], OPT[s2]):
                if o1 != cur[s1] and o2 != cur[s2]:
                    c = dict(cur, **{s1: o1, s2: o2})
                    if cost_of(c) <= BUDGET * 1.1:
                        moves.append((climb_value(evaluate(c)) - 1e-9, f"{s1} -> {o1}, {s2} -> {o2}", c))
        val, what, cand = max(moves, key=lambda m: m[0])
        if not val > best + 1e-6:
            return cur, best
        cur, best = cand, val
        if log:
            log(f"    {val:+.4f}  {what}")


def enumerate_budget():
    """Every parts set whose bill of materials fits the budget (the price filter runs before any simulation)."""
    rows = []
    for combo in itertools.product(*(OPT[s] for s in SLOTS)):
        st = dict(zip(SLOTS, combo))
        if cost_of(st) <= BUDGET:
            rows.append(evaluate(st))
    return rows


def pareto(rows):
    keys = (("endurance_min", 1), ("range_km", 1), ("vmax", 1), ("tw_ceiling", 1), ("stall", -1), ("cost", -1))
    pts = np.array([[sgn * r[k] for k, sgn in keys] for r in rows])
    return [rows[i] for i, p in enumerate(pts) if not np.any(np.all(pts >= p, axis=1) & np.any(pts > p, axis=1))]


def line(r):
    s = r["state"]
    return (f"${r['cost']:6.2f} {r['mass_g']:5.0f} g | {r['endurance_min']:4.0f} min {r['range_km']:4.0f} km {r['vmax'] * 3.6:4.0f} km/h "
            f"T/W {r['tw']:.2f}/{r['tw_low']:.2f} ({r.get('tw_ceiling', float('nan')):.2f} at {r.get('throttle_ceiling', float('nan')) * 100:3.0f} %) stall {r['stall'] * 3.6:4.1f} hover {r['hover_w']:4.0f} W "
            f"| {r['i_fresh']:4.1f}/{r['i_unlimited']:4.1f} A, pack {r['pack_a']:3.0f}/{r['pack_a_unlimited']:3.0f} A | score {score(r):.3f} | {s['motor'][:24]:24s} | {s['prop'][4:11]:7s} | "
            f"{s['esc'][:14]:14s} | {s['battery'][:24]:24s} | {s['servo'][:13]}")


def planform_study(state):
    """Span and chord climb on harness.py's parametric airframe with the chosen parts (mass ~ wing area; the flutter
    limit, spar and print-bed splits are NOT re-derived, so this is a sensitivity, not a cleared design)."""
    d0 = dict(motor=state["motor"], prop=state["prop"], battery=state["battery"], servo=state["servo"], fc=state["fc"],
              material=BASE["material"], esc=state["esc"], cr=A.ROOT_CHORD, ct=A.TIP_CHORD, span=A.SPAN)
    ref = h.evaluate(d0, full=True)

    def val(r):
        if not r["feasible"]:
            return -math.inf
        return (r["endurance_min"] / ref["endurance_min"]) ** 0.4 * (r["range_km"] / ref["range_km"]) ** 0.4 * (r["vmax"] / ref["vmax"]) ** 0.2

    rows = []
    for span in (1.72, 1.80, 1.88, 1.96, 2.04, 2.12):
        d, s = h.climb_wing(dict(d0, span=span), val)
        r = h.evaluate(d, full=True)
        rows.append(dict(span=span, cr=d["cr"], ct=d["ct"], value=s, mass_g=r["mass_g"], endurance_min=r["endurance_min"],
                         range_km=r["range_km"], vmax=r["vmax"], v_stall_trimmed=r["v_stall_trimmed"], feasible=r["feasible"]))
    return dict(reference=dict(span=A.SPAN, cr=A.ROOT_CHORD, ct=A.TIP_CHORD, mass_g=ref["mass_g"], endurance_min=ref["endurance_min"],
                               range_km=ref["range_km"], vmax=ref["vmax"], v_stall_trimmed=ref["v_stall_trimmed"]), rows=rows)


def main():
    saved = dict(h.OPTIONS)
    h.OPTIONS.update(off_aliexpress=True)
    print("options per slot: " + ", ".join(f"{s} {len(OPT[s])}" for s in SLOTS)
          + f"  ({math.prod(len(OPT[s]) for s in SLOTS):,} parts sets)")
    print(f"\nv9.0 reference:\n  {line(BASE_R)}")
    if not BASE_R["ok"]:
        print("  v9.0 no longer passes: " + "; ".join(BASE_R["why"]))

    print("\nhill climb from v9.0:")
    best_state, best_val = hill_climb({s: BASE[s] for s in SLOTS}, log=print)
    rng = random.Random(1)
    ends = [(best_val, best_state)]
    for _ in range(4):
        st, v = hill_climb({s: rng.choice(OPT[s]) for s in SLOTS})
        ends.append((v, st))
    ends.sort(key=lambda e: -e[0])
    print(f"  5 starts end at scores {', '.join(f'{v:.4f}' for v, _ in ends)}")
    climbed = evaluate(ends[0][1])
    print(f"  best end point:\n  {line(climbed)}")

    rows = enumerate_budget()
    good = sorted((r for r in rows if r["ok"]), key=lambda r: -score(r))
    print(f"\nfull enumeration: {len(rows):,} parts sets fit ${BUDGET:.0f}, {len(good):,} pass every gate")
    if not good:
        print("  nothing passes: raise --budget or relax a gate")
        return None
    print(f"  global best score {score(good[0]):.4f}; the climb found {score(climbed):.4f}"
          f" ({'the same build' if good[0]['state'] == climbed['state'] else 'a DIFFERENT build'})")
    print("  top 12 by score:")
    for r in good[:12]:
        print("  " + line(r))
    dom = [r for r in good if all(g >= 1.0 - 1e-9 for g in gains(r).values()) and r["state"] != BASE_R["state"]]
    print(f"\n{len(dom)} builds are at least as good as v9.0 on EVERY stat; best of them:")
    for r in dom[:5]:
        print("  " + line(r))
    print("\nbest on each single stat (passing builds):")
    for name, key, sgn in (("endurance", "endurance_min", 1), ("range", "range_km", 1), ("top speed", "vmax", 1),
                           ("thrust/weight", "tw_ceiling", 1), ("lowest stall", "stall", -1), ("cheapest", "cost", -1)):
        r = max(good, key=lambda r: (sgn * r[key], score(r)))
        print(f"  {name:13s} {line(r)}")
    front = sorted(pareto(good), key=lambda r: -score(r))
    print(f"\nPareto front (endurance, range, speed, T/W, stall, cost): {len(front)} builds")
    winner = dom[0] if dom else good[0]
    plan = planform_study(winner["state"])
    ref = plan["reference"]
    print(f"\nplanform sensitivity with the winning parts (parametric airframe; current wing {ref['span'] * 1000:.0f} mm, "
          f"{ref['cr'] * 1000:.0f}/{ref['ct'] * 1000:.0f} mm: {ref['endurance_min']:.0f} min, {ref['range_km']:.0f} km, {ref['vmax'] * 3.6:.0f} km/h):")
    for p in plan["rows"]:
        print(f"  span {p['span'] * 1000:4.0f} mm, chords {p['cr'] * 1000:3.0f}/{p['ct'] * 1000:3.0f} mm: {p['mass_g']:5.0f} g, {p['endurance_min']:4.0f} min "
              f"({100 * (p['endurance_min'] / ref['endurance_min'] - 1):+.1f} %), {p['range_km']:4.0f} km ({100 * (p['range_km'] / ref['range_km'] - 1):+.1f} %), "
              f"{p['vmax'] * 3.6:4.0f} km/h, stall {p['v_stall_trimmed'] * 3.6:.1f} km/h{'' if p['feasible'] else '  NOT FEASIBLE'}")
    h.OPTIONS.update(saved)
    if "--budget" in sys.argv:
        return winner
    (HERE / "hillclimb_v9_results.json").write_text(json.dumps(dict(
        budget=BUDGET, weights=WEIGHTS, base=BASE_R, winner=winner, climb_end=climbed, global_best=good[0],
        n_fit_budget=len(rows), n_pass=len(good), n_dominate_base=len(dom), top=good[:25], pareto=front[:60],
        planform=plan), indent=1, default=float))
    return winner


if __name__ == "__main__":
    main()
