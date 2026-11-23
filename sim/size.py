"""How small can the aircraft be? Re-optimise the parts and chords at each span against a reference design.

Run:  .venv/Scripts/python sim/size.py

At each span the hill climb maximises the worst of the five ratios to the reference (top speed, endurance,
range up; mass, cost down), so a score >= 1.00 means every stat is kept or better. The winner at each span
is then checked for handling against the reference with sim/stability.py.
"""
import json
import math

import numpy as np

import harness as h
import stability

h.OPTIONS["off_aliexpress"] = True          # the reference uses Molicel cells bought elsewhere
REF = dict(motor="T-Motor AT2306 1900KV", prop="7x6E", battery="Molicel M65A 3S1P (not on AliExpress)",
           servo="Emax ES9051 4.3g", fc="JHEMCU F405 Wing", material="LW-PLA (foaming)", cr=0.160, ct=0.100, span=0.610)
SPANS = (0.61, 0.60, 0.59, 0.58, 0.56, 0.54, 0.52, 0.50, 0.48, 0.46)


def ratios(r, ref):
    return dict(speed=r["vmax"] / ref["vmax"], endurance=r["endurance_min"] / ref["endurance_min"],
                range=r["range_km"] / ref["range_km"], mass=ref["mass_g"] / r["mass_g"], cost=ref["cost"] / r["cost"])


def maximin(ref):
    return lambda r: min(ratios(r, ref).values()) if h.meets_floors(r) else -math.inf


def best_at(span, ref, starts=3, seed=2):
    f = maximin(ref)
    rng = np.random.default_rng(seed)
    best = (-math.inf, None)
    for i in range(starts):
        d = dict(REF, span=span) if i == 0 else dict(
            {k: h.CHOICES[k][rng.integers(len(h.CHOICES[k]))] for k in h.KEYS}, cr=0.20, ct=0.12, span=span)
        d, s = h.hill_climb(d, f)
        if s > best[0]:
            best = (s, d)
    return best[1], best[0]


if __name__ == "__main__":
    ref = h.evaluate(REF, full=True)
    ref_st = stability.analyse(REF)
    print(f"reference, 610 mm: {h.line(ref)}")
    print("\n span | score | speed endur range  mass  cost | top speed | endur. | range |  mass |   cost  | stall | handling vs reference")
    out = []
    for span in SPANS:
        d, score = best_at(span, ref)
        if d is None or not math.isfinite(score):
            print(f" {span * 1000:4.0f} | no feasible design")
            continue
        r = h.evaluate(d, full=True)
        k = ratios(r, ref)
        st = stability.analyse(d)
        why = stability.handling_ok(st, ref_st)
        print(f" {span * 1000:4.0f} | {score:5.3f} | " + " ".join(f"{100 * (v - 1):+5.1f}" for v in k.values())
              + f" | {h.line(r)} | {r['v_stall']:4.1f}  | {'kept' if not why else '; '.join(why)}")
        print(f"        {d['motor']} / {d['prop']} / {d['battery'][:28]} / {d['servo'][:12]} / {d['fc'][:18]} / wing {d['cr'] * 1000:.0f}-{d['ct'] * 1000:.0f}", flush=True)
        out.append(dict(d, score=score, handling=why, stall=r["v_stall"], **{m: r[m] for m, _ in h.METRICS},
                        stability={lab: p for lab, p in st["points"].items()}, x_batt_mm=st["x_batt_mm"]))
    (h.HERE / "results_size.json").write_text(json.dumps(out, indent=1))
    print("\nwrote sim/results_size.json")
