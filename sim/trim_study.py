"""Which geometry change brings the trimmed stall speed down?   .venv/Scripts/python sim/trim_study.py

Varies the wing position (WING_SHIFT), washout and battery position, and for each reports the neutral point
(AeroBuildup with fins), the centre of gravity, the stability margin by both aero models and the trimmed stall.
The CFD neutral point is taken as the AeroBuildup one moved 20 mm aft (the offset found on the built shape).
"""
import math
import sys

import numpy as np

import blended as B
import stability
import readiness as R

BASE = dict(B.GEO)
WINGY = ("wing_", "elevon_", "spar tubes", "stub tubes", "servos")
CFD_OFFSET = 20.0


def setup(shift, washout):
    d = BASE["wing_shift"] - shift                       # mm the wing moves forward
    g = dict(BASE, wing_shift=shift, washout=washout, root_le=BASE["root_le"] + d, tip_le=BASE["tip_le"] + d,
             servo_x=BASE["servo_x"] + d)
    B.GEO.clear(); B.GEO.update(g)
    stability.GEO0.clear(); stability.GEO0.update(g)
    return d


def cg(d, battery_x, extra=()):
    t = B.mass_table(battery_x)
    items = [(g, x + (d if k.startswith(WINGY) else 0.0)) for k, (g, x) in t.items()] + list(extra)
    m = sum(g for g, _ in items)
    return sum(g * x for g, x in items) / m, m


def neutral_point(v=18.0):
    a = [R.aero(v, al, x_cg=0.0) for al in (2.0, 6.0)]
    return -(a[1]["Cm"] - a[0]["Cm"]) / (a[1]["CL"] - a[0]["CL"]) * R.plane().c_ref * 1000 * -1


def evaluate(shift, washout, battery_x, extra=()):
    d = setup(shift, washout)
    R.PLANE_S = R.plane().s_ref
    mac = R.plane().c_ref * 1000
    x_np = neutral_point()
    x_cg, m = cg(d, battery_x, extra)
    w = m / 1000 * 9.81
    R.W = w
    v = 14.0
    for _ in range(3):
        t = R.trim(v, 1.0, x_cg)
        v = math.sqrt(2 * w / (R.RHO * R.PLANE_S * t["cl_max_trimmed"]))
    margin_cfd = (x_cg - (x_np - CFD_OFFSET)) / mac
    clean = max(R.aero(v, al, x_cg=x_cg)["CL"] for al in (11.0, 12.0, 13.0, 14.0))
    cl_cfd = clean / (1 + 0.029 * margin_cfd / 0.011)     # zero-lift moment about zero, elevon power from AeroBuildup
    cruise = R.trim(max(18.3, 1.3 * v), 1.0, x_cg)
    return dict(shift=shift, washout=washout, battery_x=battery_x, mass=m, x_np=x_np, x_cg=x_cg, margin=(x_cg - x_np) / mac, margin_cfd=margin_cfd,
                v_stall=v, cl_trim=t["cl_max_trimmed"], elevon_stall=t["elevon_up_at_clmax"], v_stall_cfd=math.sqrt(2 * w / (R.RHO * R.PLANE_S * cl_cfd)),
                cruise_elevon=cruise["elevon_up_deg"], cruise_v=cruise["v"])


if __name__ == "__main__":
    cases = [(30, 2, 28)] + [(s, w, b) for s in (30, 20, 12) for w in (2, 4, 6) for b in (28, 10)][1:]
    if len(sys.argv) > 1:
        cases = [tuple(float(x) for x in a.split(",")) for a in sys.argv[1:]]
    print("wing shift  washout  battery x |  NP     CG   margin  (CFD-type margin) | trimmed stall km/h  (CFD-type) | elevon up at stall / cruise")
    for s, w, b in cases:
        r = evaluate(s, w, b)
        print(f"   {s:4.0f}      {w:3.0f}      {b:4.0f}    | {r['x_np']:5.1f} {r['x_cg']:5.1f}  {r['margin'] * 100:5.1f} %     ({r['margin_cfd'] * 100:4.1f} %)      |"
              f"     {r['v_stall'] * 3.6:4.0f}            ({r['v_stall_cfd'] * 3.6:3.0f})    |   {r['elevon_stall']:+5.1f} / {r['cruise_elevon']:+5.1f}", flush=True)
