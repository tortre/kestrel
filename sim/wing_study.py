"""Span & chord trade study for Kestrel-X Stealth v7 (composite 8S vectored).   .venv/Scripts/python sim/wing_study.py"""
import json
import math
import pathlib

import blended as B
import harness as h
import readiness as R
import stability

BASE = dict(B.GEO)
MARGIN = 0.10
BED = 220.0


def evaluate(span, cr, ct):
    half = span * 1000 / 2
    ey0, ey1 = round(0.59 * half, 1), round(0.79 * half, 1)
    g = dict(BASE, half=half, root_chord=cr * 1000, tip_chord=ct * 1000, elevon_y=[ey0, ey1])
    g["tip_le"] = g["root_le"] - math.tan(math.radians(g["sweep_deg"])) * (half - g["blend_y"])
    B.GEO.clear(); B.GEO.update(g)
    stability.GEO0.clear(); stability.GEO0.update(g)
    R.WING = (span, cr, ct)
    p = R.plane()
    R.PLANE_S, mac = p.s_ref, p.c_ref * 1000
    r = h.evaluate(dict(h.BASELINE, span=span, cr=cr, ct=ct), full=True)
    mass = r["mass_g"]
    w = mass / 1000 * 9.81
    R.W = w
    a2, a6 = R.aero(17.0, 2.0, x_cg=0.0), R.aero(17.0, 6.0, x_cg=0.0)
    x_np = (a6["Cm"] - a2["Cm"]) / (a6["CL"] - a2["CL"]) * mac
    x_cg = x_np + MARGIN * mac
    v = 13.0
    for _ in range(3):
        t = R.trim(v, 1.0, x_cg)
        v = math.sqrt(2 * w / (R.RHO * R.PLANE_S * t["cl_max_trimmed"]))
    elevon_len = ey1 - ey0
    return dict(span=span, cr=cr, ct=ct, area_cm2=R.PLANE_S * 1e4, mass_g=mass, v_stall=v, cl_trim=t["cl_max_trimmed"],
                vmax=r["vmax"], endurance=r["endurance_min"], range_km=r["range_km"], v_cruise=r["v_cruise"], feasible=bool(r["feasible"]),
                reasons=r["reasons"], elevon_len=elevon_len, elevon_fits=elevon_len <= BED, hover_w=r["p_hover"],
                wing_loading=mass / (R.PLANE_S * 100))


if __name__ == "__main__":
    rows = []
    print("span  root/tip chord | area cm2  mass g  loading g/dm2 | trimmed stall | top speed  endurance  range | elevon length")
    for span in (1.960, 2.120, 2.260):
        for cr, ct in ((0.172, 0.076), (0.184, 0.082), (0.198, 0.090)):
            r = evaluate(span, cr, ct)
            rows.append(r)
            print(f"{span * 1000:4.0f}   {cr * 1000:3.0f} / {ct * 1000:3.0f}      |  {r['area_cm2']:5.0f}    {r['mass_g']:4.0f}      {r['wing_loading']:4.0f}       |"
                  f"   {r['v_stall'] * 3.6:4.0f} km/h   |  {r['vmax'] * 3.6:4.0f} km/h   {r['endurance']:4.0f} min  {r['range_km']:4.0f} km |"
                  f"  {r['elevon_len']:4.0f} mm{'' if r['elevon_fits'] else ' (over 220 mm bed)'}", flush=True)
    (pathlib.Path(__file__).resolve().parent.parent / "readiness" / "wing_study.json").write_text(json.dumps(rows, indent=1))
