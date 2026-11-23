"""Close-out check of Devin's 3 final items (Claude, 2026-11-20). Read-only. build123d is stubbed (not in .venv)."""
import json, math, pathlib, sys
from unittest import mock
import numpy as np
ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))
sys.modules.setdefault("build123d", mock.MagicMock())
import importlib.util
spec = importlib.util.spec_from_file_location("P", ROOT / "src" / "printable.py")
P = importlib.util.module_from_spec(spec); spec.loader.exec_module(P)

print("1. Straight CAD channel (rect_beam between its two spar_point ends) vs skin, across the cap width")
for name, y0, y1, w, h in (("inner", P.SPAR_Y0, 415.0, P.CAP_W_IN, P.SPAR_H_IN), ("mid", 395.0, 735.0, P.CAP_W_MID, P.SPAR_H_MID),
                           ("outer", 715.0, P.HALF - 16.0, P.CAP_W_OUT, P.SPAR_H_OUT)):
    a, b = P.spar_point(P.SPAR_FRAC, y0), P.spar_point(P.SPAR_FRAC, y1)
    worst = (9e9,)
    for y in np.linspace(max(y0, P.BLEND_Y), y1, 60):
        f = (y - y0) / (y1 - y0)
        zb = a[2] + f * (b[2] - a[2])                    # beam centre-line z at this y
        c = P.chord(y)
        for ds in np.linspace(-w / 2, w / 2, 9):
            s = P.SPAR_FRAC + ds / c
            cu, cl = P.surf(s, y, True) - (zb + h / 2), (zb - h / 2) - P.surf(s, y, False)
            if min(cu, cl) < worst[0]:
                worst = (min(cu, cl), y, ds, cu, cl, zb, P.spar_point(P.SPAR_FRAC, y)[2])
    m, y, ds, cu, cl, zb, zmid = worst
    print(f"  {name:5s} h {h:4.1f}: min cover {m:+5.2f} mm at y={y:4.0f}, dx {ds:+5.1f} (upper {cu:+5.2f} / lower {cl:+5.2f});"
          f" beam z {zb:5.2f} vs camber-mid {zmid:5.2f} -> {'BREAKS SKIN' if m < 0 else 'thin (<0.65)' if m < 0.65 else 'ok'}")

import aircraft as A, aeroelastic as AE  # noqa: E402
sp = A.spar_design()
print("\n2. Spar:", [(z["name"], z["h_box_mm"], z["t_mm"], z["t_web_mm"], z["tau_web_mpa"], z["g_limit"]) for z in sp["schedule"]])
print("   root box", sp["root_box_t_mm"], "mm, tau", sp["tau_box_mpa"], "MPa, g_tors", sp["g_lim_root_torsion"], "| min", sp["min_g_limit"], "g | mass", round(sp["mass_g"], 1), "g")
print("   CAD CAP_T", (P.CAP_T_IN, P.CAP_T_MID, P.CAP_T_OUT), "SPAR_H", (P.SPAR_H_IN, P.SPAR_H_MID, P.SPAR_H_OUT))
for z in sp["schedule"][1:]:
    print(f"   {z['name']:9s}: cap t/h = {z['t_mm'] / z['h_box_mm']:.2f} (sizing loop stops at 0.35); web clear depth {z['h_box_mm'] - 2 * z['t_mm']:.1f} mm")

# ---- 3. Does THR_MAX = 78 % really cap level speed <= 200 km/h?  harness "full throttle" = rpm capped at n_max.
import harness as H  # noqa: E402
ac = H.make_aircraft(A.harness_design(A.CURRENT_BUILD))
print(f"\n3. pack {ac.ns}S, Kv {ac.kv}, prop rpm limit {ac.n_max * 60:.0f}, J-table max {max(ac.J):.2f}")
def level_state(v, v_avail):
    """Highest rpm whose motor volts fit in v_avail and thrust >= drag -> (thrust-drag, rpm, motor V) at airspeed v."""
    va = np.atleast_1d(float(v))
    drag, _ = ac.drag(va)
    lo, hi = np.full(1, 5.0), np.full(1, 2000.0)
    for _ in range(50):
        mid = (lo + hi) / 2
        _, vm, im, _ = ac.electrical(mid, va, v_avail)
        lo, hi = np.where(vm <= v_avail, mid, lo), np.where(vm <= v_avail, hi, mid)
    return float(ac.thrust(lo, va)[0] * 2 - drag[0]), float(lo[0] * 60)
v_pack = ac.ns * ac.v_cell_nom * 0.97                      # nominal cell, ~3 % sag at cruise-dash current
for thr in (1.00, 0.78, 0.65, 0.55):
    vmax, rpm = None, None
    for v in np.arange(30.0, 110.0, 0.5):
        if v / ((level_state(v, thr * v_pack)[1] / 60) * ac.d) > max(ac.J):
            break                                          # beyond the prop table: stop (treat as windmill limit)
        x, n = level_state(v, thr * v_pack)
        if x < 0:
            break
        vmax, rpm = v, n
    print(f"   THR {thr:4.0%} (~{thr * v_pack:4.1f} V at motor, linear duty): level Vmax ~{vmax * 3.6:4.0f} km/h at {rpm:6.0f} rpm"
          f" {'> APC limit' if rpm > ac.n_max * 60 else ''}")
for thr, v_kmh in ((0.78, 265.0), (0.65, 223.0), (0.55, 189.0)):
    v = v_kmh / 3.6
    _, rpm = level_state(v, thr * v_pack)
    p, vm, im, _ = ac.electrical(np.atleast_1d(rpm / 60), np.atleast_1d(v), thr * v_pack)
    print(f"   THR {thr:.0%} @ {v_kmh:.0f} km/h: motor {im[0]:5.1f} A (limit {ac.motor['imax']}), {2 * p[0]:5.0f} W electrical")

# ---- 4. V_NE sensitivity to the cap modulus (230 GPa HM/M40J strip assumed; standard pultruded T700 strip ~135 GPa)
import copy, blended as B  # noqa: E402
p = B.airplane()
print("\n4. V_NE vs cap modulus (EI_caps scaled; skin EI unchanged)")
for e_gpa in (230.0, 180.0, 135.0):
    s2 = copy.deepcopy(sp)
    for z in s2["schedule"]:
        z["i_mm4"] *= e_gpa / 230.0
    ae = AE.aeroelastic_check(A.all_up_mass_kg(), 0.0562, p.s_ref, p.c_ref, A.SPAN, A.GEO["blend_y"] / 1000.0,
                              A.ROOT_CHORD, A.TIP_CHORD, A.GEO["sweep_deg"], A.GEO["elevon_y"][0] / 1000.0,
                              A.GEO["elevon_y"][1] / 1000.0, A.GEO["hinge"], s2, sm_frac=0.082)
    print(f"   E {e_gpa:5.0f} GPa: f_bend {ae['f_bend_hz']:5.1f} Hz, V_bff {ae['v_flutter_bff_kmh']:5.0f}, V_bt {ae['v_flutter_bt_kmh']:5.0f}"
          f" -> V_NE {ae['v_ne_aeroelastic_kmh']:5.0f} km/h")
