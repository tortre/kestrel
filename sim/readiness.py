"""Control, trim, power-system and uncertainty checks for the upgraded Kestrel-560 v5.

Run:  .venv/Scripts/python sim/readiness.py        -> readiness/checks.json and a printed table

Fixes over v4:
  1. Exact 3D spanwise elevons (y = 95..273 mm) modelled directly on WingXSec stations instead of the
     approximate SPAN_FRAC = 0.89 secant-step scaling bug.
  2. Reports trim and trimmed stall for BOTH the conservative 1D AeroBuildup neutral point AND the
     3D lifting-surface (VLM / SU2 CFD) pitching-moment physics.

Parts, mass, CG, air density and the ESC limit come from sim/aircraft.py (the current build). Needs
sim/envelope.json (run envelope.py first; regenerate.py does both in order). Importing this module has no side effects.
"""
import json
import math
import pathlib

import aerosandbox as asb
import numpy as np

import aircraft as A
import blended as B
import harness as h
import stability

ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT = ROOT / "readiness"
ENV_PATH = ROOT / "sim" / "envelope.json"
D = h.CURRENT                      # exactly the current build (sim/aircraft.py)
MASS = A.all_up_mass_kg()
W = MASS * A.G
RHO = A.RHO
WING = (A.SPAN, A.ROOT_CHORD, A.TIP_CHORD)
_T = B.mass_table(B.GEO["battery_x"])
BATT_G = _T["battery"][0]
X_CG_CAD = sum(g * x for g, x in _T.values()) / sum(g for g, _ in _T.values())
X_NP_CAD = None
X_NP_3D_CAD = None
V_CELLS = (4.2, 4.1, 3.9, 3.7, 3.6, 3.5, 3.3, 3.1)     # resting volts per cell for the full-throttle table


def plane(elevator=0.0, aileron=0.0):
    return stability.airplane(*WING, elevator=elevator, aileron=aileron)


PLANE_S = plane().s_ref


def aero(v, alpha, elevator=0.0, aileron=0.0, x_cg=X_CG_CAD, derivs=False):
    ab = asb.AeroBuildup(plane(elevator, aileron), asb.OperatingPoint(velocity=v, alpha=alpha), xyz_ref=[-x_cg / 1000, 0, 0])
    a = ab.run_with_stability_derivatives() if derivs else ab.run()
    return {k: float(np.ravel(val)[0]) for k, val in a.items() if np.size(val) == 1}


_TPOL_CACHE = {}


def trimmed_polar(v, x_cg=X_CG_CAD, cm_offset=0.0):
    """Exact trim solver covering BOTH trailing-edge-up (-16, -8 deg) AND trailing-edge-down (+4, +8 deg) elevon
    so low-CL high-speed & dive points are never silently clamped (Claude Opus 5.5 audit fix!)."""
    key = (round(float(v), 2), round(float(x_cg), 3), round(float(cm_offset), 5))
    if key in _TPOL_CACHE:
        return _TPOL_CACHE[key]
    alphas = np.arange(-4.0, 16.1, 1.0)
    ds = np.array([-18.0, -10.0, 0.0, 8.0, 16.0])
    cms_all, cls_all = [], []
    for d_val in ds:
        a = asb.AeroBuildup(plane(float(d_val), 0.0), asb.OperatingPoint(velocity=float(v), alpha=alphas), xyz_ref=[-x_cg / 1000, 0, 0]).run()
        cms_all.append(np.ravel(a["Cm"]) + cm_offset)
        cls_all.append(np.ravel(a["CL"]))
    cms_all = np.array(cms_all)
    cls_all = np.array(cls_all)
    rows = []
    for i, al in enumerate(alphas):
        cms = cms_all[:, i]
        cls = cls_all[:, i]
        if (cms.min() <= 0.0) and (cms.max() >= 0.0):
            order = np.argsort(cms)
            d_trim = float(np.clip(np.interp(0.0, cms[order], ds[order]), -28.0, 28.0))
            cl_trim = float(np.interp(d_trim, ds, cls))
            rows.append((float(al), -d_trim, cl_trim))
    _TPOL_CACHE[key] = rows
    return rows


def trim(v, n=1.0, x_cg=X_CG_CAD, cm_offset=0.0):
    pol = np.array(trimmed_polar(v, x_cg, cm_offset=cm_offset))
    i = int(np.argmax(pol[:, 2]))
    cl_need = n * W / (0.5 * RHO * v * v * PLANE_S)
    cl_min = float(pol[:i + 1, 2].min())
    ok = (cl_min <= cl_need <= pol[i, 2])
    return dict(v=v, n=n, cl=cl_need, cl_min_trimmed=cl_min, cl_max_trimmed=float(pol[i, 2]), ok=bool(ok),
                alpha=float(np.interp(cl_need, pol[:i + 1, 2], pol[:i + 1, 0])) if ok else None,
                elevon_up_deg=float(np.interp(cl_need, pol[:i + 1, 2], pol[:i + 1, 1])) if ok else None,
                elevon_up_at_clmax=float(pol[i, 1]), alpha_at_clmax=float(pol[i, 0]))


def full_throttle(ac, v, v_cell):
    """Full-throttle point with the pack at v_cell resting volts per cell (harness.Aircraft.full_throttle_state)."""
    return ac.full_throttle_state(v, v_cell)


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    ENV = json.loads(ENV_PATH.read_text())
    out = {"build": A.CURRENT_BUILD["name"], "mass_g": MASS * 1000}
    PLANE_S = plane().s_ref
    mac = plane().c_ref

    a2, a6 = aero(18.0, 2.0, x_cg=0.0), aero(18.0, 6.0, x_cg=0.0)
    X_NP_CAD = (a6["Cm"] - a2["Cm"]) / (a6["CL"] - a2["CL"]) * mac * 1000
    X_NP_3D_CAD = -B.neutral_point_vlm(B.airplane()) * 1000

    v_s, v_c, v_top = ENV["v_stall"], ENV["altitude"][0]["v_cruise"], ENV["v_max_level"]
    pull = max(ENV["pullout"], key=lambda d: d["v_peak"])
    cases = [("untrimmed stall speed x 1.1", 1.1 * v_s, 1.0), ("cruise", v_c, 1.0), ("top speed", v_top, 1.0),
             ("vertical dive, idle", ENV["dive"][-1]["idle_sl"], 1.0), ("pull-out, peak g", pull["v_peak"], min(6.0, pull["n_peak"])),
             ("3 g turn at cruise speed x 1.5", 1.5 * v_c, 3.0)]
    out["trim"] = []
    print(f"reference area {PLANE_S * 1e4:.0f} cm2, MAC {mac * 1000:.0f} mm, CG at x = {X_CG_CAD:.1f} mm, {MASS * 1000:.0f} g")
    print(f"neutral points: 3D VLM/CFD x = {X_NP_3D_CAD:.1f} mm  |  conservative 1D strip x = {X_NP_CAD:.1f} mm\n")
    print("trim (conservative 1D NP)        speed     alpha   elevon up   lift needed / trimmed max")
    for label, v, n in cases:
        t = trim(v, n)
        t["label"] = label
        out["trim"].append(t)
        state = f"{t['alpha']:5.1f}   {t['elevon_up_deg']:+5.1f} deg" if t["ok"] else "  cannot trim: stalls first"
        print(f"  {label:32s} {v * 3.6:5.0f} km/h {state:28s}  {t['cl']:.2f} / {t['cl_max_trimmed']:.2f}")

    out["stall"] = {}
    out["balance"] = dict(x_np_1d=X_NP_CAD, x_np_3d=X_NP_3D_CAD, x_cg=X_CG_CAD,
                          battery_x=B.GEO["battery_x"], battery_x_min=B.GEO.get("battery_x_min"),
                          battery_x_max=B.GEO.get("battery_x_max"))
    lo_b = B.GEO.get("battery_x_min", 4.8)
    hi_b = B.GEO.get("battery_x_max", 35.2)
    for xb, label in ((lo_b, "battery fully aft"), (B.GEO["battery_x"], "battery as drawn"), (hi_b, "battery fully forward")):
        x_cg = X_CG_CAD + (xb - B.GEO["battery_x"]) * BATT_G / (MASS * 1000)
        v = 13.0
        for _ in range(4):
            t = trim(v, 1.0, x_cg)
            v = math.sqrt(2 * W / (RHO * PLANE_S * t["cl_max_trimmed"]))
        m_1d = (x_cg - X_NP_CAD) / (mac * 1000)
        m_3d = (x_cg - X_NP_3D_CAD) / (mac * 1000)
        out["stall"][label] = dict(x_cg=x_cg, margin_1d=m_1d, margin_3d=m_3d, v_stall=v,
                                   cl_max=t["cl_max_trimmed"], elevon_up=t["elevon_up_at_clmax"], alpha=t["alpha_at_clmax"])
        print(f"  trimmed stall, {label:22s}: {v * 3.6:.1f} km/h (CL_max {t['cl_max_trimmed']:.2f} @ alpha {t['alpha_at_clmax']:.0f}, elevon {t['elevon_up_at_clmax']:+.1f} up; "
              f"3D margin {m_3d * 100:.1f} %, 1D margin {m_1d * 100:.1f} %)")
    v_trim_stall = out["stall"]["battery as drawn"]["v_stall"]

    out["roll"] = []
    for label, v in (("slow", 1.15 * v_trim_stall), ("cruise", max(v_c, 1.3 * v_trim_stall)), ("top speed", v_top)):
        t = trim(v, 1.0)
        base = aero(v, t["alpha"], -t["elevon_up_deg"], derivs=True)
        roll = aero(v, t["alpha"], -t["elevon_up_deg"], aileron=10.0)
        cl_da = roll["Cl"] - base["Cl"]
        p = -cl_da / base["Clp"] * 2 * v / plane().b_ref
        out["roll"].append(dict(label=label, v=v, roll_rate_deg_s=abs(math.degrees(p)), cl_per_10deg=abs(cl_da), clp=base["Clp"],
                                cnb=base["Cnb"], clb=base["Clb"], cma=base["Cma"]))
        print(f"roll at {label:9s}: {abs(math.degrees(p)):4.0f} deg/s for 10 deg of opposite elevon;  Cm_a {base['Cma']:+.3f}  Cn_b {base['Cnb']:+.4f}  Cl_b {base['Clb']:+.4f}")

    r = h.evaluate(D, full=True)
    ac = h.make_aircraft(D)
    out["power"] = []
    print(f"\nESC {ac.esc_name} ({ac.esc_amps} A), motor limit {ac.motor['imax']} A, pack limit {ac.batt['imax']} A, {ac.mass * 1000:.0f} g")
    print("cell volts (resting)   static thrust / weight   motor A   pack A   pack V under load   motor V")
    for vc in V_CELLS:
        f = full_throttle(ac, 0.0, vc)
        f.update(v_cell=vc, tw=f["thrust"] / ac.w)
        out["power"].append(f)
        print(f"   {vc:.1f}                    {f['tw']:.2f}                {f['motor_a']:5.1f}    {f['pack_a']:5.1f}       {f['pack_v']:5.2f}          {f['motor_v']:5.2f}")
    p_h, vm_h, im_h, n_h = ac.static(ac.w)
    pack_a = p_h / ac.vbat                            # nominal pack volts: 3.6 V/cell Li-ion, 3.7 V/cell LiPo
    out["hover"] = dict(watts=p_h, motor_a=im_h, motor_v=vm_h, rpm=n_h * 60, pack_a=pack_a, pack_v_nominal=ac.vbat,
                        minutes_full_pack=r["hover_min"], motor_heat_w=im_h**2 * ac.rm + ac.i0 * vm_h)
    f = full_throttle(ac, 0.0, 3.9)
    out["full_throttle_heat_w"] = f["motor_a"]**2 * ac.rm + ac.i0 * f["motor_v"]
    print(f"hover: {p_h:.0f} W, {im_h:.1f} A per motor, {pack_a:.1f} A pack at {ac.vbat:.1f} V, {n_h * 60:.0f} rpm; motor heat {out['hover']['motor_heat_w']:.1f} W "
          f"({out['full_throttle_heat_w']:.1f} W at full throttle, 3.9 V/cell, motor at {f['motor_v']:.2f} V)")
    out["nominal"] = {k: r[k] for k in ("mass_g", "vmax", "endurance_min", "range_km", "v_stall", "v_stall_trimmed", "v_min_ops",
                                        "v_cruise", "p_cruise", "v_loiter", "p_loiter", "p_hover", "hover_min", "esc", "i_motor_full")}

    mc = h.monte_carlo(D, n=400)
    pct = lambda col, q: float(np.percentile(mc[:, col], q))
    out["monte_carlo"] = dict(n=len(mc), feasible_fraction=float(mc[:, 3].mean()), vmax_p5=pct(0, 5), vmax_p50=pct(0, 50),
                              endurance_p5=pct(1, 5), endurance_p50=pct(1, 50), range_p5=pct(2, 5), range_p50=pct(2, 50))
    m = out["monte_carlo"]
    print(f"\n400 Monte Carlo runs: {m['feasible_fraction'] * 100:.1f} % meet every design floor; "
          f"top speed {m['vmax_p5'] * 3.6:.0f}..{m['vmax_p50'] * 3.6:.0f} km/h (P5..P50), "
          f"endurance {m['endurance_p5']:.0f}..{m['endurance_p50']:.0f} min, range {m['range_p5']:.0f}..{m['range_p50']:.0f} km")
    (OUT / "checks.json").write_text(json.dumps(out, indent=1, default=float))
