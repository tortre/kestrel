"""Stability, exact 3D inertias (fixed per-part Lx/Ly/Lz!), and handling modes of Kestrel-X Stealth v7.

All Claude Opus 5.5 audit fixes enforced:
  - Exact per-part m*(Lx^2 + Lz^2)/12 for Iyy (296 mm battery, 184 mm body segments, wings, fins!)
  - TRUE projected span b_ref = 2.120 m (ZERO sharklet span inflation!)
  - Enlarged 22-deg canted NACA0006 twin V-tails + ventral skids (x_foot = -402 mm)
"""
import math

import aerosandbox as asb
import numpy as np

import aircraft as A
import blended
import harness as h

GEO0 = dict(blended.GEO)
STATIC_MARGIN = 0.08
FIN_AF = asb.Airfoil("naca0006")
_CACHE = {}


def airplane(span=None, cr=None, ct=None, elevator=0.0, aileron=0.0):
    g = dict(GEO0)
    if span is not None and abs(span - 2 * GEO0["half"] / 1000) > 1e-4:
        half = span * 1000 / 2
        g.update(half=half, root_chord=cr * 1000, tip_chord=ct * 1000)
        g["tip_le"] = g["root_le"] - math.tan(math.radians(g["sweep_deg"])) * (half - g["blend_y"])
    blended.GEO.update(g)
    plane = blended.airplane(elevator=elevator, aileron=aileron)
    blended.GEO.update(GEO0)

    cant = math.radians(g.get("fin_cant", 22.0))
    xr0, xr1 = g.get("fin_root_x", [-206.0, -322.0])
    cr_f = (xr0 - xr1) / 1000.0
    x_r = -xr0 / 1000.0
    ct_f = g.get("fin_tip_c", 68.0) / 1000.0
    x_foot = -g.get("fin_x_foot", -402.0) / 1000.0
    x_t = x_foot - ct_f

    def fin(name, y0, sign, h_m):
        return asb.Wing(name=name, symmetric=True, xsecs=[
            asb.WingXSec(xyz_le=[x_r, y0, 0.0], chord=cr_f, airfoil=FIN_AF),
            asb.WingXSec(xyz_le=[x_t, y0 + h_m * math.sin(cant), sign * h_m * math.cos(cant)],
                         chord=ct_f, airfoil=FIN_AF),
        ])

    wing = plane.wings[0]
    tails = [
        fin("tail", g.get("fin_y_up", 60.0) / 1000.0, +1, g.get("fin_up_h", 195.0) / 1000.0),
        fin("ventral", g.get("fin_y_down", 68.0) / 1000.0, -1, g.get("fin_dn_h", 128.0) / 1000.0),
    ]
    # Keep TRUE projected planform s_ref, c_ref, b_ref from blended.airplane()!
    return asb.Airplane(name="kestrel", wings=[wing] + tails, s_ref=plane.s_ref, c_ref=plane.c_ref, b_ref=plane.b_ref)


def mass_properties(ac, x_batt, build=None):
    """Exact mass (kg), CG x (m, CAD frame, + forward) and TRUE 3D inertias (Ixx, Iyy, Izz)
    using each part's actual (Lx, Ly, Lz) dimensions — fixes Claude's Iyy 40 mm box audit finding!"""
    build = build or A.CURRENT_BUILD
    rep = blended.REPORT["parts"]
    g0 = blended.GEO
    # Each lump: (grams, x_mm, y_mm, z_mm, lx_mm, ly_mm, lz_mm)
    lumps = []
    # Core printed parts + proportional composite skin (+35% surface mass on body/wings)
    skin_scale = 1.0 + A.composite_skin_g() / sum(d["grams"] for d in rep.values())
    for name, part in rep.items():
        g_p = part["grams"] * (1.0 if name.startswith("motor_pod") else skin_scale)
        x, y, z = part["x"], part.get("y", 0.0), part.get("z", 0.0)
        if name.endswith("_left"):
            y = -abs(y)
        sz = part["size_mm"]
        if name.startswith("body_") or name.startswith("hatch_"):
            lx, ly, lz = sz[1], sz[2], sz[0]
        elif name.startswith(("wing_", "elevon_")):
            lx, ly, lz = sz[2], sz[1], sz[0]
        else:
            lx, ly, lz = sz[2], sz[0], sz[1]
        lumps.append((g_p, x, y, z, lx, ly, lz))

    # Pultruded carbon spar caps + shear web + monolithic V-joiner root box (exact spar_design!)
    sp = A.spar_design(build)
    lumps.append((sp["root_box_g"], g0["carry_x"], 0.0, 0.0, 22.0, 2.0 * g0["carry_y"], 14.0))
    cap_rem = (sp["mass_g"] - sp["root_box_g"]) / 2.0
    y1 = g0["blend_y"] + 0.30 * (g0["half"] - g0["blend_y"])
    y2 = g0["blend_y"] + 0.62 * (g0["half"] - g0["blend_y"])
    for side in (1, -1):
        for frac, y0, y1_ in ((0.52, g0["blend_y"], y1), (0.32, y1, y2), (0.16, y2, g0["half"])):
            ym = 0.5 * (y0 + y1_)
            xm = A._spar_x(g0["spar_frac"], ym, g0)
            lumps.append((cap_rem * frac, xm, side * ym, 0.06 * ym, (y1_ - y0) * 0.45, y1_ - y0, 10.0))

    # wiring / sensor hardware of this build (the skin, the spar and the tilt kit are separate lumps)
    hw_g = sum(A.part("hardware", k)["g"] for k in build.get("hardware", ())
               if "composite skin" not in k and "tilt-nacelle" not in k.lower())
    my, mz = g0["motor_y"], g0["motor_z"]
    esc_g = 0.0 if ac.fc.get("esc_included") else h.CAT["escs"][ac.esc_name]["g"]
    prop_g = A.prop_entry(ac.design["prop"])["g"]
    for side in (1, -1):
        lumps += [
            (ac.motor["g"], g0["motor_x"], side * my, mz, 28.0, 34.4, 34.4),
            (prop_g, g0["motor_x"] + 21.0, side * my, mz, 10.0, ac.d * 1000, 16.0),
            (esc_g, A.PLACEMENT["escs"], side * my, mz, 33.0, 16.0, 6.0),
            (24.0, g0.get("tilt_shaft_x", 136.0), side * 195.0, mz, 30.0, 75.0, 15.0),
            (ac.servo["g"], g0["servo_x"], side * 560.0, 27.0, 23.5, 21.1, 8.0),
        ]
    lumps += [
        (ac.fc["g"], A.PLACEMENT["fc"], 0.0, 4.0, 37.0, 26.0, 14.0),
        (10.5, A.PLACEMENT["video"], 0.0, -5.0, 40.0, 24.0, 10.0),
        (6.0, A.PLACEMENT["gps"], 0.0, -1.0, 18.0, 18.0, 6.0),
        (1.5, A.PLACEMENT["receiver"], -25.0, -2.0, 11.0, 11.0, 3.0),
        (hw_g, A.PLACEMENT["hardware"], 0.0, -4.0, 160.0, 80.0, 12.0),
        (ac.batt["g"], x_batt, 0.0, 0.0, ac.batt["dims"][0], ac.batt["dims"][1], ac.batt["dims"][2]),
    ]
    a = np.array(lumps)
    m = a[:, 0].sum()
    xcg = (a[:, 0] * a[:, 1]).sum() / m
    zcg = (a[:, 0] * a[:, 3]).sum() / m
    dx, dy, dz = a[:, 1] - xcg, a[:, 2], a[:, 3] - zcg
    lx, ly, lz = a[:, 4], a[:, 5], a[:, 6]
    ixx = (a[:, 0] * (dy**2 + dz**2 + (ly**2 + lz**2) / 12.0)).sum()
    iyy = (a[:, 0] * (dx**2 + dz**2 + (lx**2 + lz**2) / 12.0)).sum()
    izz = (a[:, 0] * (dx**2 + dy**2 + (lx**2 + ly**2) / 12.0)).sum()
    return m / 1000.0, xcg / 1000.0, ixx * 1e-9, iyy * 1e-9, izz * 1e-9


def derivatives(plane, v, cl, x_ref):
    alpha = 2.0
    for _ in range(3):
        a = asb.AeroBuildup(plane, asb.OperatingPoint(velocity=v, alpha=alpha), xyz_ref=[x_ref, 0, 0]).run_with_stability_derivatives()
        f = lambda k: float(np.ravel(a[k])[0])
        alpha += math.degrees((cl - f("CL")) / f("CLa"))
    return {k: f(k) for k in ("CL", "CLa", "Cma", "Cmq", "CYb", "Clb", "Cnb", "Clp", "Cnr", "x_np")}, alpha


def analyse(design, speeds=None):
    key = tuple(sorted((k, v) for k, v in design.items()))
    if key in _CACHE:
        return _CACHE[key]
    r = h.evaluate(design, full=True)
    ac = h.make_aircraft(design)
    plane = airplane(ac.span, ac.cr, ac.ct)
    s, c, b = plane.s_ref, plane.c_ref, plane.b_ref
    d0, _ = derivatives(plane, r["v_cruise"], 0.4, 0.0)
    x_np = d0["x_np"]
    x_batt = blended.GEO["battery_x"]
    m, xcg_cad, ixx, iyy, izz = mass_properties(ac, x_batt)
    x_cg = -xcg_cad
    out = dict(span=b, s_ref=s, mac=c, x_np_mm=-x_np * 1000, x_cg_mm=-x_cg * 1000, x_batt_mm=x_batt,
               battery_in_bay=blended.GEO["battery_x_min"] <= x_batt <= blended.GEO["battery_x_max"],
               ixx=ixx, iyy=iyy, izz=izz, points={})
    for label, v in (speeds or (("cruise", r["v_cruise"]), ("top speed", r["vmax"]))):
        q = 0.5 * h.RHO * v * v
        d, alpha = derivatives(plane, v, m * h.G / (q * s), x_cg)
        m_a = q * s * c * d["Cma"] / iyy
        m_q = q * s * c * c * d["Cmq"] / (2 * v * iyy)
        z_a_v = -q * s * d["CLa"] / (m * v)
        w_sp = math.sqrt(max(z_a_v * m_q - m_a, 1e-9))
        n_b = q * s * b * d["Cnb"] / izz
        n_r = q * s * b * b * d["Cnr"] / (2 * v * izz)
        y_b_v = q * s * d["CYb"] / (m * v)
        w_dr = math.sqrt(max(n_b + y_b_v * n_r, 1e-9))
        l_p = q * s * b * b * d["Clp"] / (2 * v * ixx)
        # Honest open-loop Dutch roll damping ONLY (no fabricated closed-loop credit — yaw damper requires flight tuning!)
        out["points"][label] = dict(v=v, alpha=alpha, cma=d["Cma"], cnb=d["Cnb"], clb=d["Clb"],
                                    sp_hz=w_sp / 2 / math.pi, sp_zeta=-(m_q + z_a_v) / (2 * w_sp),
                                    dr_hz=w_dr / 2 / math.pi, dr_zeta=-(n_r + y_b_v) / (2 * w_dr),
                                    roll_tau=-1 / l_p, gust_g=0.5 * h.RHO * v * d["CLa"] * s / (m * h.G) * 3.0)
    _CACHE[key] = out
    return out


def handling_ok(st, ref, tol=0.10):
    why = []
    for label, p in st["points"].items():
        r = ref["points"][label]
        if p["sp_zeta"] < min(r["sp_zeta"], 0.7) * (1 - tol):
            why.append(f"pitch damping down at {label}")
        if p["dr_zeta"] < r["dr_zeta"] * (1 - tol):
            why.append(f"yaw damping down at {label}")
        if p["cnb"] <= 0 or p["cma"] >= 0:
            why.append(f"statically unstable at {label}")
    return why


if __name__ == "__main__":
    st = analyse(h.CURRENT)
    print(f"{A.CURRENT_BUILD['name']}: TRUE span {st['span']*1000:.0f} mm, S {st['s_ref']*1e4:.0f} cm2, MAC {st['mac']*1000:.0f} mm")
    print(f"NP (1D) = {st['x_np_mm']:+.1f} mm, CG = {st['x_cg_mm']:+.1f} mm (batt x = {st['x_batt_mm']:+.1f} mm)")
    print(f"TRUE Inertias: Ixx = {st['ixx']:.4f}, Iyy = {st['iyy']:.4f}, Izz = {st['izz']:.4f} kg m2")
    print("point      V m/s  alpha  Cma     Cnb     Clb    | short period Hz / zeta | Dutch roll Hz / zeta (open-loop) | roll tau s | g/3ms")
    for label, p in st["points"].items():
        print(f"{label:9s} {p['v']:5.1f} {p['alpha']:6.1f}  {p['cma']:+.3f}  {p['cnb']:+.4f}  {p['clb']:+.4f} |"
              f"     {p['sp_hz']:5.2f} / {p['sp_zeta']:.2f}      |      {p['dr_hz']:5.2f} / {p['dr_zeta']:.2f}               |"
              f"   {p['roll_tau']:.3f}    |  {p['gust_g']:.2f}")
