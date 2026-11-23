"""Aerodynamics, balance and performance of Kestrel-X Stealth v7 (composite 8S vectored optimum).

All Claude Opus 5.5 audit corrections enforced:
  - TRUE projected planform span b_ref = 2 * HALF (ZERO sharklet span inflation in AR!)
  - Realistic composite finish n_crit = 7.0 in AeroBuildup/NeuralFoil polars
  - Speed-scaled iron loss i0(rpm) & modern 80A BLHeli32/AM32 ESC loss model
  - Exact APC Rev 5 RPM limit (150,000/D E props, 225,000/D Sport on electric)
"""
import math
import pathlib

import aerosandbox as asb
import numpy as np

import aircraft as A
import optimize as opt

ROOT = pathlib.Path(__file__).resolve().parent.parent
REPORT = A.REPORT
GEO = REPORT["geometry"]
STATIC_MARGIN = 0.08


def _bernstein(n, i, s):
    return math.comb(n, i) * (s ** i) * ((1.0 - s) ** (n - i))


def stations(n=40):
    return [0.5 * (1 - math.cos(math.pi * i / n)) for i in range(n + 1)]


def wing_coords(t_scale=1.0):
    wt, wc, dz = GEO["cst_wt"], GEO["cst_wc"], GEO.get("cst_dz_te", 0.0025)
    nt, nc = len(wt) - 1, len(wc) - 1
    up, lo = [], []
    for s in stations():
        c_cls = math.sqrt(max(s, 0.0)) * (1.0 - s)
        yt = t_scale * (c_cls * sum(w * _bernstein(nt, i, s) for i, w in enumerate(wt)) + s * dz / 2.0)
        yc = s * (1.0 - s) * sum(w * _bernstein(nc, i, s) for i, w in enumerate(wc))
        up.append((s, yc + yt))
        lo.append((s, yc - yt))
    return np.array(list(reversed(up)) + lo[1:])


def body_f(s):
    nf = GEO.get("nose_frac", 0.19)
    fe = GEO.get("flat_end", 0.84)
    if s < nf:
        t = (nf - s) / nf
        return (1 - t * t) ** 0.58
    if s < fe:
        return 1.0
    return 0.02 + 0.98 * 0.5 * (1 + math.cos(math.pi * (s - fe) / (1.0 - fe)))


def body_camber(s, length):
    bref = GEO.get("body_reflex", 5.8)
    return (bref / length) * max(0.0, (s - 0.62) / 0.38) ** 2


def body_coords(dims):
    x_le, x_te, top, bot = dims
    length = x_le - x_te
    up = [(s, body_camber(s, length) + top * body_f(s) / length) for s in stations()]
    lo = [(s, body_camber(s, length) - bot * body_f(s) / length) for s in stations()]
    return np.array(list(reversed(up)) + lo[1:])


def le_x(y):
    return GEO["root_le"] - math.tan(math.radians(GEO["sweep_deg"])) * (abs(y) - GEO["blend_y"])


def chord(y):
    t = max(0.0, (abs(y) - GEO["blend_y"]) / (GEO["half"] - GEO["blend_y"]))
    return GEO["root_chord"] + (GEO["tip_chord"] - GEO["root_chord"]) * t


def twist_deg(y):
    return GEO.get("washout", 4.6) * max(0.0, abs(y) - GEO["blend_y"]) / (GEO["half"] - GEO["blend_y"])


def z_le_m(y):
    dih = math.tan(math.radians(GEO.get("dihedral_deg", 3.5))) * max(0.0, abs(y) - GEO["blend_y"]) / 1000.0
    tw = -GEO["spar_frac"] * chord(y) / 1000.0 * math.tan(math.radians(twist_deg(y)))
    return dih + tw


def esc_loss_modern(vbat, im, esc_loss=1.0):
    """Physical loss for modern 80A BLHeli32/AM32 ESCs (Rds_on ~ 2.5 mOhm, 2% switching)."""
    return esc_loss * (0.40 * vbat / 22.0 + 0.0040 * im**2 + 0.020 * vbat * im)


def airplane(elevator=0.0, aileron=0.0):
    """AeroSandbox model: TRUE projected planform span b_ref = 2*HALF (ZERO sharklet span inflation!)."""
    wing_af = asb.Airfoil("kestrel_wing", coordinates=wing_coords(1.0))
    tip_af = asb.Airfoil("kestrel_wing_tip", coordinates=wing_coords(GEO.get("tip_t_scale", 0.92)))
    d0, d_bay = GEO["d0"], GEO["d_bay"]
    y_body = GEO.get("d_bay_y", 68.0) / 1000.0
    by, half = GEO["blend_y"], GEO["half"]
    ey0, ey1 = GEO["elevon_y"]
    hinge = GEO.get("hinge", 0.70)
    cs = []
    if elevator != 0.0:
        cs.append(asb.ControlSurface(name="elevator", symmetric=True, hinge_point=hinge, deflection=elevator))
    if aileron != 0.0:
        cs.append(asb.ControlSurface(name="aileron", symmetric=False, hinge_point=hinge, deflection=aileron))
    xsecs = [
        asb.WingXSec(xyz_le=[-d0[0] / 1000, 0, 0], chord=(d0[0] - d0[1]) / 1000, airfoil=asb.Airfoil("body0", coordinates=body_coords(d0))),
        asb.WingXSec(xyz_le=[-d_bay[0] / 1000, y_body, 0], chord=(d_bay[0] - d_bay[1]) / 1000, airfoil=asb.Airfoil("body_bay", coordinates=body_coords(d_bay))),
        asb.WingXSec(xyz_le=[-le_x(by) / 1000, by / 1000, z_le_m(by)], chord=chord(by) / 1000, airfoil=wing_af, twist=-twist_deg(by)),
        asb.WingXSec(xyz_le=[-le_x(ey0) / 1000, ey0 / 1000, z_le_m(ey0)], chord=chord(ey0) / 1000, airfoil=wing_af, twist=-twist_deg(ey0), control_surfaces=cs),
        asb.WingXSec(xyz_le=[-le_x(ey1) / 1000, ey1 / 1000, z_le_m(ey1)], chord=chord(ey1) / 1000, airfoil=wing_af, twist=-twist_deg(ey1)),
        asb.WingXSec(xyz_le=[-le_x(half) / 1000, half / 1000, z_le_m(half)], chord=chord(half) / 1000, airfoil=tip_af, twist=-twist_deg(half)),
    ]
    # True planform reference wing (WITHOUT sharklets inflating b_ref or s_ref!)
    plan_wing = asb.Wing(name="planform_ref", symmetric=True, xsecs=xsecs)
    s_ref_true = plan_wing.area()
    mac_true = plan_wing.mean_aerodynamic_chord()
    b_ref_true = 2.0 * half / 1000.0     # EXACT true projected span (2.120 m!)

    if "shark_h" in GEO:
        sh = GEO["shark_h"] / 1000.0
        sc, ss = math.radians(GEO["shark_cant"]), math.radians(GEO["shark_sweep"])
        xsecs_full = xsecs + [asb.WingXSec(
            xyz_le=[(-le_x(half) / 1000.0) + sh * math.tan(ss), half / 1000.0 + sh * math.sin(sc), z_le_m(half) + sh * math.cos(sc)],
            chord=GEO["shark_tip_c"] / 1000.0, airfoil=asb.Airfoil("naca0006"), twist=0.0,
        )]
        wing = asb.Wing(name="blended", symmetric=True, xsecs=xsecs_full)
    else:
        wing = plan_wing
    return asb.Airplane(name="kestrel", wings=[wing], s_ref=s_ref_true, c_ref=mac_true, b_ref=b_ref_true)


_POL_CACHE = {}


def polar(plane, x_ref, v=17.0):
    key = (id(plane), round(float(x_ref), 5), round(float(v), 2))
    if key not in _POL_CACHE:
        alphas = np.arange(-2.0, 14.1, 1.0)
        a = asb.AeroBuildup(plane, asb.OperatingPoint(velocity=float(v), alpha=alphas), xyz_ref=[x_ref, 0, 0]).run()
        _POL_CACHE[key] = np.column_stack([alphas, np.ravel(a["CL"]), np.ravel(a["CD"]), np.ravel(a["Cm"])])
    return _POL_CACHE[key]


def neutral_point(plane, v=17.0):
    out = []
    for x_ref in (0.0, 0.05):
        p = polar(plane, x_ref, v)
        sel = (p[:, 0] >= 1) & (p[:, 0] <= 5)
        out.append(np.polyfit(p[sel, 1], p[sel, 3], 1)[0])
    return 0.0 + (0 - out[0]) * 0.05 / (out[1] - out[0])


def neutral_point_vlm(plane, v=17.0):
    v2 = asb.VortexLatticeMethod(plane, asb.OperatingPoint(velocity=v, alpha=2.0), xyz_ref=[0, 0, 0], spanwise_resolution=10, chordwise_resolution=10).run()
    v6 = asb.VortexLatticeMethod(plane, asb.OperatingPoint(velocity=v, alpha=6.0), xyz_ref=[0, 0, 0], spanwise_resolution=10, chordwise_resolution=10).run()
    f = lambda d, k: float(np.ravel(d[k])[0])
    return -(f(v6, "Cm") - f(v2, "Cm")) / (f(v6, "CL") - f(v2, "CL")) * plane.c_ref


def mass_table(battery_x, build=None):
    return A.mass_breakdown(build or A.CURRENT_BUILD, battery_x)


def performance(plane, mass_kg, build=None, extra_dq=0.0, drag_scale=1.0, esc_loss=1.0):
    build = build or A.CURRENT_BUILD
    m = A.part("motor", build["motor"])
    b = A.part("battery", build["battery"])
    kv, rm, i0 = m["kv"], m["r"], m["i0"]
    d, J, CT, CP = opt.prop_arrays(A.prop_data_key(build["prop"]))
    n_lim = A.prop_rpm_limit(build["prop"])
    s_ref = plane.s_ref
    w = mass_kg * A.G
    vbat = A.pack_volts(build)
    r_pack = b["r"] + A.WIRING_R
    avionics_w = A.avionics_power_w(build)
    i_limit = A.esc_current_limit(build)

    def electrical(n, v):
        shaft = np.interp(v / (n * d), J, CP) * A.RHO * n**3 * d**5
        e = n * 60 / kv
        i0_eff = i0 * (1.0 + 0.35 * np.maximum(e / 10.0 - 1.0, 0.0))
        im = shaft / e + i0_eff
        p_esc = esc_loss_modern(vbat, im, esc_loss)
        return (e + im * rm) * im + p_esc, e + im * rm, im

    n_h = opt.thrust_solve(np.array([w / 2]), np.zeros(1), d, J, CT)
    p_hover = 2 * electrical(n_h, np.zeros(1))[0][0] + avionics_w
    n_m = opt.thrust_solve(np.array([opt.TW_MIN * w / 2]), np.zeros(1), d, J, CT)
    p_m, v_m, i_m = electrical(n_m, np.zeros(1))
    i_pack_m = 2 * p_m[0] / vbat
    energy = b["wh"] * b.get("honesty", 1.0) * A.USABLE - p_hover * A.HOVER_S / 3600
    v_stall_trim = A.stall_speed(mass_kg, s_ref)
    v_min = A.MIN_SPEED_FACTOR * v_stall_trim
    best_rng, best_end = None, None
    cl_max = None
    for v in np.arange(11.0, 34.0, 0.5):
        pol = polar(plane, 0.0, float(v))
        cl_max = float(pol[:, 1].max())
        if v < v_min - 1e-9:
            continue
        q = 0.5 * A.RHO * v * v
        cl = w / (q * s_ref)
        cd = float(np.interp(cl, pol[:, 1], pol[:, 2])) * 1.15 * drag_scale + extra_dq / s_ref * drag_scale
        drag = q * s_ref * cd
        n = opt.thrust_solve(np.array([drag / 2]), np.array([v]), d, J, CT)
        p_one, v_mot, _ = electrical(n, np.array([v]))
        if v_mot[0] > vbat or n[0] > n_lim:
            continue
        p = 2 * p_one[0] + avionics_w
        row = dict(v=float(v), p=float(p), ld=cl / cd, range_km=energy / p * v * 3.6, endurance=energy / p * 60)
        if best_rng is None or row["range_km"] > best_rng["range_km"]:
            best_rng = row
        if best_end is None or row["endurance"] > best_end["endurance"]:
            best_end = row
    v_stall = math.sqrt(2 * w / (A.RHO * s_ref * cl_max))
    return dict(best_rng, endurance_loiter=best_end["endurance"], v_loiter=best_end["v"], p_loiter=best_end["p"],
                p_hover=float(p_hover), i_motor=float(i_m[0]), i_pack=float(i_pack_m), i_limit=i_limit,
                i_cell_limit=b["imax"], v_stall=v_stall, v_stall_trimmed=v_stall_trim, v_min_ops=v_min,
                thrust_ok=bool(i_m[0] <= i_limit and i_pack_m <= b["imax"]
                               and v_m[0] <= vbat - i_pack_m * r_pack))


# Conformal tilt-nacelles (CD=0.12, r=18 mm) + enlarged 22-deg NACA0006 V-tails & ventral skids (S_wet=0.145 m2)
EXTRA_DQ = 0.12 * 2 * math.pi * 0.018**2 + 0.0068 * 1.12 * 0.145


if __name__ == "__main__":
    plane = airplane()
    mac, s_ref = plane.c_ref, plane.s_ref
    x_np_1d = neutral_point(plane)
    x_np_3d = neutral_point_vlm(plane)
    print(f"planform (TRUE projected span!): area {s_ref * 1e4:.0f} cm2, span {plane.b_ref * 1000:.0f} mm, MAC {mac * 1000:.0f} mm, TRUE AR {plane.b_ref**2 / s_ref:.2f}")
    print(f"neutral point (3D VLM): CAD x = {-x_np_3d * 1000:.1f} mm  |  conservative 1D strip: CAD x = {-x_np_1d * 1000:.1f} mm")

    builds = (A.CURRENT_BUILD, A.variant(**A.DUAL_PACK_UPGRADE))
    for build in builds:
        table = mass_table(GEO["battery_x"], build)
        fixed_m = sum(g for k, (g, x) in table.items() if k != "battery")
        fixed_mx = sum(g * x for k, (g, x) in table.items() if k != "battery")
        batt_g = table["battery"][0]
        total = fixed_m + batt_g
        x_cg_now = (fixed_mx + batt_g * GEO["battery_x"]) / total
        m_1d = (x_cg_now / 1000 + x_np_1d) / mac * 100
        m_3d = (x_cg_now / 1000 + x_np_3d) / mac * 100
        print(f"\n=== {build['name']}: {build['battery']} ({total:.0f} g all-up) ===")
        print(f"  battery at x = {GEO['battery_x']:.1f} mm -> CG x = {x_cg_now:.1f} mm | 3D margin {m_3d:.1f} % (1D strip {m_1d:.1f} %)")
        for prop in (build["prop"], "APC 9x9E / 9x9EP CW+CCW pair (LP09090E + LP09090EP)"):
            for label, kw in (("nominal", {}), ("pessimistic", dict(drag_scale=1.25, esc_loss=1.5))):
                r = performance(plane, total / 1000, A.variant(build, prop=prop), EXTRA_DQ, **kw)
                print(f"    {prop[:32]:32s} {label:11s}: range {r['range_km']:4.0f} km @ {r['v'] * 3.6:3.0f} km/h ({r['p']:.0f} W, L/D {r['ld']:.1f}), "
                      f"loiter {r['endurance_loiter']:4.0f} min @ {r['v_loiter'] * 3.6:2.0f} km/h ({r['p_loiter']:.0f} W), "
                      f"hover {r['p_hover']:.0f} W, motor {r['i_motor']:.1f}/{r['i_limit']} A, pack {r['i_pack']:.1f}/{r['i_cell_limit']} A")
