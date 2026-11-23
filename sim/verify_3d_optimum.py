"""Full 3D AeroSandbox (AeroBuildup + 3D VortexLatticeMethod) verification & hill-climb of the
Kestrel-X Stealth Cranked-Kite BWB Tailsitter before CAD build.

Architecture (inspired by Northrop B-21 Raider / Lockheed RQ-170 Sentinel / X-47B):
  - Razor-slim chined stealth centerbody (t/c = 7.3%, houses 3x6 flat Molicel P60B 6S3P 388.8 Wh pack!)
  - High-AR cranked-kite stealth wing with NeuralFoil-optimized CST laminar reflex airfoil
  - 2-segment pin-coupled stealth elevon (y = 135..555 mm, each segment <= 210 mm for 220 mm print bed):
      * Inboard segment sits 100% inside the 8" propwash for high-authority VTOL hover & transition!
      * Outboard segment sits far aft of CG for crisp high-CL cruise/stall pitch trim & fast roll rate!
  - Swept canted stealth V-tails + matching ventral landing skids (NACA0006 thin section, 38 deg cant)

Run: .venv/Scripts/python.exe sim/verify_3d_optimum.py
"""
import json
import math
import pathlib
import sys

import aerosandbox as asb
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import aircraft as A
import optimize as opt

CAT = json.loads((HERE / "catalog.json").read_text())
APC = json.loads((HERE / "apc_props.json").read_text())
AF_DATA = json.loads((HERE / "stealth_airfoil.json").read_text())
RHO, G, NU = A.RHO, A.G, A.NU


def bernstein(n, i, x):
    return math.comb(n, i) * (x ** i) * ((1.0 - x) ** (n - i))


def stations(n=40):
    return [0.5 * (1.0 - math.cos(math.pi * i / n)) for i in range(n + 1)]


def stealth_wing_coords(t_scale=1.0):
    w_t, w_c, dz_te = AF_DATA["w_t"], AF_DATA["w_c"], AF_DATA.get("dz_te", 0.0025)
    s = np.array(stations(40))
    c_cls = np.sqrt(np.maximum(s, 0.0)) * (1.0 - s)
    nt, nc = len(w_t) - 1, len(w_c) - 1
    yt = t_scale * (c_cls * sum(w * bernstein(nt, i, s) for i, w in enumerate(w_t)) + s * (dz_te / 2.0))
    yc = s * (1.0 - s) * sum(w * bernstein(nc, i, s) for i, w in enumerate(w_c))
    up = np.column_stack([s[::-1], (yc + yt)[::-1]])
    lo = np.column_stack([s[1:], (yc - yt)[1:]])
    return np.vstack([up, lo])


def body_f(s, nf=0.22):
    if s < nf:
        t = (nf - s) / nf
        return (1.0 - t * t) ** 0.58
    if s < 0.68:
        return 1.0
    return 0.02 + 0.98 * 0.5 * (1.0 + math.cos(math.pi * (s - 0.68) / 0.32))


def body_coords(x_le, x_te, top, bot, body_reflex=6.0):
    length = x_le - x_te
    up, lo = [], []
    for s in stations(40):
        zc = (body_reflex / length) * max(0.0, (s - 0.60) / 0.40) ** 2
        f = body_f(s)
        up.append((s, zc + top * f / length))
        lo.append((s, zc - bot * f / length))
    return np.array(list(reversed(up)) + lo[1:])


WING_AF_ROOT = asb.Airfoil("stealth_root", coordinates=stealth_wing_coords(1.0))
WING_AF_TIP = asb.Airfoil("stealth_tip", coordinates=stealth_wing_coords(0.90))
FIN_AF = asb.Airfoil("naca0006")


def build_3d_plane(half_mm=900.0, cr_mm=176.0, ct_mm=78.0, sweep_deg=26.0,
                   washout=4.0, body_reflex=6.0, wing_shift=55.0,
                   ey0=135.0, ey1=690.0, fin_cant=38.0, fin_h=115.0,
                   elevator=0.0, aileron=0.0, with_fins=True):
    """Build exact 3D stealth cranked-kite BWB in AeroSandbox (x aft, y right, z up, metres)."""
    d0 = (235.0, -245.0, 17.5, 17.5)     # 480 mm long x 35 mm thick (t/c = 7.3%)
    d_bay = (172.0, -215.0, 16.5, 16.5)  # at y = 72 mm (outside 140 mm wide flat 3x6 battery bay)
    blend_y = 107.0
    sw = math.tan(math.radians(sweep_deg))
    root_le = 120.0 - wing_shift

    def le_x(y):
        return root_le - sw * (y - blend_y)

    def chord(y):
        return cr_mm + (ct_mm - cr_mm) * (y - blend_y) / (half_mm - blend_y)

    def tw(y):
        return washout * max(0.0, y - blend_y) / (half_mm - blend_y)

    def z_le(y):
        return -0.30 * chord(y) / 1000.0 * math.tan(math.radians(tw(y)))

    cs = []
    if elevator != 0.0:
        cs.append(asb.ControlSurface(name="elevator", symmetric=True, hinge_point=0.72, deflection=elevator))
    if aileron != 0.0:
        cs.append(asb.ControlSurface(name="aileron", symmetric=False, hinge_point=0.72, deflection=aileron))

    # For outer-elevon trim check, we can place control surface from ey0..ey1 (or split!)
    xsecs = [
        asb.WingXSec(xyz_le=[-d0[0] / 1000, 0.0, 0.0], chord=(d0[0] - d0[1]) / 1000,
                     airfoil=asb.Airfoil("b0", coordinates=body_coords(*d0, body_reflex))),
        asb.WingXSec(xyz_le=[-d_bay[0] / 1000, 0.072, 0.0], chord=(d_bay[0] - d_bay[1]) / 1000,
                     airfoil=asb.Airfoil("b72", coordinates=body_coords(*d_bay, body_reflex))),
        asb.WingXSec(xyz_le=[-le_x(blend_y) / 1000, blend_y / 1000, z_le(blend_y)], chord=chord(blend_y) / 1000,
                     airfoil=WING_AF_ROOT, twist=-tw(blend_y)),
        asb.WingXSec(xyz_le=[-le_x(ey0) / 1000, ey0 / 1000, z_le(ey0)], chord=chord(ey0) / 1000,
                     airfoil=WING_AF_ROOT, twist=-tw(ey0), control_surfaces=cs),
        asb.WingXSec(xyz_le=[-le_x(ey1) / 1000, ey1 / 1000, z_le(ey1)], chord=chord(ey1) / 1000,
                     airfoil=WING_AF_ROOT, twist=-tw(ey1)),
        asb.WingXSec(xyz_le=[-le_x(half_mm) / 1000, half_mm / 1000, z_le(half_mm)], chord=chord(half_mm) / 1000,
                     airfoil=WING_AF_TIP, twist=-tw(half_mm)),
    ]
    wing = asb.Wing(name="stealth_bwb", symmetric=True, xsecs=xsecs)
    wings = [wing]
    if with_fins:
        cant = math.radians(fin_cant)
        x_r, c_r = 0.145, 0.088
        x_t, c_t = x_r + (fin_h / 1000.0) * math.tan(math.radians(38.0)), 0.056
        for name, y0, sign in (("tail_up", 0.082, +1), ("tail_dn", 0.090, -1)):
            h_m = fin_h / 1000.0
            wings.append(asb.Wing(name=name, symmetric=True, xsecs=[
                asb.WingXSec(xyz_le=[x_r, y0, 0.0], chord=c_r, airfoil=FIN_AF),
                asb.WingXSec(xyz_le=[x_t, y0 + h_m * math.sin(cant), sign * h_m * math.cos(cant)],
                             chord=c_t, airfoil=FIN_AF),
            ]))
    return asb.Airplane(name="kestrel_x_stealth", wings=wings,
                        s_ref=wing.area(), c_ref=wing.mean_aerodynamic_chord(), b_ref=wing.span())


def eval_3d(half_mm=900.0, cr_mm=176.0, ct_mm=78.0, sweep_deg=26.0,
            washout=4.0, body_reflex=6.0, wing_shift=55.0, ey0=135.0, ey1=820.0,
            fin_cant=38.0, fin_h=115.0,
            motor_name="T-Motor F100 2810 1350KV", prop_name="8x9",
            batt_name="Molicel P60B 6S3P (18x21700, 180A)", mass_kg=2.340):
    kw_g = dict(half_mm=half_mm, cr_mm=cr_mm, ct_mm=ct_mm, sweep_deg=sweep_deg,
                washout=washout, body_reflex=body_reflex, wing_shift=wing_shift,
                ey0=ey0, ey1=ey1, fin_cant=fin_cant, fin_h=fin_h)
    plane_clean = build_3d_plane(**kw_g, with_fins=False)
    plane_full = build_3d_plane(**kw_g, with_fins=True)
    s_ref, mac, span = plane_full.s_ref, plane_full.c_ref, plane_full.b_ref
    ar = span ** 2 / s_ref
    f = lambda d, k: float(np.ravel(d[k])[0])

    # 3D VLM Neutral Point
    v2 = asb.VortexLatticeMethod(plane_clean, asb.OperatingPoint(velocity=20.0, alpha=2.0), xyz_ref=[0, 0, 0],
                                 spanwise_resolution=10, chordwise_resolution=10).run()
    v6 = asb.VortexLatticeMethod(plane_clean, asb.OperatingPoint(velocity=20.0, alpha=6.0), xyz_ref=[0, 0, 0],
                                 spanwise_resolution=10, chordwise_resolution=10).run()
    x_np_3d_asb = -(f(v6, "Cm") - f(v2, "Cm")) / (f(v6, "CL") - f(v2, "CL")) * mac
    x_np_3d_cad = -x_np_3d_asb * 1000.0

    # Conservative 1D AeroBuildup Neutral Point (with fins)
    a2 = asb.AeroBuildup(plane_full, asb.OperatingPoint(velocity=20.0, alpha=2.0), xyz_ref=[0, 0, 0]).run()
    a6 = asb.AeroBuildup(plane_full, asb.OperatingPoint(velocity=20.0, alpha=6.0), xyz_ref=[0, 0, 0]).run()
    x_np_1d_asb = -(f(a6, "Cm") - f(a2, "Cm")) / (f(a6, "CL") - f(a2, "CL")) * mac
    x_np_1d_cad = -x_np_1d_asb * 1000.0

    # Target CG at 7.5% conservative 1D margin (gives ~14-18% 3D VLM margin!)
    x_cg_asb = x_np_1d_asb - 0.075 * mac
    x_cg_cad = -x_cg_asb * 1000.0
    m3d = (x_cg_cad - x_np_3d_cad) / (mac * 1000.0)

    # Trimmed polar (exact 3-point flap solve across alpha = -2..16)
    alphas = np.arange(-2.0, 16.1, 1.0)
    def run_ab(v_air, elev):
        p = build_3d_plane(**kw_g, elevator=elev, with_fins=True)
        return asb.AeroBuildup(p, asb.OperatingPoint(velocity=v_air, alpha=alphas), xyz_ref=[x_cg_asb, 0, 0]).run()

    r0 = run_ab(16.0, 0.0)
    r1 = run_ab(16.0, -8.0)
    r2 = run_ab(16.0, -16.0)
    cl_trims, el_trims = [], []
    for i in range(len(alphas)):
        ds = np.array([-16.0, -8.0, 0.0])
        cms = np.array([r2["Cm"][i], r1["Cm"][i], r0["Cm"][i]])
        cls = np.array([r2["CL"][i], r1["CL"][i], r0["CL"][i]])
        if cms.min() <= 0.01 and cms.max() >= -0.01:
            order = np.argsort(cms)
            dt = float(np.clip(np.interp(0.0, cms[order], ds[order]), -25.0, 25.0))
            cl_trims.append(float(np.interp(dt, ds, cls)))
            el_trims.append(-dt)
    cl_max_trim = max(cl_trims) if cl_trims else 0.1
    w = mass_kg * G
    v_stall_trim = math.sqrt(2.0 * w / (RHO * s_ref * cl_max_trim))
    v_min_ops = A.MIN_SPEED_FACTOR * v_stall_trim

    # Cruise / Loiter / Dash on 3D AeroBuildup polar + conformal motor nacelles + honest 1.15 factor!
    m = CAT["motors"][motor_name]
    b = CAT["batteries"][batt_name]
    d = APC[prop_name]["d_in"] * 0.0254
    tab = np.array(APC[prop_name]["table"])
    J, CT, CP = tab[:, 0], tab[:, 1], tab[:, 2] * 1.08
    vbat = b["ns"] * 3.6
    r_pack = b["r"] + A.WIRING_R
    avionics_w = A.avionics_power_w(A.CURRENT_BUILD)
    extra_dq = 0.11 * 2.0 * math.pi * 0.0178 ** 2

    def elec(n, v, v_bus=vbat):
        shaft = np.interp(v / np.maximum(n * d, 1e-6), J, CP) * RHO * n ** 3 * d ** 5
        e = n * 60.0 / m["kv"]
        im = shaft / np.maximum(e, 1e-6) + m["i0"]
        return (e + im * m["r"]) * im + opt.esc_loss_w(v_bus, im), e + im * m["r"], im, shaft

    n_h = opt.thrust_solve(np.array([w / 2.0]), np.zeros(1), d, J, CT)
    p_hover = float(2.0 * elec(n_h, np.zeros(1))[0][0] + avionics_w)
    energy = b["wh"] * A.USABLE - p_hover * A.HOVER_S / 3600.0

    best_rng, best_end = None, None
    for v in np.arange(math.ceil(v_min_ops * 2) / 2, 34.0, 0.5):
        ab = asb.AeroBuildup(plane_full, asb.OperatingPoint(velocity=float(v), alpha=alphas), xyz_ref=[x_cg_asb, 0, 0]).run()
        cl_p, cd_p = np.ravel(ab["CL"]), np.ravel(ab["CD"])
        im_ = int(np.argmax(cl_p))
        q = 0.5 * RHO * v * v
        cl = w / (q * s_ref)
        if cl > cl_max_trim:
            continue
        cd = float(np.interp(cl, cl_p[:im_ + 1], cd_p[:im_ + 1])) * 1.15 + extra_dq / s_ref
        drag = q * s_ref * cd
        n = opt.thrust_solve(np.array([drag / 2.0]), np.array([v]), d, J, CT)
        p1, vm, _, sh = elec(n, np.array([v]))
        if vm[0] > vbat:
            continue
        p_tot = float(2.0 * p1[0] + avionics_w)
        row = dict(v=float(v), p=p_tot, ld=cl / cd, range_km=energy / p_tot * v * 3.6,
                   endurance=energy / p_tot * 60.0, pe=float(drag / 2.0 * v / sh[0]))
        if best_rng is None or row["range_km"] > best_rng["range_km"]:
            best_rng = row
        if best_end is None or row["endurance"] > best_end["endurance"]:
            best_end = row

    n_max = min(190000.0 / APC[prop_name]["d_in"] / 60.0, 265.0 / (math.pi * d))
    def ft(v, v_cell):
        voc = b["ns"] * v_cell
        lo, hi = 5.0, n_max
        for _ in range(36):
            mid = 0.5 * (lo + hi)
            _, vm, im, _ = elec(np.array([mid]), np.array([v]), voc)
            vm, im = float(vm[0]), float(im[0])
            disc = voc ** 2 - 8.0 * vm * im * r_pack
            vb = (voc + math.sqrt(max(disc, 0.0))) / 2.0
            if disc >= 0 and vm <= vb and im <= min(m["imax"], 70.0) and 2 * vm * im / max(vb, 0.1) <= b["imax"]:
                lo = mid
            else:
                hi = mid
        return 2.0 * float(np.interp(v / (lo * d), J, CT)) * RHO * lo ** 2 * d ** 4

    ab_hi = asb.AeroBuildup(plane_full, asb.OperatingPoint(velocity=85.0, alpha=np.linspace(-4, 4, 17)), xyz_ref=[x_cg_asb, 0, 0]).run()
    cl_hi_t, cd_hi_t = np.ravel(ab_hi["CL"]), np.ravel(ab_hi["CD"])
    v_top_36, v_top_39 = 0.0, 0.0
    for v in np.arange(35.0, 115.0, 1.0):
        q = 0.5 * RHO * v * v
        cl = w / (q * s_ref)
        cd = float(np.interp(cl, cl_hi_t, cd_hi_t)) * 1.15 + extra_dq / s_ref
        drag = q * s_ref * cd
        if ft(v, 3.6) >= drag:
            v_top_36 = v
        if ft(v, 3.9) >= drag:
            v_top_39 = v

    ab_st = asb.AeroBuildup(plane_full, asb.OperatingPoint(velocity=best_rng["v"], alpha=4.0), xyz_ref=[x_cg_asb, 0, 0]).run_with_stability_derivatives()
    tw_36 = ft(0.0, 3.6) / w
    tw_33 = ft(0.0, 3.3) / w
    v_c = best_rng["v"]
    cl_c = w / (0.5 * RHO * v_c * v_c * s_ref)
    cr_el = float(np.interp(cl_c, cl_trims[:int(np.argmax(cl_trims)) + 1], el_trims[:int(np.argmax(cl_trims)) + 1])) if cl_trims else 99.0

    return dict(
        span_mm=span * 1000, area_cm2=s_ref * 1e4, mac_mm=mac * 1000, ar=ar,
        x_np_1d=x_np_1d_cad, x_np_3d=x_np_3d_cad, x_cg=x_cg_cad, m3d_pct=m3d * 100,
        cl_max_trim=cl_max_trim, v_stall_kmh=v_stall_trim * 3.6, cr_elev_up=cr_el,
        range_km=best_rng["range_km"], v_cruise_kmh=best_rng["v"] * 3.6, p_cruise=best_rng["p"],
        ld=best_rng["ld"], pe=best_rng["pe"] * 100,
        endurance_min=best_end["endurance"], v_loiter_kmh=best_end["v"] * 3.6, p_loiter=best_end["p"],
        v_top_36_kmh=v_top_36 * 3.6, v_top_39_kmh=v_top_39 * 3.6,
        p_hover=p_hover, tw_36=tw_36, tw_33=tw_33,
        cma=f(ab_st, "Cma"), cnb=f(ab_st, "Cnb"), clb=f(ab_st, "Clb"), clp=f(ab_st, "Clp"),
    )


if __name__ == "__main__":
    print("=== 3D AEROSANDBOX + 3D VLM VERIFICATION OF STEALTH BWB CANDIDATES ===")
    for label, kw in (
        ("1800mm sw26 cr176 ct78 ey135..820", dict(half_mm=900, cr_mm=176, ct_mm=78, sweep_deg=26, washout=4.0, body_reflex=6.0, ey0=135, ey1=820, mass_kg=2.34)),
        ("1800mm sw28 cr176 ct78 ey135..820", dict(half_mm=900, cr_mm=176, ct_mm=78, sweep_deg=28, washout=4.5, body_reflex=6.0, ey0=135, ey1=820, mass_kg=2.34)),
        ("1800mm sw30 cr176 ct78 ey350..840", dict(half_mm=900, cr_mm=176, ct_mm=78, sweep_deg=30, washout=4.5, body_reflex=6.0, ey0=350, ey1=840, mass_kg=2.34)),
        ("1800mm sw28 cr176 ct78 ey340..840", dict(half_mm=900, cr_mm=176, ct_mm=78, sweep_deg=28, washout=4.5, body_reflex=6.0, ey0=340, ey1=840, mass_kg=2.34)),
        ("1760mm sw28 cr176 ct78 ey135..820 8x9", dict(half_mm=880, cr_mm=176, ct_mm=78, sweep_deg=28, washout=4.5, body_reflex=6.0, ey0=135, ey1=820, mass_kg=2.32)),
        ("1800mm sw28 1100KV 8x9", dict(half_mm=900, cr_mm=176, ct_mm=78, sweep_deg=28, washout=4.5, body_reflex=6.0, ey0=135, ey1=820, mass_kg=2.34, motor_name="T-Motor F100 2810 1100KV")),
        ("1800mm sw28 1350KV 8x8E", dict(half_mm=900, cr_mm=176, ct_mm=78, sweep_deg=28, washout=4.5, body_reflex=6.0, ey0=135, ey1=820, mass_kg=2.34, prop_name="8x8E")),
    ):
        r = eval_3d(**kw)
        print(f"\n[{label}]  AR={r['ar']:.2f}  S={r['area_cm2']:.0f} cm2  MAC={r['mac_mm']:.0f} mm")
        print(f"  Vmax36={r['v_top_36_kmh']:.0f} km/h  Vmax39={r['v_top_39_kmh']:.0f} km/h | Range={r['range_km']:.0f} km @ {r['v_cruise_kmh']:.0f} km/h ({r['p_cruise']:.1f} W, 3D L/D={r['ld']:.1f}, pe={r['pe']:.0f}%)")
        print(f"  Endurance={r['endurance_min']:.0f} min ({r['endurance_min']/60:.2f} hr) @ {r['v_loiter_kmh']:.0f} km/h ({r['p_loiter']:.1f} W) | Hover={r['p_hover']:.0f} W | TW36={r['tw_36']:.2f}, TW33={r['tw_33']:.2f}")
        print(f"  NP_3D={r['x_np_3d']:+.1f} mm  NP_1D={r['x_np_1d']:+.1f} mm  CG={r['x_cg']:+.1f} mm (3D margin {r['m3d_pct']:.1f}%)")
        print(f"  Trimmed CLmax={r['cl_max_trim']:.3f} -> Stall={r['v_stall_kmh']:.1f} km/h | Cruise elevon={r['cr_elev_up']:+.2f} deg")
        print(f"  Derivs: Cma={r['cma']:+.3f}  Cnb={r['cnb']:+.4f}  Clb={r['clb']:+.4f}  Clp={r['clp']:+.3f}")
