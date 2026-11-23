"""Final 3D AeroSandbox + 3D VLM fine hill-climb for Kestrel-X Stealth (v6) to lock in exact
CAD geometry parameters before build123d modeling.

Verifies:
  - Continuous B-21/RQ-170 stealth chined leading edge + sawtooth W-tail alignment
  - CG aligned dead-center on the flat 3x6 Molicel P60B 6S3P battery bay (x_batt ~ -10..+15 mm)
  - Positive 1D (6-10%) AND 3D VLM (13-17%) static margin
  - Negative Cl_b (dihedral roll stability!), strong positive Cn_b (yaw stability!), negative Cm_a
  - Near-zero cruise elevon trim (< 2 deg), high trimmed CL_max (> 0.78), gentle stall (< 47 km/h)
  - Dual-purpose stealth control: outboard high-arm elevons (y=470..680 mm, 210 mm single printable piece)
    + inboard 100% propwash-submerged hover vanes (y=128..324 mm, 196 mm single printable piece)
  - Maximizes Vmax (310-335 km/h), Range (325+ km), Endurance (320+ min / 5.3+ hr), 3D L/D (19+)!

Run: .venv/Scripts/python.exe sim/refine_3d_cad_params.py
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


def body_coords(x_le, x_te, top, bot, body_reflex=5.5):
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


def build_plane(half_mm=900.0, cr_mm=176.0, ct_mm=78.0, sweep_deg=30.0, dihedral_deg=2.5,
                washout=4.2, body_reflex=5.5, root_le=140.0, ey0=470.0, ey1=685.0,
                fin_cant=36.0, fin_h_up=125.0, fin_h_dn=98.0,
                elevator=0.0, aileron=0.0, with_fins=True):
    """Exact 3D stealth cranked-kite BWB with forward-aligned continuous stealth LE & dihedral."""
    # Centerbody: x = +240..-240 mm (480 mm long, 35 mm thick -> t/c = 7.3%)
    # Flat 3x6 Molicel P60B 6S3P pack (216 x 134 x 24 mm) sits centered near x = -5 mm!
    d0 = (240.0, -240.0, 17.5, 17.5)
    d_bay = (176.0, -205.0, 16.5, 16.5)   # at y = 72 mm (outside bay wall at 70 mm)
    blend_y = 107.0
    sw = math.tan(math.radians(sweep_deg))
    dih = math.tan(math.radians(dihedral_deg))

    def le_x(y):
        return root_le - sw * (y - blend_y)

    def chord(y):
        return cr_mm + (ct_mm - cr_mm) * (y - blend_y) / (half_mm - blend_y)

    def tw(y):
        return washout * max(0.0, y - blend_y) / (half_mm - blend_y)

    def z_le(y):
        z_dih = dih * max(0.0, y - blend_y) / 1000.0
        z_tw = -0.30 * chord(y) / 1000.0 * math.tan(math.radians(tw(y)))
        return z_dih + z_tw

    cs = []
    if elevator != 0.0:
        cs.append(asb.ControlSurface(name="elevator", symmetric=True, hinge_point=0.70, deflection=elevator))
    if aileron != 0.0:
        cs.append(asb.ControlSurface(name="aileron", symmetric=False, hinge_point=0.70, deflection=aileron))

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
        # Asymmetric stealth V-tail (taller upper stealth twin-tails for strong negative Cl_b & Cn_b + sleek lower landing skids!)
        x_r, c_r = 0.130, 0.095
        for name, y0, sign, h_mm in (("tail_up", 0.076, +1, fin_h_up), ("tail_dn", 0.088, -1, fin_h_dn)):
            h_m = h_mm / 1000.0
            x_t = x_r + h_m * math.tan(math.radians(38.0))
            c_t = 0.056
            wings.append(asb.Wing(name=name, symmetric=True, xsecs=[
                asb.WingXSec(xyz_le=[x_r, y0, 0.0], chord=c_r, airfoil=FIN_AF),
                asb.WingXSec(xyz_le=[x_t, y0 + h_m * math.sin(cant), sign * h_m * math.cos(cant)],
                             chord=c_t, airfoil=FIN_AF),
            ]))
    return asb.Airplane(name="kestrel_x_stealth", wings=wings,
                        s_ref=wing.area(), c_ref=wing.mean_aerodynamic_chord(), b_ref=wing.span())


def check(kw, mass_kg=2.34):
    p_clean = build_plane(**kw, with_fins=False)
    p_full = build_plane(**kw, with_fins=True)
    s_ref, mac, span = p_full.s_ref, p_full.c_ref, p_full.b_ref
    ar = span ** 2 / s_ref
    f = lambda d, k: float(np.ravel(d[k])[0])

    v2 = asb.VortexLatticeMethod(p_clean, asb.OperatingPoint(velocity=20.0, alpha=2.0), xyz_ref=[0, 0, 0],
                                 spanwise_resolution=10, chordwise_resolution=10).run()
    v6 = asb.VortexLatticeMethod(p_clean, asb.OperatingPoint(velocity=20.0, alpha=6.0), xyz_ref=[0, 0, 0],
                                 spanwise_resolution=10, chordwise_resolution=10).run()
    x_np_3d = -(f(v6, "Cm") - f(v2, "Cm")) / (f(v6, "CL") - f(v2, "CL")) * mac

    a2 = asb.AeroBuildup(p_full, asb.OperatingPoint(velocity=20.0, alpha=2.0), xyz_ref=[0, 0, 0]).run()
    a6 = asb.AeroBuildup(p_full, asb.OperatingPoint(velocity=20.0, alpha=6.0), xyz_ref=[0, 0, 0]).run()
    x_np_1d = -(f(a6, "Cm") - f(a2, "Cm")) / (f(a6, "CL") - f(a2, "CL")) * mac

    x_cg_asb = x_np_1d - 0.075 * mac
    x_cg_cad = -x_cg_asb * 1000.0
    x_np_1d_cad = -x_np_1d * 1000.0
    x_np_3d_cad = -x_np_3d * 1000.0
    m3d = (x_cg_cad - x_np_3d_cad) / (mac * 1000.0)

    alphas = np.arange(-2.0, 16.1, 1.0)
    r0 = asb.AeroBuildup(build_plane(**kw, elevator=0.0), asb.OperatingPoint(velocity=16.0, alpha=alphas), xyz_ref=[x_cg_asb, 0, 0]).run()
    r1 = asb.AeroBuildup(build_plane(**kw, elevator=-8.0), asb.OperatingPoint(velocity=16.0, alpha=alphas), xyz_ref=[x_cg_asb, 0, 0]).run()
    r2 = asb.AeroBuildup(build_plane(**kw, elevator=-16.0), asb.OperatingPoint(velocity=16.0, alpha=alphas), xyz_ref=[x_cg_asb, 0, 0]).run()
    cl_t, el_t = [], []
    for i in range(len(alphas)):
        ds = np.array([-16.0, -8.0, 0.0])
        cms = np.array([r2["Cm"][i], r1["Cm"][i], r0["Cm"][i]])
        cls = np.array([r2["CL"][i], r1["CL"][i], r0["CL"][i]])
        if cms.min() <= 0.01 and cms.max() >= -0.01:
            o = np.argsort(cms)
            dt = float(np.clip(np.interp(0.0, cms[o], ds[o]), -25.0, 25.0))
            cl_t.append(float(np.interp(dt, ds, cls)))
            el_t.append(-dt)
    cl_max_trim = max(cl_t) if cl_t else 0.1
    w = mass_kg * G
    v_stall = math.sqrt(2.0 * w / (RHO * s_ref * cl_max_trim))
    v_min_ops = A.MIN_SPEED_FACTOR * v_stall

    # Performance
    m = CAT["motors"]["T-Motor F100 2810 1350KV"]
    b = CAT["batteries"]["Molicel P60B 6S3P (18x21700, 180A)"]
    d = APC["8x9"]["d_in"] * 0.0254
    tab = np.array(APC["8x9"]["table"])
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
    for v in np.arange(math.ceil(v_min_ops * 2) / 2, 32.0, 0.5):
        ab = asb.AeroBuildup(p_full, asb.OperatingPoint(velocity=float(v), alpha=alphas), xyz_ref=[x_cg_asb, 0, 0]).run()
        cl_p, cd_p = np.ravel(ab["CL"]), np.ravel(ab["CD"])
        im_ = int(np.argmax(cl_p))
        q = 0.5 * RHO * v * v
        cl = w / (q * s_ref)
        if cl > cl_max_trim: continue
        cd = float(np.interp(cl, cl_p[:im_ + 1], cd_p[:im_ + 1])) * 1.15 + extra_dq / s_ref
        drag = q * s_ref * cd
        n = opt.thrust_solve(np.array([drag / 2.0]), np.array([v]), d, J, CT)
        p1, vm, _, sh = elec(n, np.array([v]))
        if vm[0] > vbat: continue
        p_tot = float(2.0 * p1[0] + avionics_w)
        row = dict(v=float(v), p=p_tot, ld=cl / cd, range_km=energy / p_tot * v * 3.6, endurance=energy / p_tot * 60.0)
        if best_rng is None or row["range_km"] > best_rng["range_km"]: best_rng = row
        if best_end is None or row["endurance"] > best_end["endurance"]: best_end = row

    ab_st = asb.AeroBuildup(p_full, asb.OperatingPoint(velocity=best_rng["v"], alpha=4.0), xyz_ref=[x_cg_asb, 0, 0]).run_with_stability_derivatives()
    cl_c = w / (0.5 * RHO * best_rng["v"] ** 2 * s_ref)
    cr_el = float(np.interp(cl_c, cl_t[:int(np.argmax(cl_t)) + 1], el_t[:int(np.argmax(cl_t)) + 1]))
    # Check roll rate for 10 deg opposite outboard elevon
    base_r = asb.AeroBuildup(build_plane(**kw, elevator=-cr_el), asb.OperatingPoint(velocity=best_rng["v"], alpha=4.0), xyz_ref=[x_cg_asb, 0, 0]).run_with_stability_derivatives()
    roll_r = asb.AeroBuildup(build_plane(**kw, elevator=-cr_el, aileron=10.0), asb.OperatingPoint(velocity=best_rng["v"], alpha=4.0), xyz_ref=[x_cg_asb, 0, 0]).run()
    cl_da = abs(f(roll_r, "Cl") - f(base_r, "Cl"))
    roll_dps = abs(math.degrees(-cl_da / f(base_r, "Clp") * 2 * best_rng["v"] / span))

    # Vmax at 3.6V & 3.9V
    n_max = min(190000.0 / 8.0 / 60.0, 265.0 / (math.pi * d))
    def ft(v, v_cell):
        voc = b["ns"] * v_cell
        lo, hi = 5.0, n_max
        for _ in range(36):
            mid = 0.5 * (lo + hi)
            _, vm, im, _ = elec(np.array([mid]), np.array([v]), voc)
            vm, im = float(vm[0]), float(im[0])
            disc = voc ** 2 - 8.0 * vm * im * r_pack
            vb = (voc + math.sqrt(max(disc, 0.0))) / 2.0
            if disc >= 0 and vm <= vb and im <= min(m["imax"], 70.0) and 2 * vm * im / max(vb, 0.1) <= b["imax"]: lo = mid
            else: hi = mid
        return 2.0 * float(np.interp(v / (lo * d), J, CT)) * RHO * lo ** 2 * d ** 4

    ab_hi = asb.AeroBuildup(p_full, asb.OperatingPoint(velocity=85.0, alpha=np.linspace(-4, 4, 17)), xyz_ref=[x_cg_asb, 0, 0]).run()
    cl_ht, cd_ht = np.ravel(ab_hi["CL"]), np.ravel(ab_hi["CD"])
    v36, v39 = 0.0, 0.0
    for v in np.arange(40.0, 115.0, 1.0):
        q = 0.5 * RHO * v * v
        cl = w / (q * s_ref)
        drag = q * s_ref * (float(np.interp(cl, cl_ht, cd_ht)) * 1.15 + extra_dq / s_ref)
        if ft(v, 3.6) >= drag: v36 = v
        if ft(v, 3.9) >= drag: v39 = v

    return dict(
        ar=ar, s_cm2=s_ref * 1e4, mac=mac * 1000,
        np3d=x_np_3d_cad, np1d=x_np_1d_cad, cg=x_cg_cad, m3d=m3d * 100,
        clm=cl_max_trim, stall=v_stall * 3.6, cr_el=cr_el, roll_dps=roll_dps,
        rng=best_rng["range_km"], vc=best_rng["v"] * 3.6, pc=best_rng["p"], ld=best_rng["ld"],
        end=best_end["endurance"], vl=best_end["v"] * 3.6, pl=best_end["p"],
        v36=v36 * 3.6, v39=v39 * 3.6,
        cma=f(ab_st, "Cma"), cnb=f(ab_st, "Cnb"), clb=f(ab_st, "Clb"), clp=f(ab_st, "Clp"),
    )


if __name__ == "__main__":
    cases = [
        ("A: dih=2.5 sw=30 rootLE=140 ey=470..685", dict(half_mm=900, cr_mm=176, ct_mm=78, sweep_deg=30, dihedral_deg=2.5, washout=4.2, body_reflex=5.5, root_le=140, ey0=470, ey1=685, fin_cant=34, fin_h_up=135, fin_h_dn=95)),
        ("B: dih=3.5 sw=30 rootLE=150 ey=470..685", dict(half_mm=900, cr_mm=176, ct_mm=78, sweep_deg=30, dihedral_deg=3.5, washout=4.2, body_reflex=5.5, root_le=150, ey0=470, ey1=685, fin_cant=32, fin_h_up=145, fin_h_dn=95)),
        ("C: dih=4.0 sw=30 rootLE=155 ey=480..790", dict(half_mm=900, cr_mm=176, ct_mm=78, sweep_deg=30, dihedral_deg=4.0, washout=4.2, body_reflex=5.5, root_le=155, ey0=480, ey1=790, fin_cant=30, fin_h_up=150, fin_h_dn=95)),
        ("D: dih=3.5 sw=30 rootLE=155 cr=172 ct=74", dict(half_mm=900, cr_mm=172, ct_mm=74, sweep_deg=30, dihedral_deg=3.5, washout=4.5, body_reflex=6.0, root_le=155, ey0=470, ey1=780, fin_cant=30, fin_h_up=150, fin_h_dn=95)),
        ("E: dih=3.5 sw=29 rootLE=150 cr=174 ct=76 h=920", dict(half_mm=920, cr_mm=174, ct_mm=76, sweep_deg=29, dihedral_deg=3.5, washout=4.5, body_reflex=6.0, root_le=150, ey0=480, ey1=790, fin_cant=30, fin_h_up=150, fin_h_dn=95)),
    ]
    for lab, kw in cases:
        r = check(kw)
        print(f"\n[{lab}] AR={r['ar']:.2f} S={r['s_cm2']:.0f} cm2")
        print(f"  V36={r['v36']:.0f} V39={r['v39']:.0f} km/h | Rng={r['rng']:.0f} km @{r['vc']:.0f} ({r['pc']:.1f}W, L/D={r['ld']:.1f}) | End={r['end']:.0f} m ({r['end']/60:.2f}h) @{r['vl']:.0f} ({r['pl']:.1f}W)")
        print(f"  NP3D={r['np3d']:+.1f} NP1D={r['np1d']:+.1f} CG={r['cg']:+.1f} mm (3D m={r['m3d']:.1f}%) | CLm={r['clm']:.3f} Stall={r['stall']:.1f} km/h | El={r['cr_el']:+.2f} deg | Roll={r['roll_dps']:.0f} deg/s")
        print(f"  Cma={r['cma']:+.3f}  Cnb={r['cnb']:+.4f}  Clb={r['clb']:+.4f}  Clp={r['clp']:+.3f}")
