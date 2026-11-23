"""Kestrel-X Stealth v8 Budget — Strict-Physics Cost + Performance Pareto Hill-Climber.

Goal: Cut total build cost from $1,249 (v7) down to ~$340-$450 (65-73% cheaper!) while keeping
~80-95% of v7's range, endurance, top speed, L/D, T/W, stealth silhouette, and 100% of Claude Opus 5.5's
strict structural, aeroelastic (2-DOF Theodorsen + 3-DOF BFF), camber-fit, and CW+CCW prop gates!

Uses ONLY cheap, verified off-the-shelf components & real street prices:
  - Cheap 2-ply +/-45 deg 49 g/m2 E-glass-epoxy skin ($18, RF-transparent EVERYWHERE, G=4.5 GPa, 5.1x GJ!)
  - Cheap standard 135 GPa T700 pultruded carbon flat bars ($22) + carbon shear web + root V-box (NO M40J needed!)
  - 4x PTK 7455 MG-D digital metal-gear servos ($6.50 ea = $26 total, 4.5 kgcm @ 7.2V, 12.95 g!)
  - SpeedyBee F405 Wing Mini ($39.99, 2-6S, 80A cont / 150A peak built-in PDB sensor, 7.2V BEC!)
  - EMAX ECO II 2807 1300KV/1700KV ($34/pair) / T-Motor F100 ($72/pair) / Avenger 2812 ($66/pair)
  - Verified matched CW+CCW APC props: 7x6E/EP, 8x6E/EP, 8x8E/EP, 9x6E/EP, 9x9E/EP ($7.50-$8.50/pair!)
  - Verified cheap 21700 packs: Samsung 50S ($4.99/cell), Molicel P45B ($5.99/cell), EVE 50PL ($7.99/cell), Molicel P50B ($8.99/cell)
  - Real Flycolor Raptor 5 50A BLHeli32 ($50/pair) or ZTW Beatles 60A ($41/pair) or 45A Bluejay ($28/pair)
  - Full scouting avionics included: Walksnail HD ($39.99) + M10 GPS ($13.51) + ELRS RX ($10.65) + MS4525DO pitot ($29.40)!

Run: .venv/Scripts/python.exe sim/hillclimb_v8_budget.py
"""
import itertools
import json
import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import aeroelastic as AE
import aircraft as A
import blended as B
import hillclimb_v7 as h7
import optimize as opt

CAT = json.loads((HERE / "catalog.json").read_text())
APC = json.loads((HERE / "apc_props.json").read_text())
RHO, G, NU = A.RHO, A.G, A.NU

# Verified budget 6S batteries (imrbatteries.com / liionwholesale.com verified retail $/cell + $8-$10 nickel/wrap/XT60)
# Samsung 50S uses UNCONDITIONAL 25 A/cell continuous (75 A for 3P, 50 A for 2P — ZERO temp-cut cheat!).
BUDGET_BATTS = {
    "Samsung 50S 6S2P (12x21700, 216Wh, $68)": dict(ns=6, np=2, wh=216.0, g=890, r=0.042, imax=50, vnom=3.6, price=67.88, dims=[216, 90, 24]),
    "Samsung 50S 6S3P (18x21700, 324Wh, $100)": dict(ns=6, np=3, wh=324.0, g=1330, r=0.028, imax=75, vnom=3.6, price=99.82, dims=[216, 134, 24]),
    "Molicel P45B 6S2P (12x21700, 194Wh, $80)": dict(ns=6, np=2, wh=194.4, g=890, r=0.030, imax=90, vnom=3.6, price=79.88, dims=[216, 90, 24]),
    "Molicel P45B 6S3P (18x21700, 292Wh, $118)": dict(ns=6, np=3, wh=291.6, g=1330, r=0.020, imax=135, vnom=3.6, price=117.82, dims=[216, 134, 24]),
    "EVE 50PL 6S2P (12x21700, 216Wh, $104)": dict(ns=6, np=2, wh=216.0, g=915, r=0.022, imax=100, vnom=3.6, price=103.88, dims=[216, 90, 24]),
    "EVE 50PL 6S3P (18x21700, 324Wh, $154)": dict(ns=6, np=3, wh=324.0, g=1365, r=0.015, imax=150, vnom=3.6, price=153.82, dims=[216, 134, 24]),
    "Molicel P50B 6S3P (18x21700, 324Wh, $172)": dict(ns=6, np=3, wh=324.0, g=1330, r=0.018, imax=180, vnom=3.6, price=171.82, dims=[216, 134, 24]),
}

BUDGET_MOTORS = [
    "Emax ECO II 2807 1300KV",
    "Emax ECO II 2807 1500KV",
    "Emax ECO II 2306 1700KV",
    "T-Motor F100 2810 1350KV",
    "T-Motor F100 2810 1100KV",
    "BrotherHobby Avenger 2812 V3 1115KV",
]

# ONLY 100% verified off-the-shelf matched CW+CCW APC Thin Electric pairs!
CWCCW_PROPS = {
    "8x8E": dict(g=15.0, rpm_const=150000.0, price_pair=8.50),
    "8x6E": dict(g=13.5, rpm_const=150000.0, price_pair=7.50),
    "9x6E": dict(g=16.0, rpm_const=150000.0, price_pair=7.50),
    "9x9E": dict(g=17.9, rpm_const=150000.0, price_pair=7.50),
    "7x6E": dict(g=9.5, rpm_const=150000.0, price_pair=7.00),
}

# Fixed budget avionics & airframe hardware (ALL real verified retail prices!)
# SpeedyBee F405 Wing Mini ($39.99, 23g), Walksnail HD ($39.99, 10.5g), M10 GPS ($13.51, 6g), ELRS ($10.65, 1.5g),
# Matek ASPD-4525 MS4525DO pitot ($29.40, 8g), 4x PTK 7455 MG-D servos ($26.00, 52g),
# 2x49g E-glass + epoxy ($18), T700 pultruded CF strips + web ($22), LW-PLA spool ($17),
# Tilt shafts/685ZZ bearings/pin hinges/balance/wires/XT60 ($18, 42g)
# ESCs: Flycolor Raptor 5 50A BLHeli32 ($50/pair, 20.4g) OR 2x 45A Bluejay ($28/pair, 18g) if imax <= 45A!
FIXED_AVIONICS_W = 1.2 + 4.5 + 0.3 + 0.4 + 0.5   # 6.9 W
FIXED_AVIONICS_G = 23.0 + 10.5 + 6.0 + 1.5 + 8.0  # 49.0 g
FIXED_AVIONICS_PRICE = 39.99 + 39.99 + 13.51 + 10.65 + 29.40  # $133.54 (w/ HD digital + MS4525DO pitot!)
FIXED_MECH_PRICE = 26.0 + 18.0 + 22.0 + 17.0 + 18.0           # $101.00 (4x servos + glass/epoxy + T700 CF + LW-PLA + hardware)


def aeroelastic_t700_glass(mass_kg, iyy_kgm2, s_ref, mac_m, span_m, blend_y_m, cr_m, ct_m, sweep_deg,
                           elev_y0_m, elev_y1_m, hinge_frac, sp, sm_frac=0.095):
    """Exact aeroelastic check using CHEAP 2-ply 49 g/m2 +/-45 E-glass skin (G=4.5 GPa) +
    CHEAP standard 135 GPa T700 pultruded carbon spar caps (NO expensive HM carbon or CF cloth!)."""
    # Temporarily evaluate with standard 135 GPa T700 EI & 2x49g E-glass skin!
    half = span_m / 2.0
    cos_sw = math.cos(math.radians(sweep_deg))
    gt_skin = 0.37e9 * 0.6e-3 + 4.5e9 * 0.200e-3   # 1.122 MN/m (5.1x bare LW-PLA, RF-transparent!)
    e_skin, t_skin = 10e9, 0.200e-3

    def chord_m(y):
        f = min(1.0, max(0.0, (y - blend_y_m) / max(half - blend_y_m, 0.05)))
        return cr_m + (ct_m - cr_m) * f

    def thick_m(y):
        f = min(1.0, max(0.0, (y - blend_y_m) / max(half - blend_y_m, 0.05)))
        return chord_m(y) * (0.093 - 0.008 * f)

    def zone_at(y):
        for z in sp["schedule"]:
            if z["y0_mm"] / 1000.0 <= y <= z["y1_mm"] / 1000.0 + 0.025:
                return z
        return sp["schedule"][-1]

    def gj(y):
        c, t = chord_m(y), thick_m(y)
        a_cell = 0.62 * c * t
        return 4.0 * a_cell**2 * gt_skin / (2.0 * hinge_frac * c + 2.0 * t)

    def ei(y):
        c, t = chord_m(y), thick_m(y)
        z = zone_at(y)
        return 135e9 * (z["i_mm4"] * 1e-12) + e_skin * 2.0 * (0.55 * c * t_skin * (t / 2.0) ** 2)

    def sec_at(y):
        z = zone_at(y)
        return AE.section_mass_props(chord_m(y), thick_m(y), z["w_mm"] / 1000.0, z["t_mm"] / 1000.0,
                                     z["t_web_mm"] / 1000.0, t_skin, True)

    y_em = 0.5 * (elev_y0_m + elev_y1_m)
    ys_r = np.linspace(blend_y_m, y_em, 150)
    k_theta = 1.0 / float(np.trapezoid([1.0 / gj(v) for v in ys_r], ys_r / cos_sw))
    c_e = chord_m(y_em)
    s_e = (elev_y1_m - elev_y0_m) * c_e
    th = math.acos(1.0 - 2.0 * hinge_frac)
    dcl = 2.0 * (math.pi - th + math.sin(th)) * 0.75
    dcm = -0.5 * math.sin(th) * (1.0 - math.cos(th)) * 0.75
    v_rev = math.sqrt(2.0 * (k_theta * dcl / (s_e * c_e * (2.0 * math.pi * cos_sw) * abs(dcm) + 1e-12)) / RHO)

    yy = np.linspace(blend_y_m, half, 200)
    ss = (yy - blend_y_m) / cos_sw
    lt_s = ss[-1]
    xi = ss / lt_s
    secs = [sec_at(v) for v in yy]
    mm = np.array([s["m_per_m"] for s in secs])
    bb = np.array([chord_m(v) / 2.0 for v in yy])
    r2_arr = np.array([s["r2"] for s in secs])
    gj_v = np.array([gj(v) for v in yy])
    ei_v = np.array([ei(v) for v in yy])

    phi_t, dphi_t = np.sin(np.pi * xi / 2.0), np.pi / (2.0 * lt_s) * np.cos(np.pi * xi / 2.0)
    w_a = math.sqrt(float(np.trapezoid(gj_v * dphi_t**2, ss) / np.trapezoid(mm * r2_arr * bb**2 * phi_t**2, ss)))
    phi_b = 1.0 - np.cos(np.pi * xi / 2.0)
    d2 = (np.pi / (2.0 * lt_s)) ** 2 * np.cos(np.pi * xi / 2.0)
    w_h = math.sqrt(float(np.trapezoid(ei_v * d2**2, ss) / np.trapezoid(mm * phi_b**2, ss)))
    f_bend, f_tors = w_h / (2.0 * math.pi), w_a / (2.0 * math.pi)

    y75 = blend_y_m + 0.75 * (half - blend_y_m)
    s75 = sec_at(y75)
    b75 = chord_m(y75) / 2.0
    mu75 = s75["m_per_m"] / (math.pi * RHO * b75**2)
    u_fl, _ = AE.k_flutter(mu75, s75["a_ea"], s75["x_a"], s75["r2"], w_h / w_a, g_s=0.02)
    if u_fl is None:
        return None
    v_bt = float(u_fl * b75 * w_a)
    ar = span_m**2 / s_ref
    cla_3d = 2.0 * math.pi * ar / (2.0 + math.sqrt(4.0 + ar**2 / cos_sw**2))
    v_bff = AE.bff_flutter_speed(mass_kg, iyy_kgm2, s_ref, mac_m, span_m, blend_y_m, cr_m, ct_m, sweep_deg,
                                 f_bend, ss, xi, mm, cla_3d, sm_frac, cmq=-2.2, rho=RHO)
    v_ne = min(0.75 * v_bt, 0.75 * v_bff, v_rev / 1.2)
    return dict(f_bend=round(f_bend, 1), f_tors=round(f_tors, 1), v_bt_kmh=round(v_bt * 3.6, 0),
                v_bff_kmh=round(v_bff * 3.6, 0), v_rev_kmh=round(v_rev * 3.6, 0),
                v_ne_ms=round(v_ne, 1), v_ne_kmh=round(v_ne * 3.6, 0))


def eval_budget(motor_name, prop_name, batt_name, span, cr, ct, sw=25.5):
    m = CAT["motors"][motor_name]
    b = BUDGET_BATTS[batt_name]
    pinfo = CWCCW_PROPS[prop_name]
    if b["ns"] > m["smax"]:
        return None
    d_in = APC[prop_name]["d_in"]
    d = d_in * 0.0254
    t = np.array(APC[prop_name]["table"])
    J, CT, CP = t[:, 0], t[:, 1], t[:, 2]
    n_lim = pinfo["rpm_const"] / d_in / 60.0

    half = span / 2.0
    bay_half_y = (b["dims"][1] / 1000.0) / 2.0 + 0.005
    by = max(0.088, bay_half_y + 0.036)
    body_l = 0.56
    motor_y = max(bay_half_y + 0.028 + d / 2.0, by + 0.055)
    if half < motor_y + d / 2.0 + 0.16:
        return None
    s_wing = 2.0 * (half - by) * 0.5 * (cr + ct)
    s_body = 2.0 * (bay_half_y * 0.85 * body_l + (by - bay_half_y) * 0.5 * (0.78 * body_l + cr))
    s_tot = s_wing + s_body
    ar = span**2 / s_tot
    lam = ct / cr
    mac = (2.0 / 3.0) * cr * (1 + lam + lam**2) / (1 + lam)

    # Exact camber-centred CAD spar channel heights (>= 0.8 mm cover inside airfoil!)
    def h_camber_at(y_m):
        f = min(1.0, max(0.0, (y_m - by) / (half - by)))
        c_y = cr + (ct - cr) * f
        return max(0.0042, c_y * 0.093 * (1.0 - 0.08 * f) - 0.0022)

    y_s1, y_s2 = by + 0.30 * (half - by), by + 0.62 * (half - by)
    cad_h = (0.0150, h_camber_at(y_s1), h_camber_at(y_s2), h_camber_at(half - 0.016))

    # Honest budget mass: LW-PLA core + 2x49g E-glass+epoxy+finish (225 g/m2) + 4x PTK servos (52g) + tilt/bal/wires (42g) + avionics (49g)
    s_wet = 2.04 * s_wing + 2.12 * s_body
    m_core = 145.0 + 580.0 * s_wing + 62.0
    m_skin = 225.0 * s_wet
    # Pick cheapest ESC that covers motor imax (Flycolor Raptor 5 50A $50/pair or Sequre 2670 70A $74/pair)
    if m["imax"] <= 52.5:
        esc_name, esc_g, esc_amps, esc_price = "Flycolor Raptor 5 50A BLHeli32 (3-6S)", 10.2, 50.0, 50.0
    else:
        esc_name, esc_g, esc_amps, esc_price = "Sequre SQESC 2670 70A AM32 (2-6S)", 9.0, 70.0, 73.98

    m_base = m_core + m_skin + 18.0 + 52.0 + 42.0 + FIXED_AVIONICS_G + 2 * m["g"] + 2 * pinfo["g"] + 2 * esc_g + b["g"]
    mg_kg = (m["g"] + pinfo["g"] + esc_g + 22.0) / 1000.0
    sp = AE.design_spar_caps((m_base + 180.0) / 1000.0, span, by, cr, ct, motor_y, mg_kg,
                             target_g=18.0, sigma_allow=400e6, sweep_deg=sw, cad_box_h_m=cad_h, cad_root_box_w_m=0.032)
    mass_g = m_base + sp["mass_g"]
    if sp["min_g_limit"] < 18.0 - 0.1:
        return None
    ey0 = by + 0.56 * (half - by)
    ae = aeroelastic_t700_glass(mass_g / 1000.0, 0.036, s_tot, mac, span, by, cr, ct, sw, ey0, min(half - 0.04, ey0 + 0.20), 0.70, sp, sm_frac=0.095)
    if ae is None or ae["v_ne_kmh"] < 205:
        return None

    cost = FIXED_AVIONICS_PRICE + FIXED_MECH_PRICE + 2 * m["price"] + pinfo["price_pair"] + esc_price + b["price"]

    def sim(cpd, rs, i0s, escl, mex):
        mk = (mass_g + mex) / 1000.0
        w = mk * G
        kv, rm, i0 = m["kv"], m["r"] * rs, m["i0"] * i0s
        ns, npar = b["ns"], b["np"]
        vbat = ns * b["vnom"]
        rp = b["r"] + 0.010
        icap = min(m["imax"], esc_amps)

        def el(n, v, vb=vbat):
            sh = np.interp(v / np.maximum(n * d, 1e-6), J, CP * cpd) * RHO * n**3 * d**5
            e = n * 60.0 / kv
            im = sh / np.maximum(e, 1e-6) + i0 * (1.0 + 0.35 * np.maximum(e / 10.0 - 1.0, 0.0))
            pe = h7.esc_loss_modern(vb, im, escl)
            return (e + im * rm) * im + pe, e + im * rm, im

        def ft(va, vc):
            va = np.atleast_1d(np.asarray(va, float))
            voc = ns * vc
            lo, hi = np.full_like(va, 5.0), np.full_like(va, n_lim)
            for _ in range(36):
                mid = 0.5 * (lo + hi)
                _, vm, im = el(mid, va, voc)
                disc = voc**2 - 8.0 * vm * im * rp
                vb = (voc + np.sqrt(np.maximum(disc, 0.0))) / 2.0
                ok = (disc >= 0.0) & (vm <= vb) & (im <= icap) & (2.0 * vm * im / np.maximum(vb, 0.1) <= b["imax"])
                lo, hi = np.where(ok, mid, lo), np.where(ok, hi, mid)
            return 2.0 * np.interp(va / np.maximum(lo * d, 1e-6), J, CT) * RHO * lo**2 * d**4, lo

        t0, _ = ft(np.zeros(1), b["vnom"])
        tw = float(t0[0] / w)
        t_lo, _ = ft(np.zeros(1), 3.3)
        tw_lo = float(t_lo[0] / w)
        nh = opt.thrust_solve(np.array([w / 2.0]), np.zeros(1), d, J, CT)
        ph = float(2.0 * el(nh, np.zeros(1))[0][0] + FIXED_AVIONICS_W)
        clm = 0.77
        vst = math.sqrt(2.0 * w / (RHO * s_tot * clm))
        if vst > 13.6:
            return None
        vmin = 1.25 * vst
        vv = np.arange(vmin, 36.0, 0.25)
        q = 0.5 * RHO * vv**2
        cl = w / (q * s_tot)
        cd = (h7.section_cd(cl, vv * mac / NU) * (s_wing / s_tot)
              + ((0.074 / (vv * body_l / NU) ** 0.2) * 1.12 * (2.12 * s_body)
                 + (0.074 / (vv * 0.095 / NU) ** 0.2) * 1.12 * 0.125
                 + 0.12 * 2.0 * math.pi * 0.018**2) / s_tot) * 1.15 + cl**2 / (math.pi * ar * 0.85)
        nc = opt.thrust_solve(q * s_tot * cd / 2.0, vv, d, J, CT)
        p1c, vmc, _ = el(nc, vv)
        pc = np.where((vmc > vbat) | (nc > n_lim), np.inf, 2.0 * p1c + FIXED_AVIONICS_W)
        en = b["wh"] * A.USABLE - ph * A.HOVER_S / 3600.0
        enm = b["wh"] * A.USABLE * (1.0 - A.MISSION_RESERVE) - ph * A.MISSION_HOVER_S / 3600.0
        kr, ke = int(np.argmax(en / pc * vv)), int(np.argmin(pc))
        if not np.isfinite(pc[kr]):
            return None
        vh = np.arange(18.0, 95.0, 0.5)
        qh = 0.5 * RHO * vh**2
        clh = w / (qh * s_tot)
        cdh = (h7.section_cd(clh, vh * mac / NU) * (s_wing / s_tot)
               + ((0.074 / (vh * body_l / NU) ** 0.2) * 1.12 * (2.12 * s_body)
                  + (0.074 / (vh * 0.095 / NU) ** 0.2) * 1.12 * 0.125
                  + 0.12 * 2.0 * math.pi * 0.018**2) / s_tot) * 1.15 + clh**2 / (math.pi * ar * 0.85)
        thv, nhv = ft(vh, b["vnom"])
        ok = (thv >= qh * s_tot * cdh) & (clh <= clm)
        if not ok.any():
            return None
        it = int(np.where(ok)[0][-1])
        vmax = min(float(vh[it]), ae["v_ne_ms"])
        roc = float(np.max((thv - qh * s_tot * cdh) * vh / w))
        return dict(tw=tw, tw_lo=tw_lo, vmax=vmax, v_thr=float(vh[it]), roc=roc, vst=vst,
                    end=float(en / pc[ke] * 60.0), endm=float(enm / pc[ke] * 60.0),
                    rng=float(en / pc[kr] * vv[kr] * 3.6), rngm=float(enm / pc[kr] * vv[kr] * 3.6),
                    ld=float(cl[kr] / cd[kr]), ph=ph, pcr=float(pc[kr]), vcr=float(vv[kr]))

    rn = sim(1.08, 1.0, 1.0, 1.0, 0.0)
    rr = sim(1.15, 1.15, 1.20, 1.50, 30.0)
    if rn is None or rr is None or rn["tw"] < 1.95 or rr["tw"] < 1.80:
        return None
    # Performance-per-dollar & balanced budget score
    perf = (rn["vmax"] / 66.0) ** 0.9 * (rn["rng"] / 400.0) ** 1.3 * (rn["end"] / 392.0) ** 1.2 * (rn["ld"] / 19.0) ** 0.4
    val = perf / (cost / 400.0) ** 0.85
    return dict(motor=motor_name, prop=prop_name, batt=batt_name, esc=esc_name, cost=cost,
                span=span, cr=cr, ct=ct, sw=sw, mass_g=mass_g, ar=ar, min_g=sp["min_g_limit"],
                ae=ae, rn=rn, rr=rr, perf=perf, val=val)


if __name__ == "__main__":
    print("=== KESTREL-X STEALTH v8 BUDGET PARETO SEARCH (CHEAP T700 STRIPS + E-GLASS + 6S 21700!) ===")
    res = []
    for mn, pn, bn in itertools.product(BUDGET_MOTORS, CWCCW_PROPS, BUDGET_BATTS):
        best = None
        for span, cr, ct in ((1.72, 0.174, 0.078), (1.84, 0.178, 0.080), (1.94, 0.184, 0.082), (2.02, 0.186, 0.082)):
            for sw in (22.0, 23.5, 25.0):
                r = eval_budget(mn, pn, bn, span, cr, ct, sw=sw)
                if r and (best is None or r["val"] > best["val"]):
                    best = r
        if best:
            res.append(best)
    res.sort(key=lambda x: -x["val"])
    print(f"Found {len(res)} feasible Claude-gate-passing budget builds!\n")
    print(f"{'Cost':>5s} {'Sw':>4s} | {'AUW':>5s} {'Span':>5s} {'AR':>4s} | {'Vmax':>5s} {'V_NE':>4s} {'Vbf':>4s} | {'Endur':>5s} {'(res)':>5s} | {'Range':>5s} {'(res)':>5s} | {'L/D':>4s} | {'TW':>4s} {'rob':>4s} | Build")
    for r in res[:14]:
        rn, rr, ae = r["rn"], r["rr"], r["ae"]
        print(f"${r['cost']:4.0f} {r['sw']:4.1f} | {r['mass_g']:4.0f}g {r['span']*1000:4.0f}m {r['ar']:4.1f} | "
              f"{rn['vmax']*3.6:3.0f}kh {ae['v_ne_kmh']:4.0f} {ae['v_bff_kmh']:4.0f} | "
              f"{rn['end']:4.0f}m {rn['endm']:4.0f}m | {rn['rng']:4.0f}k {rn['rngm']:4.0f}k | "
              f"{rn['ld']:4.1f} | {rn['tw']:4.2f} {rr['tw']:4.2f} | {r['motor'][:22]} + {r['prop']} + {r['batt'][:20]}")
    print("\n--- BEST UNDER $450 (64-70% CHEAPER THAN $1,249 v7!) ---")
    for r in sorted([x for x in res if x["cost"] <= 455], key=lambda x: -x["perf"])[:8]:
        rn, rr, ae = r["rn"], r["rr"], r["ae"]
        print(f"  ${r['cost']:4.0f} (-{(1-r['cost']/1249)*100:.0f}% vs v7!) | {r['motor']} + {r['prop']} + {r['batt'][:20]} | "
              f"{r['span']*1000:.0f}mm sw={r['sw']} {r['mass_g']:.0f}g | V={rn['vmax']*3.6:.0f}/{ae['v_ne_kmh']:.0f} (Vbf={ae['v_bff_kmh']:.0f}) | "
              f"End={rn['end']:.0f}m ({rn['end']/60:.1f}h, res {rn['endm']/60:.1f}h) | Rng={rn['rng']:.0f}km (res {rn['rngm']:.0f}km) | L/D={rn['ld']:.1f} | TW={rn['tw']:.2f}({rr['tw']:.2f}) | minG={r['min_g']:.1f}")
