"""Review of Devin's sim/hillclimb_v7.py + sim/aeroelastic.py (Claude, 2026-11-20). Read-only.

Run from the repo root:  .venv/Scripts/python.exe audit/claude_v7_hillclimb_review.py
"""
import json
import math
import pathlib
import sys

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))
import aeroelastic as AE      # noqa: E402
import hillclimb_v7 as HC     # noqa: E402

RES = json.loads((ROOT / "sim" / "hillclimb_v7_results.json").read_text())
R1, R2 = RES[0], RES[1]
MOT, PROP = "BrotherHobby Avenger 2812 V3 1115KV", "8x9"
SA504, SA88 = R1["battery"], R2["battery"]


def hdr(t):
    print("\n" + "=" * 100 + f"\n{t}\n" + "=" * 100)


hdr("1. WHICH CONSTRAINTS ARE ACTIVE AT THE 'INTERIOR' OPTIMUM")
for i, r in enumerate(RES[:12], 1):
    pins = []
    if abs(r["sweep_deg"] - 26.0) < 0.3:
        pins.append("sweep @ 26.0 lower bound")
    if abs(r["ct"] - 0.072) < 0.0005:
        pins.append("ct @ 72 mm lower bound")
    if r["span"] > 2.99:
        pins.append("span @ 3.0 upper bound")
    if r["min_g_limit"] < 15.0:
        pins.append(f"spar {r['min_g_limit']:.1f} g < 15 g target (not enforced)")
    vmin = 1.25 * r["v_stall"]
    if abs(r["v_cruise"] - vmin) < 0.3:
        pins.append("cruise speed @ 1.25 x stall floor")
    print(f"  #{i:2d} {r['motor'][:22]:22s} {r['prop']:6s}: " + "; ".join(pins))
print("  -> sweep has no stability/trim model in the climber, so the optimizer deletes it to save spar length.")
print("  -> best-range AND best-endurance speeds sit on the 1.25 x V_stall floor: range/endurance scale ~directly with")
print("     the ASSUMED clmax_trim = 0.74 x section CLmax (no trim, static margin or elevon-deflection model behind it).")

hdr("2. SPAR CAPS: moment about the SWEPT spar axis is M_y / cos(sweep)")
for r in (R1, R2):
    c = math.cos(math.radians(r["sweep_deg"]))
    print(f"  {r['battery'][:26]:26s}: reported min {r['min_g_limit']:.1f} g -> about the swept spar {r['min_g_limit']*c:.1f} g"
          f" (x cos {r['sweep_deg']:.0f} deg = {c:.3f})")
w = R2["mass_g"] / 1000 * 9.81
v_root = 15 * w / 2 - 15 * 9.81 * 0.17
for web, tau_allow in (("1.0 mm balsa (in the mass model)", 2.0e6), ("0.3 mm +/-45 carbon", 60e6)):
    t = 1.0e-3 if "balsa" in web else 0.3e-3
    tau = 1.5 * v_root / (0.014 * t)
    print(f"  shear web, 15 g root shear {v_root:.0f} N on a 14 mm web, {web}: tau {tau/1e6:5.1f} MPa vs ~{tau_allow/1e6:.0f} MPa"
          f" allowable -> {'FAIL' if tau > tau_allow else 'ok'}")
print("  The 24 g root box is a constant; it must also react the sweep-kink torsion M sin(26 deg) = 44 % of root moment.")

hdr("3. AEROELASTIC MODEL: beam length, BFF, fall-back value")
for r in (R1, R2):
    ae = r["aeroelastic"]
    c = math.cos(math.radians(r["sweep_deg"]))
    print(f"  {r['battery'][:26]:26s}: f_bend {ae['f_bend_hz']} Hz (beam measured along y); along the swept spar with the"
          f" same EI and mass ~{ae['f_bend_hz']*c**1.5:.1f} Hz (x cos^1.5)")
# short period at Vmax: rank 2 geometry, corrected-style Iyy
s = R2["span"] ** 2 / R2["ar"]
c_ref = 0.14
ar = R2["ar"]
cla = 2 * math.pi * ar / (2 + math.sqrt(ar**2 * (1 + math.tan(math.radians(26))**2) + 4))
v = R2["vmax_safe"]
q = 0.5 * 1.225 * v * v
m = R2["mass_g"] / 1000
for iyy in (0.035, 0.045, 0.055):
    for sm in (0.08,):
        m_a = q * s * c_ref * cla * sm / iyy
        z_a = q * s * cla / (m * v)
        m_q = q * s * c_ref**2 * (-1.2) / (2 * v * iyy)           # ASSUMED Cmq ~ -1.2 for a tailless swept wing
        w_sp = math.sqrt(m_a + z_a * (-m_q))
        fb = R2["aeroelastic"]["f_bend_hz"] * math.cos(math.radians(26)) ** 1.5
        print(f"  short period @ {v*3.6:.0f} km/h, SM {sm:.0%}, Iyy {iyy:.3f} kg m2 (296 mm pack alone = 0.012): "
              f"{w_sp/2/math.pi:4.1f} Hz -> f_sp/f_bend {w_sp/2/math.pi/R2['aeroelastic']['f_bend_hz']:.2f} (code) / "
              f"{w_sp/2/math.pi/fb:.2f} (swept)")
print("  aeroelastic.py has NO body-freedom check; ratio 0.5-0.75 at V_NE with aft sweep (bending -> washout -> pitch)")
print("  is inside the zone where BFF appears in flying wings. Needs a 3-DOF plunge/pitch/symmetric-bending p-k model.")
print("  Also: k_flutter() returning None falls back to V_flutter = 250 m/s (900 km/h) -> optimizer can exploit a solver miss.")
print("  x_a = 0.06 and r2 = 0.30 are fixed inputs, not computed from the section mass layout (elevon balance, servo, caps).")

hdr("4. MOTOR / ESC THERMAL AT Vmax (their own numbers)")
for r, cp_j in ((R1, 46.0), (R2, 46.0)):
    tm = r["motor_heat_w"] / cp_j
    te = r["esc_heat_w"] / 13.5
    print(f"  {r['battery'][:26]:26s}: motor {r['motor_heat_w']:.0f} W into 77.5 g (~{cp_j:.0f} J/K) = {tm:.1f} K/s adiabatic"
          f" -> 80 K in {80/tm:.0f} s; ESC {r['esc_heat_w']:.0f} W into 15 g = {te:.1f} K/s -> 80 K in {80/te:.0f} s")
print("  Copper R rises 0.39 %/K, so heat grows ~30 % by the end of the dash. '30-45 s' needs >= 50 % of the heat removed by")
print("  airflow: plausible in a ducted nacelle, unproven. Log motor + ESC temperature and gate the dash on it.")
print("  esc_loss_modern: 0.004 I^2 + 2 % V I is defensible for a 2.5 mOhm-FET 80 A ESC (2 FETs conducting = 0.005 I^2),")
print("  but ROBUST does not scale esc_loss at all (r_scale/i0_scale/cp only). Add esc_loss=1.5 to ROBUST.")
print("  i0 is the 10 V figure; at 24k rpm iron loss is ~1.3-1.6x higher (adds ~10-20 W per motor at Vmax).")

hdr("5. SENSITIVITY of the two finalists (Devin's evaluate_v7)")
print(f"  {'case':48s} {'AUW':>5s} {'T/W':>4s} {'Vnom':>4s} {'end min':>7s} {'range':>5s} {'Vfl':>4s} {'f_b':>5s} {'g':>5s}")
for batt, base in ((SA88, R2), (SA504, R1)):
    for lbl, span, sw, kw in (("as converged", base["span"], base["sweep_deg"], {}),
                              ("span 2.25, sweep 26", 2.25, 26.0, {}), ("span 2.30, sweep 28", 2.30, 28.0, {}),
                              ("span 2.36, sweep 28", 2.36, 28.0, {}),
                              ("as converged, ROBUST + esc_loss 1.5", base["span"], base["sweep_deg"],
                               dict(cp_derate=1.15, r_scale=1.15, i0_scale=1.2, mass_extra=30.0, esc_loss=1.5, tw_floor=1.9)),
                              ("as converged, drag x1.15", base["span"], base["sweep_deg"], dict(drag_scale=1.15))):
        r = HC.evaluate_v7(MOT, PROP, batt, span, base["cr"], base["ct"], sw, do_flutter=True, **kw)
        name = f"{'SA88' if batt == SA88 else 'SA504'}: {lbl}"
        if r is None:
            print(f"  {name:48s} infeasible")
            continue
        ae = r["aeroelastic"]
        print(f"  {name:48s} {r['mass_g']:5.0f} {r['tw_nom']:4.2f} {r['vmax_safe']*3.6:4.0f} {r['endurance_min']:7.0f}"
              f" {r['range_km']:5.0f} {ae['v_flutter_kmh']:4.0f} {ae['f_bend_hz']:5.1f} {r['min_g_limit']:5.1f}")
# CLmax_trim sensitivity: endurance/range both sit on the 1.25 Vstall floor
for scale in (0.92, 0.85):
    orig = HC.np.interp
    clm = HC.CLMAX_TAB.copy()
    HC.CLMAX_TAB[:] = clm * scale
    r = HC.evaluate_v7(MOT, PROP, SA88, R2["span"], R2["cr"], R2["ct"], R2["sweep_deg"])
    HC.CLMAX_TAB[:] = clm
    if r:
        print(f"  SA88: trimmed CLmax x{scale:.2f}                              {r['mass_g']:5.0f} {r['tw_nom']:4.2f}"
              f" {r['vmax_safe']*3.6:4.0f} {r['endurance_min']:7.0f} {r['range_km']:5.0f}")
