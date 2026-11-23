"""Independent audit checks for Kestrel-X Stealth v6 (Claude, 2026-11-20).

Run from the repo root:  .venv/Scripts/python.exe audit/claude_v6_checks.py
Reads only; writes nothing. Every number printed is a first-order engineering estimate; the
material constants marked ASSUMED are the dominant uncertainties and must be replaced by coupon tests.
"""
import math
import pathlib
import sys

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))

import aircraft as A          # noqa: E402
import blended as B           # noqa: E402
import harness as h           # noqa: E402

G, RHO = A.G, A.RHO
GEO = A.GEO
M = A.all_up_mass_kg()
W = M * G
HALF = GEO["half"] / 1000


def hdr(t):
    print("\n" + "=" * 100 + f"\n{t}\n" + "=" * 100)


# ---------------------------------------------------------------------------------------------------
hdr("1. SPANWISE SPAR CAPACITY (tube only, their 600 MPa allowable) vs bending moment per g")
y = np.linspace(0, HALF, 2001)
c = np.array([B.chord(v * 1000) / 1000 for v in y])
lift_chord = c / np.trapezoid(c, y) * W / 2                          # their assumption (envelope.py)
lift_ell = np.sqrt(np.clip(1 - (y / HALF) ** 2, 0, None))
lift_ell = lift_ell / np.trapezoid(lift_ell, y) * W / 2
motor_group = (66.5 + 11.2 + 9.0 + 14.3) / 1000 * G                 # motor + prop + ESC + pod at y = 225 mm


def moment(lift, y0):
    sel = y >= y0
    m = np.trapezoid(lift[sel] * (y[sel] - y0), y[sel])
    if y0 < 0.225:
        m -= motor_group * (0.225 - y0)                              # inertia relief of the motor group
    return m


def z_mod(od, idd):
    return math.pi * (od**4 - idd**4) / 64 / (od / 2) * 1e-9         # m3


cap = {"12x9": 600e6 * z_mod(12, 9), "8x6": 600e6 * z_mod(8, 6), "6x4": 600e6 * z_mod(6, 4)}
stations = [(0.074, "12x9", "root of 12x9 (only printed socket holds it)"), (0.106, "12x9", "wing/body joint"),
            (0.330, "8x6", "12x9 ends -> 8x6 alone"), (0.530, "8x6", "rear spar ended at 520"),
            (0.600, "6x4", "8x6 ends -> 6x4 alone"), (0.710, "6x4", "panel 4/5 split")]
print(f"  tube capacities at 600 MPa: " + ", ".join(f"{k} {v:.1f} N m" for k, v in cap.items()))
worst = 1e9
for y0, tube, note in stations:
    mc, me = moment(lift_chord, y0), moment(lift_ell, y0)
    g_lim = cap[tube] / max(mc, me)
    worst = min(worst, g_lim) if y0 > 0.11 else worst
    print(f"  y={y0*1000:4.0f} mm  {tube:4s}  M/g chord-lift {mc:5.2f}  elliptic {me:5.2f} N m  -> tube limit {g_lim:5.1f} g   ({note})")
print(f"  envelope.py claims 19 g from the 12x9 tube at the CENTRELINE only. Spanwise tube-only minimum: {worst:.1f} g")

hdr("1b. ROOT JOINT: does the 12x9 inner spar actually meet the 12x9 carry-through?")
sx = lambda frac, yy: B.le_x(yy) - frac * B.chord(yy)
x_sp74 = (GEO["root_le"] + math.tan(math.radians(GEO["sweep_deg"])) * (GEO["blend_y"] - 74.0)) - 0.3 * GEO["root_chord"]
print(f"  inner spar root  : x = {x_sp74:6.1f} mm at y = 74 mm (spar runs to x = {sx(0.3, 330):.1f} mm at y = 330)")
print(f"  carry-through    : x = {GEO['carry_x']:6.1f} mm, y = -{GEO['carry_y']:.0f}..+{GEO['carry_y']:.0f} mm")
print(f"  -> chordwise offset {x_sp74 - GEO['carry_x']:.0f} mm, spanwise overlap {GEO['carry_y'] - 74:.0f} mm, tubes never touch.")
m74 = moment(lift_chord, 0.074)
L_emb, d = 0.032, 0.012
p1g = 6 * m74 / (d * L_emb**2)
print(f"  Moment at y=74 is {m74:.2f} N m/g; reacted as bearing on a 32 mm embedment in LW-PLA: peak ~{p1g/1e6:.1f} MPa per g.")
print(f"  ASSUMED foamed LW-PLA local bearing/crush allowable 4-8 MPa (0.6 g/cc, 4-wall socket) -> root good for ~{4e6/p1g:.1f}-{8e6/p1g:.1f} g.")

# ---------------------------------------------------------------------------------------------------
hdr("2. HOVER PITCH CONTROL ARM (transition.py clamps it to 85 mm)")
y_mid = 0.5 * (GEO["hover_vane_y"][0] + GEO["hover_vane_y"][1])
le, ch = B.le_x(y_mid), B.chord(y_mid)
x_cg = A.cg_x_mm()
x_hinge = le - 0.72 * ch
th = math.acos(1 - 2 * GEO["hinge"])                       # thin-aerofoil flap theory, 30 % flap
dcl = 2 * (math.pi - th + math.sin(th))
dcm = -0.5 * math.sin(th) * (1 - math.cos(th))
x_cp_frac = 0.25 - dcm / dcl
x_cp = le - x_cp_frac * ch
print(f"  vane mid-span y={y_mid:.0f} mm: LE x={le:.1f}, chord {ch:.1f}, hinge x={x_hinge:.1f}, CG x={x_cg:.1f}")
print(f"  code: arm = max(0.085, (x_cg - x_hinge)/1000) = max(0.085, {(x_cg - x_hinge)/1000:.4f}) -> 0.085 m (FAKE)")
print(f"  flap-increment load acts at {x_cp_frac*100:.0f}% chord = x {x_cp:.1f} mm, i.e. {x_cp - x_cg:+.0f} mm AHEAD of the CG")
t_one = W / 2
v_jet = 0.88 * math.sqrt(2 * t_one / (RHO * math.pi * 0.1016**2))
q_s = 0.5 * RHO * v_jet**2
s_sub = 2 * (GEO["hover_vane_y"][1] - GEO["hover_vane_y"][0]) / 1000 * ch / 1000
f25 = q_s * s_sub * 2.65 * math.radians(25)
print(f"  25 deg vanes in hover: normal force {f25:.1f} N (={f25/W:.2f} W!) but pitch moment only {f25*(x_cp-x_cg)/1000:+.2f} N m "
      f"with REVERSED sign vs code's +1.55 N m. It is a translation control, not a pitch control.")

# ---------------------------------------------------------------------------------------------------
hdr("3. PITCH INERTIA Iyy (stability.mass_properties uses a 40 mm x-extent for EVERY lump)")
iyy_add_batt = 1.418 * ((0.216**2 - 0.040**2) / 12)
print(f"  battery is 216 mm long: missing m*L^2/12 = {iyy_add_batt:.4f} kg m2 on a quoted Iyy of 0.0156 (+{iyy_add_batt/0.0156*100:.0f} %)")
print("  wing panels/body parts (chords 76..532 mm) are also treated as 40 mm long -> Iyy is low by ~40-60 %;"
      " short-period, hover pitch accel (99 rad/s2) and transition results all shift.")

# ---------------------------------------------------------------------------------------------------
hdr("4. SURFACE FINISH: NeuralFoil free transition (n_crit 9) vs a printed LW-PLA wing")
try:
    import aerosandbox as asb
    import neuralfoil as nf
    af = asb.Airfoil("kestrel", coordinates=B.wing_coords(1.0))
    re = 16.3 * 0.1314 / A.NU
    rows = []
    for label, kw in (("smooth, n_crit 9 (repo)", dict(n_crit=9)), ("painted/sanded, n_crit 5", dict(n_crit=5)),
                      ("raw FDM layer lines, trip 10%/10%", dict(n_crit=9, xtr_upper=0.10, xtr_lower=0.10)),
                      ("trip 5%/5% (worst)", dict(n_crit=9, xtr_upper=0.05, xtr_lower=0.05))):
        al = np.linspace(-2, 10, 49)
        a = nf.get_aero_from_airfoil(af, alpha=al, Re=re, model_size="large", **kw)
        cd_at = float(np.interp(0.46, np.ravel(a["CL"]), np.ravel(a["CD"])))
        a2 = nf.get_aero_from_airfoil(af, alpha=al, Re=87.75 * 0.1314 / A.NU, model_size="large", **kw)
        cd_hi = float(np.interp(0.02, np.ravel(a2["CL"]), np.ravel(a2["CD"])))
        rows.append((label, cd_at, cd_hi))
    base = rows[0]
    for label, cd_c, cd_h in rows:
        print(f"  {label:36s} cd(CL .46, Re {re/1e3:.0f}k) {cd_c:.5f} ({cd_c/base[1]-1:+5.0%})   cd at Vmax {cd_h:.5f} ({cd_h/base[2]-1:+5.0%})")
    worst_ratio = rows[2][1] / base[1]
except Exception as e:  # pragma: no cover
    print("  NeuralFoil check skipped:", e)
    worst_ratio = 1.4

# ---------------------------------------------------------------------------------------------------
hdr("5. HEADLINE SENSITIVITY (harness model, same build)")
base_r = h.evaluate(h.CURRENT, full=True)
for label, kw in (("as published", {}), ("drag x1.25 (finish+gaps+trim+interference)", dict(drag_scale=1.25)),
                  ("drag x1.25, e 0.75", dict(drag_scale=1.25, e_osw=0.75)),
                  ("drag x1.35, e 0.75, cp x1.15, usable 0.80", dict(drag_scale=1.35, e_osw=0.75, cp_derate=1.15, usable=0.80))):
    r = h.evaluate(h.CURRENT, full=True, **kw)
    print(f"  {label:46s} Vmax {r['vmax']*3.6:4.0f} km/h | loiter {r['endurance_min']:4.0f} min | range {r['range_km']:4.0f} km | L/D {r['ld']:4.1f}")
ac = h.make_aircraft(h.CURRENT)
vv = np.arange(60, 100, 0.25)
for vc in (4.2, 3.9, 3.6, 3.3):
    t, _ = ac.full_throttle(vv, v_cell=vc)
    dd, _ = ac.drag(vv)
    ok = vv[t >= dd]
    print(f"  level Vmax at {vc:.1f} V/cell resting: {ok.max()*3.6 if ok.size else float('nan'):5.0f} km/h")
print("  -> 316 km/h is the harness value at 3.9 V/cell (first ~30 % of the pack); envelope.py's 299 km/h is at 3.6 V.")

# ---------------------------------------------------------------------------------------------------
hdr("6. POWER-TRAIN THERMAL AND CURRENT (their own models)")
import optimize as opt  # noqa: E402
for v, lbl in ((0.0, "static full throttle"), (87.75, "Vmax level")):
    s = ac.full_throttle_state(v, 3.9)
    pe = opt.esc_loss_w(ac.vbat, s["motor_a"])
    pm = s["motor_a"]**2 * ac.rm + ac.i0 * s["motor_v"]
    cell_a = s["pack_a"] / 3
    print(f"  {lbl:20s}: motor {s['motor_a']:.1f} A -> motor heat {pm:4.0f} W into 66 g (~{pm/30:.1f} K/s adiabatic); "
          f"ESC model loss {pe:4.0f} W in a 9 g board; pack {s['pack_a']:.0f} A = {cell_a:.0f} A/cell "
          f"-> {cell_a**2*0.0128:.0f} W/cell, ~{cell_a**2*0.0128/(70*1.0):.2f} K/s/cell in a sealed bay")
print(f"  single XT60: ~60 A continuous rating vs {ac.full_throttle_state(0.0, 3.3)['pack_a']:.0f} A static at 3.3 V/cell -> use XT90/AS150 + 10 AWG")
print("  FC: printable.py itself labels the SpeedyBee Wing Mini '80A sensor': static 88-119 A and Vmax ~114 A exceed it ->"
      " route pack power to the ESCs directly, use an external 200 A shunt/hall sensor.")

# ---------------------------------------------------------------------------------------------------
hdr("7. AEROELASTICITY (first-order; ASSUMED foamed LW-PLA E=1.0 GPa, G=0.37 GPa, 0.6 mm skin; pultruded tube G=5 GPa)")
G_skin, t_skin, G_tube = 0.37e9, 0.6e-3, 5e9


def gj(y_mm):
    cc = B.chord(y_mm) / 1000
    tt = 0.093 * cc * (1 - 0.08 * min(1, max(0, (y_mm - 106) / 794)))
    a_cell = 0.62 * cc * tt                                          # cell closed at the 70 % hinge line
    perim = 2 * 0.70 * cc + 2 * tt
    k = 4 * a_cell**2 * G_skin * t_skin / perim
    j = math.pi / 32 * ((12**4 - 9**4) if y_mm < 330 else (8**4 - 6**4) if y_mm < 600 else (6**4 - 4**4)) * 1e-12
    return k + G_tube * j


ys = np.linspace(106, 640, 300)
k_theta = 1 / np.trapezoid([1 / gj(v) for v in ys], ys / 1000)
c_e = B.chord(640) / 1000
s_e = (GEO["elevon_y"][1] - GEO["elevon_y"][0]) / 1000 * c_e
cla = 2 * math.pi * math.cos(math.radians(GEO["sweep_deg"]))
cl_d, cm_d = dcl * 0.75, dcm * 0.75                                    # 3D/gap knockdown on thin-aerofoil flap
e_arm = (0.40 - 0.25) * c_e                                            # EA ~40 % c -> aerodynamic centre offset
q_rev = k_theta * cl_d / (s_e * c_e * cla * abs(cm_d) + 1e-12)
v_rev = math.sqrt(2 * q_rev / RHO)
print(f"  GJ at y=200/450/700 mm: {gj(200):.1f} / {gj(450):.1f} / {gj(700):.1f} N m2; root-to-elevon K_theta {k_theta:.1f} N m/rad")
print(f"  elevon (roll) reversal speed, typical-section: ~{v_rev:.0f} m/s ({v_rev*3.6:.0f} km/h)  vs Vmax 299-316 km/h")
for vk in (60, 75, 87.75):
    print(f"     elevon effectiveness at {vk*3.6:3.0f} km/h ~ {max(0.0, 1 - (vk / v_rev)**2):.0%} of rigid")
print(f"  -> readiness.py's 311 deg/s roll rate at top speed is a RIGID-wing number.")

# typical-section 2-DOF flutter (Theodorsen, k-method, Scanlan-Rosenbaum coefficients), Rayleigh modal frequencies
from scipy.special import hankel2  # noqa: E402


def theod(k):
    return hankel2(1, k) / (hankel2(1, k) + 1j * hankel2(0, k))


def k_flutter(mu, a, x_a, r2, sig, g_s=0.02):
    """Return (V/(b w_a), k) of the lowest flutter crossing, or (None, None)."""
    prev = None
    for k in np.geomspace(3.0, 0.005, 3000):
        C = theod(k)
        Lh, La, Mh, Ma = 1 - 2j * C / k, 0.5 - 1j * (1 + 2 * C) / k - 2 * C / k**2, 0.5, 3 / 8 - 1j / k
        A11, A12 = Lh, La - (0.5 + a) * Lh
        A21 = Mh - (0.5 + a) * Lh
        A22 = Ma - (0.5 + a) * (La + Mh) + (0.5 + a)**2 * Lh
        P, Q, R, S = mu + A11, mu * sig**2, mu * r2 + A22, mu * r2
        roots = np.roots([Q * S, -(P * S + Q * R), P * R - (mu * x_a + A12) * (mu * x_a + A21)])
        out = []
        for Z in roots:
            if Z.real > 0:
                out.append((1 / math.sqrt(Z.real), Z.imag / Z.real))       # omega/omega_a, required damping g
        out.sort()
        gmax = max((g for _, g in out), default=-1)
        if prev is not None and prev < g_s <= gmax:
            w = [wr for wr, g in out if g == gmax][0]
            return w / k, k
        prev = gmax
    return None, None


vf_ref, _ = k_flutter(20.0, -0.2, 0.1, 6 / 25, 0.4, g_s=0.0)
print(f"  solver check, Hodges & Pierce case (mu 20, a -0.2, x_a 0.1, r2 0.24, sigma 0.4): U/(b w_a) = {vf_ref:.2f}  (textbook ~2.2)")

yy = np.linspace(GEO["blend_y"], GEO["half"], 400)
Lt = (yy[-1] - yy[0]) / 1000
s_ = (yy - yy[0]) / 1000 / Lt


def m_span(y_mm):
    skin = 0.183 if y_mm < 225 else 0.160 if y_mm < 365 else 0.124 if y_mm < 530 else 0.073 if y_mm < 710 else 0.077
    tube = 0.0816 if y_mm < 330 else 0.0363 + (0.026 if y_mm > 530 else 0) if y_mm < 600 else 0.026
    rear = 0.026 if y_mm < 520 else 0.0
    elev = 0.022 if GEO["elevon_y"][0] < y_mm < GEO["elevon_y"][1] else 0.0
    return skin + tube + rear + elev


def ei(y_mm):
    I = (12**4 - 9**4) if y_mm < 330 else (8**4 - 6**4) if y_mm < 600 else (6**4 - 4**4)
    return 120e9 * math.pi / 64 * I * 1e-12 + (0.15 * 120e9 * math.pi / 64 * (6**4 - 4**4) * 1e-12 if y_mm < 520 else 0)


r2 = 0.30
mm = np.array([m_span(v) for v in yy]); bb = np.array([B.chord(v) / 2000 for v in yy])
GJ = np.array([gj(v) for v in yy]); EIv = np.array([ei(v) for v in yy])
phi_t, dphi_t = np.sin(np.pi * s_ / 2), np.pi / (2 * Lt) * np.cos(np.pi * s_ / 2)
w_a = math.sqrt(np.trapezoid(GJ * dphi_t**2, yy) / np.trapezoid(mm * r2 * bb**2 * phi_t**2, yy))
phi_b = 1 - np.cos(np.pi * s_ / 2); d2 = (np.pi / (2 * Lt))**2 * np.cos(np.pi * s_ / 2)
w_h = math.sqrt(np.trapezoid(EIv * d2**2, yy) / np.trapezoid(mm * phi_b**2, yy))
y75 = 0.75 * GEO["half"]
b75, m75 = B.chord(y75) / 2000, m_span(y75)
mu75 = m75 / (math.pi * RHO * b75**2)
print(f"  Rayleigh modes: 1st bending {w_h/2/math.pi:.1f} Hz, 1st torsion {w_a/2/math.pi:.1f} Hz; station y={y75:.0f} mm b={b75*1000:.0f} mm mu={mu75:.1f}")
for lbl, xa, a_ in (("CG 9 % c aft of EA (as built, no balance)", 0.18, -0.24), ("CG 4 % c aft of EA", 0.08, -0.24),
                    ("mass-balanced elevon, CG on EA", 0.0, -0.24)):
    U, kf = k_flutter(mu75, a_, xa, r2, w_h / w_a)
    if U is None:
        print(f"    {lbl:42s}: no bending-torsion crossing found")
    else:
        vf = U * b75 * w_a
        print(f"    {lbl:42s}: flutter ~{vf:4.0f} m/s ({vf*3.6:4.0f} km/h), reduced freq k {kf:.2f}")
w_h_hz = w_h / 2 / math.pi
print("  (typical-section ignores sweep, which usually RAISES bending-torsion flutter speed for aft-swept wings, and ignores the")
print("   much softer elevon-hinge (tape + 1 mm pushrod) mode, which usually LOWERS it. Ground vibration test before any dash.)")
f_sp_vmax = 17.93
print(f"  body-freedom check: short period 3.16 Hz @16 m/s -> 17.9 Hz @88 m/s (stability.py) crosses the ~{w_h_hz:.0f} Hz "
      f"first wing bending near {w_h_hz/0.204:.0f} m/s ({w_h_hz/0.204*3.6:.0f} km/h): classic flying-wing body-freedom flutter zone.")
q_vmax = 0.5 * RHO * 87.75**2
s_v = 0.188 * 0.3 * B.chord(224) / 1000
hm = q_vmax * s_v * 0.3 * B.chord(224) / 1000 * 0.6 * math.radians(10)
print(f"  hover vanes at Vmax: 10 deg needs {hm/0.0981:.1f} kg cm vs ES9051 0.8 kg cm -> vanes back-drive / flutter; no mass balance anywhere.")

# ---------------------------------------------------------------------------------------------------
hdr("8. THINGS THE PUBLISHED FILES ALREADY SAY (but the headline ignores)")
import json  # noqa: E402
env = json.loads((ROOT / "sim" / "envelope.json").read_text())
tr = json.loads((ROOT / "readiness" / "transition.json").read_text())
cfd = json.loads((ROOT / "readiness" / "cfd_summary_stealth.json").read_text())
print(f"  vertical idle dive {env['v_dive_vertical']:.1f} m/s > harness V_NE 95 m/s (no airbrake / prop-brake logic)")
print(f"  inbound transition ends at pitch {tr['inbound']['final_pitch_deg']} deg, Vx {tr['inbound']['final_vx_ms']} m/s: the sim LOOPED; aircraft lost")
print(f"  SU2 Euler CD = {[round(p['CD'], 4) for p in cfd['points']]}: negative drag ~ -0.027 (inviscid CDi should be ~+0.007 at CL .5)"
      f" -> surface integration/mesh is broken; the 13.3 % CFD margin is not trustworthy until CD(alpha=0) ~ 0")
print(f"  Dutch roll zeta 0.10 cruise / 0.09 top speed: below MIL-F-8785C Cat A Level 1 (0.19); no rudder, no yaw damper")
print(f"  firmware/kestrel560_tailsitter.param is still 3S/5000 mAh: BATT_LOW_VOLT 9.9 V never trips on 6S; Q_M_BAT_CURR_MAX 40 A caps"
      f" VTOL power at ~{40*21.6/725:.2f}x hover power (T/W ~{(40*21.6/725)**(2/3):.2f}), Q_TAILSIT_DSKLD 11 vs real {M/(2*math.pi*0.1016**2):.0f} kg/m2")
