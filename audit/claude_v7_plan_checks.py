"""Stress-test of Devin's proposed Kestrel-X Stealth v7 architecture (Claude, 2026-11-20).

Run from the repo root:  .venv/Scripts/python.exe audit/claude_v7_plan_checks.py
Read-only: adds candidate parts to the in-memory catalogue only; writes nothing.
Inputs marked QUOTED are Devin's datasheet numbers (not independently verified: web search unavailable).
Inputs marked ASSUMED are engineering estimates that must be replaced by coupon tests or datasheets.
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


def hdr(t):
    print("\n" + "=" * 100 + f"\n{t}\n" + "=" * 100)


# ---------------------------------------------------------------------------------------------------
hdr("1. CARBON SKIN: mass, torsional stiffness, flutter/reversal, Tg, RF")
ac6 = h.make_aircraft(h.CURRENT)
s_wet = 2.04 * ac6.s                                   # thin wing, both faces (+2 % for curvature)
cloth, vf, rho_f, rho_r = 90.0, 0.42, 1.78, 1.15       # g/m2, hand lay-up + vacuum bag on a soft core (ASSUMED Vf)
resin = cloth * (rho_r / rho_f) * (1 - vf) / vf
lam = cloth + resin
t_lam = cloth / (rho_f * vf) / 1000                    # mm
print(f"  wetted area wing+body ~{s_wet:.2f} m2; laminate {cloth:.0f} g cloth + {resin:.0f} g resin = {lam:.0f} g/m2, t = {t_lam:.3f} mm")
print(f"  laminate alone {s_wet*lam:.0f} g, +10 % overlaps/LE wraps, + ~50 g/m2 filler/primer for a laminar finish"
      f" -> {s_wet*lam*1.10 + s_wet*50:.0f} g  (plan says +100 g)")

G45 = 20e9                                             # ASSUMED G12 of a +/-45 carbon laminate at Vf 0.42 (19-24 GPa)
G_skin6, t_skin6 = 0.37e9, 0.6e-3


def gj(y_mm, g_t):
    cc = B.chord(y_mm) / 1000
    tt = 0.093 * cc
    a_cell = 0.62 * cc * tt
    perim = 2 * 0.70 * cc + 2 * tt
    return 4 * a_cell**2 * g_t / perim


gt6, gt7 = G_skin6 * t_skin6, G_skin6 * t_skin6 + G45 * t_lam / 1000
print(f"  skin shear flow stiffness G*t: v6 LW-PLA {gt6/1e6:.2f} MN/m -> v7 carbon+core {gt7/1e6:.2f} MN/m = x{gt7/gt6:.1f}")
print(f"  closed-cell GJ at y=450: {gj(450, gt6):.1f} -> {gj(450, gt7):.1f} N m2  (but ONLY if the cell is closed: the elevon/vane")
print(f"  cut-outs, hatch and the tilt-nacelle slots open the cell; open-section GJ is ~100x lower -> skin must wrap continuously")
print(f"  over a rear shear web at the hinge line, and the hatch must be a stressed, fastened panel)")
fl_scale = math.sqrt(gt7 / gt6)
print(f"  flutter speed ~ sqrt(GJ) -> x{fl_scale:.1f}: 203-265 km/h becomes ~{203*fl_scale:.0f}-{265*fl_scale:.0f} km/h (typical-section, +/-40 %)")
print(f"  reversal ~ sqrt(K_theta) -> ~{352*fl_scale:.0f} km/h. Plan's '>500 km/h flutter, >900 km/h reversal' is the right order of")
print("  magnitude, but cannot set V_NE without a GVT: the elevon-on-servo rotation mode and body-freedom are NOT fixed by GJ.")
e45 = 16e9                                             # ASSUMED Ex of +/-45 carbon (shear-dominated, low axial)
for y_mm in (200, 450, 700):
    cc = B.chord(y_mm) / 1000
    hh = 0.093 * cc
    ei_skin = e45 * 2 * (0.55 * cc * t_lam / 1000 * (hh / 2) ** 2)
    od, idd = (12, 9) if y_mm < 330 else (8, 6) if y_mm < 600 else (6, 4)
    ei_tube = 120e9 * math.pi / 64 * (od**4 - idd**4) * 1e-12
    print(f"  y={y_mm}: bending EI tube {ei_tube:.0f} + +/-45 skin {ei_skin:.0f} N m2 -> skin adds {ei_skin/ei_tube:+.0%} "
          f"(torsion/bending frequency ratio moves, re-run the mode solve)")
print("  Tg TRAP: a 100 C Tg needs a ~80-100 C post-cure; LW-PLA softens at ~55-60 C, so the core slumps. A room-temp cure")
print("  gives Tg ~50-60 C. Either accept ~55 C Tg (white/light paint mandatory, no hot-car storage) or cure on a")
print("  PETG/ASA/PC core, or use a 'low-temp post-cure' system and validate with a DMA/heat-deflection coupon.")
print("  VACUUM-BAG TRAP: 0.8 bar on a 4 % infill, 0.5 mm foamed wall is ~8 t/m2; panels will dish between ribs ->")
print("  use 0.3-0.5 bar, or print 8-10 % infill under the skin, or wet-layup + peel ply + spray-on release film.")
print("  RF/STEALTH TRAP: carbon is a conductor. A full carbon skin (a) blinds the internal GPS, ELRS, VTX, telemetry and")
print("  Remote-ID antennas, (b) turns every panel gap, hatch edge and nacelle slot into a radar scatterer. Use E-glass or")
print("  aramid (RF-transparent) over the antenna bays and the nose, or a glass skin + a carbon-only torque box.")
gt_glass = G_skin6 * t_skin6 + 4.5e9 * 0.10e-3 * 2
print(f"  for comparison, 2 x 49 g/m2 +/-45 E-glass: G*t {gt_glass/1e6:.2f} MN/m (x{gt_glass/gt6:.1f}), RF-transparent, similar mass.")

# ---------------------------------------------------------------------------------------------------
hdr("2. SPARS: 12x10 vs 12x9, telescoping fits, and the GUST loads that actually size them")


def z_mod(od, idd):
    return math.pi * (od**4 - idd**4) / 64 / (od / 2) * 1e-9


z129, z1210 = z_mod(12, 9), z_mod(12, 10)
print(f"  section modulus 12x10 = {z1210*1e9:.0f} mm3 vs v6 12x9 = {z129*1e9:.0f} mm3 -> 12x10 is {1 - z1210/z129:.0%} WEAKER at the root")
for outer, inner in (((12, 10), 10), ((10, 8), 8), ((8, 6), 6)):
    print(f"  {inner} mm OD into {outer[0]}x{outer[1]} (ID {outer[1]}): nominal radial gap {(outer[1]-inner)/2:.2f} mm "
          f"-> a 0.1 mm bondline needs ID - OD = 0.20 mm; buy MATCHED telescoping sets or sand/sleeve, check with gauge pins")
sig = 600e6 / 1.5
# moment per g scales with W and span; re-use the v6 elliptic estimate scaled to a v7 mass
for m_kg, span in ((2.25, 1.8), (3.4, 2.2)):
    w = m_kg * G
    half = span / 2
    y = np.linspace(0, half, 2001)
    ell = np.sqrt(np.clip(1 - (y / half) ** 2, 0, None))
    ell = ell / np.trapezoid(ell, y) * w / 2
    m_root = np.trapezoid(ell * y, y) - 0.10 * G * 0.225 * (span / 1.8)
    m15 = 15 * m_root
    need = [f"{od}x{idd}" for od, idd in ((12, 10), (12, 9), (14, 12), (14, 11), (16, 14), (16, 13))
            if sig * z_mod(od, idd) >= m15]
    print(f"  {m_kg:.2f} kg, {span} m span: root M = {m_root:.2f} N m/g -> 15 g needs {m15:.0f} N m; tubes that pass at 400 MPa: {need}")

ac = ac6
cla = 2 * math.pi * ac.ar / (2 + math.sqrt(4 + ac.ar**2))   # Helmbold, finite wing
for m_kg, s_ref, lbl in ((2.25, ac.s, "v6"), (3.4, 0.33, "v7 8S1P SA30 class")):
    ws = m_kg * G / s_ref
    mug = 2 * ws / (RHO * ac.mac * cla * G)
    kg = 0.88 * mug / (5.3 + mug)
    for v_kmh in (200, 316, 400):
        v = v_kmh / 3.6
        for ude in (7.6, 15.2):
            dn = kg * RHO * v * cla * ude / (2 * ws)
            print(f"  {lbl:20s} W/S {ws:5.1f} N/m2  V {v_kmh} km/h, {ude:4.1f} m/s gust (CS-23: 7.6 = 25 ft/s at V_D, 15.2 = 50 ft/s at V_C): dn = {dn:4.1f} g")
print("  -> at 300-400 km/h a light wing sees 15-30 g in a CS-23 50 ft/s V_C gust. A 15 g structure needs a rough-air")
print("     speed V_B ~ 150-200 km/h and a firmware airspeed limit; dash only in smooth air. This, not flutter, sizes the spar.")

# ---------------------------------------------------------------------------------------------------
hdr("3. TILT-NACELLE PITCH/YAW CONTROL")
x_cg = A.cg_x_mm()
arm = (GEO["motor_x"] - x_cg) / 1000
m3 = 3.4
w = m3 * G
for tilt in (10, 18):
    print(f"  {tilt:2d} deg tilt, thrust = W ({m3} kg), arm {arm*1000:.0f} mm: pitch {w*math.sin(math.radians(tilt))*arm:.2f} N m, "
          f"diff-tilt heading yaw {w*math.sin(math.radians(tilt))*GEO['motor_y']/1000:.2f} N m (motor_y {GEO['motor_y']:.0f} mm),"
          f" side-force loss of lift {1-math.cos(math.radians(tilt)):.1%}")
for d_in, g_p, rpm, lbl in ((8, 11.2, 12500, "8 in hover"), (8, 11.2, 22800, "8 in dash"), (10, 20.0, 15000, "10 in dash")):
    ip = g_p / 1000 * (d_in * 0.0254) ** 2 / 12
    hmom = ip * rpm * 2 * math.pi / 60
    print(f"  {lbl:11s}: prop H = {hmom:.3f} N m s; 3 rad/s airframe pitch/yaw rate -> {3*hmom:.2f} N m ({3*hmom/0.0981:.1f} kg cm)"
          f" on EACH tilt shaft/lock; a 6 rad/s tilt slew -> {6*hmom:.2f} N m")
print("  CW+CCW props cancel gyro torque on the airframe for SYMMETRIC (pitch) tilt, but differential (heading) tilt ADDS")
print("  the two gyro torques -> a coupled moment about the third axis; feed it forward or the hover yaw loop cross-couples.")
m_pod = (113 + 20 + 15) / 1000
print(f"  15 g manoeuvre on a {m_pod*1000:.0f} g motor+prop+mount group with a 15 mm CG offset from the tilt axis: "
      f"{15*G*m_pod*0.015:.2f} N m. A 0-deg 'hard stop' only resists ONE direction -> need a positive lock or an over-centre")
print("  linkage; KST X08 Plus ~0.46 N m stall is then not the structural path. Design load on the tilt pivot ~1 N m + 30 N.")
print("  ArduPlane already supports vectored tailsitters (Q_TAILSIT_ENABLE + SERVOn_FUNCTION 75/76 tilt-motor L/R,")
print("  Q_TAILSIT_VHGAIN/VFGAIN/VHPOW) -> use that, verify against your ArduPlane version; do not write a custom mixer.")
print("  Elevons stay outboard (y 536-744) of the 8-in prop wash (motor y 225 +/- 102): no elevon authority in hover,")
print("  so tilt is the ONLY pitch control in hover. Tilt servo failure = loss of aircraft -> need dual-path or fail-safe.")

# ---------------------------------------------------------------------------------------------------
hdr("4. QUOTED CELL DATA: internal consistency and what each pack can actually deliver")
cells = {  # QUOTED by Devin: Ah, Wh/kg, cont A, g (derived if missing), ACIR mOhm (ASSUMED where not quoted)
    "SA30 26Ah": dict(ah=26.0, whkg=340, ia=103, g=266, ir=8.0),
    "SA08 11.05Ah": dict(ah=11.05, whkg=345, ia=54, g=None, ir=3.0),
    "SA88 10.2Ah": dict(ah=10.2, whkg=365, ia=102, g=None, ir=2.5),
    "P60B 21700 (v6)": dict(ah=6.0, whkg=None, ia=60, g=70, ir=8.7),
}
for n, c in cells.items():
    wh = c["ah"] * 3.6
    g = c["g"] or wh / c["whkg"] * 1000
    vnom = c["whkg"] * g / 1000 / c["ah"] if c["whkg"] else 3.6
    c.update(wh=wh, g_eff=g, vnom=vnom)
    heat = c["ia"] ** 2 * c["ir"] / 1000
    print(f"  {n:16s}: {c['ah']:5.2f} Ah, ~{wh:5.1f} Wh at 3.6 V, {g:5.0f} g -> implied V_nom {vnom:.2f} V; at its rated {c['ia']} A"
          f" with {c['ir']:.1f} mOhm: drop {c['ia']*c['ir']/1000:.2f} V, {heat:4.0f} W/cell")
print("  SA30: 8 mOhm at 103 A = 0.82 V sag and 85 W of heat PER CELL. That is not a 4C cell; either the ACIR is ~1-2 mOhm")
print("  or the 103 A is a 10 s pulse. Get the DCIR vs SOC and temperature curve and the continuous rating at 25 C before")
print("  committing. Silicon-anode 340-365 Wh/kg figures are usually quoted at 0.2-0.5C; check energy at YOUR loiter and dash C.")

packs = [("6S1P SA30", "SA30 26Ah", 6, 1), ("8S1P SA30", "SA30 26Ah", 8, 1), ("8S2P SA08", "SA08 11.05Ah", 8, 2),
         ("8S2P SA88", "SA88 10.2Ah", 8, 2), ("6S3P P60B (v6)", "P60B 21700 (v6)", 6, 3)]
print(f"  {'pack':16s} {'Wh':>5s} {'cells g':>8s} {'pack g*':>8s} {'I cont':>7s} {'P cont kW':>9s} {'R pack mOhm':>11s}")
for lbl, cn, ns, npar in packs:
    c = cells[cn]
    wh = ns * npar * c["ah"] * c["vnom"]
    g_cells = ns * npar * c["g_eff"]
    g_pack = g_cells * 1.06 + (30 if "SA" in cn else 0)            # +6 % tabs/bus/wrap, +30 g compression plates
    i_c = npar * c["ia"]
    p_c = i_c * ns * (c["vnom"] - 0.0)
    print(f"  {lbl:16s} {wh:5.0f} {g_cells:8.0f} {g_pack:8.0f} {i_c:6.0f}A {p_c/1000:9.2f} {ns*c['ir']/npar:11.1f}")
print("  * +6 % tabs/bus bars/wrap, +30 g pouch compression plates. Pouch cells are not self-contained: they need")
print("    restraint, puncture protection and tab strain relief. A 30 g plate is the right instinct for ANY Si pouch (Si")
print("    swells ~10-30 % more than graphite); use Amprius's stated stack pressure for each SKU, not only for SA107.")

# ---------------------------------------------------------------------------------------------------
hdr("5. PERFORMANCE SWEEP with the repo's own harness (parametric airframe, candidate parts added in memory)")
CAT = h.CAT
for lbl, cn, ns, npar in packs[:4]:
    c = cells[cn]
    wh = ns * npar * c["ah"] * c["vnom"]
    CAT["batteries"][lbl] = dict(chem="liion", ns=ns, np=npar, wh=wh, honesty=0.93, g=ns * npar * c["g_eff"] * 1.06 + 30,
                                 r=ns * c["ir"] / npar / 1000 * 1.6, imax=npar * c["ia"], dims=[150, 110, 40], price=0,
                                 aliexpress=True)                # honesty 0.93: Si energy at real C-rate; r x1.6 = DCIR/ACIR
CAT["motors"]["T-Motor Velox V3115 640KV (QUOTED)"] = dict(kv=640, r=0.0671, i0=1.10, g=113, imax=62, smax=8, price=0)
CAT["motors"]["T-Motor Velox V3115 900KV (QUOTED)"] = dict(kv=900, r=0.0381, i0=1.6, g=113, imax=83, smax=6, price=0)
CAT["escs"]["generic 8S 100A ESC (ASSUMED 30 g)"] = dict(amps=100, g=30.0, price=0)
CAT["fcs"]["Matek H743-WING V3 (QUOTED)"] = dict(CAT["fcs"][h.BASELINE["fc"]], g=30, price=0)
h.OPTIONS.update(free_bay=True, off_aliexpress=True)
# v7 structural and control adds over the parametric v6 airframe (grams)
extra = 140 + 22 + 14 + (2 * 9 + 24) - 25 + 25 + 40       # skin+finish, root box, elevon balance, tilt, -vanes, XT90/10AWG/sensor, bigger fins
print(f"  v7 mass adds applied to every case: +{extra} g (skin+finish 140, root box 22, balance 14, tilt 42, -vanes 25,"
      f" XT90/10 AWG/200 A sensor 25, bigger V-tails 40)")
base = dict(h.BASELINE)
cases = [
    ("v6 baseline (no v7 adds)", dict(), 0),
    ("v6 parts + v7 adds", dict(), extra),
    ("F100 1100KV + 8x9, P60B 6S3P", dict(motor="T-Motor F100 2810 1100KV"), extra),
]
for pk in ("6S1P SA30", "8S1P SA30", "8S2P SA08", "8S2P SA88"):
    if pk.startswith("6S"):
        cases += [(f"{pk}, F100 1350KV 8x9", dict(battery=pk), extra),
                  (f"{pk}, F100 1100KV 8x9", dict(battery=pk, motor="T-Motor F100 2810 1100KV"), extra),
                  (f"{pk}, Velox 900KV 9x9E", dict(battery=pk, motor="T-Motor Velox V3115 900KV (QUOTED)", prop="9x9E"), extra)]
    else:
        cases += [(f"{pk}, Velox 640KV 9x9E", dict(battery=pk, motor="T-Motor Velox V3115 640KV (QUOTED)", prop="9x9E",
                                                    esc="generic 8S 100A ESC (ASSUMED 30 g)"), extra),
                  (f"{pk}, Velox 640KV 10x10E", dict(battery=pk, motor="T-Motor Velox V3115 640KV (QUOTED)", prop="10x10E",
                                                      esc="generic 8S 100A ESC (ASSUMED 30 g)"), extra)]
print(f"  {'case':38s} {'span':>4s} {'AUW g':>6s} {'T/W':>4s} {'Vthr km/h':>9s} {'loiter min':>10s} {'range km':>8s} {'L/D':>5s} {'Ihov+A':>6s}  flags")
for span in (1.8, 2.2):
    for lbl, d, mx in cases:
        des = dict(base, **d, span=span)
        if span != 1.8:
            des.update(cr=base["cr"], ct=base["ct"])
        try:
            r = h.evaluate(des, full=True, mass_extra=mx)
            a_ = h.make_aircraft(des, mass_extra=mx)
            t0, _ = a_.full_throttle(np.zeros(1), v_cell=3.6)
            tw = float(t0[0]) / a_.w
            i_h = (r["p_hover"] - a_.avionics_w) / a_.vbat
            flags = "; ".join(r["reasons"])[:60]
            print(f"  {lbl:38s} {span:4.1f} {r['mass_g']:6.0f} {tw:4.2f} {r['v_thrust']*3.6:9.0f} {r['endurance_min']:10.0f}"
                  f" {r['range_km']:8.0f} {r['ld']:5.1f} {i_h:6.0f}  {flags}")
        except Exception as e:  # pragma: no cover
            print(f"  {lbl:38s} {span:4.1f} failed: {e}")
print("  Vthr is thrust-limited level speed at 3.6 V/cell (V_NE cap removed); loiter/range at ZERO reserve, 60 s hover,")
print("  smooth-finish drag (v7 skin is assumed to deliver it). Subtract ~25 % for a reserve-rule mission.")
print("  The 2.2 m rows keep v6 chords (AR up, S up), body unchanged; spar/skin mass grows only through the per-m2 term.")

# ---------------------------------------------------------------------------------------------------
hdr("6. PROP RPM CEILING (APC limits QUOTED by Devin: 150,000/D thin-electric, 225,000/D sport) -> hard Vmax cap")
import optimize as opt  # noqa: E402
print(f"  {'prop':8s} {'type':5s} {'rpm lim':>7s} {'J(CT=0)':>7s} {'V(CT=0) km/h':>12s} {'V(CT=40% static)':>16s} {'helical tip M':>13s}")
for key in ("8x9", "7x10", "9x10", "8x10E", "9x9E", "10x10E", "11x10E", "12x10E", "8x8E"):
    d_in = opt.APC[key]["d_in"]
    elec = key.endswith("E")
    rpm = (150000 if elec else 225000) / d_in
    dd, J, CT, CP = opt.prop_arrays(key)
    j0 = float(np.interp(0.0, CT[::-1], J[::-1]))
    j40 = float(np.interp(0.40 * CT[0], CT[::-1], J[::-1]))
    n = rpm / 60
    v0, v40 = j0 * n * dd, j40 * n * dd
    tip = math.hypot(math.pi * dd * n, v40) / 340.3
    print(f"  {key:8s} {'E' if elec else 'sport':5s} {rpm:7.0f} {j0:7.2f} {v0*3.6:12.0f} {v40*3.6:16.0f} {tip:13.2f}")
print("  Level flight needs CT well above zero (drag/2 per prop), so V(CT=40 %) is the realistic ceiling at the APC rpm limit.")
print("  -> 350-400 km/h needs P/D >= ~1.4 sport props at tip Mach ~0.9: big compressibility loss, loud (stealth), and a")
print("     P/D 1.4 blade is deep-stalled at static -> poor hover thrust. One fixed-pitch prop cannot do T/W 2 AND 400 km/h.")

hdr("7. RE-RUN OF THE BEST CANDIDATES WITH THE CORRECT APC RPM LIMITS (harness uses 190,000/D for every prop)")
_orig_init = h.Aircraft.__init__


def _init(self, design, **assume):
    _orig_init(self, design, **assume)
    lim = 150000.0 if self.prop_data.endswith("E") else 225000.0
    self.n_max = min(lim / self.d_in / 60.0, 265.0 / (math.pi * self.d))


h.Aircraft.__init__ = _init
h._CACHE.clear()
for span in (1.8, 2.2):
    for lbl, d, mx in cases:
        if not any(s in lbl for s in ("baseline", "SA88", "SA08")):
            continue
        des = dict(base, **d, span=span)
        r = h.evaluate(des, full=True, mass_extra=mx)
        a_ = h.make_aircraft(des, mass_extra=mx)
        t0, _ = a_.full_throttle(np.zeros(1), v_cell=3.6)
        print(f"  {lbl:38s} {span:4.1f}  AUW {r['mass_g']:5.0f} g  T/W {float(t0[0])/a_.w:4.2f}  Vthr {r['v_thrust']*3.6:4.0f} km/h"
              f"  loiter {r['endurance_min']:4.0f} min  range {r['range_km']:4.0f} km  {'; '.join(r['reasons'])[:50]}")
