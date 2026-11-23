"""Review of Devin's sim/hillclimb_v8_budget.py (Claude, 2026-11-20). Read-only.
Run: .venv/Scripts/python.exe audit/claude_v8_budget_checks.py"""
import json, math, pathlib, sys
import numpy as np
ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))
import hillclimb_v7 as h7  # noqa: E402
CAT = json.loads((ROOT / "sim" / "catalog.json").read_text())
APC = json.loads((ROOT / "sim" / "apc_props.json").read_text())
RHO = 1.225
BATT = {"P45B 6S3P": dict(ns=6, r=0.020 + 0.010, imax=135), "P45B 6S2P": dict(ns=6, r=0.030 + 0.010, imax=90)}

def full_throttle_static(motor, prop, batt, v_cell, cpd=1.08):
    """Wide-open throttle, standing, NO current clamp and NO rpm clamp (what the hardware actually does)."""
    m, b = CAT["motors"][motor], BATT[batt]
    t = np.array(APC[prop]["table"]); J, CT, CP = t[:, 0], t[:, 1], t[:, 2]
    d = APC[prop]["d_in"] * 0.0254
    voc = b["ns"] * v_cell
    def el(n):
        sh = CP[0] * cpd * RHO * n**3 * d**5
        e = n * 60 / m["kv"]
        im = sh / e + m["i0"] * (1 + 0.35 * max(e / 10 - 1, 0))
        return e + im * m["r"], im, sh
    lo, hi = 20.0, 1500.0
    for _ in range(60):
        n = 0.5 * (lo + hi)
        vm, im, _ = el(n)
        vb = voc - 2 * im * b["r"] * (vm / voc)          # pack sags with BOTH motors' pack current (~im*vm/vb each)
        lo, hi = (n, hi) if vm <= vb else (lo, n)
    vm, im, sh = el(lo)
    i_pack = 2 * vm * im / voc
    thrust = 2 * CT[0] * RHO * lo**2 * d**4
    return dict(rpm=lo * 60, motor_a=im, pack_a=i_pack, thrust_kg=thrust / 9.81, shaft_w=sh,
                rpm_lim=150000 / APC[prop]["d_in"])

print("1. Wide-open-throttle STATIC on a fresh pack (no clamps). ESC = Flycolor 50 A; FC/PDB = 80 A cont / 150 A peak")
for motor, prop, batt in (("BrotherHobby Avenger 2812 V3 1115KV", "9x9E", "P45B 6S3P"),
                          ("BrotherHobby Avenger 2812 V3 1115KV", "8x8E", "P45B 6S3P"),
                          ("Emax ECO II 2807 1300KV", "8x8E", "P45B 6S2P"),
                          ("BrotherHobby Avenger 2812 V3 1115KV", "8x6E", "P45B 6S3P")):
    m = CAT["motors"][motor]
    for vc in (4.20, 3.60):
        r = full_throttle_static(motor, prop, batt, vc)
        print(f"   {motor[:24]:24s} {prop} {batt} @{vc:.2f} V/cell: {r['rpm']:6.0f} rpm (APC limit {r['rpm_lim']:5.0f}),"
              f" motor {r['motor_a']:5.1f} A (motor max {m['imax']}, ESC 50), pack {r['pack_a']:5.0f} A (cells {BATT[batt]['imax']}, PDB 80/150),"
              f" shaft {r['shaft_w']:4.0f} W, static {r['thrust_kg']:.2f} kg")

# ---- 2. Servo fit: local airfoil thickness over the servo footprint (v8 planform, same CST airfoil as v7 CAD)
from unittest import mock
sys.modules.setdefault("build123d", mock.MagicMock())
import importlib.util
spec = importlib.util.spec_from_file_location("P", ROOT / "src" / "printable.py")
P = importlib.util.module_from_spec(spec); spec.loader.exec_module(P)
def t_over_c(s, t_scale):
    nt = len(P.CST_WT) - 1
    c_cls = math.sqrt(s) * (1 - s)
    yt = t_scale * (c_cls * sum(w * P.bernstein(nt, i, s) for i, w in enumerate(P.CST_WT)) + s * P.CST_DZ_TE / 2)
    return 2 * yt
half, by, cr, ct = 0.965, 0.108, 0.184, 0.082
print("\n2. Wing thickness over a servo footprint (needs servo thickness + 2 x 0.8 mm skin cover)")
for name, thick, chord_len in (("Corona DS-239MG", 10.0, 30.0), ("PTK 7455 MG-D (Devin: 11.7 mm)", 11.7, 23.0),
                               ("Corona DS-939MG", 9.0, 23.0), ("KST X08 (v7)", 8.0, 23.5)):
    for y in (0.60, 0.68):
        f = (y - by) / (half - by)
        c = (cr + (ct - cr) * f) * 1000
        tsc = 1 + (P.TIP_T_SCALE - 1) * f
        best = None
        for s_c in np.arange(0.36, 0.62, 0.01):          # servo centre chord fraction (aft of 30 % spar cap, ahead of 70 % hinge)
            ss = np.linspace(s_c - chord_len / 2 / c, s_c + chord_len / 2 / c, 9)
            if ss[0] < 0.30 + 9.0 / c:                    # stay clear of the 18 mm spar cap
                continue
            tmin = min(t_over_c(s, tsc) for s in ss) * c
            if best is None or tmin > best[0]:
                best = (tmin, s_c)
        need = thick + 1.6
        print(f"   {name:31s} at y={y*1000:3.0f} mm (c {c:5.1f}): best min-t {best[0]:5.2f} mm at {best[1]:.2f}c vs needed {need:4.1f}"
              f" -> {'FITS' if best[0] >= need else 'BULGES %.1f mm' % (need - best[0])}")

# ---- 3. Devin's two candidates in his own eval_budget, as coded vs. with the 50 A ESC respected
import hillclimb_v8_budget as HB  # noqa: E402
AV, EM = "BrotherHobby Avenger 2812 V3 1115KV", "Emax ECO II 2807 1300KV"
print("\n3. eval_budget (span 1.94, cr .184, ct .082) -- as coded, then motor current capped at 50 A (Flycolor)")
for motor, prop, batt, sw in ((AV, "9x9E", "Molicel P45B 6S3P (18x21700, 292Wh, $118)", 23.5),
                              (AV, "8x8E", "Molicel P45B 6S3P (18x21700, 292Wh, $118)", 23.5),
                              (EM, "8x8E", "Molicel P45B 6S2P (12x21700, 194Wh, $80)", 23.5)):
    for cap in (None, 50.0):
        orig = HB.CAT["motors"][motor]["imax"]
        if cap:
            HB.CAT["motors"][motor]["imax"] = min(orig, cap)
        r = HB.eval_budget(motor, prop, batt, 1.94, 0.184, 0.082, sw=sw)
        HB.CAT["motors"][motor]["imax"] = orig
        if r is None:
            print(f"   {motor[:22]} {prop} {batt[:17]} cap {cap}: INFEASIBLE (fails a gate)")
            continue
        rn, rr, ae = r["rn"], r["rr"], r["ae"]
        print(f"   {motor[:22]} {prop} {batt[:17]} {'as coded' if cap is None else '50 A cap'}: ${r['cost']:.0f}, ESC {r['esc'][:22]}, {r['mass_g']:.0f} g,"
              f" T/W {rn['tw']:.2f} (robust {rr['tw']:.2f}), Vmax {rn['vmax']*3.6:.0f} (thrust {rn['v_thr']*3.6:.0f}) / V_NE {ae['v_ne_kmh']:.0f},"
              f" V_bff {ae['v_bff_kmh']:.0f}, f_b {ae['f_bend']}, end {rn['end']:.0f} min, range {rn['rng']:.0f} km, minG {r['min_g']}")
# BFF sensitivity to the two hard-coded inputs (Iyy 0.036, SM 9.5 %)
r = HB.eval_budget(AV, "9x9E", "Molicel P45B 6S3P (18x21700, 292Wh, $118)", 1.94, 0.184, 0.082, sw=23.5)
orig_ae = HB.aeroelastic_t700_glass
for iyy, sm in ((0.036, 0.095), (0.036, 0.06), (0.036, 0.12), (0.028, 0.095), (0.045, 0.095)):
    def patched(*a, **k):
        a = list(a); a[1] = iyy; k["sm_frac"] = sm
        return orig_ae(*a, **k)
    HB.aeroelastic_t700_glass = patched
    rr2 = HB.eval_budget(AV, "9x9E", "Molicel P45B 6S3P (18x21700, 292Wh, $118)", 1.94, 0.184, 0.082, sw=23.5)
    HB.aeroelastic_t700_glass = orig_ae
    print(f"   BFF sensitivity Iyy {iyy:.3f}, SM {sm:.1%}: " + (f"V_bff {rr2['ae']['v_bff_kmh']:.0f}, V_NE {rr2['ae']['v_ne_kmh']:.0f} km/h" if rr2 else "infeasible (V_NE < 205)"))

print("\n4. Which motor+prop stays inside a 50 A ESC and the APC rpm limit at wide-open throttle on a FRESH pack?")
for motor in ("Emax ECO II 2807 1300KV", "T-Motor F100 2810 1100KV", "BrotherHobby Avenger 2812 V3 1115KV"):
    for prop in ("7x6E", "8x6E", "9x6E", "8x8E", "9x9E"):
        r = full_throttle_static(motor, prop, "P45B 6S3P", 4.20)
        r3 = full_throttle_static(motor, prop, "P45B 6S3P", 3.60)
        ok = r["motor_a"] <= 48 and r["rpm"] <= r["rpm_lim"]
        print(f"   {motor[:24]:24s} {prop}: fresh {r['motor_a']:5.1f} A / {r['rpm']:6.0f} rpm (lim {r['rpm_lim']:5.0f}), pack {r['pack_a']:4.0f} A;"
              f" static thrust fresh {r['thrust_kg']:.2f} kg, at 3.6 V {r3['thrust_kg']:.2f} kg -> {'OK' if ok else 'needs THR cap'}")

print("\n5. At Devin's chosen 22.0 deg (grid lower bound) -- V_NE vs the ASSUMED static margin (not computed anywhere in v8)")
P3 = "Molicel P45B 6S3P (18x21700, 292Wh, $118)"; P2 = "Molicel P45B 6S2P (12x21700, 194Wh, $80)"
for motor, prop, batt in ((AV, "9x9E", P3), (EM, "8x8E", P2), (EM, "8x8E", P3)):
    out = []
    for sm in (0.06, 0.08, 0.095, 0.12):
        def patched(*a, _sm=sm, **k):
            k["sm_frac"] = _sm
            return orig_ae(*a, **k)
        HB.aeroelastic_t700_glass = patched
        r = HB.eval_budget(motor, prop, batt, 1.94, 0.184, 0.082, sw=22.0)
        HB.aeroelastic_t700_glass = orig_ae
        out.append(f"SM {sm:.1%}: " + (f"V_NE {r['ae']['v_ne_kmh']:.0f} (bff {r['ae']['v_bff_kmh']:.0f})" if r else "INFEASIBLE"))
    r = HB.eval_budget(motor, prop, batt, 1.94, 0.184, 0.082, sw=22.0)
    tag = (f"${r['cost']:.0f} {r['esc'][:14]} {r['mass_g']:.0f} g T/W {r['rn']['tw']:.2f}/{r['rr']['tw']:.2f} end {r['rn']['end']:.0f} min"
           if r else "infeasible as coded")
    print(f"   {motor[:16]} {prop} {batt[:17]} [{tag}]:  " + " | ".join(out))
