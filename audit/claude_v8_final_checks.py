"""v8 Budget final sign-off checks (Claude, 2026-11-20). Read-only; touches none of Devin's files.
Run: .venv/Scripts/python.exe audit/claude_v8_final_checks.py"""
import json, math, pathlib, sys
from unittest import mock
import numpy as np
ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim")); sys.path.insert(0, str(ROOT / "src"))
CAT = json.loads((ROOT / "sim" / "catalog.json").read_text())
APC = json.loads((ROOT / "sim" / "apc_props.json").read_text())
RHO = 1.225
BATT = {"P45B 6S2P": dict(ns=6, r=0.030 + 0.010, imax=90), "P45B 6S3P": dict(ns=6, r=0.020 + 0.010, imax=135)}


def wot_static(motor, prop, batt, v_cell, cpd=1.08):
    """Standing WOT, no current/rpm clamps (what an ESC at 100 % actually does)."""
    m, b = CAT["motors"][motor], BATT[batt]
    t = np.array(APC[prop]["table"]); CT, CP = t[0, 1], t[0, 2]
    d = APC[prop]["d_in"] * 0.0254
    voc = b["ns"] * v_cell
    def el(n):
        sh = CP * cpd * RHO * n**3 * d**5
        e = n * 60 / m["kv"]
        im = sh / e + m["i0"] * (1 + 0.35 * max(e / 10 - 1, 0))
        return e + im * m["r"], im, sh
    lo, hi = 20.0, 1500.0
    for _ in range(60):
        n = 0.5 * (lo + hi); vm, im, _ = el(n)
        vb = voc - 2 * im * b["r"] * (vm / voc)
        lo, hi = (n, hi) if vm <= vb else (lo, n)
    vm, im, sh = el(lo)
    return dict(rpm=lo * 60, motor_a=im, pack_a=2 * vm * im / voc, kg=2 * CT * RHO * lo**2 * d**4 / 9.81,
                rpm_lim=150000 / APC[prop]["d_in"], imax=m["imax"])


MOT = next(k for k in CAT["motors"] if "Cine66" in k and "925" in k)
PROP = "8x8E" if "8x8E" in APC else next(iter(APC))
print(f"1. Unclamped static WOT, {MOT} + {PROP} (ESC 50 A, motor imax {CAT['motors'][MOT]['imax']}, 6S2P cells 90 A)")
for batt in BATT:
    for vc in (4.20, 4.00, 3.80, 3.60):
        r = wot_static(MOT, PROP, batt, vc)
        print(f"   {batt} @{vc:.2f} V/cell: {r['rpm']:6.0f} rpm (APC lim {r['rpm_lim']:5.0f}) motor {r['motor_a']:5.1f} A"
              f"  pack {r['pack_a']:5.1f} A  static {r['kg']:.2f} kg")
# throttle fraction that keeps the fresh-pack motor current <= 45 A (approx: shaft ~ throttle^3 -> I ~ thr^2 region)
r = wot_static(MOT, PROP, "P45B 6S2P", 4.20)
print(f"   -> THR_MAX for <=45 A/motor fresh: ~{100 * math.sqrt(45 / r['motor_a']):.0f} % (I ~ thr^2 rule of thumb)")

# ---- 2. Servo pocket skin, using the CAD's own surf()/line_x() functions
sys.modules.setdefault("build123d", mock.MagicMock())
import importlib.util
spec = importlib.util.spec_from_file_location("P", ROOT / "src" / "printable.py")
P = importlib.util.module_from_spec(spec); spec.loader.exec_module(P)
print("\n2. Servo pocket fit (CAD surf(), pocket floor = lower skin, so the 'skin' left above the servo is the whole margin)")
for lbl, y, dim, real_t in (("tilt PTK 7455", P.TILT_SERVO_Y, P.TILT_SERVO, 11.7), ("elevon PTK 7308", P.ELEV_SERVO_Y, P.ELEV_SERVO, 8.0)):
    fr = (P.SERVO_FRAC - 0.08, P.SERVO_FRAC, P.SERVO_FRAC + 0.08)
    zl = min(P.surf(s, y, False) for s in fr); zu = min(P.surf(s, y, True) for s in fr)
    zl_c = P.surf(P.SERVO_FRAC, y, False); zu_c = P.surf(P.SERVO_FRAC, y, True)
    # thickness across actual pocket chord extent
    c = P.line_x(1.0, y) - P.line_x(0.0, y)
    ss = np.linspace(P.SERVO_FRAC - dim[0] / 2 / c, P.SERVO_FRAC + dim[0] / 2 / c, 9)
    tmin = min(P.surf(s, y, True) - P.surf(s, y, False) for s in ss)
    print(f"   {lbl:16s} y={y:5.0f} chord {c:5.1f} mm: local t at 0.50c {zu_c - zl_c:5.2f}, min over footprint {tmin:5.2f} mm;"
          f" pocket {dim[2]} mm (servo {real_t}) -> skin above {zu - (zl + dim[2]):4.2f} mm; lower-skin offset zl {zl:+.2f} vs centre {zl_c:+.2f}")

# ---- 3. Spar cap vs channel and wing-root carry box (from GEO)
print("\n3. Dive/V_NE cross-check from sim/envelope.json")
E = json.loads((ROOT / "sim" / "envelope.json").read_text())
def find(d, key, path=""):
    if isinstance(d, dict):
        for k, v in d.items():
            p = f"{path}.{k}" if path else k
            if key in k.lower() and not isinstance(v, (dict, list)):
                print(f"   {p} = {v}")
            find(v, key, p)
for k in ("v_ne", "v_bff", "v_bt", "v_rev", "v_b"):
    find(E, k)
