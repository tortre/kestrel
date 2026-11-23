"""Final v7 verification of Devin's five blocker fixes (Claude, 2026-11-20). Read-only."""
import json, math, pathlib, sys
import numpy as np
ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))
import aircraft as A  # noqa: E402
G = A.GEO
mr = json.loads((ROOT / "print" / "mass_report.json").read_text())["geometry"]
print("GEO spar_h", G.get("spar_h"), "mass_report spar_h", mr.get("spar_h"), "carry", G.get("carry_box_w"), G.get("carry_box_h"))
print("GEO tilt_shaft_x", G.get("tilt_shaft_x"), "motor_x", G["motor_x"])
sp = A.spar_design()
for z in sp["schedule"]:
    print("  ", {k: z[k] for k in ("name", "y0_mm", "y1_mm", "h_box_mm", "t_mm", "t_web_mm", "tau_web_mpa", "g_limit")})
print("  root box", {k: sp[k] for k in ("root_box_w_mm", "root_box_h_mm", "root_box_t_mm", "tau_box_mpa", "g_lim_root_torsion", "min_g_limit", "root_box_g")}, "spar total g", round(sp["mass_g"], 1))

# 1. Does a constant-height channel fit inside the local airfoil at the spar line?
af = json.loads((ROOT / "sim" / "stealth_airfoil.json").read_text())
print("airfoil keys", list(af)[:12])

# ---- 1b. CAD channel (centred on z_dih, i.e. the chord line) vs local airfoil skin at the spar line
import importlib.util, types
from unittest import mock
sys.modules.setdefault("build123d", mock.MagicMock())  # geometry helpers only; no solids built
spec = importlib.util.spec_from_file_location("P", ROOT / "src" / "printable.py")
P = importlib.util.module_from_spec(spec); spec.loader.exec_module(P)
print("\n1b. Spar channel vs airfoil (channel centred on chord line, rect_beam(spar_point...))")
for name, y0, y1, w, h in (("inner", P.SPAR_Y0, 415.0, P.CAP_W_IN, P.SPAR_H_IN), ("mid", 395.0, 735.0, P.CAP_W_MID, P.SPAR_H_MID),
                           ("outer", 715.0, P.HALF - 16.0, P.CAP_W_OUT, P.SPAR_H_OUT)):
    worst = None
    for y in np.linspace(max(y0, P.BLEND_Y), y1, 40):
        c = P.chord(y)
        zc = P.z_dih(y)
        for ds in np.linspace(-w / 2, w / 2, 7):
            s = P.SPAR_FRAC + ds / c
            up = P.surf(s, y, True) - zc
            lo = zc - P.surf(s, y, False)
            m_up, m_lo = up - h / 2, lo - h / 2      # material left above / below the channel (mm)
            m = min(m_up, m_lo)
            if worst is None or m < worst[0]:
                worst = (m, y, ds, m_up, m_lo, up + lo)
    m, y, ds, mu, ml, t = worst
    print(f"  {name:5s} h={h:4.1f}: worst cover {m:+5.2f} mm at y={y:5.0f} (dx {ds:+4.1f}); upper {mu:+5.2f}, lower {ml:+5.2f}, local t {t:5.2f} mm"
          f" -> {'BREAKS THROUGH SKIN' if m < 0 else ('< 0.8 mm cover' if m < 0.8 else 'ok')}")
print("  printed wall WALL =", P.WALL, "mm; cap mm (CAD const vs sim):",
      (P.CAP_T_IN, P.CAP_T_MID, P.CAP_T_OUT), [z["t_mm"] for z in sp["schedule"][1:]])
print("\n1c. Along-span: skin z above/below chord line at the 30 % spar line vs channel half-height")
for name, y0, y1, h in (("inner", 108.0, 415.0, P.SPAR_H_IN), ("mid", 395.0, 735.0, P.SPAR_H_MID), ("outer", 715.0, 1044.0, P.SPAR_H_OUT)):
    for y in (y0, 0.5 * (y0 + y1), y1):
        zc = P.z_dih(y)
        up, lo = P.surf(P.SPAR_FRAC, y, True) - zc, zc - P.surf(P.SPAR_FRAC, y, False)
        print(f"  {name:5s} y={y:6.0f}: upper +{up:5.2f}, lower -{lo:5.2f} (t {up+lo:5.2f}, camber {0.5*(up-lo):+4.2f}) | channel +/-{h/2:4.1f}"
              f" -> lower protrusion {h/2-lo:+5.2f} mm")
# What the structure gives if each channel is centred on the camber line and kept 0.8 mm inside the skin at its
# outboard end (constant-height channels, Devin's own sizing routine picks the caps):
hs = []
for y1 in (415.0, 735.0, 1044.0):
    zc = P.z_dih(y1)
    t = (P.surf(P.SPAR_FRAC, y1, True) - zc) + (zc - P.surf(P.SPAR_FRAC, y1, False))
    hs.append(round(t - 1.6, 2))
print("\n1d. Max constant channel heights that fit (centred on camber, 0.8 mm cover):", hs)
import aeroelastic as AE
sp2 = AE.design_spar_caps(*[None] * 0, **{}) if False else None
orig = A.GEO["spar_h"]
for lbl, hh in (("as CAD 14.0/10.6/7.2 (does NOT fit)", orig), ("fits: camber-centred, 0.8 mm cover", hs)):
    A.GEO["spar_h"] = list(hh)
    s = A.spar_design()
    print(f"  {lbl:38s}: caps t {[z['t_mm'] for z in s['schedule'][1:]]} mm, g {[z['g_limit'] for z in s['schedule'][1:]]},"
          f" min {s['min_g_limit']} g, spar+box {s['mass_g']:.0f} g")
A.GEO["spar_h"] = orig
# same CAD heights but caps as written in printable.py (1.5 / 1.0 / 0.5 mm) instead of the sim's 1.75 / 1.25 / 0.5
def g_scale(h, t_sim, t_cad):
    i = lambda t: 2 * t * ((h - t) / 2) ** 2 + 2 * t**3 / 12
    return i(t_cad) / i(t_sim)
z = sp["schedule"]
print("  CAD cap constants 1.5/1.0 mm in sim heights -> inner", round(z[1]["g_limit"] * g_scale(14.0, 1.75, 1.5), 1),
      "g, mid", round(z[2]["g_limit"] * g_scale(10.6, 1.25, 1.0), 1), "g")
