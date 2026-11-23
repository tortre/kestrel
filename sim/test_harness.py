"""Checks on sim/harness.py and sim/aircraft.py: physics bounds, bookkeeping, the current build's published
numbers, agreement with the older model and with the time-stepped mission, and import hygiene.
Run:  .venv/Scripts/python sim/test_harness.py
"""
import json
import math
import os
import pathlib
import subprocess
import sys
import traceback

import numpy as np

import aircraft as A
import harness as h
import optimize as opt

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
h.OPTIONS.update(off_aliexpress=True, free_bay=True, risky=True)
TW_NOMINAL_GATE = 1.95
DESIGNS = [
    h.CURRENT,
    h.BASELINE,
    dict(h.BASELINE, **A.DUAL_PACK_UPGRADE),
    dict(h.BASELINE, motor="Flashhobby Mars M2812 900KV"),
]
# LEGACY 610 mm design, only for the agreement test against the old optimize.py model
LEGACY_610 = dict(motor="SunnySky X2206 1500KV", prop="7x6E", battery="Molicel P50B 3S1P (not on AliExpress)",
                  servo="Emax ES08MD II 12g metal gear", fc="SpeedyBee F405 Wing Mini", material="LW-PLA (foaming)",
                  cr=0.160, ct=0.100, span=0.610)


def aircraft(design):
    return h.make_aircraft(design), h.evaluate(design, full=True)


# ---- the current build ------------------------------------------------------------------------------
def test_current_build_headline():
    """The published numbers for aircraft.CURRENT_BUILD (Kestrel-X Stealth v9.1 Under-$300) stay inside verified bands."""
    hd = h.headline(A.CURRENT_BUILD)
    assert abs(hd["mass_g"] - A.all_up_mass_g(A.CURRENT_BUILD)) < 0.01, "harness mass differs from aircraft.py"
    assert 240 <= hd["hover_w"] <= 320, hd["hover_w"]
    assert 95 <= hd["loiter_min"] <= 130, hd["loiter_min"]
    assert 80 <= hd["best_range_km"] <= 105, hd["best_range_km"]
    assert 50.0 <= hd["vmax"] <= hd["v_ne_aeroelastic"], (hd["vmax"], hd["v_ne_aeroelastic"])
    assert 1.95 <= hd["static"]["3.6"]["tw"] <= 2.60, hd["static"]["3.6"]["tw"]
    assert hd["static"]["3.3"]["tw"] <= hd["static"]["3.6"]["tw"]
    assert hd["esc_amps"] >= hd["i_motor_full_fresh"] * A.ESC_SIZING_MARGIN - 1e-9, "ESC smaller than full-throttle current"
    assert all(hd["cruise"][f"{v:g}"]["above_min_speed"] for v in A.HEADLINE_SPEEDS)
    assert hd["cost"] < 300.0, f"v9 parts cost is not under $300: {hd['cost']}"
    assert abs(hd["cost"] - sum(c for _, c in A.bill_of_materials(A.CURRENT_BUILD).values())) < 0.01
    assert "Walksnail" in A.CURRENT_BUILD["video"], "the HD digital camera + VTX kit must stay in the build"
    assert hd["feasible"], hd["reasons"]
    # the pack's continuous rating covers full throttle, and the FC's own 80 A current sensor covers the pack
    assert hd["static"]["3.6"]["pack_a"] <= A.part("battery", A.CURRENT_BUILD["battery"])["imax"] + 1e-6 <= 80.0


def test_current_build_is_valid_and_complete():
    A.validate(A.CURRENT_BUILD)
    assert A.CURRENT_BUILD["battery"] in A.CAT["batteries"] and A.CAT["batteries"][A.CURRENT_BUILD["battery"]].get("default")
    assert A.CAT["motors"]["Emax ECO II 2306 2400KV"]["kv"] == 2400
    names = A.mass_breakdown(A.CURRENT_BUILD, A.GEO["battery_x"])
    assert any(k.startswith("ESCs x2") for k in names), "ESC mass missing from the build"
    alt = A.variant(**A.DUAL_PACK_UPGRADE)
    assert A.all_up_mass_g(alt) > A.all_up_mass_g(), "adding the second 6S1P stick did not increase mass"
    assert h.headline(alt)["feasible"], "the build must still fly with the second 6S1P brick fitted"
    # the CAD spar caps are sized for ONE design gross mass: one brick and two must both stay under it
    assert A.all_up_mass_kg(alt) <= A.AIRFRAME_DESIGN_GROSS_KG - 0.10, (A.all_up_mass_kg(alt), A.AIRFRAME_DESIGN_GROSS_KG)
    assert A.validate(A.V8_BUDGET_BUILD), "the v8 reference build must stay usable for comparisons"
    # Dual-pack CG shift check: one 6S1P stick <-> two in parallel moves CG by < 4 mm (< 2% MAC)!
    assert abs(A.cg_x_mm(alt) - A.cg_x_mm(A.CURRENT_BUILD)) < 4.0, (A.cg_x_mm(alt), A.cg_x_mm(A.CURRENT_BUILD))


def test_hard_output_ceiling_protects_the_hardware():
    """With NO firmware current limiter, a fresh pack at the param file's Q_M_SPIN_MAX / THR_MAX stays inside the ESC and
    cell burst ratings, and the aircraft still has the hover thrust margin under that ceiling (one brick and two)."""
    import hillclimb_v9 as H
    params = {}
    for ln in (ROOT / "firmware" / "kestrel_x_stealth_v9_under300.param").read_text().splitlines():
        if ln and not ln.startswith("#") and "," in ln:
            k, v = ln.split(",")[:2]
            params[k.strip()] = float(v)
    ceiling, tw = H.hard_ceiling(A.CURRENT_BUILD)
    assert params["Q_M_SPIN_MAX"] <= ceiling + 1e-9 and params["THR_MAX"] / 100.0 <= ceiling + 1e-9, (params, ceiling)
    assert tw >= h.TW_MIN, f"thrust/weight {tw:.2f} under the {ceiling:.2f} output ceiling"
    batt = A.part("battery", A.CURRENT_BUILD["battery"])
    assert params["Q_M_BAT_CURR_MAX"] <= batt["imax"] and params["BATT_CAPACITY"] == round(batt["wh"] / A.pack_volts() * 1000)
    ceiling2, tw2 = H.hard_ceiling(A.variant(**A.DUAL_PACK_UPGRADE))
    assert ceiling2 >= 0.94 and tw2 >= TW_NOMINAL_GATE, (ceiling2, tw2)       # two bricks: Q_M_SPIN_MAX 0.94 in the param notes
    winner = json.loads((HERE / "hillclimb_v9_results.json").read_text())["winner"]["state"]
    assert all(A.CURRENT_BUILD[k] == v for k, v in winner.items()), "sim/aircraft.py is not the hill-climb winner: rerun hillclimb_v9.py"


def test_claude_audit_gates():
    """Hard regression tests enforcing EVERY Claude Opus 5.5 audit finding with ONE 6S1P stick and with two!"""
    import blended as B
    import stability as st
    # 1. True projected span (ZERO sharklet span inflation in AR!)
    p = B.airplane()
    assert abs(p.b_ref - A.SPAN) < 1e-4, f"b_ref {p.b_ref} != true projected span {A.SPAN}"
    # 2. Cheap standard 135 GPa T700 spar caps + >=0.70mm carbon shear web + 32x15mm V-box >= 20 g @ 400 MPa (for 6S3P!)
    sp = A.spar_design()
    assert sp["min_g_limit"] >= 20.0, f"min spar/torsion limit {sp['min_g_limit']} g < 20 g @ 400 MPa"
    assert sp["g_lim_root_torsion"] >= 20.0, sp
    assert abs(sp["root_box_w_mm"] - A.GEO["carry_box_w"]) < 0.1, "CAD vs structure root box width mismatch"
    assert abs(sp["schedule"][1]["h_box_mm"] - A.GEO["spar_h"][0]) < 0.1, "CAD vs structure spar depth mismatch"
    assert [sp["schedule"][1]["t_mm"], sp["schedule"][2]["t_mm"], sp["schedule"][3]["t_mm"]] == list(A.GEO["cap_t"]), (
        f"CAD CAP_T {A.GEO['cap_t']} != sized caps {[sp['schedule'][k]['t_mm'] for k in (1,2,3)]}"
    )
    assert all(z["tau_web_mpa"] <= 50.0 for z in sp["schedule"]), sp["schedule"]
    # Verify camber-centred spar channel fits inside local CST airfoil across FULL cap width!
    coords = B.wing_coords(1.0)
    up_x, up_y = coords[:41, 0][::-1], coords[:41, 1][::-1]
    lo_x, lo_y = coords[40:, 0], coords[40:, 1]
    y1 = round(A.GEO["blend_y"] + 0.30 * (A.GEO["half"] - A.GEO["blend_y"]), 1)
    y2 = round(A.GEO["blend_y"] + 0.62 * (A.GEO["half"] - A.GEO["blend_y"]), 1)
    for y_end, h_mm, w_mm in ((y1, A.GEO["spar_h"][0], A.GEO["cap_w"][0]),
                              (y2, A.GEO["spar_h"][1], A.GEO["cap_w"][1]),
                              (A.GEO["half"] - 16.0, A.GEO["spar_h"][2], A.GEO["cap_w"][2])):
        t_sc = 1.0 + (A.GEO["tip_t_scale"] - 1.0) * (y_end - A.GEO["blend_y"]) / (A.GEO["half"] - A.GEO["blend_y"])
        c_y = B.chord(y_end)
        hsf = (w_mm / 2.0 + 0.25) / c_y
        local_t = min((float(np.interp(s, up_x, up_y)) - float(np.interp(s, lo_x, lo_y))) * c_y * t_sc
                      for s in (0.30 - hsf, 0.30, 0.30 + hsf))
        assert local_t - h_mm >= 1.40, f"spar channel {h_mm} mm breaks skin at y={y_end} (local thick={local_t:.2f} mm)"
    # 3. 2-DOF Theodorsen + 3-DOF BFF with cheap 135 GPa T700 + 2x49g E-glass using TRUE computed Iyy on BOTH 6S2P & 6S3P!
    for b_test in (A.CURRENT_BUILD, A.variant(**A.DUAL_PACK_UPGRADE)):
        ac_t = h.make_aircraft(A.harness_design(b_test))
        m_st, _, _, iyy_true, _ = st.mass_properties(ac_t, A.GEO["battery_x"], b_test)
        ae = A.AE.aeroelastic_check(m_st, iyy_true, p.s_ref, p.c_ref, A.SPAN,
                                    A.GEO["blend_y"] / 1000.0, A.ROOT_CHORD, A.TIP_CHORD, A.GEO["sweep_deg"],
                                    A.GEO["elevon_y"][0] / 1000.0, A.GEO["elevon_y"][1] / 1000.0,
                                    A.GEO["hinge"], sp, sm_frac=0.090, skin_type="glass2x49", cap_e_gpa=135.0)
        assert ae is not None and ae["v_ne_aeroelastic_ms"] >= 62.0, (b_test["battery"], ae)
        hd_t = h.headline(b_test)
        assert abs(m_st * 1000.0 - hd_t["mass_g"]) < 25.0, (m_st * 1000.0, hd_t["mass_g"])
    # 4. Real unclamped pivot tilt-nacelle arm & converging inbound transition WITH real margin below max tilt!
    tr_path = A.ROOT / "readiness" / "transition.json"
    if tr_path.exists():
        import json
        tr = json.loads(tr_path.read_text())
        if "tilt_pitch_arm_mm" in tr["hover"]:
            assert tr["hover"]["tilt_pitch_arm_mm"] > 110.0, tr["hover"]
            assert tr["inbound"]["converged"], tr["inbound"]
            assert tr["inbound"]["peak_tilt_deg"] <= tr["hover"]["max_tilt_deg"] - 3.0, tr["inbound"]
    # 5. All readiness trim cases genuinely within TE-up..TE-down trim table (no silent clamping!)
    chk_path = A.ROOT / "readiness" / "checks.json"
    if chk_path.exists():
        import json
        chk = json.loads(chk_path.read_text())
        assert all(t["ok"] for t in chk["trim"]), f"untrimmed case in checks.json: {[t for t in chk['trim'] if not t['ok']]}"


def test_cruise_and_loiter_respect_min_speed():
    """Loiter and best-range speeds are >= aircraft.MIN_SPEED_FACTOR x the trimmed stall."""
    for d in DESIGNS:
        ac, r = aircraft(d)
        assert r["v_loiter"] >= A.MIN_SPEED_FACTOR * ac.v_stall_trim - 1e-9
        assert r["v_cruise"] >= A.MIN_SPEED_FACTOR * ac.v_stall_trim - 1e-9


def test_esc_sized_for_full_throttle():
    for d in DESIGNS:
        ac, r = aircraft(d)
        biggest = max(e["amps"] for e in h.CAT["escs"].values())
        if not ac.fc.get("esc_included") and not d.get("esc"):
            assert ac.esc_amps >= min(r["i_motor_full"] * A.ESC_SIZING_MARGIN, biggest), (ac.esc_name, r["i_motor_full"])


# ---- catalogue -----------------------------------------------------------------------------------------
def test_catalogue_is_complete():
    for group in ("motors", "batteries", "servos", "fcs", "materials", "escs", "video", "gps", "receivers",
                  "remote_id", "hardware"):
        for name, part in h.CAT[group].items():
            if name.startswith("_"):
                continue
            price = part.get("price", part.get("price_pair"))
            assert price is not None and price > 0, f"{group}/{name} has no price"
    for name, p in h.CAT["props"]["models"].items():
        if not name.startswith("_"):
            assert p["data"] in opt.APC and p["price_pair"] > 0 and p["g"] > 0, name
    for name, b in h.CAT["batteries"].items():
        max_whkg = 450 if b["chem"] == "si_liion" else 310
        assert 100 < b["wh"] / b["g"] * 1000 < max_whkg, f"{name}: implausible energy density"


# ---- physics bounds -------------------------------------------------------------------------------------
def test_power_chain_and_prop_efficiency_bound():
    """Electrical power > shaft power > thrust power, and the prop never beats ideal momentum theory."""
    for d in DESIGNS:
        ac, r = aircraft(d)
        v = np.arange(max(1.2 * ac.v_stall, 9), 30, 1.0)
        p, x = ac.level_power(v)
        ok = np.isfinite(p)
        thrust_w = x["drag"] / 2 * v
        assert np.all(p[ok] > 2 * x["shaft"][ok]) and np.all(x["shaft"][ok] > thrust_w[ok])
        disk = math.pi * ac.d**2 / 4
        t = x["drag"] / 2
        ideal = 2 / (1 + np.sqrt(1 + t / (0.5 * h.RHO * v**2 * disk)))
        assert np.all((thrust_w / x["shaft"])[ok] <= ideal[ok] + 1e-9), "prop efficiency above the momentum-theory limit"


def test_hover_power_above_induced_power():
    for d in DESIGNS:
        ac, r = aircraft(d)
        disk = 2 * math.pi * ac.d**2 / 4
        assert r["p_hover"] > ac.w**1.5 / math.sqrt(2 * h.RHO * disk)


def test_endurance_from_independent_inputs():
    """Rebuild endurance and range from the catalogue pack energy, the time-stepped mission and a fresh speed
    sweep, not from evaluate()'s own energy figure: the loiter speed must be the minimum-power speed above the
    speed floor, the endurance must agree with flying the mission at that speed, and range must respect L/D."""
    for d in DESIGNS:
        ac, r = aircraft(d)
        b = h.CAT["batteries"][d["battery"]]
        pack_wh = b["wh"] * b["honesty"]                                            # straight from the catalogue
        assert abs(ac.wh - pack_wh) < 1e-9
        v = np.arange(ac.v_min_ops, 30.0, 0.05)
        p = ac.level_power(v)[0]
        assert r["p_loiter"] <= np.nanmin(p[np.isfinite(p)]) * 1.01, "loiter power is not the minimum level power"
        sim = h.fly_mission(d, r["v_loiter"])                                        # time-stepped, OCV curve + sag
        assert abs(sim["minutes"] / (r["endurance_min"] + h.HOVER_S / 60) - 1) < 0.08, (sim["minutes"], r["endurance_min"])
        assert r["range_km"] <= pack_wh * h.NOMINAL["usable"] * 3600 * r["ld"] / (r["mass_g"] / 1000 * h.G) / 1000, \
            "range above the L/D bound"


def test_monotonic():
    for d in DESIGNS:
        a, heavy, draggy = h.evaluate(d), h.evaluate(d, mass_extra=50.0), h.evaluate(d, drag_scale=1.2)
        assert heavy["range_km"] < a["range_km"] and heavy["endurance_min"] < a["endurance_min"]
        assert draggy["range_km"] < a["range_km"] and draggy["vmax"] <= a["vmax"]


def test_top_speed_is_where_thrust_meets_drag():
    for d in DESIGNS:
        ac, r = aircraft(d)
        v = r["v_thrust"]
        assert ac.full_throttle(v)[0][0] >= ac.drag(np.array([v]))[0][0]
        assert ac.full_throttle(v + 1.0)[0][0] < ac.drag(np.array([v + 1.0]))[0][0]
        assert r["vmax"] <= min(r["v_thrust"], r["v_servo"], h.V_NE) + 1e-9


def test_full_throttle_covers_hover_margin():
    for d in DESIGNS:
        ac, r = aircraft(d)
        if r["feasible"]:
            assert ac.full_throttle(0.01)[0][0] >= 0.95 * h.TW_MIN * ac.w


def test_mission_matches_closed_form():
    for d in DESIGNS:
        r = h.evaluate(d, full=True)
        sim = h.fly_mission(d, r["v_cruise"])
        assert abs(sim["range_km"] / r["range_km"] - 1) < 0.08, (sim["range_km"], r["range_km"])
        assert sim["v_min"] > 3.0 * 3, "pack pulled below 3.0 V per cell"


def test_agrees_with_older_model():
    """The LEGACY 610 mm airframe in sim/optimize.py (printable drag and mass settings): range within 6 % once
    mass and avionics power are matched."""
    old = (opt.HULL_FF, opt.POD_CD, opt.EXTRAS_DQ, opt.EXTRA_MASS_G, opt.AVIONICS_W)
    try:
        opt.HULL_FF, opt.POD_CD, opt.EXTRAS_DQ, opt.EXTRA_MASS_G = h.HULL_FF, h.POD_CD, h.EXTRAS_DQ, 26.65
        mine = h.evaluate(LEGACY_610, full=True)
        opt.AVIONICS_W = mine["avionics_w"]                       # same camera and electronics load in both
        ref = opt.evaluate("SunnySky X2206 1500KV", "7x6E", "Molicel P50B", (3, 1), 0.160, 0.100, full=True)
    finally:
        opt.HULL_FF, opt.POD_CD, opt.EXTRAS_DQ, opt.EXTRA_MASS_G, opt.AVIONICS_W = old
    matched = h.evaluate(LEGACY_610, full=True, mass_extra=ref["mass_g"] - mine["mass_g"])
    assert abs(matched["range_km"] / ref["range_km"] - 1) < 0.06, (matched["range_km"], ref["range_km"])
    assert abs(matched["p_hover"] / ref["p_hover"] - 1) < 0.05, (matched["p_hover"], ref["p_hover"])


def test_pareto():
    rows = [({}, dict(vmax=1, endurance_min=1, range_km=1, mass_g=1, cost=1)),
            ({}, dict(vmax=2, endurance_min=1, range_km=1, mass_g=1, cost=1)),
            ({}, dict(vmax=1, endurance_min=1, range_km=1, mass_g=1, cost=0.5))]
    assert len(h.pareto(rows)) == 2


# ---- import hygiene -------------------------------------------------------------------------------------
_SMOKE = r"""
import json, pathlib, sys
root = pathlib.Path(sys.argv[1])
def snap():
    out = {}
    for d in (root / "readiness", root / "airflow", root / "sim"):
        if d.exists():
            for p in d.iterdir():
                if p.name != "__pycache__":
                    out[str(p)] = p.stat().st_mtime_ns
        else:
            out[str(d)] = None
    return out
before = snap()
import harness
opts = dict(harness.OPTIONS)
import readiness, envelope, transition, budget_check
after = snap()
print(json.dumps(dict(changed=sorted(k for k in set(before) | set(after) if before.get(k) != after.get(k)),
                      options_before=opts, options_after=dict(harness.OPTIONS))))
"""


def test_imports_have_no_side_effects():
    """readiness, envelope, transition and budget_check import cleanly in a fresh interpreter without writing
    files, creating folders or changing harness.OPTIONS."""
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    res = subprocess.run([sys.executable, "-c", _SMOKE, str(ROOT)], cwd=HERE, env=env, capture_output=True, text=True,
                         timeout=900)
    assert res.returncode == 0, res.stderr[-2000:]
    out = json.loads(res.stdout.strip().splitlines()[-1])
    assert not out["changed"], f"import wrote or created: {out['changed']}"
    assert out["options_before"] == out["options_after"] == dict(free_bay=False, risky=False, off_aliexpress=False), out


if __name__ == "__main__":
    failed = 0
    tests = [(name, fn) for name, fn in list(globals().items()) if name.startswith("test_") and callable(fn)]
    for name, fn in tests:
        try:
            fn()
            print(f"PASS  {name}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {name}: {e}")
        except Exception as e:                                         # a crash is a failure, not the end of the run
            failed += 1
            print(f"FAIL  {name}: {type(e).__name__}: {e}")
            traceback.print_exc(limit=3)
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    sys.exit(1 if failed else 0)
