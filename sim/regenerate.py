"""Regenerate every published number for the current build (sim/aircraft.py) in dependency order, then print
and save the headline table.

Run:  .venv/Scripts/python sim/regenerate.py             (all steps, ~10-15 min)
      .venv/Scripts/python sim/regenerate.py --headline  (only the headline table, ~30 s)
      .venv/Scripts/python sim/regenerate.py --tests     (all steps, then sim/test_harness.py)
      .venv/Scripts/python sim/regenerate.py --search    (also the harness.py design search -> sim/results.json, slow)

Steps (each a separate process; its console output is also saved to readiness/log_<step>.txt):
  envelope.py      -> sim/envelope.json, airflow/limits_{altitude,dive_speed,load_factor,pullout}.png
  readiness.py     -> readiness/checks.json                     (reads sim/envelope.json)
  transition.py    -> readiness/transition.json, airflow/limits_transition.png
  budget_check.py  -> readiness/budget_check.json
  blended.py, stability.py, mission.py   -> logs only
  wing_study.py    -> readiness/wing_study.json
Nothing here runs SU2/CFD (tools/); scripts that read CFD results reuse the files already on disk.
Headline -> readiness/headline.json.
"""
import json
import pathlib
import subprocess
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
RDY = ROOT / "readiness"
STEPS = (
    ("envelope", ["envelope.py"]),
    ("readiness", ["readiness.py"]),
    ("transition", ["transition.py"]),
    ("budget_check", ["budget_check.py"]),
    ("blended", ["blended.py"]),
    ("stability", ["stability.py"]),
    ("wing_study", ["wing_study.py"]),
    ("mission", ["mission.py"]),
)
CLMAX_DRIFT = 0.03          # warn if readiness's trimmed CL_max differs from aircraft.TRIMMED_CLMAX by more than this


def run_step(name, args):
    t0 = time.time()
    print(f"\n=== {name}: python sim/{' '.join(args)}", flush=True)
    log = RDY / f"log_{name}.txt"
    with subprocess.Popen([sys.executable, "-u", *args], cwd=HERE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          text=True, encoding="utf-8", errors="replace") as p, open(log, "w", encoding="utf-8") as f:
        for line in p.stdout:
            f.write(line)
            print("    " + line, end="", flush=True)
    dt = time.time() - t0
    print(f"=== {name}: {'ok' if p.returncode == 0 else 'FAILED (exit %d)' % p.returncode} in {dt:.0f} s", flush=True)
    return p.returncode == 0, dt


def headline():
    import aircraft as A
    import harness as h
    hd = h.headline(A.CURRENT_BUILD)
    checks_path = RDY / "checks.json"
    checks = json.loads(checks_path.read_text()) if checks_path.exists() else {}
    stall = checks.get("stall", {})
    drawn = stall.get("battery as drawn", {})
    margins_3d = [s["margin_3d"] * 100 for s in stall.values() if "margin_3d" in s]
    margins_1d = [s["margin_1d"] * 100 for s in stall.values() if "margin_1d" in s]
    warnings = []
    if checks.get("mass_g") is not None and abs(checks["mass_g"] - hd["mass_g"]) > 0.5:
        warnings.append(f"readiness/checks.json is for {checks['mass_g']:.1f} g, the build is {hd['mass_g']:.1f} g: rerun readiness")
    if drawn.get("cl_max"):
        drift = drawn["cl_max"] / A.TRIMMED_CLMAX - 1
        if abs(drift) > CLMAX_DRIFT:
            warnings.append(f"trimmed CL_max from readiness is {drawn['cl_max']:.3f}, aircraft.TRIMMED_CLMAX is "
                            f"{A.TRIMMED_CLMAX:.3f} ({drift * 100:+.1f} %): update aircraft.TRIMMED_CLMAX")
    else:
        warnings.append("no readiness/checks.json trim result: trimmed stall below is the aircraft.py estimate only")
    for v, c in hd["cruise"].items():
        if not c["above_min_speed"]:
            warnings.append(f"{v} m/s is below {A.MIN_SPEED_FACTOR} x trimmed stall ({hd['v_min_ops']:.1f} m/s)")

    out = dict(
        build=A.CURRENT_BUILD,
        generated=time.strftime("%Y-%m-%d %H:%M"),
        mass_g=hd["mass_g"], cost=hd["cost"], wh=hd["wh"], usable=A.USABLE, rho=A.RHO,
        hover_w=hd["hover_w"], hover_pack_a=hd["hover_pack_a"], pack_v_nominal=hd["pack_v_nominal"],
        cruise=hd["cruise"],
        reserve_rule=dict(usable=A.USABLE, hover_s=A.MISSION_HOVER_S, reserve=A.MISSION_RESERVE,
                          note="energy = Wh x usable x (1 - reserve) - hover W x hover_s"),
        closed_form_rule=dict(usable=A.USABLE, hover_s=A.HOVER_S, note="energy = Wh x usable - hover W x hover_s"),
        static={k: dict(tw=s["tw"], thrust_n=s["thrust"], motor_a=s["motor_a"], pack_a=s["pack_a"], pack_v=s["pack_v"])
                for k, s in hd["static"].items()},
        esc=hd["esc"], esc_amps=hd["esc_amps"], i_motor_full_fresh_pack=hd["i_motor_full_fresh"],
        v_stall_trimmed_readiness=drawn.get("v_stall"), cl_max_trimmed_readiness=drawn.get("cl_max"),
        v_stall_trimmed_estimate=hd["v_stall_trimmed"], trimmed_clmax_used=A.TRIMMED_CLMAX,
        v_min_ops=hd["v_min_ops"], min_speed_factor=A.MIN_SPEED_FACTOR,
        static_margin_3d_pct=[min(margins_3d), max(margins_3d)] if margins_3d else None,
        static_margin_1d_pct=[min(margins_1d), max(margins_1d)] if margins_1d else None,
        static_margin_as_drawn_pct=dict(m3d=drawn["margin_3d"] * 100, m1d=drawn["margin_1d"] * 100) if drawn else None,
        best_range=dict(v=hd["best_range_v"], km=hd["best_range_km"]), loiter=dict(v=hd["loiter_v"], min=hd["loiter_min"]),
        vmax=hd["vmax"], warnings=warnings,
    )
    RDY.mkdir(exist_ok=True)
    (RDY / "headline.json").write_text(json.dumps(out, indent=1, default=float))

    s0, s1 = [f"{v:g}" for v in A.HEADLINE_SPEEDS[:2]]
    c0, c1 = hd["cruise"][s0], hd["cruise"][s1]
    vs = drawn.get("v_stall", hd["v_stall_trimmed"])
    rows = [
        ("all-up mass", f"{hd['mass_g']:.1f} g  (${hd['cost']:.0f} parts)"),
        ("hover", f"{hd['hover_w']:.1f} W, {hd['hover_pack_a']:.2f} A from the pack at {hd['pack_v_nominal']:.1f} V"),
        (f"cruise {s0} m/s ({c0['kmh']:.0f} km/h)", f"{c0['watts']:.1f} W, {c0['pack_a']:.2f} A"),
        (f"cruise {s1} m/s ({c1['kmh']:.0f} km/h)", f"{c1['watts']:.1f} W, {c1['pack_a']:.2f} A"),
        (f"{s1} m/s, {A.USABLE:.2f} usable, {A.HOVER_S:.0f} s hover", f"{c1['endurance_min']:.1f} min, {c1['range_km']:.1f} km"),
        (f"{s1} m/s, mission reserve", f"{c1['mission_min']:.1f} min, {c1['mission_km']:.1f} km "
                                     f"({A.MISSION_HOVER_S / 60:.0f} min hover, {A.MISSION_RESERVE * 100:.0f} % kept back)"),
        (f"{s0} m/s, {A.USABLE:.2f} usable / mission", f"{c0['endurance_min']:.1f} min, {c0['range_km']:.1f} km / "
                                                    f"{c0['mission_min']:.1f} min, {c0['mission_km']:.1f} km"),
        ("top speed (level, harness 3.9V)", f"{hd['vmax']:.1f} m/s ({hd['vmax'] * 3.6:.0f} km/h)"),
        ("trimmed stall (battery as drawn)", f"{vs:.2f} m/s ({vs * 3.6:.0f} km/h); estimate {hd['v_stall_trimmed']:.2f} m/s; "
                                             f"min cruise/loiter {hd['v_min_ops']:.2f} m/s"),
        ("static T/W at 3.6 V/cell", f"{hd['static']['3.6']['tw']:.2f}  ({hd['static']['3.6']['motor_a']:.1f} A per motor, "
                                     f"{hd['static']['3.6']['pack_a']:.1f} A pack)"),
        ("static T/W at 3.3 V/cell", f"{hd['static']['3.3']['tw']:.2f}  ({hd['static']['3.3']['motor_a']:.1f} A per motor, "
                                     f"{hd['static']['3.3']['pack_a']:.1f} A pack)"),
        ("ESC", f"{hd['esc']} ({hd['esc_amps']} A; full throttle on a fresh pack {hd['i_motor_full_fresh']:.1f} A)"),
    ]
    if margins_3d:
        rows.append(("static margin, battery slide", f"3D {min(margins_3d):.1f}..{max(margins_3d):.1f} % MAC, "
                                                     f"1D {min(margins_1d):.1f}..{max(margins_1d):.1f} % MAC"))
    print(f"\nHEADLINE: {A.CURRENT_BUILD['name']}  (rho {A.RHO}, no wind)")
    w = max(len(k) for k, _ in rows)
    for k, v in rows:
        print(f"  {k:{w}s}  {v}")
    for msg in warnings:
        print(f"  WARNING: {msg}")
    print(f"wrote {(RDY / 'headline.json').relative_to(ROOT)}")
    return out


def main(argv):
    t0 = time.time()
    ok_all = True
    if "--headline" not in argv:
        steps = list(STEPS) + ([("search", ["harness.py"])] if "--search" in argv else [])
        times = {}
        for name, args in steps:
            ok, times[name] = run_step(name, args)
            ok_all &= ok
            if not ok and name in ("envelope",):
                print("envelope failed: readiness needs sim/envelope.json; stopping")
                return 1
        print("\nstep times: " + ", ".join(f"{k} {v:.0f} s" for k, v in times.items()))
    headline()
    if "--tests" in argv:
        ok, _ = run_step("tests", ["test_harness.py"])
        ok_all &= ok
    print(f"\nregenerate: {'all steps ok' if ok_all else 'SOME STEPS FAILED'} in {(time.time() - t0) / 60:.1f} min")
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
