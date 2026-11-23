"""Airflow around the Kestrel-560 from a vortex-lattice solution (AeroSandbox), for four flight cases.

Run:  .venv/Scripts/python sim/airflow.py      (after sim/envelope.py)
Writes airflow/flow_<case>.npz: streamlines, local speed along them, and the lift loading on the wing.
sim/airflow_plot.py draws them (airflow/flow_<case>.png); renders/render.py can draw them round the CAD model in Blender.

What this is: inviscid potential flow over the wing's mean surface. It shows where the lift is carried,
how the air bends over the wing and the tip vortices. It has no boundary layer, no separation, no
props and no body thickness, so it says nothing about stall or drag.
"""
import json
import pathlib

import aerosandbox as asb
import numpy as np

import blended as B

ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT = ROOT / "airflow"
ENV = json.loads((ROOT / "sim" / "envelope.json").read_text())
PLANE = B.airplane()
W = ENV["mass_kg"] * 9.81
S = PLANE.s_ref
RHO = 1.225

cruise = ENV["altitude"][0]["v_cruise"]
pull = max(ENV["pullout"], key=lambda d: d["v_peak"])
CASES = {
    # name: (airspeed m/s, load factor g, label)
    "cruise": (cruise, 1.0, f"Cruise, {cruise * 3.6:.0f} km/h, 1 g"),
    "slow": (1.08 * ENV["v_stall"], 1.0, f"Slow flight just above the stall, {1.08 * ENV['v_stall'] * 3.6:.0f} km/h"),
    "dive": (ENV["dive"][-1]["idle_sl"], 0.0, f"Vertical dive, {ENV['dive'][-1]['idle_sl'] * 3.6:.0f} km/h, no lift"),
    "pullout": (pull["v_peak"], pull["n_peak"], f"Dive pull-out, {pull['v_peak'] * 3.6:.0f} km/h, {pull['n_peak']:.0f} g"),
}


def solve(v, alpha):
    vlm = asb.VortexLatticeMethod(PLANE, asb.OperatingPoint(velocity=v, alpha=alpha), spanwise_resolution=10,
                                  chordwise_resolution=12)
    return vlm, vlm.run()


def seeds():
    x0 = -0.26                                                   # ahead of the nose (AeroSandbox x runs aft)
    pts = []
    for z, y_min in ((0.030, 0.0), (-0.030, 0.0), (0.013, 0.095), (-0.013, 0.095)):
        ys = np.linspace(y_min, 0.31, 15 if y_min == 0 else 10)
        ys = np.concatenate([-ys[::-1], ys]) if y_min > 0 else np.concatenate([-ys[:0:-1], ys])
        pts += [(x0, y, z) for y in ys]
    half = B.GEO["half"] / 1000
    x_tip = -B.le_x(B.GEO["half"]) / 1000 + 0.02
    for side in (-1, 1):                                         # rings round each tip: these wind into the tip vortex
        for r in (0.008, 0.018, 0.030):
            for a in np.linspace(0, 2 * np.pi, 7)[:-1]:
                pts.append((x_tip, side * (half + r * np.cos(a)), r * np.sin(a)))
    return np.array(pts)


if __name__ == "__main__":
    summary = {}
    for name, (v, n, label) in CASES.items():
        q = 0.5 * RHO * v * v
        cl_need = n * W / (q * S)
        a_lo, a_hi = 0.0, 4.0
        c_lo, c_hi = float(solve(v, a_lo)[1]["CL"]), float(solve(v, a_hi)[1]["CL"])
        for _ in range(4):                                           # secant steps to the angle that gives the lift
            alpha = a_hi + (cl_need - c_hi) * (a_hi - a_lo) / (c_hi - c_lo)
            vlm, aero = solve(v, alpha)
            a_lo, c_lo, a_hi, c_hi = a_hi, c_hi, alpha, float(aero["CL"])
        vlm.vortex_core_radius = 4e-3                                # smooth the filaments for drawing only
        lines = vlm.calculate_streamlines(seeds(), n_steps=520, length=1.05)          # (seed, 3, step)
        lines = np.transpose(lines, (0, 2, 1))
        flat = lines.reshape(-1, 3)
        speed = np.concatenate([np.linalg.norm(vlm.get_velocity_at_points(c), axis=1) for c in np.array_split(flat, 40)]).reshape(lines.shape[:2]) / v
        normal_force = np.einsum("ij,ij->i", vlm.forces_geometry, vlm.normal_directions)
        loading = normal_force / (q * vlm.areas)                                     # pressure difference / dynamic pressure
        centres = (vlm.front_left_vertices + vlm.back_left_vertices + vlm.back_right_vertices + vlm.front_right_vertices) / 4
        flip = np.array([-1.0, -1.0, 1.0])                                           # to the CAD frame: x forward, metres
        wind = np.array([np.cos(np.radians(alpha)), 0, np.sin(np.radians(alpha))]) * flip
        np.savez(OUT / f"flow_{name}.npz", lines=lines * flip, speed=speed, centres=centres * flip, loading=loading,
                 pressure_pa=loading * q, corners=np.stack([vlm.front_left_vertices, vlm.back_left_vertices, vlm.back_right_vertices, vlm.front_right_vertices], axis=1) * flip, alpha=alpha, v=v, n=n, label=label, wind=wind)
        summary[name] = dict(label=label, alpha_deg=alpha, CL=float(aero["CL"]), lift_N=float(aero["CL"] * q * S),
                             peak_loading_pa=float((loading * q).max()), speed_ratio_max=float(np.percentile(speed, 99.5)))
        print(f"{label:55s} alpha {alpha:5.1f} deg  CL {float(aero['CL']):5.2f}  lift {aero['CL'] * q * S:5.1f} N  "
              f"peak pressure difference {(loading * q).max():6.0f} Pa  fastest air {np.percentile(speed, 99.5):.2f} x flight speed")
    (OUT / "flow_summary.json").write_text(json.dumps(summary, indent=1))
