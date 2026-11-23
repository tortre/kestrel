"""Final v7 sign-off: re-run Devin's transition sims with the tilt-pivot pitch arm (Claude, 2026-11-20). Read-only."""
import pathlib, sys
ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))
import transition as T  # noqa: E402

PIVOT_X = 134.0  # src/printable.py TILT_SHAFT_X: thrust line passes through the tilt pivot
for lbl, arm in (("as coded (motor_x - CG)", T.ARM_TILT_M), ("pivot - CG", (PIVOT_X - T.X_CG_MM) / 1000.0)):
    T.ARM_TILT_M = arm
    hv = T.hover_envelope()
    _, ob = T.sim_outbound()
    _, ib = T.sim_inbound()
    print(f"\n--- {lbl}: arm {arm*1000:.1f} mm")
    print("  hover   :", {k: hv[k] for k in ("hover_pitch_max_Nm", "hover_pitch_accel_rad_s2", "hover_wind_hold_ms")})
    print("  outbound:", ob)
    print("  inbound :", ib)
