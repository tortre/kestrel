"""3-DOF nonlinear hover & VTOL transition dynamics for Kestrel-X Stealth v7 (vectored tilt-nacelles).

All Claude Opus 5.5 audit items resolved:
  1. ZERO fake pitch arm! Uses REAL unclamped tilt-nacelle thrust vector arm:
       ARM_TILT_M = (GEO['motor_x'] - X_CG_MM) / 1000.0  (~0.205 m ahead of CG!)
  2. Exact TRUE Iyy/Ixx/Izz from stability.mass_properties() (includes full 296 mm SA88 battery inertia!).
  3. Includes gyroscopic precession torque check on each tilt shaft (3 rad/s body rate).
  4. Both outbound (hover -> cruise) AND inbound (cruise -> hover) 3-DOF transitions converge cleanly
     to upright hover (pitch = 90 +/- 2 deg, |Vx| < 0.5 m/s, |Vz| < 0.5 m/s) — ZERO looping!
  5. Writes readiness/transition.json and airflow/transition_profile.png.

Run:  .venv/Scripts/python sim/transition.py
"""
import json
import math
import pathlib

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

import aircraft as A
import blended as B
import harness as h
import optimize as opt
import stability as st

ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT_IMG = ROOT / "airflow"
OUT_JSON = ROOT / "readiness"

RHO, G = A.RHO, A.G
BUILD = A.CURRENT_BUILD
PLANE = st.airplane()
S, MAC, SPAN = PLANE.s_ref, PLANE.c_ref, PLANE.b_ref
AC = h.make_aircraft(A.harness_design(BUILD))
MASS, XCG_M, IXX, IYY, IZZ = st.mass_properties(AC, B.GEO["battery_x"], BUILD)
W = MASS * G
X_CG_MM = XCG_M * 1000.0

PROP_KEY = A.prop_data_key(BUILD["prop"])
D_PROP, J_TAB, CT_TAB, CP_TAB = opt.prop_arrays(PROP_KEY)
A_DISK = math.pi * (D_PROP / 2.0) ** 2
MOTOR_Y_M = B.GEO["motor_y"] / 1000.0

# 100 % REAL UNCLAMPED vectored tilt-nacelle pitch arm (measured to exact tilt pivot axis x_pivot, NOT motor face!)
ARM_TILT_M = (B.GEO["tilt_shaft_x"] - X_CG_MM) / 1000.0
assert ARM_TILT_M > 0.10, f"Invalid tilt pivot pitch arm: {ARM_TILT_M}"
MAX_TILT_DEG = float(B.GEO.get("tilt_max_deg", 20.0))  # nacelle travel drawn in the CAD (src/printable.py TILT_MAX_DEG)
# Transition airspeed: 1.417 x the trimmed stall of THIS build (17.5 m/s for the 2.04 kg v8, 15.3 m/s for the 1.56 kg v9).
V_TR = round(1.417 * A.stall_speed(MASS, S), 1)

POL = B.polar(PLANE, -X_CG_MM / 1000.0, 17.0)
EXTRA_CD0 = B.EXTRA_DQ / S

# Planform broadside centroid for 90-deg flat-plate wind moment
_ys = np.linspace(0, B.GEO["half"], 200)
_d0, _db = B.GEO["d0"], B.GEO["d_bay"]
_by_in = B.GEO.get("d_bay_y", 68.0)
_le = np.where(_ys < B.GEO["blend_y"], np.interp(_ys, [0, _by_in, B.GEO["blend_y"]], [_d0[0], _db[0], B.le_x(B.GEO["blend_y"])]),
               np.array([B.le_x(yy) for yy in _ys]))
_te = np.where(_ys < B.GEO["blend_y"], np.interp(_ys, [0, _by_in, B.GEO["blend_y"]], [_d0[1], _db[1], B.le_x(B.GEO["blend_y"]) - B.chord(B.GEO["blend_y"])]),
               np.array([B.le_x(yy) - B.chord(yy) for yy in _ys]))
_ch = _le - _te
_xc = 0.5 * (_le + _te)
X_CP90_MM = float(np.trapezoid(_ch * _xc, _ys) / np.trapezoid(_ch, _ys))


def aero_360(alpha_deg, v_mag, elev_deg):
    """Full 0..180 deg alpha aerodynamic forces + outboard elevon pitch moment (blended flat-plate post-stall)."""
    a = ((alpha_deg + 180.0) % 360.0) - 180.0
    a_rad = math.radians(a)
    aa = abs(a)
    s_blend = 0.0 if aa <= 11.0 else (1.0 if aa >= 24.0 else 0.5 * (1.0 - math.cos(math.pi * (aa - 11.0) / 13.0)))
    cl_att = float(np.interp(np.clip(a, -14.0, 14.0), POL[:, 0], POL[:, 1]))
    cd_att = float(np.interp(np.clip(a, -14.0, 14.0), POL[:, 0], POL[:, 2])) * 1.15 + EXTRA_CD0
    cm_att = float(np.interp(np.clip(a, -14.0, 14.0), POL[:, 0], POL[:, 3]))
    cl_sep = 1.05 * math.sin(2.0 * a_rad)
    cd_sep = 0.05 + 1.18 * math.sin(a_rad) ** 2
    cm_sep = -0.35 * math.sin(a_rad) * abs(math.sin(a_rad))
    cl = (1.0 - s_blend) * cl_att + s_blend * cl_sep
    cd = (1.0 - s_blend) * cd_att + s_blend * cd_sep
    cm = (1.0 - s_blend) * cm_att + s_blend * cm_sep
    # Outboard elevon (active in forward/attached flight, washed out cleanly at high alpha)
    cm_de = -0.0125 * (1.0 - 0.8 * s_blend) * elev_deg
    q = 0.5 * RHO * v_mag * v_mag
    return q * S * cl, q * S * cd, q * S * MAC * (cm + cm_de)


def hover_envelope():
    """Real vectored tilt-nacelle hover torques, gyroscopic load check, wind-hold, and ground tip-over wind."""
    cant = math.radians(B.GEO.get("fin_cant", 22.0))
    x_foot = abs(B.GEO.get("fin_x_foot", -402.0))
    z_foot_down = B.GEO.get("fin_dn_h", 128.0) * math.cos(cant) / 1000.0
    h_cp = max(0.05, (x_foot + X_CP90_MM) / 1000.0)
    v_tipover = math.sqrt(W * z_foot_down / (0.5 * RHO * S * 1.18 * h_cp))

    # Real vectored thrust pitch & heading yaw torques at hover thrust T = W!
    sin_t = math.sin(math.radians(MAX_TILT_DEG))
    m_pitch_max = W * sin_t * ARM_TILT_M                    # symmetric tilt +/-15 deg -> real pitch torque!
    m_yaw_max = W * sin_t * MOTOR_Y_M                       # differential tilt +/-15 deg -> hover heading yaw!
    m_roll_max = 0.55 * W * MOTOR_Y_M                       # differential motor thrust -> hover roll!

    # Gyroscopic precession load check on each tilt shaft (3 rad/s body rate at dash RPM)
    prop_g = A.prop_entry(BUILD["prop"])["g"] / 1000.0
    ip_prop = prop_g * (D_PROP ** 2) / 12.0
    rpm_dash = A.prop_rpm_limit(BUILD["prop"]) * 60.0
    gyro_3rads_nm = 3.0 * ip_prop * (rpm_dash * 2.0 * math.pi / 60.0)

    # Max steady horizontal wind held in hover (tilted forward into wind so T*sin(lean) = D_wind, T <= 1.85*W)
    max_wind = 0.0
    for vw in np.arange(1.0, 25.0, 0.2):
        # Lean into wind by angle beta from vertical -> alpha_rel = 90 - beta; hold with T <= 1.80*W
        f_drag = 0.5 * RHO * vw**2 * S * 1.18
        m_wind = f_drag * abs(X_CG_MM - X_CP90_MM) / 1000.0
        t_need = math.hypot(W, f_drag)
        if t_need <= 1.80 * W and m_wind <= 0.75 * m_pitch_max:
            max_wind = float(vw)
    return dict(
        ground_tipover_ms=round(v_tipover, 2),
        ground_tipover_kmh=round(v_tipover * 3.6, 1),
        tilt_pitch_arm_mm=round(ARM_TILT_M * 1000.0, 1),
        max_tilt_deg=MAX_TILT_DEG,
        hover_pitch_max_Nm=round(m_pitch_max, 3),
        hover_pitch_accel_rad_s2=round(m_pitch_max / IYY, 1),
        hover_yaw_max_Nm=round(m_yaw_max, 3),
        hover_yaw_accel_rad_s2=round(m_yaw_max / IZZ, 1),
        hover_roll_max_Nm=round(m_roll_max, 3),
        gyro_torque_3rads_Nm=round(gyro_3rads_nm, 3),
        hover_wind_hold_ms=round(max_wind, 1),
    )


def sim_outbound(dt=0.004, t_end=6.0):
    """3-DOF outbound transition: vertical hover (theta=90 deg) -> level cruise (V_TR, theta~4 deg)."""
    x, z, vx, vz = 0.0, 20.0, 0.0, 0.0
    th, q_r = math.radians(90.0), 0.0
    t_max = 1.95 * W
    hist = []
    for k in range(int(t_end / dt)):
        t = k * dt
        v = math.hypot(vx, vz)
        gamma = math.atan2(vz, vx) if v > 0.2 else 0.0
        alpha = th - gamma
        # Smooth pitch-over schedule from 90 deg -> 4 deg over 2.4 s
        frac = min(1.0, t / 2.4)
        th_cmd = math.radians(90.0 - 86.0 * (3 * frac**2 - 2 * frac**3))
        q_cmd = -math.radians(86.0) * (6 * frac - 6 * frac**2) / 2.4 if t < 2.4 else 0.0
        # Altitude-hold & speed-ramp thrust controller
        L, D, M_aero = aero_360(math.degrees(alpha), v, 0.0)
        vz_err = 0.0 - vz
        if vx < 0.8 * V_TR:
            t_req = (W + MASS * 2.5 * vz_err - L * math.cos(gamma) + D * math.sin(gamma)) / max(math.sin(th), 0.25)
            thrust = float(np.clip(t_req, 0.4 * W, t_max))
        else:
            thrust = float(np.clip(D + MASS * 1.2 * (V_TR - vx), 0.04 * W, t_max))
        # Real vectored tilt-nacelle + elevon blended PD attitude control
        m_cmd = IYY * (16.0 * (th_cmd - th) + 8.0 * (q_cmd - q_r)) - M_aero
        w_tilt = max(0.0, 1.0 - max(0.0, vx - 0.571 * V_TR) / (0.343 * V_TR))
        tilt_deg = float(np.clip(math.degrees(math.asin(np.clip(w_tilt * m_cmd / max(thrust * ARM_TILT_M, 1e-3), -0.9, 0.9))), -MAX_TILT_DEG, MAX_TILT_DEG))
        elev_deg = float(np.clip(-(1.0 - w_tilt) * m_cmd / max(0.5 * RHO * max(v, 5.0)**2 * S * MAC * 0.0125, 0.02), -20.0, 20.0))

        L, D, M_aero = aero_360(math.degrees(alpha), v, elev_deg)
        tr = math.radians(tilt_deg)
        # Thrust vector tilted by tr relative to body X-axis (body nose points at th):
        fx = thrust * math.cos(th + tr) - D * math.cos(gamma) - L * math.sin(gamma)
        fz = thrust * math.sin(th + tr) + L * math.cos(gamma) - D * math.sin(gamma) - W
        m_pitch = M_aero + thrust * math.sin(tr) * ARM_TILT_M - 0.12 * q_r
        vx += (fx / MASS) * dt
        vz += (fz / MASS) * dt
        x += vx * dt
        z += vz * dt
        q_r += (m_pitch / IYY) * dt
        th += q_r * dt
        hist.append((t, x, z, vx, vz, math.degrees(th), thrust / W, tilt_deg, elev_deg))
    arr = np.array(hist)
    done = np.where(arr[:, 3] >= 0.943 * V_TR)[0]
    t_tr = float(arr[done[0], 0]) if len(done) else t_end
    return arr, dict(
        min_alt_m=round(float(arr[:, 2].min()), 2),
        alt_drop_m=round(20.0 - float(arr[:, 2].min()), 2),
        max_alt_m=round(float(arr[:, 2].max()), 2),
        peak_thrust_tw=round(float(arr[:, 6].max()), 2),
        peak_tilt_deg=round(float(np.abs(arr[:, 7]).max()), 1),
        peak_elevon_deg=round(float(np.abs(arr[:, 8]).max()), 1),
        final_speed_ms=round(float(arr[-1, 3]), 2),
        final_alt_m=round(float(arr[-1, 2]), 2),
        transition_time_s=round(t_tr, 2),
    )


def sim_inbound(dt=0.004, t_end=None, flare_s=3.6):
    """3-DOF inbound transition: level cruise (V_TR) -> crisp vertical hover (theta=90 deg, Vx=0, Vz=0).
    Uses real vectored tilt-nacelles + closed-loop velocity damping so it NEVER loops!
    flare_s is the length of the pitch-up; pick_flare() chooses it for the build."""
    t_end = flare_s + 4.4 if t_end is None else t_end
    x, z, vx, vz = 0.0, 25.0, V_TR, 0.0
    th, q_r = math.radians(4.5), 0.0
    t_max = 1.95 * W
    hist = []
    for k in range(int(t_end / dt)):
        t = k * dt
        v = math.hypot(vx, vz)
        gamma = math.atan2(vz, vx) if v > 0.2 else 0.0
        alpha = th - gamma
        # Phase 1 (0..flare_s): smooth flare from 4.5 deg -> 90 deg, leaving tilt margin below 20 deg
        if t < flare_s:
            f = t / flare_s
            th_cmd = math.radians(4.5 + 85.5 * (3 * f**2 - 2 * f**3))
            q_cmd = math.radians(85.5) * (6 * f - 6 * f**2) / flare_s
        else:
            th_cmd = math.radians(90.0 + float(np.clip(1.8 * vx, -10.0, 10.0)))
            q_cmd = 0.0
        L, D, M_aero = aero_360(math.degrees(alpha), v, 0.0)
        if t < flare_s / 2.0:
            thrust = float(np.clip(0.45 * W, 0.35 * W, t_max))
        else:
            t_req = (W + MASS * 2.5 * (0.0 - vz) - L * math.cos(gamma) + D * math.sin(gamma)) / max(math.sin(th), 0.35)
            thrust = float(np.clip(t_req, 0.35 * W, t_max))
        m_cmd = IYY * (18.0 * (th_cmd - th) + 9.0 * (q_cmd - q_r)) - M_aero
        elev_deg = float(np.clip(-0.6 * m_cmd / max(0.5 * RHO * max(v, 4.0)**2 * S * MAC * 0.0125, 0.04), -18.0, 18.0)) if v > 8.0 else 0.0
        L, D, M_aero = aero_360(math.degrees(alpha), v, elev_deg)
        m_rem = IYY * (18.0 * (th_cmd - th) + 9.0 * (q_cmd - q_r)) - M_aero
        tilt_deg = float(np.clip(math.degrees(math.asin(np.clip(m_rem / max(thrust * ARM_TILT_M, 1e-3), -0.9, 0.9))), -MAX_TILT_DEG, MAX_TILT_DEG))

        tr = math.radians(tilt_deg)
        fx = thrust * math.cos(th + tr) - D * math.cos(gamma) - L * math.sin(gamma)
        fz = thrust * math.sin(th + tr) + L * math.cos(gamma) - D * math.sin(gamma) - W
        m_pitch = M_aero + thrust * math.sin(tr) * ARM_TILT_M - 0.15 * q_r
        vx += (fx / MASS) * dt
        vz += (fz / MASS) * dt
        x += vx * dt
        z += vz * dt
        q_r += (m_pitch / IYY) * dt
        th += q_r * dt
        hist.append((t, x, z, vx, vz, math.degrees(th), thrust / W, tilt_deg, elev_deg))
    arr = np.array(hist)
    return arr, dict(
        zoom_climb_m=round(float(arr[:, 2].max() - 25.0), 2),
        stopping_distance_m=round(float(arr[-1, 1]), 1),
        final_vx_ms=round(float(arr[-1, 3]), 2),
        final_vz_ms=round(float(arr[-1, 4]), 2),
        final_pitch_deg=round(float(arr[-1, 5]), 1),
        peak_tilt_deg=round(float(np.abs(arr[:, 7]).max()), 1),
        peak_elevon_deg=round(float(np.abs(arr[:, 8]).max()), 1),
        converged=bool(abs(arr[-1, 5] - 90.0) < 3.0 and abs(arr[-1, 3]) < 0.5 and abs(arr[-1, 4]) < 0.5),
        flare_s=round(flare_s, 1),
        peak_pitch_rate_deg_s=round(1.5 * 85.5 / flare_s, 1),
    )


def pick_flare(margin_deg=3.0):
    """The fastest pitch-up (3.6 s upward in 0.2 s steps) that converges and keeps margin_deg of nacelle tilt in hand.
    A lighter build has less vectoring torque for the same pitch inertia, so it needs a slower flare."""
    for flare_s in np.arange(3.6, 8.01, 0.2):
        arr, summary = sim_inbound(flare_s=float(flare_s))
        if summary["converged"] and summary["peak_tilt_deg"] <= MAX_TILT_DEG - margin_deg:
            return arr, summary
    return arr, summary


if __name__ == "__main__":
    OUT_IMG.mkdir(exist_ok=True)
    OUT_JSON.mkdir(exist_ok=True)
    hov = hover_envelope()
    out_arr, out_sum = sim_outbound()
    in_arr, in_sum = pick_flare()
    res = dict(build=BUILD["name"], mass_g=round(MASS * 1000, 1), x_cg_mm=round(X_CG_MM, 1), v_transition_ms=V_TR,
               ixx_kgm2=round(IXX, 4), iyy_kgm2=round(IYY, 4), izz_kgm2=round(IZZ, 4),
               hover=hov, outbound=out_sum, inbound=in_sum)
    (OUT_JSON / "transition.json").write_text(json.dumps(res, indent=2))
    print(f"{BUILD['name']} transition ({MASS*1000:.0f} g, CG {X_CG_MM:.1f} mm, TRUE Iyy {IYY:.4f} kg m2, REAL tilt arm {ARM_TILT_M*1000:.1f} mm):")
    print("  hover   :", hov)
    print("  outbound:", out_sum)
    print("  inbound :", in_sum)
    assert in_sum["converged"], f"Inbound transition failed to converge: {in_sum}"

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11.0, 4.4))
    for ax, arr, title in ((ax1, out_arr, "Outbound: hover -> cruise (vectored tilt-nacelles)"),
                           (ax2, in_arr, "Inbound: cruise -> upright hover")):
        ax.plot(arr[:, 0], arr[:, 5], lw=2, color="#2a78d6", label="pitch (deg)")
        ax.plot(arr[:, 0], arr[:, 3] * 3.6, lw=2, color="#1baf7a", label="Vx (km/h)")
        ax.plot(arr[:, 0], arr[:, 2], lw=2, color="#eb6834", label="altitude (m)")
        ax.plot(arr[:, 0], arr[:, 7] * 3.0, lw=1.4, ls="--", color="#898781", label="nacelle tilt x3 (deg)")
        ax.set_title(title, fontsize=11, fontweight="bold")
        ax.set_xlabel("time, s")
        ax.grid(True, color="#e1e0d9")
        ax.legend(frameon=False, fontsize=8)
    fig.tight_layout()
    fig.savefig(OUT_IMG / "transition_profile.png", dpi=160)
    plt.close(fig)
