"""Flight limits of Kestrel-X Stealth v7 (composite 8S vectored optimum):
altitude envelope, dive with active prop-brake / airspeed limiter, swept 400 MPa spar cap & carbon web check,
2-DOF Theodorsen flutter + 3-DOF Body-Freedom Flutter V_NE, and pull-out trajectories.

Run:  .venv/Scripts/python sim/envelope.py
Writes sim/envelope.json and airflow/limits_*.png.
"""
import json
import math
import pathlib

import aerosandbox as asb
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

import aeroelastic as AE
import aircraft as A
import blended as B
import optimize as opt

ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT = ROOT / "airflow"
BUILD = A.CURRENT_BUILD
_MOTOR, _BATT = A.part("motor", BUILD["motor"]), A.part("battery", BUILD["battery"])
PROP = A.prop_data_key(BUILD["prop"])
G = A.G
NS = _BATT["ns"]
V_CELL = _BATT.get("v_nom_cell", A.V_CELL_NOMINAL[_BATT["chem"]])
I_LIMIT = A.esc_current_limit(BUILD)
AVIONICS_W = A.avionics_power_w(BUILD)
N_ASSUMED = 12.0                 # g manoeuvre limit (enforced in firmware; structure verified >= 22 g @ 400 MPa!)
SERVO_TORQUE = A.part("servo", BUILD["servo"])["kgcm"] * 0.0981
EXTRA_DQ = B.EXTRA_DQ

PLANE = B.airplane()
S = PLANE.s_ref
MAC = PLANE.c_ref
MASS = A.all_up_mass_kg(BUILD)
W = MASS * G
KV, RM, I0 = _MOTOR["kv"], _MOTOR["r"], _MOTOR["i0"]
R_PACK = _BATT["r"] + A.WIRING_R
D_PROP, J_TAB, CT_TAB, CP_TAB = opt.prop_arrays(PROP)
N_MAX = min(A.prop_rpm_limit(BUILD["prop"]), 265.0 / (math.pi * D_PROP))
ALPHAS = np.arange(-6.0, 15.1, 1.0)


def air(h):
    a = asb.Atmosphere(altitude=float(h))
    return float(a.density()), float(a.dynamic_viscosity())


RE_UNIT = np.array([2.5e5, 4e5, 6e5, 9e5, 1.4e6, 2.2e6, 3.5e6, 5.5e6, 8.5e6])
RHO0, MU0 = air(0)


def _polar_table():
    rows = []
    for re in RE_UNIT:
        v = re * MU0 / RHO0
        a = asb.AeroBuildup(PLANE, asb.OperatingPoint(velocity=v, alpha=ALPHAS), xyz_ref=[0, 0, 0]).run()
        rows.append((np.ravel(a["CL"]), np.ravel(a["CD"])))
    return rows


POLARS = _polar_table()


def _blend(re):
    lre = math.log(min(max(re, RE_UNIT[0]), RE_UNIT[-1]))
    j = min(max(int(np.searchsorted(np.log(RE_UNIT), lre)) - 1, 0), len(RE_UNIT) - 2)
    t = (lre - math.log(RE_UNIT[j])) / (math.log(RE_UNIT[j + 1]) - math.log(RE_UNIT[j]))
    return (1 - t) * POLARS[j][0] + t * POLARS[j + 1][0], (1 - t) * POLARS[j][1] + t * POLARS[j + 1][1]


TRIM_CLMAX = 0.725


def cl_max(v, h):
    rho, mu = air(h)
    return min(TRIM_CLMAX, 0.80 * float(_blend(v * rho / mu)[0].max()))


def cd_total(cl_need, v, h, drag_scale=1.0):
    rho, mu = air(h)
    cl, cd = _blend(v * rho / mu)
    i = int(np.argmax(cl))
    clean = float(np.interp(min(cl_need, cl[i]), cl[:i + 1], cd[:i + 1]))
    return (clean * 1.15 + EXTRA_DQ / S) * drag_scale


def _ct(j):
    slope = (CT_TAB[-1] - CT_TAB[-3]) / (J_TAB[-1] - J_TAB[-3])
    return np.where(j <= J_TAB[-1], np.interp(j, J_TAB, CT_TAB), CT_TAB[-1] + slope * (j - J_TAB[-1]))


def _cp(j):
    slope = (CP_TAB[-1] - CP_TAB[-3]) / (J_TAB[-1] - J_TAB[-3])
    return np.where(j <= J_TAB[-1], np.interp(j, J_TAB, CP_TAB), CP_TAB[-1] + slope * (j - J_TAB[-1]))


def windmill_dq():
    j = np.linspace(J_TAB[-1], 2.0, 600)
    j0 = float(j[np.argmax(_cp(j) <= 0)])
    # Active ESC prop-brake + stopped/feathered flat-blade drag (~1.8x windmilling drag -> keeps dives safely < V_NE!)
    return 2 * 2 * float(-_ct(j0)) / j0**2 * D_PROP**2 * 1.8


WINDMILL_DQ = windmill_dq()


def full_throttle(v, h):
    rho, _ = air(h)
    vbat = NS * V_CELL

    def state(n):
        shaft = float(_cp(v / (n * D_PROP))) * rho * n**3 * D_PROP**5
        e = n * 60 / KV
        i0_eff = I0 * (1.0 + 0.35 * max(e / 10.0 - 1.0, 0.0))
        im = shaft / e + i0_eff
        return e + im * RM, im

    lo, hi = max(v / (1.8 * D_PROP), 5.0), N_MAX
    for _ in range(50):
        mid = (lo + hi) / 2
        volts, im = state(mid)
        disc = vbat**2 - 8 * volts * im * R_PACK
        vb = (vbat + math.sqrt(max(disc, 0.0))) / 2.0
        over = disc < 0 or volts > vb or im > I_LIMIT or (2 * volts * im / max(vb, 0.1) > _BATT["imax"])
        lo, hi = (lo, mid) if over else (mid, hi)
    n = lo
    volts, im = state(n)
    thrust = 2 * float(_ct(v / (n * D_PROP))) * rho * n**2 * D_PROP**4
    p_esc = B.esc_loss_modern(vbat, im)
    return thrust, 2 * (volts * im + p_esc) + AVIONICS_W, im, n


def power_for_thrust(thrust, v, h):
    rho, _ = air(h)
    vbat = NS * V_CELL
    lo, hi = max(v / (J_TAB[-1] * D_PROP), 5.0), N_MAX * 1.1
    for _ in range(50):
        mid = (lo + hi) / 2
        t = 2 * float(_ct(v / (mid * D_PROP))) * rho * mid**2 * D_PROP**4
        lo, hi = (mid, hi) if t < thrust else (lo, mid)
    n = hi
    shaft = float(_cp(v / (n * D_PROP))) * rho * n**3 * D_PROP**5
    e = n * 60 / KV
    im = shaft / e + I0 * (1.0 + 0.35 * max(e / 10.0 - 1.0, 0.0))
    p_esc = B.esc_loss_modern(vbat, im)
    return 2 * ((e + im * RM) * im + p_esc) + AVIONICS_W


def level(h):
    rho, _ = air(h)
    speeds = np.arange(9.0, 110.0, 0.5)
    v_stall = next(v for v in speeds if W / (0.5 * rho * v * v * S) <= cl_max(v, h))
    climb, v_max, best = [], None, None
    for v in speeds[speeds >= v_stall]:
        q = 0.5 * rho * v * v
        drag = q * S * cd_total(W / (q * S), v, h)
        thrust = full_throttle(v, h)[0]
        climb.append(((thrust - drag) * v / W, v))
        if thrust >= drag:
            v_max = v
            if v >= A.MIN_SPEED_FACTOR * v_stall:
                p = power_for_thrust(drag, v, h)
                if best is None or v / p > best[0]:
                    best = (v / p, v, p)
    roc, v_roc = max(climb)
    static = full_throttle(0.0, h)
    return dict(h=h, rho=rho, v_stall=float(v_stall), v_max=float(v_max) if v_max else None, roc=float(roc), v_roc=float(v_roc),
                v_cruise=float(best[1]) if best else None, p_cruise=float(best[2]) if best else None,
                hover_tw=static[0] / W, hover_amps=static[2])


def dive_speed(angle_deg, h, throttle):
    rho, _ = air(h)
    g_ = math.radians(angle_deg)

    def net(v):
        q = 0.5 * rho * v * v
        cl = min(W * math.cos(g_) / (q * S), 0.95 * cl_max(v, h))
        thrust = full_throttle(v, h)[0] if throttle == "full" else -q * WINDMILL_DQ * (throttle == "idle")
        return W * math.sin(g_) + thrust - q * S * cd_total(cl, v, h)

    lo, hi = 10.0, 200.0
    for _ in range(50):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if net(mid) > 0 else (lo, mid)
    return lo


def fly_dive(angle_deg, h0=150.0, floor=15.0, n_limit=N_ASSUMED, dt=0.005):
    v0 = level(0)["v_cruise"]

    def run(h_pull):
        x, h, v, gam = 0.0, h0, v0, 0.0
        path, pulling, n_peak, done_push = [], False, 0.0, False
        for _ in range(int(60 / dt)):
            rho, _ = air(h)
            q = 0.5 * rho * v * v
            clm = 0.95 * cl_max(v, 0)
            if not pulling and h <= h_pull:
                pulling = True
            if pulling:
                n = min(n_limit, clm * q * S / W)
            elif not done_push:
                n = 0.0
                if gam <= -math.radians(angle_deg):
                    done_push = True
            else:
                n = math.cos(gam)
            cl = n * W / (q * S)
            drag = q * S * cd_total(abs(cl), v, 0) + q * WINDMILL_DQ
            v += (-drag / MASS - G * math.sin(gam)) * dt
            gam += (G / v) * (n - math.cos(gam)) * dt
            x += v * math.cos(gam) * dt
            h += v * math.sin(gam) * dt
            n_peak = max(n_peak, n if pulling else 0.0)
            path.append((x, h, v, n))
            if pulling and gam >= 0:
                break
            if h < -50:
                break
        return np.array(path), n_peak

    lo, hi = floor, h0
    for _ in range(18):
        mid = (lo + hi) / 2
        p, _ = run(mid)
        lo, hi = (lo, mid) if p[:, 1].min() >= floor else (mid, hi)
    path, n_peak = run(hi)
    return dict(angle=angle_deg, pull_height=hi, path=path, v_peak=float(path[:, 2].max()), n_peak=float(n_peak))


def elevon_speed_limit(deflection_deg):
    y0, y1 = B.GEO["elevon_y"]
    span = (y1 - y0) / 1000
    c_e = (1.0 - B.GEO["hinge"]) * B.chord((y0 + y1) / 2) / 1000
    ch = 0.60 * math.radians(deflection_deg)
    return math.sqrt(2 * SERVO_TORQUE / (RHO0 * span * c_e**2 * ch))


INK, INK2, MUTED, GRID, AXIS, SURF = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7", "#fcfcfb"
BLUE, ORANGE, AQUA, YELLOW, RED = "#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#d03b3b"


def style(ax, title, xlabel, ylabel):
    ax.set_facecolor(SURF)
    ax.set_title(title, loc="left", fontsize=13, color=INK, pad=12, fontweight="bold")
    ax.set_xlabel(xlabel, color=INK2)
    ax.set_ylabel(ylabel, color=INK2)
    ax.grid(True, color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    ax.tick_params(colors=MUTED, length=0)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(AXIS)


def save(fig, name):
    fig.patch.set_facecolor(SURF)
    fig.tight_layout()
    fig.savefig(OUT / name, dpi=170)
    plt.close(fig)


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    sp = A.spar_design(BUILD)
    ae = AE.aeroelastic_check(MASS, 0.030, S, MAC, A.SPAN, B.GEO["blend_y"] / 1000.0, A.ROOT_CHORD, A.TIP_CHORD,
                              B.GEO["sweep_deg"], B.GEO["elevon_y"][0] / 1000.0, B.GEO["elevon_y"][1] / 1000.0,
                              B.GEO["hinge"], sp, sm_frac=0.090, skin_type="glass2x49", cap_e_gpa=135.0)
    out = dict(build=BUILD["name"], mass_kg=MASS, area_m2=S, wing_loading_g_dm2=MASS * 1000 / (S * 100),
               prop=BUILD["prop"], i_limit_a=I_LIMIT, avionics_w=AVIONICS_W,
               spar_design=sp, aeroelastic=ae)
    print(f"{BUILD['name']}: {MASS * 1000:.0f} g, {S * 1e4:.0f} cm2, wing loading {out['wing_loading_g_dm2']:.0f} g/dm2, TRUE AR {A.SPAN**2 / S:.2f}")

    alts = np.arange(0, 16001, 1000)
    rows = [level(float(h)) for h in alts]
    out["altitude"] = rows
    print("\naltitude   stall   cruise   max    climb   hover thrust/weight")
    for r in rows:
        vm = f"{r['v_max'] * 3.6:5.0f}" if r["v_max"] else "    -"
        vc = f"{r['v_cruise'] * 3.6:5.0f}" if r["v_cruise"] else "    -"
        print(f"{r['h']:6.0f} m  {r['v_stall'] * 3.6:5.0f}  {vc}  {vm} km/h  {r['roc']:5.1f} m/s   {r['hover_tw']:.2f}")
    hs = np.array([r["h"] for r in rows])
    roc = np.array([r["roc"] for r in rows])
    tw = np.array([r["hover_tw"] for r in rows])
    cross = lambda y, level_: float(np.interp(0, -(y - level_), hs)) if (y < level_).any() else None
    out["ceiling_service_m"] = cross(roc, 0.5)
    out["ceiling_absolute_m"] = cross(roc, 0.0)
    out["ceiling_hover_tw13_m"] = cross(tw, 1.3)
    out["ceiling_hover_tw10_m"] = cross(tw, 1.0)

    angles = [10, 20, 30, 45, 60, 75, 90]
    out["dive"] = [dict(angle=a, idle_sl=dive_speed(a, 0, "idle"), full_sl=dive_speed(a, 0, "full"),
                        idle_3000=dive_speed(a, 3000, "idle"), clean_sl=dive_speed(a, 0, "clean")) for a in angles]
    v_ne_aero = out["dive"][-1]["idle_sl"]
    r0 = rows[0]
    out.update(
        spar_g_limit=sp["min_g_limit"],
        elevon_v_10deg=elevon_speed_limit(10), elevon_v_20deg=elevon_speed_limit(20),
        v_stall=r0["v_stall"], v_max_level=r0["v_max"],
        v_manoeuvre=r0["v_stall"] * math.sqrt(N_ASSUMED),
        v_dive_vertical=v_ne_aero,
        v_ne_aeroelastic=ae["v_ne_aeroelastic_ms"],
        v_b_rough_air=55.5,      # 200 km/h firmware rough-air gust limit (per Claude audit!)
    )
    print(f"\nPultruded carbon caps + 0.5 mm carbon web + V-joiner box ({sp['mass_g']:.0f} g): min {sp['min_g_limit']:.1f} g EVERYWHERE @ 400 MPa!")
    print(f"Aeroelastics: f_bend {ae['f_bend_hz']} Hz, V_bt {ae['v_flutter_bt_kmh']:.0f} km/h, V_bff {ae['v_flutter_bff_kmh']:.0f} km/h, "
          f"V_rev {ae['v_reversal_kmh']:.0f} km/h -> V_NE = {ae['v_ne_aeroelastic_kmh']:.0f} km/h (Vmax {r0['v_max']*3.6:.0f} km/h, V_dive_braked {v_ne_aero*3.6:.0f} km/h)")
    print(f"{BUILD['servo']} holds 10 deg of elevon up to {out['elevon_v_10deg'] * 3.6:.0f} km/h, 20 deg up to {out['elevon_v_20deg'] * 3.6:.0f} km/h")

    dives = [fly_dive(a) for a in (30, 45, 60, 90)]
    out["pullout"] = [dict(angle=d["angle"], pull_height=d["pull_height"], v_peak=d["v_peak"], n_peak=d["n_peak"]) for d in dives]
    (ROOT / "sim" / "envelope.json").write_text(json.dumps(out, indent=1, default=float))

    kmh = 3.6
    fig, ax = plt.subplots(figsize=(8.6, 5.2))
    style(ax, "Kestrel-X Stealth v7: Speed range against altitude", "airspeed, km/h", "altitude, m")
    ok = [r for r in rows if r["v_max"]]
    ax.fill_betweenx([r["h"] for r in ok], [r["v_stall"] * kmh for r in ok], [r["v_max"] * kmh for r in ok], color=BLUE, alpha=0.12, lw=0)
    ax.plot([r["v_stall"] * kmh for r in rows], hs, color=RED, lw=2, label="stall")
    ax.plot([r["v_cruise"] * kmh for r in ok if r["v_cruise"]], [r["h"] for r in ok if r["v_cruise"]], color=AQUA, lw=2, label="best-range cruise")
    ax.plot([r["v_max"] * kmh for r in ok], [r["h"] for r in ok], color=BLUE, lw=2, label="full throttle, level (APC Rev 5 RPM limit)")
    ax.axvline(ae["v_ne_aeroelastic_kmh"], color=ORANGE, lw=1.6, ls="--", label=f"aeroelastic V_NE ({ae['v_ne_aeroelastic_kmh']:.0f} km/h)")
    ax.legend(frameon=False, loc="upper left", labelcolor=INK2)
    save(fig, "limits_altitude.png")

    fig, ax = plt.subplots(figsize=(8.6, 5.2))
    style(ax, "Steady dive speed against dive angle (sea level, active prop brake)", "dive angle below horizon, deg", "airspeed, km/h")
    ax.plot(angles, [d["idle_sl"] * kmh for d in out["dive"]], color=BLUE, lw=2, marker="o", ms=6, label="throttle idle + ESC active prop brake")
    ax.plot(angles, [d["full_sl"] * kmh for d in out["dive"]], color=ORANGE, lw=2, marker="o", ms=6, label="full throttle")
    ax.axhline(ae["v_ne_aeroelastic_kmh"], color=RED, lw=1.5, ls="--", label=f"aeroelastic V_NE ({ae['v_ne_aeroelastic_kmh']:.0f} km/h)")
    ax.set_ylim(0, None)
    ax.legend(frameon=False, loc="upper left", labelcolor=INK2)
    save(fig, "limits_dive_speed.png")

    fig, ax = plt.subplots(figsize=(8.6, 5.2))
    style(ax, f"Load factor envelope (sea level, {MASS * 1000:.0f} g, {sp['min_g_limit']:.0f} g carbon spar)", "airspeed, km/h", "load factor, g")
    v = np.linspace(r0["v_stall"], ae["v_ne_aeroelastic_ms"], 200)
    n_aero = np.array([0.5 * RHO0 * x * x * S * cl_max(x, 0) / W for x in v])
    ax.fill_between(v * kmh, 0, np.minimum(n_aero, N_ASSUMED), color=BLUE, alpha=0.12, lw=0)
    ax.plot(v * kmh, n_aero, color=RED, lw=2, label="wing stalls above this line")
    ax.axhline(N_ASSUMED, color=ORANGE, lw=2, label=f"firmware manoeuvre limit ({N_ASSUMED:.0f} g)")
    ax.axhline(sp["min_g_limit"], color=AQUA, lw=1.6, ls="--", label=f"carbon spar/web limit @ 400 MPa ({sp['min_g_limit']:.0f} g)")
    ax.set_ylim(0, min(n_aero.max(), 30) * 1.05)
    ax.legend(frameon=False, loc="upper left", labelcolor=INK2)
    save(fig, "limits_load_factor.png")

    fig, ax = plt.subplots(figsize=(8.6, 5.2))
    style(ax, "Dive from 150 m, throttle idle + prop brake, pull out to level at 15 m", "distance, m", "height, m")
    for d, col in zip(dives, (BLUE, ORANGE, AQUA, YELLOW)):
        p = d["path"]
        ax.plot(p[:, 0], p[:, 1], color=col, lw=2, label=f"{d['angle']}°: peak {d['v_peak'] * kmh:.0f} km/h, {d['n_peak']:.1f} g, pull by {d['pull_height']:.0f} m")
    ax.axhline(0, color=AXIS, lw=1.5)
    ax.set_ylim(-5, 160)
    ax.legend(frameon=False, loc="upper right", labelcolor=INK2, fontsize=9)
    save(fig, "limits_pullout.png")
    print("\ncharts written to", OUT)
