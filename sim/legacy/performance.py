# LEGACY (moved from sim/): superseded Kestrel-600 single-file model; not imported by anything; current numbers come from sim/aircraft.py + harness.py.
"""Kestrel-600 performance simulation.

Inputs that are measured or published (sources in SOURCES):
  - motor + prop static bench test: Emax Eco 2306 2400KV with HQ 6x3.5, MiniQuadTestBench
  - prop behaviour with airspeed: APC 6x4E performance file (shape only, rescaled to the bench prop)
  - cell: Molicel INR-21700-P45B
Inputs that are my estimates: drag build-up, airframe mass, avionics power, ESC idle loss scaling.
Run:  python sim/performance.py
"""
import math

SOURCES = {
    "bench": "https://www.miniquadtestbench.com/assets/components/motordata/motorinfo.php?uid=259",
    "prop": "https://www.apcprop.com/files/PER3_6x4E.dat",
    "cell": "https://www.molicel.com/inr-21700-p45b/",
}
RHO, G, NU = 1.19, 9.81, 1.5e-5

# ---- geometry (matches src/kestrel.py) ------------------------------------
SPAN, ROOT_C, TIP_C, HALF = 0.610, 0.240, 0.120, 0.302
S = 2 * HALF * (ROOT_C + TIP_C) / 2
AR = SPAN**2 / S
LAM = TIP_C / ROOT_C
MAC = 2 / 3 * ROOT_C * (1 + LAM + LAM**2) / (1 + LAM)
Y_MAC = 2 * HALF / 6 * (1 + 2 * LAM) / (1 + LAM)
X_AC = (0.070 - 0.130 * Y_MAC / HALF) - 0.25 * MAC      # metres, CAD frame (+x forward)

# ---- propulsion: bench data, HQ 6x3.5 on Emax Eco 2306 (tested 2300 KV) ---
D = 6 * 0.0254
KV = 2300.0
BENCH = [  # rpm, volts, amps, thrust g
    (5434, 16.14, 0.89, 49), (10104, 16.11, 2.45, 191), (16620, 16.02, 8.31, 532),
    (22221, 15.84, 19.19, 957), (26932, 15.54, 37.35, 1461)]
MOTOR_G = 29.7
# APC 6x4E at 14000 rpm: J, Ct, Cp
APC = [(0, .119, .0498), (.0565, .1158, .0513), (.1129, .112, .0526), (.1694, .1074, .0535),
       (.2259, .1019, .0541), (.2824, .0956, .054), (.3388, .0883, .0532), (.3953, .08, .0515),
       (.4518, .0707, .0488), (.5082, .0606, .0451), (.5647, .05, .0405), (.6212, .039, .0352),
       (.6776, .0277, .0291), (.7341, .0164, .0223), (.7906, .0054, .0151), (.8188, 0, .0113)]
PITCH_SCALE = 3.5 / 4.0

# ---- battery: Molicel P45B --------------------------------------------------
CELL_WH, CELL_V, CELL_R, CELL_G, CELL_AMAX = 16.2, 3.6, 0.0138, 70.0, 45.0
USABLE = 0.85            # fraction of energy used before landing
AVIONICS_W = 5.0         # FC + GPS + receiver + analog camera/VTX (estimate)
HOVER_S = 60.0           # take-off plus landing hover per flight


def interp(table, x, col):
    if x <= table[0][0]:
        return table[0][col]
    for a, b in zip(table, table[1:]):
        if x <= b[0]:
            t = (x - a[0]) / (b[0] - a[0])
            return a[col] + t * (b[col] - a[col])
    return table[-1][col]


def fit_bench():
    """Static Ct, Cp, total resistance, no-load current and fixed ESC loss from the bench rows."""
    rpm, v, a, g = BENCH[-1]                               # full throttle: duty = 1
    e = rpm / KV
    r = (v - e) / a
    n = lambda rpm: rpm / 60
    ct0 = sum(gr / 1000 * G / (RHO * n(rp)**2 * D**4) for rp, _, _, gr in BENCH[1:]) / 4
    best = None
    for i0 in [x / 20 for x in range(4, 41)]:
        cp0 = (a - i0) * e / (RHO * n(rpm)**3 * D**5)
        for p0 in [x / 2 for x in range(0, 25)]:
            err = 0
            for rp, vv, aa, _ in BENCH:
                shaft = cp0 * RHO * n(rp)**3 * D**5
                ee = rp / KV
                im = shaft / ee + i0
                pred = (ee + im * r) * im + p0
                err += ((pred - vv * aa) / (vv * aa))**2
            if best is None or err < best[0]:
                best = (err, i0, p0, cp0)
    err, i0, p0, cp0 = best
    return dict(ct0=ct0, cp0=cp0, r=r, i0=i0, p0=p0, rms=math.sqrt(err / len(BENCH)))


FIT = fit_bench()


def prop(j):
    """Ct, Cp at advance ratio j: APC shape, rescaled to the bench prop's pitch and static values."""
    ja = j / PITCH_SCALE
    return (interp(APC, ja, 1) * FIT["ct0"] / APC[0][1],
            interp(APC, ja, 2) * FIT["cp0"] / APC[0][2])


def motor_input_w(n, v_air, vbat, loss=1.0):
    """Electrical power into one ESC at rev/s n and airspeed; None if it needs more than full duty."""
    ct, cp = prop(v_air / (n * D))
    shaft = cp * RHO * n**3 * D**5
    e = n * 60 / KV
    im = shaft / e + FIT["i0"] * loss
    vm = e + im * FIT["r"]
    if vm > vbat:
        return None
    return vm * im + FIT["p0"] * loss * vbat / 16.0, ct * RHO * n**2 * D**4


def rpm_for_thrust(thrust, v_air):
    lo, hi = max(v_air / (0.80 * PITCH_SCALE * D), 5.0), 700.0
    for _ in range(60):
        mid = (lo + hi) / 2
        ct, _ = prop(v_air / (mid * D))
        if ct * RHO * mid**2 * D**4 < thrust:
            lo = mid
        else:
            hi = mid
    return hi


def full_throttle(v_air, cells):
    """Total thrust (N), pack current (A) and pack voltage at full duty, with battery sag."""
    voc, rb = cells * 4.0, cells * CELL_R + 0.012          # 4.0 V/cell: mostly-charged, pack + wiring
    lo, hi = 20.0, 700.0
    for _ in range(60):
        n = (lo + hi) / 2
        _, cp = prop(v_air / (n * D))
        im = cp * RHO * n**3 * D**5 / (n * 60 / KV) + FIT["i0"]
        vbat = voc - 2 * im * rb
        if n * 60 / KV + im * FIT["r"] < vbat:
            lo = n
        else:
            hi = n
    ct, _ = prop(v_air / (n * D))
    return 2 * ct * RHO * n**2 * D**4, 2 * im, vbat


def drag_buildup(v):
    q = 0.5 * RHO * v * v
    cf = lambda length: 0.074 / (v * length / NU)**0.2        # turbulent flat plate
    items = {
        "wing": cf(MAC) * 1.25 * 2.05 * S,
        "fuselage": cf(0.34) * 1.40 * 0.060,
        "tip fins": cf(0.12) * 1.05 * 0.056,
        "motor pods": 0.30 * 2 * math.pi * 0.014**2,
        "sensor ball": 0.40 * 3.0e-4,
        "antenna": 1.10 * 0.060 * 0.003,
    }
    items["gaps/roughness (+15%)"] = 0.15 * sum(items.values())
    return {k: f / S for k, f in items.items()}, q


def mass_table(cells, extra=0.0):
    """name: (grams, x position in mm, CAD frame). Battery x is solved for balance."""
    return {
        "airframe (LW-PLA + carbon spar, est.)": (190 + extra, -45),
        "motors x2": (2 * MOTOR_G, 38),
        "props x2": (8, 50),
        "ESCs x2": (16, 0),
        "flight controller stack": (23, 95),
        "camera + VTX": (20, 120),
        "GPS": (8, -95),
        "receiver": (2, -60),
        "servos x2": (24, -100),
        "wiring + connectors": (20, 0),
        "battery": (cells * CELL_G + 15, None),
    }


def analyse(label, cells, cd0_scale=1.0, mass_extra=0.0, loss=1.0, e_osw=0.80):
    table = mass_table(cells, mass_extra)
    m = sum(g for g, _ in table.values()) / 1000
    w = m * G
    k = 1 / (math.pi * AR * e_osw)
    vnom = cells * CELL_V
    rows = []
    for v10 in range(100, 400, 5):
        v = v10 / 10
        parts, q = drag_buildup(v)
        cd0 = sum(parts.values()) * cd0_scale
        cl = w / (q * S)
        if cl > 0.95:
            continue
        drag = q * S * (cd0 + k * cl * cl)
        n = rpm_for_thrust(drag / 2, v)
        res = motor_input_w(n, v, vnom, loss)
        if res is None:
            break
        p = 2 * res[0] + AVIONICS_W
        rows.append((v, cl / (cd0 + k * cl * cl), drag, n * 60, p))
    # hover: each motor carries half the weight, static
    nh = rpm_for_thrust(w / 2, 0.0)
    p_hover = 2 * motor_input_w(nh, 0.0, vnom, loss)[0] + AVIONICS_W
    energy = cells * CELL_WH * USABLE - p_hover * HOVER_S / 3600
    best_e = min(rows, key=lambda r: r[4])
    best_r = min(rows, key=lambda r: r[4] / r[0])
    t_max, i_max, v_sag = full_throttle(0.0, cells)
    vmax = max((v10 / 10 for v10 in range(100, 600, 5)
                if full_throttle(v10 / 10, cells)[0] >
                0.5 * RHO * (v10 / 10)**2 * S * (sum(drag_buildup(v10 / 10)[0].values()) * cd0_scale
                                                 + k * (w / (0.5 * RHO * (v10 / 10)**2 * S))**2)),
               default=0)
    v_stall = math.sqrt(2 * w / (RHO * S * 0.95))
    # balance: battery x for a 10 % static margin
    x_cg = X_AC * 1000 + 0.10 * MAC * 1000
    fixed = sum(g * x for g, x in table.values() if x is not None)
    gb = table["battery"][0]
    x_batt = (x_cg * m * 1000 - fixed) / gb
    return dict(label=label, mass=m * 1000, table=table, v_stall=v_stall, rows=rows,
                best_e=best_e, best_r=best_r, p_hover=p_hover, hover_rpm=nh * 60,
                hover_min=cells * CELL_WH * USABLE / p_hover * 60,
                endurance=energy / best_e[4] * 60, range_km=energy / best_r[4] * best_r[0] * 3.6,
                tw=t_max / w, i_max=i_max, v_sag=v_sag, vmax=vmax, x_cg=x_cg, x_batt=x_batt,
                cd0=sum(drag_buildup(best_r[0])[0].values()) * cd0_scale, ld=max(r[1] for r in rows))


def report(r):
    print(f"\n=== {r['label']} ===")
    print(f"  mass {r['mass']:.0f} g | wing loading {r['mass'] / 1000 / S:.1f} kg/m2 | "
          f"CD0 {r['cd0']:.3f} | best L/D {r['ld']:.1f}")
    print(f"  stall {r['v_stall']:.1f} m/s | top speed {r['vmax']:.0f} m/s ({r['vmax'] * 3.6:.0f} km/h)")
    v, _, d, rpm, p = r["best_e"]
    print(f"  loiter : {v:.1f} m/s ({v * 3.6:.0f} km/h), {p:.0f} W, {rpm:.0f} rpm -> {r['endurance']:.0f} min")
    v, _, d, rpm, p = r["best_r"]
    print(f"  cruise : {v:.1f} m/s ({v * 3.6:.0f} km/h), {p:.0f} W, {rpm:.0f} rpm -> {r['range_km']:.0f} km "
          f"({p / (v * 3.6):.2f} Wh/km)")
    print(f"  hover  : {r['p_hover']:.0f} W at {r['hover_rpm']:.0f} rpm | pure-hover time {r['hover_min']:.1f} min")
    print(f"  full throttle static: thrust/weight {r['tw']:.2f}, pack {r['i_max']:.0f} A "
          f"(cell limit {CELL_AMAX:.0f} A), sag to {r['v_sag']:.1f} V")
    print(f"  balance: CG target x={r['x_cg']:.0f} mm, battery centre must sit at x={r['x_batt']:.0f} mm")


if __name__ == "__main__":
    print(f"wing: S={S * 1e4:.0f} cm2, AR={AR:.2f}, MAC={MAC * 1000:.0f} mm, aerodynamic centre x={X_AC * 1000:.0f} mm")
    print(f"bench fit: Ct0={FIT['ct0']:.4f} Cp0={FIT['cp0']:.4f} R={FIT['r'] * 1000:.0f} mOhm "
          f"I0={FIT['i0']:.2f} A ESC fixed loss={FIT['p0']:.1f} W (rms error {FIT['rms'] * 100:.0f} %)")
    parts, _ = drag_buildup(17.0)
    print("drag build-up at 17 m/s (CD0 share):")
    for kname, cdv in parts.items():
        print(f"    {kname:24s} {cdv:.4f}")
    nominal = analyse("3S1P nominal", 3)
    print("mass budget (3S):")
    for kname, (g, x) in nominal["table"].items():
        print(f"    {kname:40s} {g:6.1f} g")
    report(nominal)
    report(analyse("3S1P optimistic (drag -15 %, 20 g lighter, e=0.85)", 3, 0.85, -20, 1.0, 0.85))
    report(analyse("3S1P pessimistic (drag +30 %, 50 g heavier)", 3, 1.30, 50, 1.0, 0.75))
    report(analyse("4S1P nominal", 4))
    report(analyse("3S1P what-if: smaller motor, half the idle losses (hypothetical)", 3, loss=0.5))
    print("\nspeed sweep, 3S1P nominal:  m/s   L/D   drag N   rpm    W    min   km")
    e = 3 * CELL_WH * USABLE - nominal["p_hover"] * HOVER_S / 3600
    for v, ld, d, rpm, p in nominal["rows"][::4]:
        print(f"                           {v:5.1f} {ld:5.1f} {d:7.2f} {rpm:6.0f} {p:5.0f} {e / p * 60:5.0f} {e / p * v * 3.6:5.0f}")
