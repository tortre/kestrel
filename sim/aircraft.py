"""Kestrel-X Stealth v9.1 Under-$300 (the v8 Budget airframe with a single 6S1P pack and a hill-climbed powertrain):
the ONE single source of truth for parts, exact CAD + E-glass + standard T700 spar mass, CG, and operating rules.
V8_BUDGET_BUILD keeps the previous $506 build for comparison (sim/budget_check.py).
"""
import json
import math
import pathlib

import aeroelastic as AE

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
CATALOG_PATH = HERE / "catalog.json"
MASS_REPORT_PATH = ROOT / "print" / "mass_report.json"

CAT = json.loads(CATALOG_PATH.read_text())
REPORT = json.loads(MASS_REPORT_PATH.read_text())
GEO = REPORT["geometry"]

# ---- the build (Kestrel-X Stealth v9.2 Under-$300: v9.1's hill-climbed parts on a re-worked airframe) ----
# v9.2 CAD changes (src/printable.py): spar caps 1.0/0.75/0.5 mm for a 2.10 kg design gross mass (v8: 1.5/1.0/0.75 for
# 2.5 kg), symmetric 148 mm X-tail (v8: 178 up / 118 down), battery brick 20 mm aft, nacelle tilt +/-30 deg (v8: 20).
# v9.1 is the end point of sim/hillclimb_v9.py: a greedy climb over motor, prop, ESC, pack, servo, GPS, receiver and
# flight controller at a $300 parts cap, confirmed against a full enumeration of the 6,455 parts sets that fit the cap.
# Against v8 (prices in catalog.json, read 22 Nov 2026):
#   battery  6S2P P45B -> ONE 6S1P Reliance RS40 brick (half the cells; 40 A continuous, 70 A burst, 5 mOhm tabless)
#   motors   T-Motor Cine66 2812 925KV -> Glee Hobby 2810 900KV (same 28xx can and 19 mm bolt circle, 9 g lighter each,
#            lower no-load current, which is the largest single loss in cruise)
#   props    APC 8x8E/EP -> APC 9x7.5E/EP (the CAD clears a 9 in disc by 37.8 mm)
#   ESCs     Flycolor 50 A BLHeli32 -> Readytosky 45 A BLHeli_S (Bluejay), fed from the flight controller's own PDB
#   servos   PTK 7308 -> Emax ES9051 on the outboard elevons (outside the prop wash, loads are small)
#   sensors  no pitot and no external Hall sensor (the FC's 80 A sensor covers a 40 A pack)
CURRENT_BUILD = dict(
    name="Kestrel-X Stealth v9.2 Under-$300 (hill-climbed parts + airframe, 6S1P, Walksnail HD)",
    motor="Glee Hobby 2810 900KV",
    prop="APC 9x7.5E / 9x7.5EP CW+CCW pair",
    battery="Reliance RS40 6S1P (6x21700, 86Wh, 40A)",
    esc="Readytosky 45A BLHeli_S (2-6S)",
    servo="Emax ES9051 4.3g",
    fc="SpeedyBee F405 Wing Mini",
    video="Camera + video transmitter (Walksnail Ascent Lite+ kit)",
    gps="GPS + compass (Quescan 18 mm M10 + QMC5883L)",
    receiver="Receiver (BetaFPV ELRS Nano 2.4G)",
    remote_id=None,
    hardware=(
        "Budget E-glass composite skin (2x49g/m2 +/-45 E-glass + epoxy + laminar finish, RF-transparent)",
        "Standard 135 GPa T700 pultruded carbon flat spar caps + 0.7mm web + 32x15mm root V-box",
        "Budget tilt-nacelle kit x2 (2x PTK 7455 MG-D 4.5kgcm + 5mm steel/CF shafts + 685ZZ bearings)",
        "6S XT60 harness on the FC PDB (16AWG to the pods + 2x 470uF), pin hinges, elevon mass balance, fasteners",
    ),
    material="LW-PLA (foaming)",
)
DUAL_PACK_BATTERY = "Reliance RS40 2x 6S1P in parallel (12x21700, 173Wh, 80A)"   # the drop-in second brick
DUAL_PACK_UPGRADE = dict(battery=DUAL_PACK_BATTERY)

# ---- the previous build (v8 Budget, $506 with pitot, 6S2P), kept for comparison -----------------------
V8_BUDGET_BUILD = dict(
    name="Kestrel-X Stealth v8 Budget (6S dual-pack composite optimum)",
    motor="T-Motor Cine66 2812 925KV",
    prop="APC 8x8E / 8x8EP CW+CCW pair (LP08080E + LP08080EP)",
    battery="Molicel P45B 6S2P (12x21700, 194Wh, 90A)",
    esc="Flycolor Raptor 5 50A BLHeli32 (3-6S)",
    servo="PTK 7308-MG-D 8mm 9.2g metal gear",
    fc="SpeedyBee F405 Wing Mini",
    video="Camera + video transmitter (Walksnail Ascent Lite+ kit)",
    gps="GPS + compass (Walksnail WS-M181, M10)",
    receiver="Receiver (HappyModel EP1 ELRS 2.4G)",
    remote_id=None,
    hardware=(
        "Budget E-glass composite skin (2x49g/m2 +/-45 E-glass + epoxy + laminar finish, RF-transparent)",
        "Standard 135 GPa T700 pultruded carbon flat spar caps + 0.7mm web + 32x15mm root V-box",
        "Budget tilt-nacelle kit x2 (2x PTK 7455 MG-D 4.5kgcm + 5mm steel/CF shafts + 685ZZ bearings)",
        "6S 10AWG XT60 direct ESC bus + 150A Hall sensor + Matek ASPD-4525 MS4525DO pitot + balance/hinges",
    ),
    material="LW-PLA (foaming)",
)

RHO = 1.225
G = 9.81
NU = 1.5e-5

SPAN = 2.0 * GEO["half"] / 1000.0        # 1.880 m TRUE projected planform span!
ROOT_CHORD = GEO["root_chord"] / 1000.0  # 0.178 m
TIP_CHORD = GEO["tip_chord"] / 1000.0    # 0.080 m

USABLE = 0.85
HOVER_S = 60.0
MISSION_HOVER_S = 120.0
MISSION_RESERVE = 0.20
MIN_SPEED_FACTOR = 1.25
TRIMMED_CLMAX = 0.729                    # trimmed CL_max on the blended reference area (readiness.py -> 37.4 km/h). v9.2 moved the
                                         # brick 20 mm aft (10.3 % static margin); v9.1 was 0.682 at 12.6 %, v8 0.699
CRUISE_V = 14.0                          # m/s (50 km/h): just above 1.25 x the 39 km/h trimmed stall of the 1.57 kg build
HEADLINE_SPEEDS = (14.0, 15.5)
V_CELL_NOMINAL = {"liion": 3.6, "lipo": 3.7, "si_liion": 3.45}
V_CELL_FULL = 4.2
ESC_SIZING_MARGIN = 1.0
WIRING_R = 0.010                         # ohm, XT60 + FC PDB shunt + 16 AWG leads to the two pod ESCs
SERVO_HOLD_W = 0.5
TUBE_G_PER_M = 26.0
AIRFRAME_DESIGN_GROSS_KG = 2.10          # what the CAD spar caps are sized for: two 6S1P bricks (1.94 kg) plus margin.
                                         # v8 sized them for 2.5 kg (a 6S3P pack this airframe no longer carries)

PLACEMENT = dict(
    motors=GEO["motor_x"],
    props=GEO["motor_x"] + 21.0,
    escs=GEO["firewall_x"] - 22.0,
    fc=GEO.get("fc_x", 180.5),
    video=GEO["cam_x"] - 45.0,
    gps=-135.0,
    remote_id=-138.0,
    receiver=-135.0,
    servos=GEO["servo_x"],
    hardware=0.0,
    battery=GEO["battery_x"],
)

SLOT_GROUP = dict(motor="motors", battery="batteries", esc="escs", servo="servos", fc="fcs", video="video",
                  gps="gps", receiver="receivers", remote_id="remote_id", hardware="hardware", material="materials")
REQUIRED = dict(
    motor=("kv", "r", "i0", "g", "imax", "smax"), battery=("chem", "ns", "np", "wh", "g", "r", "imax", "dims"),
    esc=("amps", "g"), servo=("g", "kgcm"), fc=("g", "w"), video=("g", "w"), gps=("g", "w"), receiver=("g", "w"),
    remote_id=("g", "w"), hardware=("g",), material=("extra_g",), prop=("data", "d_in", "g"),
)


def part(slot, key, cat=None):
    cat = cat or CAT
    if slot == "prop":
        return prop_entry(key, cat)
    group = cat[SLOT_GROUP[slot]]
    if key not in group:
        raise KeyError(f"{slot} {key!r} is not in catalog.json[{SLOT_GROUP[slot]!r}]")
    return group[key]


def prop_entry(name, cat=None):
    cat = cat or CAT
    models = cat["props"].get("models", {})
    if name in models:
        return models[name]
    apc = json.loads((HERE / "apc_props.json").read_text())
    if name in apc:
        d_in = apc[name]["d_in"]
        rpm_const = 150000.0 if name.endswith("E") else 225000.0
        return dict(data=name, d_in=d_in, g=6.8 * (d_in / 7.0) ** 2.35, rpm_const=rpm_const,
                    price_pair=cat["props"]["pair_price_by_inch"][str(max(5, int(d_in)))], note="mass estimated")
    raise KeyError(f"prop {name!r} not found")


def prop_data_key(name):
    return prop_entry(name)["data"]


def prop_rpm_limit(name):
    pe = prop_entry(name)
    rc = pe.get("rpm_const", 150000.0 if pe["data"].endswith("E") else 225000.0)
    return rc / pe["d_in"] / 60.0


def validate(build=CURRENT_BUILD):
    problems = []
    for slot, fields in REQUIRED.items():
        keys = build.get(slot)
        if keys is None:
            if slot in ("remote_id",) or (slot == "esc" and part("fc", build["fc"]).get("esc_included")):
                continue
            problems.append(f"{slot}: not set")
            continue
        for key in (keys if isinstance(keys, (list, tuple)) else [keys]):
            try:
                e = part(slot, key)
            except KeyError as err:
                problems.append(str(err))
                continue
            problems += [f"{slot} {key!r}: missing field {f!r}" for f in fields if f not in e]
    m, p = part("motor", build["motor"]), prop_entry(build["prop"])
    if "shaft" in m and "bore_mm" in p and abs(float(m["shaft"].lstrip("M")) - p["bore_mm"]) > 0.01:
        problems.append(f"prop bore {p['bore_mm']} mm does not fit {m['shaft']} shaft")
    b = part("battery", build["battery"])
    if b["ns"] > m["smax"]:
        problems.append(f"{b['ns']}S pack over motor {m['smax']}S rating")
    if any(x > y for x, y in zip(b["dims"], CAT["bay_mm"])):
        problems.append(f"battery {b['dims']} mm does not fit bay {CAT['bay_mm']} mm")
    if problems:
        raise ValueError("build is not usable:\n  " + "\n  ".join(problems))
    return True


def variant(build=CURRENT_BUILD, **changes):
    out = dict(build, **changes)
    if "name" not in changes:
        out["name"] = build["name"] + " with " + ", ".join(f"{k}={v}" for k, v in changes.items())
    return out


def esc_current_limit(build=CURRENT_BUILD):
    m = part("motor", build["motor"])
    fc = part("fc", build["fc"])
    esc_a = fc["esc_amps"] if fc.get("esc_included") else part("esc", build["esc"])["amps"]
    return min(esc_a, m["imax"])


def avionics_power_w(build=CURRENT_BUILD):
    w = part("fc", build["fc"])["w"] + SERVO_HOLD_W
    for slot in ("video", "gps", "receiver", "remote_id"):
        if build.get(slot):
            w += part(slot, build[slot])["w"]
    return w


def pack_volts(build=CURRENT_BUILD):
    b = part("battery", build["battery"])
    return b["ns"] * b.get("v_nom_cell", V_CELL_NOMINAL[b["chem"]])


def _spar_x(frac, y, geo):
    sw = math.tan(math.radians(geo.get("sweep_deg", 23.0)))
    by, half = geo["blend_y"], geo["half"]
    lex = geo["root_le"] - sw * (abs(y) - by)
    c = geo["root_chord"] + (geo["tip_chord"] - geo["root_chord"]) * max(0.0, (abs(y) - by) / (half - by))
    return lex - frac * c


def composite_skin_g():
    """Cheap 2-ply +/-45 49 g/m2 E-glass + epoxy + laminar finish = 225 g/m2 wetted (RF-transparent!)."""
    s_wing = 2.0 * (GEO["half"] - GEO["blend_y"]) / 1000.0 * 0.5 * (ROOT_CHORD + TIP_CHORD)
    d0, db = GEO["d0"], GEO["d_bay"]
    l0, lb = (d0[0] - d0[1]) / 1000.0, (db[0] - db[1]) / 1000.0
    s_body = 2.0 * (0.070 * 0.5 * (l0 + lb) + (GEO["blend_y"] / 1000.0 - 0.070) * 0.5 * (lb + ROOT_CHORD))
    return 225.0 * (2.04 * s_wing + 2.12 * s_body)


def spar_design(build=CURRENT_BUILD):
    """Cheap standard 135 GPa T700 pultruded flat spar caps + >=0.70 mm carbon web + 32x15 mm root V-box,
    sized for the airframe design gross mass so one brick and two both pass >= 20 g @ 400 MPa."""
    m = part("motor", build["motor"])
    p = prop_entry(build["prop"])
    e = part("esc", build["esc"])
    # The caps are cut into the CAD for ONE design gross mass (AIRFRAME_DESIGN_GROSS_KG). The spar is a property of
    # the airframe, not of the parts set: test_harness checks every flyable build stays under that mass.
    m_est = AIRFRAME_DESIGN_GROSS_KG
    mg_kg = (m["g"] + p["g"] + e["g"] + 7.4 + 24.0) / 1000.0
    sh = GEO.get("spar_h", [10.8, 7.6, 4.4])
    cad_h = (GEO.get("carry_box_h", 15.0) / 1000.0, sh[0] / 1000.0, sh[1] / 1000.0, sh[2] / 1000.0)
    cad_bw = GEO.get("carry_box_w", 32.0) / 1000.0
    return AE.design_spar_caps(m_est, SPAN, GEO["blend_y"] / 1000.0, ROOT_CHORD, TIP_CHORD,
                               GEO["motor_y"] / 1000.0, mg_kg, target_g=20.0, sigma_allow=400e6,
                               sweep_deg=GEO["sweep_deg"], cad_box_h_m=cad_h, cad_root_box_w_m=cad_bw)


def parts_list(build=CURRENT_BUILD, battery_x=None):
    g = GEO
    rows = []

    def add(name, slot, grams, x, price, bom=None):
        rows.append(dict(name=name, slot=slot, g=float(grams), x=float(x), price=float(price), bom=bom or name))

    mat = part("material", build["material"])
    air = f"airframe, printed core ({build['material']}, print/mass_report.json)"
    for i, (name, d) in enumerate(REPORT["parts"].items()):
        add(name, "airframe", d["grams"], d["x"], mat["price"] if i == 0 else 0.0, air)
    if mat["extra_g"]:
        printed = sum(d["grams"] for d in REPORT["parts"].values())
        x_air = sum(d["grams"] * d["x"] for d in REPORT["parts"].values()) / printed
        add(f"{build['material']} extra over LW-PLA", "airframe", mat["extra_g"], x_air, 0.0, air)

    sp = spar_design(build)
    tubes = "standard 135 GPa T700 carbon spar caps + 0.7mm web + 32x15mm root V-box"
    add("monolithic carbon root V-joiner box", "tubes", sp["root_box_g"], g["carry_x"], 0.0, tubes)
    cap_g_rem = sp["mass_g"] - sp["root_box_g"]
    y1, y2 = g["blend_y"] + 0.30 * (g["half"] - g["blend_y"]), g["blend_y"] + 0.62 * (g["half"] - g["blend_y"])
    for frac_m, y_mid in ((0.52, 0.5 * (g["blend_y"] + y1)), (0.32, 0.5 * (y1 + y2)), (0.16, 0.5 * (y2 + g["half"]))):
        add(f"T700 carbon caps+web y={y_mid:.0f}", "tubes", cap_g_rem * frac_m, _spar_x(g["spar_frac"], y_mid, g), 0.0, tubes)

    m = part("motor", build["motor"])
    add(f"motors x2: {build['motor']}", "motor", 2 * m["g"], PLACEMENT["motors"], 2 * (m["price"] + m.get("ship", 0)))
    p = prop_entry(build["prop"])
    add(f"props x2: {build['prop']}", "prop", 2 * p["g"], PLACEMENT["props"], p.get("price_pair", 0.0))
    fc = part("fc", build["fc"])
    if not fc.get("esc_included"):
        e = part("esc", build["esc"])
        add(f"ESCs x2: {build['esc']}", "esc", 2 * e["g"], PLACEMENT["escs"], 2 * (e["price"] + e.get("ship", 0)))
    add(f"flight controller: {build['fc']}", "fc", fc["g"], PLACEMENT["fc"], fc["price"] + fc.get("ship", 0))
    s = part("servo", build["servo"])
    add(f"servos x2: {build['servo']}", "servo", 2 * s["g"], PLACEMENT["servos"], s["price_pair"])
    for slot in ("video", "gps", "receiver", "remote_id"):
        if build.get(slot):
            e = part(slot, build[slot])
            add(f"{slot}: {build[slot]}", slot, e["g"], PLACEMENT[slot], e["price"] + e.get("ship", 0))
    for key in build.get("hardware", ()):
        e = part("hardware", key)
        if "E-glass composite skin" in key:
            printed = sum(d["grams"] for d in REPORT["parts"].values())
            x_skin = sum(d["grams"] * d["x"] for d in REPORT["parts"].values()) / printed
            add(f"hardware: {key}", "hardware", composite_skin_g(), x_skin, e["price"] + e.get("ship", 0))
        elif "tilt-nacelle" in key.lower():
            add(f"hardware: {key}", "hardware", e["g"], g.get("tilt_shaft_x", 136.0), e["price"] + e.get("ship", 0))
        else:
            add(f"hardware: {key}", "hardware", e["g"], PLACEMENT["hardware"], e["price"] + e.get("ship", 0))
    b = part("battery", build["battery"])
    if battery_x is None:      # one 6S1P brick sits at the CAD station; two bricks or a full-length pack fill the bay
        battery_x = GEO.get("bay_x", PLACEMENT["battery"]) if (b["np"] >= 2 or b["dims"][0] > 150) else PLACEMENT["battery"]
    add("battery", "battery", b["g"], battery_x,
        b["price"] + b.get("ship", 0), f"battery: {build['battery']}")
    return rows


def mass_breakdown(build=CURRENT_BUILD, battery_x=None):
    return {r["name"]: (r["g"], r["x"]) for r in parts_list(build, battery_x)}


def bill_of_materials(build=CURRENT_BUILD):
    out = {}
    for r in parts_list(build):
        g, c = out.get(r["bom"], (0.0, 0.0))
        out[r["bom"]] = (g + r["g"], c + r["price"])
    return out


def all_up_mass_g(build=CURRENT_BUILD):
    return sum(r["g"] for r in parts_list(build))


def all_up_mass_kg(build=CURRENT_BUILD):
    return all_up_mass_g(build) / 1000.0


def cg_x_mm(build=CURRENT_BUILD, battery_x=None):
    rows = parts_list(build, battery_x)
    return sum(r["g"] * r["x"] for r in rows) / sum(r["g"] for r in rows)


def airframe_g(build=CURRENT_BUILD):
    return sum(r["g"] for r in parts_list(build) if r["slot"] in ("airframe", "tubes"))


def fixed_items(build=CURRENT_BUILD):
    out = {}
    for slot in ("video", "gps", "receiver", "remote_id"):
        if build.get(slot):
            e = part(slot, build[slot])
            out[build[slot]] = dict(g=e["g"], w=e["w"], price=e["price"] + e.get("ship", 0))
    for key in build.get("hardware", ()):
        e = part("hardware", key)
        g_item = composite_skin_g() if "E-glass composite skin" in key else e["g"]
        out[key] = dict(g=g_item, w=0.0, price=e["price"] + e.get("ship", 0))
    return out


def harness_design(build=CURRENT_BUILD, exact_mass=True):
    d = dict(motor=build["motor"], prop=build["prop"], battery=build["battery"], servo=build["servo"],
             fc=build["fc"], material=build["material"], cr=ROOT_CHORD, ct=TIP_CHORD, span=SPAN)
    if exact_mass:
        d.update(esc=build.get("esc"), avionics_w=avionics_power_w(build),
                 mass_table=tuple((k, round(g, 3), round(c, 2)) for k, (g, c) in bill_of_materials(build).items()))
    return d


def stall_speed(mass_kg, s_ref, cl_max=TRIMMED_CLMAX, rho=RHO):
    return math.sqrt(2.0 * mass_kg * G / (rho * s_ref * cl_max))


if __name__ == "__main__":
    validate()
    sp = spar_design()
    print(CURRENT_BUILD["name"])
    for k, (g, c) in bill_of_materials().items():
        print(f"  {k:92s} {g:6.1f} g  ${c:7.2f}")
    print(f"  {'ALL-UP':92s} {all_up_mass_g():6.1f} g  ${sum(c for _, c in bill_of_materials().values()):7.2f}")
    print(f"CG x = {cg_x_mm():.1f} mm (batt x = {PLACEMENT['battery']:.1f} mm), min T700 spar limit = {sp['min_g_limit']:.1f} g @ 400 MPa (sized for {AIRFRAME_DESIGN_GROSS_KG:.2f} kg), "
          f"pack {pack_volts():.1f} V nom, ESC/motor limit {esc_current_limit():.0f} A")
