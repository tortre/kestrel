"""Kestrel-560 multi-objective harness: top speed, endurance, range, mass and cost from real parts.

Run:  .venv/Scripts/python sim/harness.py            hill climbs, Pareto front, bill of materials
      .venv/Scripts/python sim/harness.py --free-bay  also allow batteries that need a longer bay (CAD change)
      .venv/Scripts/python sim/harness.py --risky     also allow parts whose firmware support is unverified
      .venv/Scripts/python sim/test_harness.py        physics and regression checks

Parts and prices come from sim/catalog.json (read from AliExpress, see the file header).
Aerodynamics, propeller and motor model are the ones in sim/optimize.py (NeuralFoil MH45 polars,
APC performance files, Kv / resistance / no-load-current motor), with the drag and airframe mass of the
printed blended body (sim/mission.py "printable"). New here: catalogue parts with prices, full-throttle
top speed with battery sag and current limits, servo and battery-bay checks, a time-stepped mission,
and hill climbing on every objective.

The current build (parts, mass, air density, ESC, operating rules) comes from sim/aircraft.py:
CURRENT = aircraft.harness_design() is exactly that build; BASELINE is the same parts on this file's
parametric airframe model, the start point of the design search. headline() gives the published numbers.
"""
import itertools
import json
import math
import pathlib
import sys

import numpy as np

import aircraft as A
import blended
import optimize as opt

HERE = pathlib.Path(__file__).resolve().parent
CAT = A.CAT
RHO, G, NU = A.RHO, A.G, A.NU
SPAN = A.SPAN
HALF = SPAN / 2 - 0.003                 # wing half-span used by the parametric model (tips end 3 mm short)

# printed blended body (v2): smooth hull, faired pods, no sensor ball
HULL_FF, POD_CD, EXTRAS_DQ = 1.25, 0.15, 0.0
AIRFRAME_G_PER_M2 = 1100.0
TUBE_G_PER_M = A.TUBE_G_PER_M   # 6 x 4 mm carbon tube; the spars follow the swept 30 % chord line (x 1.066)
# printed parts + carbon tubes at zero wing area, calibrated so the current wing gives exactly the CAD's
# print/mass_report.json + tube mass (sim/aircraft.py)
AIRFRAME_FIXED_G = A.airframe_g() - AIRFRAME_G_PER_M2 * HALF * (A.ROOT_CHORD + A.TIP_CHORD)
BODY_CLEARANCE = 0.123 if SPAN > 0.80 else 0.071
TW_MIN, STALL_MAX = 1.8, 12.5   # hover thrust margin for a tailsitter; gentle stall speed
MAX_PROP_IN = 9.0 if SPAN > 0.80 else 7.0
HOVER_S = A.HOVER_S             # take-off plus landing hover per flight
ESC_P0_W = 8.0                  # fixed loss per ESC at 16 V (bench fit in sim/performance.py)
CR_RANGE, CT_MIN = (0.140, 0.280), 0.070
V_CELL = A.V_CELL_NOMINAL
OCV = {  # open-circuit volts per cell against state of charge, for the time-stepped mission
    "liion": ([0, .1, .2, .5, .8, 1], [3.0, 3.3, 3.45, 3.65, 3.95, 4.2]),
    "si_liion": ([0, .1, .2, .5, .8, 1], [2.8, 3.05, 3.22, 3.45, 3.82, 4.2]),
    "lipo": ([0, .1, .2, .5, .8, 1], [3.3, 3.6, 3.7, 3.8, 4.0, 4.2]),
}
# the things I estimated rather than measured; Monte Carlo varies them
NOMINAL = dict(drag_scale=1.0, esc_loss=1.0, cp_derate=1.08, usable=A.USABLE, e_osw=0.85 if SPAN > 0.80 else 0.80,
               r_scale=1.0, i0_scale=1.0, honesty_scale=1.0, mass_extra=0.0)
# ROBUST includes esc_loss=1.5 per Claude Opus 5.5 audit!
ROBUST = dict(cp_derate=1.15, r_scale=1.15, i0_scale=1.2, esc_loss=1.5, mass_extra=30.0)
_SP0 = A.spar_design()
_PLANE0 = blended.airplane()
S_BODY_EXTRA = _PLANE0.s_ref - HALF * (A.ROOT_CHORD + A.TIP_CHORD)
B_REF_0 = _PLANE0.b_ref                # TRUE projected planform span (ZERO sharklet inflation!)
_AE0 = A.AE.aeroelastic_check(A.all_up_mass_kg(), 0.030, _PLANE0.s_ref, _PLANE0.c_ref, SPAN,
                              A.GEO["blend_y"] / 1000.0, A.ROOT_CHORD, A.TIP_CHORD, A.GEO["sweep_deg"],
                              A.GEO["elevon_y"][0] / 1000.0, A.GEO["elevon_y"][1] / 1000.0,
                              A.GEO["hinge"], _SP0, sm_frac=0.090, skin_type="glass2x49", cap_e_gpa=135.0)
V_NE = float(_AE0["v_ne_aeroelastic_ms"]) if SPAN > 0.80 else 45.0
OPTIONS = dict(free_bay=False, risky=False, off_aliexpress=False)

KEYS = ("motor", "prop", "battery", "servo", "fc", "material")
PROPS = [p for p in opt.APC if opt.APC[p]["d_in"] <= MAX_PROP_IN]
CHOICES = dict(motor=list(CAT["motors"]), prop=PROPS, battery=list(CAT["batteries"]),
               servo=list(CAT["servos"]), fc=list(CAT["fcs"]), material=list(CAT["materials"]))
FIXED = A.fixed_items(A.CURRENT_BUILD)          # avionics + hardware every candidate design carries
CURRENT = A.harness_design(A.CURRENT_BUILD)      # exactly the current build (mass table, ESC, avionics power)
BASELINE = A.harness_design(A.CURRENT_BUILD, exact_mass=False)    # same parts, parametric airframe: search start
_CACHE = {}
_STEALTH_POLARS = None


def _get_stealth_polars():
    global _STEALTH_POLARS
    if _STEALTH_POLARS is None:
        import aerosandbox as asb
        import neuralfoil as nf
        af = asb.Airfoil("kestrel_stealth_cst", coordinates=blended.wing_coords(1.0))
        res = np.array([6e4, 1e5, 1.6e5, 2.6e5, 4.5e5, 8e5, 1.4e6])
        alpha = np.linspace(-4.0, 15.0, 39)
        cl_grid = np.linspace(-0.05, 1.30, 136)
        cd_t, clm_t = [], []
        # Realistic sanded/painted bagged composite finish n_crit=7.0 (not idealized n_crit=9!)
        for re in res:
            a = nf.get_aero_from_airfoil(af, alpha=alpha, Re=re, n_crit=7.0, model_size="large")
            cl, cdv = np.ravel(a["CL"]), np.ravel(a["CD"])
            i = int(np.argmax(cl))
            clm_t.append(float(cl[i]))
            cd_t.append(np.interp(cl_grid, cl[:i + 1], cdv[:i + 1], right=0.25))
        _STEALTH_POLARS = (res, np.log(res), cl_grid, np.array(cd_t), np.array(clm_t))
    return _STEALTH_POLARS


def _stealth_section_cd(cl, re):
    res, lres, cl_grid, cd_tab, _ = _get_stealth_polars()
    lre = np.log(np.clip(re, res[0], res[-1]))
    j = np.clip(np.searchsorted(lres, lre) - 1, 0, len(res) - 2)
    t = (lre - lres[j]) / (lres[j + 1] - lres[j])
    cl_a, j_a = np.atleast_1d(cl), np.atleast_1d(j)
    lo = np.array([np.interp(c, cl_grid, cd_tab[k]) for c, k in zip(cl_a, j_a)])
    hi = np.array([np.interp(c, cl_grid, cd_tab[k + 1]) for c, k in zip(cl_a, j_a)])
    return lo + t * (hi - lo)


def prop_key(name):
    """Performance-table key (sim/apc_props.json) for a prop name (a catalogue model or a raw table key)."""
    return name if name in opt.APC else A.prop_data_key(name)


class Aircraft:
    """One design under one set of assumptions: mass, cost, drag and the electrical model."""

    def __init__(self, design, **assume):
        a = dict(NOMINAL, **assume)
        self.a, self.design = a, design
        self.motor = m = CAT["motors"][design["motor"]]
        self.batt = b = CAT["batteries"][design["battery"]]
        self.servo = CAT["servos"][design["servo"]]
        self.fc = CAT["fcs"][design["fc"]]
        self.material = CAT["materials"][design["material"]]
        self.kv, self.rm, self.i0 = m["kv"], m["r"] * a["r_scale"], m["i0"] * a["i0_scale"]
        self.prop_data = prop_key(design["prop"])
        self.d, self.J, self.CT, cp = opt.prop_arrays(self.prop_data)
        self.CP = cp / opt.CP_DERATE * a["cp_derate"]
        self.d_in = opt.APC[self.prop_data]["d_in"]
        # Exact APC Rev 5 RPM limit (150,000/D Thin Electric 'E', 225,000/D Sport/Carbon on electric) + tip Mach <= 0.80
        rpm_const = A.prop_entry(design["prop"]).get("rpm_const", 150000.0 if self.prop_data.endswith("E") else 225000.0)
        self.n_max = min(rpm_const / self.d_in / 60.0, 265.0 / (math.pi * self.d))
        self.cr, self.ct = design["cr"], design["ct"]
        self.span = design.get("span", SPAN)
        self.is_v6 = self.span > 0.80 and "cst_wt" in A.GEO
        self.v_ne = V_NE if self.is_v6 else 45.0
        self.half = self.span / 2 - 0.003
        s_trap = self.half * (self.cr + self.ct)
        self.s = (s_trap + S_BODY_EXTRA) if self.is_v6 else s_trap
        # TRUE projected planform span & AR (NO sharklet span inflation!)
        self.ar = self.span**2 / self.s
        lam = self.ct / self.cr
        self.mac = 2 / 3 * self.cr * (1 + lam + lam**2) / (1 + lam)
        self.motor_y = BODY_CLEARANCE + self.d / 2
        self.ns = b["ns"]
        self.v_cell_nom = b.get("v_nom_cell", V_CELL[b["chem"]])
        self.vbat = self.ns * self.v_cell_nom
        self.r_pack = b["r"] + A.WIRING_R
        self.esc_loss = a["esc_loss"]
        self.wh = b["wh"] * b["honesty"] * a["honesty_scale"]
        if "mass_table" in design:
            self.avionics_w = design["avionics_w"]
            self.table = {name: (g, c) for name, g, c in design["mass_table"]}
        else:
            self.avionics_w = self.fc["w"] + sum(f["w"] for f in FIXED.values()) + A.SERVO_HOLD_W
            self.table = self._mass_cost(FIXED)
        self.mass = sum(g for g, _ in self.table.values()) / 1000 + a["mass_extra"] / 1000
        self.cost = sum(c for _, c in self.table.values())
        if self.is_v6:
            res, lres, _, _, clm_t = _get_stealth_polars()
            self.clmax = 0.9 * float(np.interp(math.log(max(13 * self.mac / NU, res[0])), lres, clm_t))
            self.s_ref_trim = self.s
        else:
            self.clmax = 0.9 * float(np.interp(math.log(11 * self.mac / NU), np.log(opt.RES), opt.CLMAX_TAB))
            self.s_ref_trim = self.s + S_BODY_EXTRA
        self._set_weight()

    def _set_weight(self):
        self.w = self.mass * G
        self.v_stall = math.sqrt(2 * self.w / (RHO * self.s * self.clmax))          # clean trapezoid wing, 0.9 x section CL_max
        self.v_stall_trim = math.sqrt(2 * self.w / (RHO * self.s_ref_trim * A.TRIMMED_CLMAX))   # trimmed, blended area
        self.v_min_ops = A.MIN_SPEED_FACTOR * self.v_stall_trim                       # slowest allowed cruise / loiter

    # ---- mass and cost -------------------------------------------------------------------------
    def _mass_cost(self, fixed):
        m, b, d = self.motor, self.batt, self.design
        t = {
            "airframe, printed (" + d["material"] + ")": (AIRFRAME_FIXED_G + TUBE_G_PER_M * 2 * 1.066 * (self.half - HALF)
                                                         + AIRFRAME_G_PER_M2 * self.s + self.material["extra_g"],
                                                         self.material["price"]),
            "motors x2: " + d["motor"]: (2 * m["g"], 2 * (m["price"] + m.get("ship", 0))),
            f"props, CW+CCW pair: {d['prop']}": (2 * A.prop_entry(d["prop"])["g"], A.prop_entry(d["prop"])["price_pair"]),
            "servos x2: " + d["servo"]: (2 * self.servo["g"], self.servo["price_pair"]),
            "flight controller: " + d["fc"]: (self.fc["g"], self.fc["price"] + self.fc.get("ship", 0)),
            "battery: " + d["battery"]: (b["g"], b["price"] + b.get("ship", 0)),
        }
        if b.get("weld"):
            t["spot welder for the pack (one-time tool)"] = (0.0, CAT["spot_welder"]["price"])
        for name, f in fixed.items():
            t[name] = (f["g"], f["price"])
        return t

    def add_escs(self, full_throttle_amps):
        """Fit ESCs: the design's own (design["esc"]) or the lightest whose continuous rating covers the
        full-throttle motor current (fresh pack, see full_throttle_current) x aircraft.ESC_SIZING_MARGIN."""
        if self.fc.get("esc_included"):
            self.esc_amps, self.esc_name = self.fc["esc_amps"], "in the flight controller stack"
            return
        name = self.design.get("esc")
        if not name:
            need = A.ESC_SIZING_MARGIN * full_throttle_amps
            ok = [n for n, e in CAT["escs"].items() if e["amps"] >= need]
            name = (min(ok, key=lambda n: (CAT["escs"][n]["g"], CAT["escs"][n]["price"])) if ok
                    else max(CAT["escs"], key=lambda n: CAT["escs"][n]["amps"]))
        e = CAT["escs"][name]
        self.esc_amps, self.esc_name = e["amps"], name
        if any(k.startswith("ESCs x2") for k in self.table):          # exact build: already in the mass table
            return
        self.table["ESCs x2: " + name] = (2 * e["g"], 2 * (e["price"] + e.get("ship", 0)))
        self.mass += 2 * e["g"] / 1000
        self.cost += 2 * (e["price"] + e.get("ship", 0))
        self._set_weight()

    def full_throttle_current(self):
        """Motor amps at full throttle, standing, on a fresh pack (aircraft.V_CELL_FULL), limited only by the motor."""
        saved = getattr(self, "esc_amps", None)
        self.esc_amps = math.inf
        _, n = self.full_throttle(np.zeros(1), v_cell=A.V_CELL_FULL)
        self.esc_amps = saved
        return float(self.electrical(n, np.zeros(1))[2][0])

    # ---- physics -------------------------------------------------------------------------------
    def electrical(self, n, v, v_bus=None):
        """Per motor at rev/s n and airspeed v: input watts, motor volts, motor amps, shaft watts.
        Uses modern 80A BLHeli32/AM32 ESC loss & speed-scaled iron loss i0(rpm) for v7."""
        shaft = np.interp(v / (n * self.d), self.J, self.CP) * RHO * n**3 * self.d**5
        e = n * 60 / self.kv
        if self.is_v6:
            i0_eff = self.i0 * (1.0 + 0.35 * np.maximum(e / 10.0 - 1.0, 0.0))
            im = shaft / e + i0_eff
            vb = self.vbat if v_bus is None else v_bus
            p_esc = blended.esc_loss_modern(vb, im, self.esc_loss)
        else:
            im = shaft / e + self.i0
            p_esc = opt.esc_loss_w(self.vbat if v_bus is None else v_bus, im, self.esc_loss)
        return (e + im * self.rm) * im + p_esc, e + im * self.rm, im, shaft

    def thrust(self, n, v):
        return np.interp(v / (n * self.d), self.J, self.CT) * RHO * n**2 * self.d**4

    def cd(self, v, cl):
        cf = lambda length: 0.074 / (v * length / NU)**0.2
        if self.is_v6:
            # Sleek 660 mm chined stealth body (t/c=5.7%, FF=1.12) + enlarged 22-deg NACA0006 V-tails + tilt-nacelles
            other = (cf(0.66) * 1.12 * 0.250 + cf(0.10) * 1.12 * 0.145
                     + 0.12 * 2 * math.pi * 0.018**2)
            s_wing_frac = max(0.55, (self.s - 0.116) / self.s)
            cd_sec = _stealth_section_cd(cl, v * self.mac / NU) * s_wing_frac
            return ((cd_sec + other / self.s) * 1.15 * self.a["drag_scale"]
                    + cl**2 / (math.pi * self.ar * self.a["e_osw"]))
        other = cf(0.34) * HULL_FF * 0.060 + cf(0.12) * 1.05 * 0.056 + POD_CD * 2 * math.pi * 0.014**2 + EXTRAS_DQ
        return ((opt.section_cd(cl, v * self.mac / NU) + other / self.s) * 1.15 * self.a["drag_scale"]
                + cl**2 / (math.pi * self.ar * self.a["e_osw"]))

    def drag(self, v):
        q = 0.5 * RHO * v**2
        cl = self.w / (q * self.s)
        return q * self.s * self.cd(v, cl), cl

    def level_power(self, v):
        """Total electrical watts in level flight at airspeeds v (array); inf where the pack voltage is too low."""
        v = np.atleast_1d(np.asarray(v, float))
        drag, cl = self.drag(v)
        n = opt.thrust_solve(drag / 2, v, self.d, self.J, self.CT)
        p_one, v_mot, im, shaft = self.electrical(n, v)
        p = 2 * p_one + self.avionics_w
        bad = (v_mot > self.vbat) | (cl > self.clmax) | (n > self.n_max)
        return np.where(bad, np.inf, p), dict(drag=drag, cl=cl, n=n, shaft=shaft, im=im)

    def static(self, thrust_total):
        n = opt.thrust_solve(np.array([thrust_total / 2]), np.zeros(1), self.d, self.J, self.CT)
        p, vm, im, _ = self.electrical(n, np.zeros(1))
        return float(2 * p[0] + self.avionics_w), float(vm[0]), float(im[0]), float(n[0])

    def full_throttle(self, v, v_cell=None):
        """Thrust (N, both motors) at airspeeds v with the battery sagging and every current limit applied."""
        v = np.atleast_1d(np.asarray(v, float))
        vc = self.v_cell_nom if v_cell is None else v_cell
        voc = self.ns * vc
        icap = min(self.motor["imax"], self.esc_amps)
        lo, hi = np.full_like(v, 5.0), np.full_like(v, self.n_max)
        for _ in range(40):
            mid = (lo + hi) / 2
            _, vm, im, _ = self.electrical(mid, v, voc)
            disc = voc**2 - 8 * vm * im * self.r_pack
            vb = (voc + np.sqrt(np.maximum(disc, 0))) / 2
            ok = (disc >= 0) & (vm <= vb) & (im <= icap) & (2 * vm * im / vb <= self.batt["imax"])
            lo, hi = np.where(ok, mid, lo), np.where(ok, hi, mid)
        return 2 * self.thrust(lo, v), lo

    def full_throttle_state(self, v, v_cell):
        """Full-throttle operating point at one airspeed: thrust, motor and pack amps, pack and motor volts, rpm."""
        voc = self.ns * v_cell
        thrust, n = self.full_throttle(v, v_cell)
        _, vm, im, _ = self.electrical(n, np.atleast_1d(np.asarray(v, float)))
        disc = voc**2 - 8 * vm * im * self.r_pack
        vb = (voc + np.sqrt(np.maximum(disc, 0))) / 2
        return dict(thrust=float(thrust[0]), motor_a=float(im[0]), pack_a=float((2 * vm * im / vb)[0]), pack_v=float(vb[0]),
                    motor_v=float(vm[0]), rpm=float(n[0] * 60))

    def servo_speed_limit(self):
        """Airspeed where 10 degrees of elevon needs half the servo's rated torque."""
        if self.is_v6:
            ey0, ey1 = A.GEO.get("elevon_y", [536.0, 744.0])
            el_span = (ey1 - ey0) / 1000.0
            c_mid = blended.chord(0.5 * (ey0 + ey1)) / 1000.0
            chord_e = (1.0 - A.GEO.get("hinge", 0.70)) * c_mid
            k = el_span * chord_e * chord_e * 0.08
            return math.sqrt(0.5 * self.servo["kgcm"] * 0.0981 / k / (0.5 * RHO))
        chord_e = 0.35 * (self.cr + self.ct) / 2
        k = max(self.half - 0.102, 0.05) * chord_e * chord_e * 0.08                 # hinge moment per unit dynamic pressure
        return math.sqrt(0.5 * self.servo["kgcm"] * 0.0981 / k / (0.5 * RHO))


def make_aircraft(design, **assume):
    """An Aircraft with its ESCs fitted (sized for full-throttle current unless the design names its ESC)."""
    ac = Aircraft(design, **assume)
    ac.add_escs(0.0 if ac.fc.get("esc_included") else ac.full_throttle_current())
    return ac


def evaluate(design, full=False, **assume):
    key = (tuple(design[k] for k in KEYS), round(design["cr"], 4), round(design["ct"], 4), round(design.get("span", SPAN), 4),
           design.get("esc"), design.get("mass_table"), design.get("avionics_w"),
           tuple(sorted(assume.items())), tuple(sorted(OPTIONS.items())), full)
    if key in _CACHE:
        return _CACHE[key]
    ac = make_aircraft(design, **assume)
    i_full = ac.full_throttle_current() if not ac.fc.get("esc_included") else ac.esc_amps
    b, m = ac.batt, ac.motor
    p_hover, _, _, _ = ac.static(ac.w)
    p_m, v_m, i_m, n_m = ac.static(TW_MIN * ac.w)
    i_pack = (p_m - ac.avionics_w) / ac.vbat
    reasons = []
    if v_m > ac.vbat - i_pack * ac.r_pack:
        reasons.append("not enough voltage for the hover thrust margin")
    if i_m > m["imax"]:
        reasons.append("motor over its current limit at the hover thrust margin")
    if i_m > ac.esc_amps:
        reasons.append("ESC too small")
    if i_pack > b["imax"]:
        reasons.append("battery over its current limit at the hover thrust margin")
    if n_m > ac.n_max:
        reasons.append("prop over its rpm limit")
    if ac.ns > m["smax"]:
        reasons.append("too many cells for the motor")
    if ac.half < ac.motor_y + ac.d / 2:
        reasons.append("wing too short to carry the whole prop wash (hover control)")
    if ac.v_stall > STALL_MAX:
        reasons.append("stall speed too high")
    if not OPTIONS["free_bay"] and any(x > y for x, y in zip(b["dims"], CAT["bay_mm"])):
        reasons.append("battery does not fit the bay")
    if not OPTIONS["risky"] and ac.fc.get("risky"):
        reasons.append("flight controller support unverified")
    if not OPTIONS["off_aliexpress"] and not b.get("aliexpress", True) and not b.get("default"):
        reasons.append("battery not sold on AliExpress")

    if not assume and not reasons:
        worse = evaluate(design, **ROBUST)["reasons"]
        if worse:
            reasons.append("no margin (" + worse[0] + " with 30 g extra and a 15 % worse motor and prop)")

    v_lo_start = max(ac.v_min_ops, 9.0)
    v = np.concatenate(([v_lo_start], np.arange(math.ceil(v_lo_start * 4) / 4 + 0.1, 34.0, 0.25)))     # >= 1.25 x trimmed stall
    p, x = ac.level_power(v)
    energy = ac.wh * ac.a["usable"] - p_hover * HOVER_S / 3600
    rng = energy / p * v * 3.6
    k, ke = int(np.argmax(rng)), int(np.argmin(p))
    if not np.isfinite(p[k]) or energy <= 0:
        reasons.append("cannot cruise")
    vv = np.arange(10.0, 120.0, 0.25)
    t_full, _ = ac.full_throttle(vv)
    d_full, cl_full = ac.drag(vv)
    can = (t_full >= d_full) & (cl_full <= ac.clmax)
    v_thrust = float(vv[can].max()) if can.any() else 0.0
    v_servo = ac.servo_speed_limit()
    vmax = min(v_thrust, v_servo, ac.v_ne)
    out = dict(feasible=not reasons, reasons=reasons, mass_g=ac.mass * 1000, cost=ac.cost, vmax=vmax,
               endurance_min=float(energy / p[ke] * 60) if np.isfinite(p[ke]) else 0.0,
               range_km=float(rng[k]) if np.isfinite(p[k]) else 0.0)
    if full:
        out.update(v_stall=ac.v_stall, v_stall_trimmed=ac.v_stall_trim, v_min_ops=ac.v_min_ops, i_motor_full=i_full,
                   esc_amps=ac.esc_amps, v_cruise=float(v[k]), p_cruise=float(p[k]), v_loiter=float(v[ke]), p_loiter=float(p[ke]),
                   ld=float(x["cl"][k] / ac.cd(v[k:k + 1], x["cl"][k:k + 1])[0]), p_hover=p_hover,
                   hover_min=ac.wh * ac.a["usable"] / p_hover * 60, i_motor_margin=i_m, i_pack_margin=i_pack,
                   wh_usable=ac.wh * ac.a["usable"], v_thrust=v_thrust, v_servo=v_servo, esc=ac.esc_name,
                   prop_eff=float(x["drag"][k] / 2 * v[k] / x["shaft"][k]), area_cm2=ac.s * 1e4, table=ac.table,
                   avionics_w=ac.avionics_w)
    _CACHE[key] = out
    return out


# ---- time-stepped mission (independent check of the closed-form range) -----------------------------
def fly_mission(design, v_cruise, dt=5.0, **assume):
    """Hover up, cruise at constant airspeed until only the landing reserve is left, hover down.

    Battery modelled as open-circuit voltage against state of charge plus series resistance, so current
    rises as the pack empties. Returns distance (km), time (min) and the lowest loaded pack voltage.
    """
    r = evaluate(design, full=True, **assume)
    ac = make_aircraft(design, **assume)
    soc_x, ocv_y = OCV[ac.batt["chem"]]
    cap_wh = ac.wh
    amp_hours = cap_wh / ac.vbat
    floor = 1 - ac.a["usable"]
    p_cruise = float(ac.level_power(v_cruise)[0][0])
    soc, t, dist, v_min = 1.0, 0.0, 0.0, 99.0

    def step(power, seconds):
        nonlocal soc, t, v_min
        ocv = ac.ns * float(np.interp(soc, soc_x, ocv_y))
        disc = ocv**2 - 4 * ac.r_pack * power
        if disc < 0:
            return False
        amps = (ocv - math.sqrt(disc)) / (2 * ac.r_pack)
        v_min = min(v_min, ocv - amps * ac.r_pack)
        soc -= amps * seconds / 3600 / amp_hours
        t += seconds
        return True

    landing_soc = r["p_hover"] * (HOVER_S / 2) / 3600 / cap_wh * 1.05
    step(r["p_hover"], HOVER_S / 2)
    while soc > floor + landing_soc and step(p_cruise, dt):
        dist += v_cruise * dt / 1000
    step(r["p_hover"], HOVER_S / 2)
    return dict(range_km=dist, minutes=t / 60, v_min=v_min, soc_end=soc)


# ---- published numbers for a build -------------------------------------------------------------------
def headline(build=None, speeds=A.HEADLINE_SPEEDS, v_cells=(3.6, 3.3)):
    """Hover, cruise, endurance, range and static thrust/weight for an aircraft.py build (default: the current one),
    from exactly its mass table. Endurance/range: aircraft.USABLE of the pack minus aircraft.HOVER_S of hover;
    'mission' figures: MISSION_RESERVE of the usable energy kept back and MISSION_HOVER_S of hover. No wind."""
    build = build or A.CURRENT_BUILD
    design = A.harness_design(build)
    r = evaluate(design, full=True)
    ac = make_aircraft(design)
    p_hover = ac.static(ac.w)[0]
    wh_use = ac.wh * ac.a["usable"]
    e_closed = wh_use - p_hover * A.HOVER_S / 3600
    e_mission = wh_use * (1 - A.MISSION_RESERVE) - p_hover * A.MISSION_HOVER_S / 3600
    cruise = {}
    for v in speeds:
        p = float(ac.level_power(v)[0][0])
        cruise[f"{v:g}"] = dict(v=v, kmh=v * 3.6, watts=p, pack_a=p / ac.vbat, endurance_min=e_closed / p * 60,
                                range_km=e_closed / p * v * 3.6, mission_min=e_mission / p * 60,
                                mission_km=e_mission / p * v * 3.6, above_min_speed=bool(v >= ac.v_min_ops - 1e-9))
    static = {f"{vc:g}": dict(v_cell=vc, **ac.full_throttle_state(0.0, vc)) for vc in tuple(dict.fromkeys((ac.v_cell_nom,) + v_cells))}
    for s in static.values():
        s["tw"] = s["thrust"] / ac.w
    return dict(build=build["name"], mass_g=ac.mass * 1000, wh=ac.wh, wh_usable=wh_use, avionics_w=ac.avionics_w,
                hover_w=p_hover, hover_pack_a=p_hover / ac.vbat, pack_v_nominal=ac.vbat, cruise=cruise, static=static,
                esc=ac.esc_name, esc_amps=ac.esc_amps, i_motor_full_fresh=r["i_motor_full"],
                v_stall_trimmed=ac.v_stall_trim, v_min_ops=ac.v_min_ops, best_range_v=r["v_cruise"], best_range_km=r["range_km"],
                loiter_v=r["v_loiter"], loiter_min=r["endurance_min"], vmax=r["vmax"], v_ne_aeroelastic=ac.v_ne,
                cost=ac.cost, feasible=r["feasible"], reasons=r["reasons"])


# ---- objectives --------------------------------------------------------------------------------------
FLOORS = dict(vmax=25.0, endurance_min=30.0, range_km=30.0)       # a corner design must still be a usable aircraft
METRICS = (("vmax", +1), ("endurance_min", +1), ("range_km", +1), ("mass_g", -1), ("cost", -1))


def meets_floors(r):
    return r["feasible"] and all(r[k] >= f for k, f in FLOORS.items())


def objective(name, base=None):
    if name == "balanced":
        def f(r):
            if not meets_floors(r):
                return -math.inf
            return (r["vmax"] / base["vmax"] * r["endurance_min"] / base["endurance_min"] * r["range_km"] / base["range_km"]
                    * base["mass_g"] / r["mass_g"] * base["cost"] / r["cost"]) ** 0.2
        return f
    sign = dict(METRICS)[name]
    return lambda r: sign * r[name] if meets_floors(r) else -math.inf


def climb_wing(design, f):
    cr, ct = design["cr"], design["ct"]
    best = f(evaluate(design))
    step = 0.020
    while step >= 0.0025:
        moved = False
        for dcr, dct in ((step, 0), (-step, 0), (0, step), (0, -step), (step, step), (-step, -step)):
            a, b = round(cr + dcr, 4), round(ct + dct, 4)
            if not (CR_RANGE[0] <= a <= CR_RANGE[1] and CT_MIN <= b <= a):
                continue
            s = f(evaluate(dict(design, cr=a, ct=b)))
            if s > best + 1e-9:
                cr, ct, best, moved = a, b, s, True
        if not moved:
            step /= 2
    return dict(design, cr=cr, ct=ct), best


def hill_climb(start, f, log=None):
    """Greedy: take the single best move (one part swapped, a motor+prop or motor+battery pair, or the wing re-climbed)."""
    cur, best = dict(start), f(evaluate(start))
    while True:
        moves = []
        for key in KEYS:
            for o in CHOICES[key]:
                if o != cur[key]:
                    c = dict(cur, **{key: o})
                    moves.append((f(evaluate(c)), f"{key} -> {o}", c))
        for k1, k2 in (("motor", "prop"), ("motor", "battery"), ("battery", "prop")):
            for o1, o2 in itertools.product(CHOICES[k1], CHOICES[k2]):
                c = dict(cur, **{k1: o1, k2: o2})
                moves.append((f(evaluate(c)) - 1e-9, f"{k1} -> {o1}, {k2} -> {o2}", c))
        wd, ws = climb_wing(cur, f)
        moves.append((ws, f"wing -> root {wd['cr'] * 1000:.0f} mm, tip {wd['ct'] * 1000:.0f} mm", wd))
        s, what, cand = max(moves, key=lambda m: m[0])
        if not s > best + 1e-6:
            return cur, best
        cur, best = cand, s
        if log:
            log(f"      {what}")


def multi_start(f, starts=6, seed=1, log=None):
    """Hill climb from the baseline and from random designs; keep the best end point."""
    rng = np.random.default_rng(seed)
    best = (-math.inf, None)
    for i in range(starts):
        d = dict(BASELINE, battery=CHOICES["battery"][2]) if i == 0 else dict(
            {k: CHOICES[k][rng.integers(len(CHOICES[k]))] for k in KEYS}, cr=0.20, ct=0.12)
        d, s = hill_climb(d, f, log if i == 0 else None)
        if s > best[0]:
            best = (s, d)
    return best[1], best[0]


def pareto(rows):
    """Non-dominated designs among (design, result) rows on the five metrics."""
    pts = np.array([[sign * r[k] for k, sign in METRICS] for _, r in rows])
    keep = []
    for i, p in enumerate(pts):
        if not np.any(np.all(pts >= p, axis=1) & np.any(pts > p, axis=1)):
            keep.append(rows[i])
    return keep


def monte_carlo(design, n=300, seed=0):
    rng = np.random.default_rng(seed)
    out = []
    for _ in range(n):
        r = evaluate(design, full=True, drag_scale=rng.uniform(0.9, 1.35), esc_loss=rng.uniform(0.4, 1.6),
                     cp_derate=rng.uniform(1.0, 1.2), usable=rng.uniform(0.78, 0.90), e_osw=rng.uniform(0.70, 0.88),
                     r_scale=rng.uniform(0.85, 1.3), i0_scale=rng.uniform(0.8, 1.5), honesty_scale=rng.uniform(0.85, 1.05),
                     mass_extra=rng.uniform(-10, 40))
        out.append((r["vmax"], r["endurance_min"], r["range_km"], r["feasible"]))
    return np.array(out)


# ---- reporting ---------------------------------------------------------------------------------------
def line(r):
    return (f"{r['vmax'] * 3.6:4.0f} km/h | {r['endurance_min']:4.0f} min | {r['range_km']:4.0f} km | "
            f"{r['mass_g']:4.0f} g | ${r['cost']:6.2f}")


def describe(label, design, base=None):
    r = evaluate(design, full=True)
    print(f"\n  {label}")
    print(f"    {design['motor']} | {design['prop']} | {design['battery']}")
    print(f"    {design['servo']} | {design['fc']} | {design['material']} | wing {design['cr'] * 1000:.0f}/{design['ct'] * 1000:.0f} mm")
    print(f"    top speed {r['vmax'] * 3.6:.0f} km/h ({'airframe' if r['vmax'] == V_NE else 'servo' if r['v_servo'] < r['v_thrust'] else 'thrust'}-limited), "
          f"endurance {r['endurance_min']:.0f} min at {r['v_loiter'] * 3.6:.0f} km/h, range {r['range_km']:.0f} km at {r['v_cruise'] * 3.6:.0f} km/h")
    print(f"    mass {r['mass_g']:.0f} g, cost ${r['cost']:.2f}, stall {r['v_stall']:.1f} m/s, hover {r['p_hover']:.0f} W, "
          f"cruise {r['p_cruise']:.0f} W, L/D {r['ld']:.1f}, prop eff {r['prop_eff'] * 100:.0f} %")
    if base:
        print("    against baseline: " + ", ".join(
            f"{name} {100 * (r[k] / base[k] - 1):+.0f} %" for name, k in
            (("speed", "vmax"), ("endurance", "endurance_min"), ("range", "range_km"), ("mass", "mass_g"), ("cost", "cost"))))
    if not r["feasible"]:
        print("    NOT FEASIBLE: " + "; ".join(r["reasons"]))
    return r


def bom(design):
    r = evaluate(design, full=True)
    print(f"    {'part':66s} {'g':>6s} {'$':>8s}")
    for name, (g, c) in r["table"].items():
        print(f"    {name:66s} {g:6.1f} {c:8.2f}")
    print(f"    {'TOTAL':66s} {r['mass_g']:6.1f} {r['cost']:8.2f}")


def main():
    for flag in ("free_bay", "risky", "off_aliexpress"):
        OPTIONS[flag] = ("--" + flag.replace("_", "-")) in sys.argv
    print("options:", {k: v for k, v in OPTIONS.items()})
    saved = dict(OPTIONS)
    OPTIONS.update(off_aliexpress=True)
    print(f"\nCURRENT BUILD, exact mass table from sim/aircraft.py ({A.CURRENT_BUILD['name']})")
    describe("current build", CURRENT)
    print("\nBASELINE (the current build's parts on this file's parametric airframe model; the search starts here)")
    base = describe("baseline", BASELINE)
    OPTIONS.update(saved)

    results = {"baseline": dict(BASELINE, **{k: base[k] for k, _ in METRICS})}
    print(f"\nHILL CLIMBS (each from the baseline and 5 random starts; floors: top speed >= {FLOORS['vmax'] * 3.6:.0f} km/h, "
          f"endurance >= {FLOORS['endurance_min']:.0f} min, range >= {FLOORS['range_km']:.0f} km)")
    winners = {}
    for name, label in (("vmax", "fastest"), ("endurance_min", "longest endurance"), ("range_km", "longest range"),
                        ("mass_g", "lightest"), ("cost", "cheapest"), ("balanced", "best all-round")):
        print(f"\n  climbing: {label}")
        d, _ = multi_start(objective(name, base), log=print)
        winners[label] = d
        r = describe(label.upper(), d, base)
        results[label] = dict(d, **{k: r[k] for k, _ in METRICS})

    rows = {}
    for key, r in _CACHE.items():
        # key: parts, cr, ct, span, esc, mass_table, avionics_w, assumptions, options, full
        if (key[7] or key[9] or key[3] != round(SPAN, 4) or key[4] or key[5] or key[8] != tuple(sorted(OPTIONS.items()))
                or not meets_floors(r)):
            continue
        rows[key[:3]] = (dict(zip(KEYS, key[0]), cr=key[1], ct=key[2]), r)
    front = pareto(list(rows.values()))
    print(f"\nPARETO FRONT: {len(front)} non-dominated designs out of {len(rows)} feasible ones evaluated. Cheapest ten:")
    print("    top speed |  endur. | range |  mass |   cost  | motor / prop / battery")
    for d, r in sorted(front, key=lambda t: t[1]["cost"])[:10]:
        print(f"    {line(r)} | {d['motor']} / {d['prop']} / {d['battery'][:34]}")
    results["pareto"] = [dict(d, **{k: r[k] for k, _ in METRICS}) for d, r in sorted(front, key=lambda t: t[1]["cost"])]

    pick = winners["best all-round"]
    print("\nBILL OF MATERIALS, best all-round")
    bom(pick)
    full = evaluate(pick, full=True)
    sim = fly_mission(pick, full["v_cruise"])
    print(f"\nTIME-STEPPED MISSION, best all-round: {sim['range_km']:.0f} km in {sim['minutes']:.0f} min, lowest pack voltage "
          f"{sim['v_min']:.1f} V  (closed form: {full['range_km']:.0f} km)")
    mc = monte_carlo(pick)
    print(f"MONTE CARLO, best all-round, 300 runs ({100 * mc[:, 3].mean():.0f} % meet every constraint):")
    for label, col, k in (("top speed km/h", 0, 3.6), ("endurance min", 1, 1), ("range km", 2, 1)):
        p10, p50, p90 = np.percentile(mc[:, col] * k, [10, 50, 90])
        print(f"    {label:15s} 10th pct {p10:5.0f}   median {p50:5.0f}   90th pct {p90:5.0f}")
    results["mission_check"] = sim
    (HERE / "results.json").write_text(json.dumps(results, indent=1))
    print("\nwrote sim/results.json")


if __name__ == "__main__":
    main()
