"""Kestrel-X Stealth v9 Under-$300 — the v8 Budget composite stealth VTOL airframe, re-pocketed for the $295 parts set.

v9.2 airframe changes (src/printable_v9_1_backup.py is the previous CAD): spar caps 1.0 / 0.75 / 0.5 mm sized for a 2.10 kg
design gross mass, symmetric 148 mm X-tail fins, battery brick 20 mm aft, nacelle tilt +/-30 deg.

v9 changes against v8 (src/printable_v8_backup.py): the outboard elevon servo pocket is cut for the Emax ES9051
(19.7 x 8.3 x 23.3 mm), the nacelle ESC envelope is 35 x 17 mm, the props are 9 in (v9.1, 37.8 mm from the nose), and the
external Hall sensor is gone (the pack feeds the flight controller PDB). Planform, spar, body, bay, fins and nacelles are
unchanged, so every v8 aero, structure and CFD result still applies. The bay keeps its 216 x 134 x 24 mm size: one 6S1P
brick (144 x 66 x 24 mm) on the centreline, or two side by side. Parts and prices: sim/aircraft.py and sim/catalog.json.

The v8 notes below describe the shared airframe; where they name v8 parts (Cine66, Raptor 5, PTK 7308, 6S2P/6S3P, $490),
read the v9 parts above.


Run:  cad run src/check_print.py       (writes 24 printable STLs + print/mass_report.json, checks 0 clashes incl. +/-20 deg tilt)
      cad run src/check_prop.py        (checks 8" prop clearance to nose & centrebody)
      cad run src/cfd_shape.py         (writes cfd/half.step for 3D SU2 INC_EULER CFD)

Cost-optimized architecture (~$490 complete w/ HD digital & pitot, -61% vs $1,249 v7) keeping ALL Claude Opus 5.5 gates:
  - 1,880 mm true projected span (TRUE AR ~ 11.5, L/D ~ 19.4-20.0), 23.0 deg stealth sweep, +3.5 deg dihedral,
    integrated 36 deg outward-canted stealth wingtip sharklets, NeuralFoil CST stealth reflex airfoil.
  - Dual-pack 216 x 134 x 24 mm bay centred near CG: flies BOTH cheap 6S2P Molicel P45B ($80, 1.94 kg, 3.9 hr / 200 km)
    AND drop-in 6S3P P45B/EVE 50PL ($118-$154, 2.38-2.42 kg, 4.9-5.3 hr / 275-305 km) with < 2 mm CG shift!
  - Cheap $18 2-ply +/-45 deg 49 g/m2 E-glass-epoxy skin (RF-transparent EVERYWHERE -> zero antenna blinding!) +
    cheap $22 standard 135 GPa T700 pultruded carbon flat spar caps (1.25 / 1.00 / 0.50 mm) inside camber-centred
    10.8 / 7.6 / 4.4 mm CAD channels (verified >= 0.70 mm skin cover across FULL cap width s=0.22..0.38!) +
    >= 0.70 mm carbon shear web + monolithic 32 x 15 x 1.5 mm root V-joiner torsion box -> >= 20 g @ 400 MPa, V_NE = 242 km/h!
  - 100% WOT-safe 6S powertrain (ZERO fresh-pack ESC overload & ZERO prop overspeed even at 100% throttle!):
    2x T-Motor Cine66 2812 925KV (or $66/pr Avenger 2812 900KV) + matched CW+CCW APC 8x8E/8x8EP + 2x Flycolor Raptor 5 50A BLHeli32.
  - Flush budget metal-gear servos ($49 all 4!): 2x PTK 7308-MG-D (8.0 mm thin, 9.2 g, 3.2 kgcm) on outboard elevons +
    2x PTK 7455 MG-D (11.7 mm inside 15.9 mm wing_1 root, 4.5 kgcm) on +/-20 deg vectored tilt-nacelles!
"""
import json
import math
import pathlib

import build123d as bd

# ---- planform (converged Devin + Claude Opus 5.5 budget optimum — cheap T700 + E-glass!) --------
HALF = 940.0                      # 1,880 mm true projected span (TRUE AR ~ 11.5, L/D ~ 19.4-20.0)
BLEND_Y = 106.0                   # chined body width = 212 mm (<= 220 mm printer bed!)
ROOT_LE = 160.0                   # wing LE x at blend_y -> dead-on 8.5-11% SM on BOTH 6S2P & 6S3P!
ROOT_CHORD = 178.0                # wing chord at blend_y (local thickness = 16.5 mm -> fits 11.7 mm tilt servo!)
TIP_CHORD = 80.0                  # wing chord at tip y = 940 mm
SWEEP_DEG = 23.0                  # optimal sweep for cheap 135 GPa T700 V_bff > 700 km/h & V_NE = 242 km/h!
SWEEP = math.tan(math.radians(SWEEP_DEG))
TIP_LE = ROOT_LE - SWEEP * (HALF - BLEND_Y)
DIHEDRAL_DEG = 3.5
DIHEDRAL = math.tan(math.radians(DIHEDRAL_DEG))
WASHOUT = 4.5

# ---- NeuralFoil-optimized CST stealth reflex airfoil (sim/stealth_airfoil.json) -----------------
_AF_JSON = pathlib.Path(__file__).resolve().parent.parent / "sim" / "stealth_airfoil.json"
_AF = json.loads(_AF_JSON.read_text()) if _AF_JSON.exists() else {}
CST_WT = tuple(_AF.get("w_t", [0.130219, 0.144404, 0.114957, 0.123775, 0.117387]))
CST_WC = tuple(_AF.get("w_c", [0.068089, 0.056603, -0.029446, -0.015524]))
CST_DZ_TE = float(_AF.get("dz_te", 0.0025))
THICKNESS = 0.093
TIP_T_SCALE = 0.92
CAMBER_K, CAMBER_A = 0.14, 0.68

# ---- stealth faceted chined centerbody (dual-pack 216x134x24 mm bay fits BOTH 6S2P & 6S3P!) -----
D0 = (278.0, -286.0, 18.6, 18.6)          # 564 mm x 37.2 mm (t/c = 6.6% razor stealth delta diamond!)
D_BAY = (206.0, -214.0, 17.6, 17.6)       # at y = 70 mm: unbroken stealth chine + diamond beaver-tail
BAY_Y = 69.0                              # 138 mm clear interior width for 134 mm wide 6S3P (or 90 mm 6S2P w/ foam!)
NOSE_FRAC = 0.20                          # full-height bay starts ahead of pack front (+116 mm)
FLAT_END = 0.80                           # full-height bay continues past pack rear (-100 mm)
BODY_REFLEX = 5.4                         # mm upward aft reflex for self-trimming
WALL = 1.2                                # LW-PLA core wall wrapped in 2-ply +/-45 49g/m2 E-glass RF radome!
NOSE_JOINT, MID_JOINT, TAIL_JOINT = 122.0, 4.0, -114.0    # 4 bed-sized body segments (all <= 172 x 212 mm!)

# ---- 100 % ALL-CARBON primary spar box & swept V-joiner (CHEAP 135 GPa T700 flat bars!) ---------
SPAR_FRAC, RWEB_FRAC, WIRE_FRAC, SERVO_FRAC = 0.30, 0.70, 0.12, 0.50
SPAR_Y0 = 70.0
CARRY_X, CARRY_Y = 138.0, 106.0           # monolithic 1.5 mm woven root V-joiner box (axes intersect!)
CARRY_BOX_W, CARRY_BOX_H = 32.0, 15.0     # EXACT 32 x 15 mm carbon root V-box matching aeroelastic.py!
CAP_W_IN, CAP_W_MID, CAP_W_OUT = 18.0, 16.0, 12.0
CAP_T_IN, CAP_T_MID, CAP_T_OUT = 1.00, 0.75, 0.50   # v9.2: sized for 2.10 kg (two 6S1P bricks + margin), not v8's 2.5 kg 6S3P
SPAR_H_IN, SPAR_H_MID, SPAR_H_OUT = 10.8, 7.6, 4.4  # camber-centred (>= 0.75 mm cover across FULL cap width!)
WEB_T = 1.0                               # slot for >= 0.70 mm +/-45 carbon shear web (tau_web <= 40 MPa!)
PIN_D = 2.0

# ---- enlarged 22 deg canted swept NACA0006 stealth twin V-tails & ventral landing skids ---------
FIN_CANT = 22.0
FIN_ROOT_CLIP = 1.5
FIN_Y_UP, FIN_Y_DOWN = 58.0, 66.0
FIN_X_FOOT = -342.0                       # common vertical tailsitter landing plane
FIN_UP_H, FIN_DN_H = 148.0, 148.0         # v9.2: symmetric X-tail, same total fin area as v8's 178/118, wider ground stance
FIN_ROOT_X = (-162.0, -264.0)             # 102 mm root chord cleanly inside body_tail behind TAIL_JOINT=-114!
FIN_TIP_C = 64.0
FIN_THICK = 0.06

# ---- integrated stealth wingtip sharklets (printed as part of wing_5_right/left!) ---------------
SHARK_H = 62.0
SHARK_CANT = 36.0
SHARK_SWEEP = 38.0
SHARK_TIP_C = 38.0

# ---- +/-20 deg pitch-vectoring tilt-nacelles & 100% WOT-safe 6S budget powertrain ---------------
MOTOR_Y = 236.0                           # 8" prop R=101.6 mm -> inner tip at y=134.4 mm (>28 mm body clearance!)
MOTOR_Z = DIHEDRAL * (MOTOR_Y - BLEND_Y)  # +7.95 mm dead-center on dihedral wing leading edge!
MOTOR_D, MOTOR_L = 34.4, 28.0             # 28xx envelope: Glee 2810 900KV (33.5 x 20.5); also Glee 2812 / Flashhobby Mars M2812 (34.3 x 27)
MOUNT_SLOT = (8.5, 10.5)
POD_R, POD_WALL, FIREWALL_X, FIREWALL_T = 18.2, 1.4, 174.0, 3.5
TILT_SHAFT_X, TILT_SHAFT_D = 136.0, 5.2   # Ø5 mm tilt pivot at x=+136 mm
TILT_MAX_DEG = 30.0                       # v9.2: nacelle travel (v8: 20); src/check_print.py checks the pod sweep to this angle
PROP_D = 228.6                            # 9 in: APC 9x7.5E (LP09075E) + APC 9x7.5EP (LP09075EP) matched CW+CCW pair
ESC_DIM = (35.5, 17.5, 6.0)               # v9.1: Readytosky 45 A BLHeli_S (28.5 x 13 x 5 mm); the envelope also takes 35 x 17 mm ESCs

# Budget servos ($23 all 4), all on the F405 Wing Mini's 5 V servo rail (the Mini has no 7.2 V setting):
#   - Elevons: 2x Emax ES9051 (8.3 mm thick, 4.3 g, 0.8 kgcm, 4.0-5.5 V) flush in the 10 mm outer wing
#   - Tilt:    2x PTK 7455 MG-D (11.7 mm thick, 12.95 g, 2.5-3.0 kgcm @ 5 V) flush in the 15.9 mm wing_1 root
ELEV_SERVO = (20.2, 23.6, 8.5)            # v9: Emax ES9051 pocket (19.7 x 23.3 x 8.3 mm servo); the metal-gear ES9052MD
ELEV_LUGS = (28.4, 4.0, 12.0)             # upgrade is 8.8 mm thick and needs the depth set to 9.0 (leaves 0.99 mm of skin)
TILT_SERVO = (24.0, 27.2, 11.9)           # PTK 7455 MG-D pocket in wing_1 root (15.9 mm thick -> >= 2.0 mm skin!)
TILT_LUGS = (32.5, 4.5, 14.0)
TILT_SERVO_Y = 162.0
ELEV_SERVO_Y = 560.0

# Dual-pack bay (216 x 134 x 24 mm): fits BOTH 6S2P (216x90x24) AND 6S3P (216x134x24) centred near CG!
PACK_DIM = (216.0, 134.0, 24.0)
BAY_X = 6.0                               # bay envelope runs x = -102 .. +114 mm
BRICK_DIM = (144.0, 66.0, 24.0)           # one 6S1P brick (2 cells long x 3 wide); two sit side by side
BATTERY_X = -14.0                         # v9.2: brick station, 20 mm aft of the bay centre, for a 10.5 % static margin
HATCH_X, HATCH_Y = (-108.0, 118.0), 68.0  # 226 x 136 mm stressed bolted hatch opening (split at MID_JOINT=+4!)
FC_DIM = (37.0, 26.0, 14.0)               # SpeedyBee F405 Wing Mini ($39.99, 2-6S, 7.2V servo BEC, 23 g)
TRAY_Z = -2.2
VTX = (23.5, 23.5, 8.0)                   # Walksnail Ascent HD Lite+ VTX
VTX_ANT_X, VTX_ANT_D = 208.0, 6.5
CAM = (228.0, 0.0, -1.5)                  # Recessed stealth EO/IR nano camera
CAM_TILT = 30.0
CAM_W = 14.0
GPS_DIM = (18.0, 18.0, 6.0)
RID_DIM = (24.0, 16.0, 6.5)
RX_DIM = (10.5, 10.5, 3.0)
WIRE_ENTRY_X, WIRE_ENTRY_Z = 68.0, -8.5

# ---- outboard mass-balanced stealth elevons & 5 bed-sized wing panels/side ----------------------
HINGE = 0.70
ELEVON_Y = (550.0, 750.0)                 # 200 mm span (<= 220 mm bed!)
WING_SPLITS = (BLEND_Y, 240.0, 410.0, 580.0, 760.0, HALF)   # 5 panels/side, ALL fit 220x220x250 bed!

SKIN, DARK, PETG = "#1c2026", "#0d1014", "#15191f"
C = bd.Align.CENTER


# ---- geometry helpers ---------------------------------------------------------------------------
def le_x(y):
    return ROOT_LE - SWEEP * (abs(y) - BLEND_Y)


def chord(y):
    t = max(0.0, (abs(y) - BLEND_Y) / (HALF - BLEND_Y))
    return ROOT_CHORD + (TIP_CHORD - ROOT_CHORD) * t


def z_dih(y):
    return DIHEDRAL * max(0.0, abs(y) - BLEND_Y)


def line_x(frac, y):
    return le_x(y) - frac * chord(y)


def stations(n=36):
    return [0.5 * (1.0 - math.cos(math.pi * i / n)) for i in range(n + 1)]


def bernstein(n, i, s):
    return math.comb(n, i) * (s ** i) * ((1.0 - s) ** (n - i))


def airfoil_pts(c, t_scale=1.0):
    nt, nc = len(CST_WT) - 1, len(CST_WC) - 1
    up, lo = [], []
    for s in stations():
        c_cls = math.sqrt(max(s, 0.0)) * (1.0 - s)
        yt = t_scale * (c_cls * sum(w * bernstein(nt, i, s) for i, w in enumerate(CST_WT)) + s * CST_DZ_TE / 2.0)
        yc = s * (1.0 - s) * sum(w * bernstein(nc, i, s) for i, w in enumerate(CST_WC))
        up.append((s * c, (yc + yt) * c))
        lo.append((s * c, (yc - yt) * c))
    return up, lo


def surf(s, y, upper=True):
    c = chord(y)
    t_scale = 1.0 + (TIP_T_SCALE - 1.0) * max(0.0, (abs(y) - BLEND_Y) / (HALF - BLEND_Y))
    nt, nc = len(CST_WT) - 1, len(CST_WC) - 1
    c_cls = math.sqrt(max(s, 0.0)) * (1.0 - s)
    yt = t_scale * (c_cls * sum(w * bernstein(nt, i, s) for i, w in enumerate(CST_WT)) + s * CST_DZ_TE / 2.0)
    yc = s * (1.0 - s) * sum(w * bernstein(nc, i, s) for i, w in enumerate(CST_WC))
    z_local = (yc + (yt if upper else -yt)) * c
    tw = math.radians(WASHOUT * max(0.0, abs(y) - BLEND_Y) / (HALF - BLEND_Y))
    dx = (SPAR_FRAC - s) * c
    return z_dih(y) + z_local * math.cos(tw) - dx * math.sin(tw)


def spar_point(frac, y):
    """Exact cambered airfoil midline point at chord fraction frac (prevents lower-skin breakout!)."""
    if abs(y) <= BLEND_Y:
        zc = 0.5 * (surf(frac, BLEND_Y, True) + surf(frac, BLEND_Y, False))
        f = max(0.0, (abs(y) - SPAR_Y0) / max(BLEND_Y - SPAR_Y0, 1.0))
        return (line_x(frac, y), y, f * zc)
    zc = 0.5 * (surf(frac, y, True) + surf(frac, y, False))
    return (line_x(frac, y), y, zc)


def body_f(s):
    if s < NOSE_FRAC:
        t = (NOSE_FRAC - s) / NOSE_FRAC
        return (1.0 - t * t) ** 0.58
    if s < FLAT_END:
        return 1.0
    return 0.02 + 0.98 * 0.5 * (1.0 + math.cos(math.pi * (s - FLAT_END) / (1.0 - FLAT_END)))


def body_camber(s):
    return BODY_REFLEX * max(0.0, (s - 0.62) / 0.38) ** 2


def profile_face(y, upper, lower):
    up = [(x, y, z) for x, z in upper]
    lo = [(x, y, z) for x, z in reversed(lower)]
    edges = [bd.Edge.make_spline([bd.Vector(*p) for p in up]),
             bd.Edge.make_spline([bd.Vector(*p) for p in lo])]
    if (bd.Vector(*up[-1]) - bd.Vector(*lo[0])).length > 1e-4:
        edges.insert(1, bd.Edge.make_line(bd.Vector(*up[-1]), bd.Vector(*lo[0])))
    if (bd.Vector(*lo[-1]) - bd.Vector(*up[0])).length > 1e-4:
        edges.append(bd.Edge.make_line(bd.Vector(*lo[-1]), bd.Vector(*up[0])))
    return bd.Face(bd.Wire(edges))


def airfoil_face(y):
    c = chord(y)
    t_scale = 1.0 + (TIP_T_SCALE - 1.0) * max(0.0, (abs(y) - BLEND_Y) / (HALF - BLEND_Y))
    up, lo = airfoil_pts(c, t_scale)
    lx = le_x(y)
    tw = WASHOUT * max(0.0, abs(y) - BLEND_Y) / (HALF - BLEND_Y)
    piv = (lx - SPAR_FRAC * c, z_dih(y))

    def rot(px, pz):
        dx, dz = px - piv[0], pz - piv[1]
        a = math.radians(tw)
        return (piv[0] + dx * math.cos(a) + dz * math.sin(a),
                piv[1] - dx * math.sin(a) + dz * math.cos(a))

    zd = z_dih(y)
    return profile_face(y, [rot(lx - x, zd + z) for x, z in up], [rot(lx - x, zd + z) for x, z in lo])


def body_face(y, dims, inset=0.0):
    x_le, x_te, top, bot = dims
    x_le, x_te = x_le - inset, x_te + inset
    top, bot = max(top - inset, 0.5), max(bot - inset, 0.5)
    up, lo = [], []
    for s in stations():
        x = x_le - s * (x_le - x_te)
        f, zc = body_f(s), body_camber(s)
        up.append((x, zc + top * f))
        lo.append((x, zc - bot * f))
    return profile_face(y, up, lo)


def box(x0, x1, y0, y1, z0, z1):
    return bd.Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * bd.Box(x1 - x0, y1 - y0, z1 - z0)


def along_x(shape):
    return shape.rotate(bd.Axis.Y, 90)


def rod(a, b, r):
    v = bd.Vector(*b) - bd.Vector(*a)
    return bd.Plane(origin=bd.Vector(*a), z_dir=v.normalized()) * bd.Cylinder(r, v.length, align=(C, C, bd.Align.MIN))


def rect_beam(a, b, w, h):
    va, vb = bd.Vector(*a), bd.Vector(*b)
    v = vb - va
    zdir = v.normalized()
    xdir = (bd.Vector(0, 0, 1).cross(zdir)).normalized()
    pl = bd.Plane(origin=va, x_dir=xdir, z_dir=zdir)
    return pl * bd.Box(w, h, v.length, align=(C, C, bd.Align.MIN))


def both(shape):
    return shape + shape.mirror(bd.Plane.XZ)


def named(shape, label, color=SKIN):
    shape.label, shape.color = label, color
    return shape


# ---- camber-centred T700 carbon spar channels (full cap-width skin-cover verified!) -------------
Y_S1 = round(BLEND_Y + 0.30 * (HALF - BLEND_Y), 1)   # 356.2 mm
Y_S2 = round(BLEND_Y + 0.62 * (HALF - BLEND_Y), 1)   # 623.1 mm


def spar_cutouts(clear=0.25):
    vbox = box(CARRY_X - CARRY_BOX_W / 2 - clear, CARRY_X + CARRY_BOX_W / 2 + clear,
               -CARRY_Y, CARRY_Y, -CARRY_BOX_H / 2 - clear, CARRY_BOX_H / 2 + clear)
    # Full cap-width skin-cover assertion (s = 0.22..0.38, closing Claude's front-edge check!)
    for y_chk, h_chk, w_chk in ((BLEND_Y, SPAR_H_IN, CAP_W_IN), (Y_S1, SPAR_H_IN, CAP_W_IN),
                                (Y_S1, SPAR_H_MID, CAP_W_MID), (Y_S2, SPAR_H_MID, CAP_W_MID),
                                (Y_S2, SPAR_H_OUT, CAP_W_OUT), (HALF - 16.0, SPAR_H_OUT, CAP_W_OUT)):
        half_sf = (w_chk / 2.0 + clear) / chord(y_chk)
        ss_chk = (SPAR_FRAC - half_sf, SPAR_FRAC, SPAR_FRAC + half_sf)
        zu = min(surf(s, y_chk, True) for s in ss_chk)
        zl = max(surf(s, y_chk, False) for s in ss_chk)
        zc = spar_point(SPAR_FRAC, y_chk)[2]
        assert zu - (zc + h_chk / 2.0) >= 0.70 and (zc - h_chk / 2.0) - zl >= 0.70, (
            f"spar channel at y={y_chk} leaves < 0.70 mm cover: zu={zu:.2f}, zl={zl:.2f}, box=[{zc-h_chk/2:.2f},{zc+h_chk/2:.2f}]"
        )
    s_in = rect_beam(spar_point(SPAR_FRAC, SPAR_Y0), spar_point(SPAR_FRAC, Y_S1), CAP_W_IN + 2 * clear, SPAR_H_IN)
    s_mid = rect_beam(spar_point(SPAR_FRAC, Y_S1 - 20.0), spar_point(SPAR_FRAC, Y_S2), CAP_W_MID + 2 * clear, SPAR_H_MID)
    s_out = rect_beam(spar_point(SPAR_FRAC, Y_S2 - 20.0), spar_point(SPAR_FRAC, HALF - 16.0), CAP_W_OUT + 2 * clear, SPAR_H_OUT)
    rweb = rect_beam(spar_point(RWEB_FRAC, BLEND_Y + 4.0), spar_point(RWEB_FRAC, HALF - 20.0), WEB_T + 2 * clear, 3.4)
    t_bore = rod((TILT_SHAFT_X, 155.0, MOTOR_Z), (TILT_SHAFT_X, MOTOR_Y + 22.0, MOTOR_Z), TILT_SHAFT_D / 2)
    return vbox + both(s_in + s_mid + s_out + rweb + t_bore)


def wire_runs():
    w_in = rod((WIRE_ENTRY_X, BAY_Y - 12, WIRE_ENTRY_Z), (TILT_SHAFT_X - 18.0, MOTOR_Y, MOTOR_Z - 4.0), 3.6)
    w_sv = rod(spar_point(WIRE_FRAC, BLEND_Y - 15), spar_point(WIRE_FRAC, ELEV_SERVO_Y + 12), 2.5)
    return both(w_in + w_sv)


def naca0006_section(x_le, c, y, z, cant_rad, thick=FIN_THICK):
    ny, nz = math.cos(cant_rad), -math.sin(cant_rad)
    up, lo = [], []
    for s in stations(22):
        yt = 5.0 * thick * c * (0.2969 * math.sqrt(s) - 0.1260 * s - 0.3516 * s**2 + 0.2843 * s**3 - 0.1015 * s**4)
        x = x_le - s * c
        up.append((x, y + yt * ny, z + yt * nz))
        lo.append((x, y - yt * ny, z - yt * nz))
    up_v = [bd.Vector(*p) for p in up]
    lo_v = [bd.Vector(*p) for p in reversed(lo)]
    edges = [bd.Edge.make_spline(up_v), bd.Edge.make_spline(lo_v)]
    if (up_v[-1] - lo_v[0]).length > 1e-4:
        edges.insert(1, bd.Edge.make_line(up_v[-1], lo_v[0]))
    if (lo_v[-1] - up_v[0]).length > 1e-4:
        edges.append(bd.Edge.make_line(lo_v[-1], up_v[0]))
    return bd.Face(bd.Wire(edges))


def fin_plate(upper, clear=0.0):
    y0 = FIN_Y_UP if upper else FIN_Y_DOWN
    sign = 1.0 if upper else -1.0
    h = FIN_UP_H if upper else FIN_DN_H
    cant = math.radians(FIN_CANT)
    x_root_le, x_root_te = FIN_ROOT_X
    c_root = (x_root_le - x_root_te) + 2.0 * clear
    c_tip = FIN_TIP_C + 2.0 * clear
    x_tip_le = FIN_X_FOOT + FIN_TIP_C + clear
    dy_in, dz_in = -18.0 * math.sin(cant), -sign * 18.0 * math.cos(cant)
    dy_tip, dz_tip = h * math.sin(cant), sign * h * math.cos(cant)
    t_eff = FIN_THICK + (2.0 * clear / max(c_root, 40.0))
    f0 = naca0006_section(x_root_le + clear, c_root, y0 + dy_in, dz_in, sign * cant, t_eff)
    f1 = naca0006_section(x_tip_le, c_tip, y0 + dy_tip, dz_tip, sign * cant, t_eff)
    blade = bd.loft([f0, f1], ruled=True)
    if upper:
        return blade - box(-420, 0, 0, 260, -220, FIN_ROOT_CLIP - clear)
    return blade - box(-420, 0, 0, 260, -FIN_ROOT_CLIP + clear, 220)


def fin_slots():
    return both(fin_plate(True, 0.25) + fin_plate(False, 0.25))


def pin_holes_at(y):
    return (rod(spar_point(0.18, y - 8), spar_point(0.18, y + 8), PIN_D / 2)
            + rod(spar_point(0.55, y - 8), spar_point(0.55, y + 8), PIN_D / 2))


def joint_pins():
    pins = pin_holes_at(WING_SPLITS[0])
    for y in WING_SPLITS[1:-1]:
        pins += pin_holes_at(y)
    return both(pins)


# ---- faceted chined stealth centerbody (4 bed-sized parts + 2 stressed bolted hatches) ----------
def body_outer():
    guide = BLEND_Y + 26.0
    blend = bd.loft([
        airfoil_face(-guide), airfoil_face(-BLEND_Y),
        body_face(-70.0, D_BAY), body_face(0.0, D0), body_face(70.0, D_BAY),
        airfoil_face(BLEND_Y), airfoil_face(guide),
    ], ruled=True)
    return blend & box(-360, 320, -BLEND_Y, BLEND_Y, -60, 60)


def body_inset(inset):
    return bd.loft([
        body_face(-BAY_Y, D_BAY, inset + 0.5),
        body_face(0.0, D0, inset),
        body_face(BAY_Y, D_BAY, inset + 0.5),
    ], ruled=True)


def collar(x_joint, direction, length=5.0, thick=1.2, clear=0.2):
    xa, xb = (x_joint - 0.6, x_joint + length) if direction > 0 else (x_joint - length, x_joint + 0.6)
    return (body_inset(WALL + clear) - body_inset(WALL + clear + thick)) & box(xa, xb, -BAY_Y, BAY_Y, -35, 35)


def hatch_opening(shrink=0.0):
    return box(HATCH_X[0] + shrink, HATCH_X[1] - shrink, -HATCH_Y + shrink, HATCH_Y - shrink, 11.0, 45.0)


def cam_axis(length):
    a = math.radians(CAM_TILT)
    return (CAM[0] + length * math.cos(a), CAM[1], CAM[2] - length * math.sin(a))


def cooling_ducts():
    inlet = both(rod((218.0, 24.0, -5.0), (182.0, 18.0, -6.5), 3.0))
    exhaust = box(-236.0, -214.0, -10.0, 10.0, 4.0, 28.0)
    return inlet + exhaust


def body_parts():
    outer, cavity = body_outer(), body_inset(WALL)
    shell = outer - cavity
    carry_sleeve = box(CARRY_X - CARRY_BOX_W / 2 - 2.0, CARRY_X + CARRY_BOX_W / 2 + 2.0,
                       -BAY_Y - 2, BAY_Y + 2, -CARRY_BOX_H / 2 - 2.0, CARRY_BOX_H / 2 + 2.0) & cavity
    ledge = ((cavity - body_inset(WALL + 1.2))
             & box(HATCH_X[0] - 3.0, HATCH_X[1] + 3.0, -HATCH_Y - 1.5, HATCH_Y + 1.5, 5.0, 40.0)
             ) - box(HATCH_X[0] + 2.0, HATCH_X[1] - 2.0, -HATCH_Y + 1.2, HATCH_Y - 1.2, 0.0, 40.0)
    cuts = spar_cutouts() + wire_runs() + fin_slots() + joint_pins() + cooling_ducts()

    fwd_zone = box(MID_JOINT, NOSE_JOINT, -120, 120, -45, 45)
    aft_zone = box(TAIL_JOINT, MID_JOINT, -120, 120, -45, 45)
    hatch_skin_all = shell & hatch_opening(0.3)
    hatch_fwd = hatch_skin_all & box(MID_JOINT + 0.2, NOSE_JOINT, -120, 120, -45, 45)
    hatch_aft = hatch_skin_all & box(TAIL_JOINT, MID_JOINT - 0.2, -120, 120, -45, 45)

    bay_fwd_raw = ((shell & fwd_zone) + (ledge & fwd_zone) + collar(NOSE_JOINT, 1) + collar(MID_JOINT, -1)
                   - hatch_opening() - cuts)
    bay_aft_raw = ((shell & aft_zone) + (ledge & aft_zone) + collar(TAIL_JOINT, -1)
                   - collar(MID_JOINT, -1, thick=1.6, clear=0.0) - hatch_opening() - cuts)
    bay_fwd = max(bay_fwd_raw.solids(), key=lambda s: s.volume)
    bay_aft = max(bay_aft_raw.solids(), key=lambda s: s.volume)

    nose_zone = box(NOSE_JOINT, 300, -120, 120, -45, 45)
    fc_tray_raw = box(160.0, 206.0, -54, 54, TRAY_Z - 1.2, TRAY_Z) & outer
    vtx_air_slot = box(164.0, 186.0, -9.0, 9.0, TRAY_Z - 3.0, TRAY_Z + 1.0)
    fc_tray = fc_tray_raw - vtx_air_slot
    rails = both(box(CAM[0] - 8.5, CAM[0] + 8.5, CAM_W / 2 + 0.2, CAM_W / 2 + 2.2, -40, 40)) & outer
    screws = rod((CAM[0], -24, CAM[2]), (CAM[0], 24, CAM[2]), 1.1)
    window = rod(CAM, cam_axis(55), 5.8)
    antenna = bd.Pos(VTX_ANT_X, 0, -32) * bd.Cylinder(VTX_ANT_D / 2, 32, align=(C, C, bd.Align.MIN))
    nose_raw = (shell & nose_zone) + (carry_sleeve & nose_zone) + fc_tray + rails - screws - window - antenna - cuts
    nose = max(nose_raw.solids(), key=lambda s: s.volume)

    tail_zone = box(-310, TAIL_JOINT, -120, 120, -45, 45)
    tail_shelf = (box(-168.0, -120.0, -50, 50, -4.5, -3.3) & outer) - box(-156.0, -132.0, -7.0, 7.0, -6.0, -2.5)
    tail_raw = (shell & tail_zone) + tail_shelf - cuts
    tail = max(tail_raw.solids(), key=lambda s: s.volume)
    return dict(nose=nose, bay_fwd=bay_fwd, bay_aft=bay_aft, tail=tail,
                hatch_fwd=hatch_fwd, hatch_aft=hatch_aft, outer=outer)


# ---- swept wing (5 bed-sized panels/side + integrated sharklet + flush budget servos) -----------
def winglet_right():
    lx0, zd0 = le_x(HALF), z_dih(HALF)
    cant, sw = math.radians(SHARK_CANT), math.radians(SHARK_SWEEP)
    ny, nz = math.cos(cant), -math.sin(cant)
    y_tip = HALF + SHARK_H * math.sin(cant)
    z_tip = zd0 + SHARK_H * math.cos(cant)
    x_tip_le = lx0 - SHARK_H * math.tan(sw)
    f0 = airfoil_face(HALF)
    up, lo = airfoil_pts(SHARK_TIP_C, 0.72)
    up_3d = [bd.Vector(x_tip_le - x, y_tip + z * ny, z_tip + z * nz) for x, z in up]
    lo_3d = [bd.Vector(x_tip_le - x, y_tip + z * ny, z_tip + z * nz) for x, z in reversed(lo)]
    edges = [bd.Edge.make_spline(up_3d), bd.Edge.make_spline(lo_3d)]
    if (up_3d[-1] - lo_3d[0]).length > 1e-4:
        edges.insert(1, bd.Edge.make_line(up_3d[-1], lo_3d[0]))
    if (lo_3d[-1] - up_3d[0]).length > 1e-4:
        edges.append(bd.Edge.make_line(lo_3d[-1], up_3d[0]))
    return bd.loft([f0, bd.Face(bd.Wire(edges))], ruled=True)


def wing_full():
    ys = (BLEND_Y, 240.0, 410.0, 580.0, 760.0, HALF)
    return bd.loft([airfoil_face(y) for y in ys], ruled=True)


def elevon_prism(gap=0.0):
    y0, y1 = ELEVON_Y
    pts = [(line_x(HINGE, y0) - gap, y0 + gap), (line_x(HINGE, y1) - gap, y1 - gap),
           (line_x(1.0, y1) - 10, y1 - gap), (line_x(1.0, y0) - 10, y0 + gap)]
    return bd.Pos(0, 0, -40) * bd.extrude(bd.Polygon(*pts, align=None), 180)


def servo_pocket_at(y_sv, dim, lugs, min_skin=0.8):
    sx = line_x(SERVO_FRAC, y_sv)
    x0, x1 = sx - dim[0] / 2, sx + dim[0] / 2
    y0, y1 = y_sv - dim[1] / 2, y_sv + dim[1] / 2
    zl = min(surf(s, y_sv, False) for s in (SERVO_FRAC - 0.08, SERVO_FRAC, SERVO_FRAC + 0.08))
    zu = min(surf(s, y_sv, True) for s in (SERVO_FRAC - 0.08, SERVO_FRAC, SERVO_FRAC + 0.08))
    top = zl + dim[2]
    assert zu - top >= min_skin, f"servo pocket at y={y_sv} leaves only {zu - top:.2f} mm of skin (zu={zu:.1f}, top={top:.1f})"
    body_b = box(x0, x1, y0, y1, zl - 10.0, top)
    lug_b = box(sx - lugs[0] / 2, sx + lugs[0] / 2, y_sv - lugs[1] / 2, y_sv + lugs[1] / 2, zl - 10.0, zl + 2.4)
    lead = box(x1 - 2.0, x1 + 6.0, y_sv - 3.5, y_sv + 3.5, zl - 10.0, top)
    return body_b + lug_b + lead


def wing_right_panels():
    full = wing_full() + winglet_right()
    cuts = (elevon_prism(0.0) + spar_cutouts() + wire_runs()
            + servo_pocket_at(TILT_SERVO_Y, TILT_SERVO, TILT_LUGS, min_skin=1.0)
            + servo_pocket_at(ELEV_SERVO_Y, ELEV_SERVO, ELEV_LUGS, min_skin=0.8)
            + joint_pins())
    clean = full - cuts
    panels = []
    for i in range(len(WING_SPLITS) - 1):
        y0, y1 = WING_SPLITS[i], WING_SPLITS[i + 1]
        y_hi = HALF + 140.0 if i == len(WING_SPLITS) - 2 else y1
        panels.append(clean & box(-550, 320, y0, y_hi, -60, 220))
    return panels, full


def elevon_right_solid(full):
    y0, y1 = ELEVON_Y
    raw = full & elevon_prism(0.8)
    chamfer = bd.Pos(line_x(HINGE, (y0 + y1) / 2), (y0 + y1) / 2, z_dih((y0 + y1) / 2) - 22.0) * bd.Box(2.5, y1 - y0 + 20, 44.0)
    return raw - chamfer


def pod_right(full):
    """Pivoting PETG-CF stealth tilt-nacelle (+/-20 deg clearance ahead of wing LE, Flycolor 50A ESC bay!)."""
    x_aft = le_x(MOTOR_Y) + 9.0
    x_cone = x_aft + 14.0
    outer = (bd.Pos((x_cone + FIREWALL_X) / 2, MOTOR_Y, MOTOR_Z) * along_x(bd.Cylinder(POD_R, FIREWALL_X - x_cone))
             + bd.Pos((x_aft + x_cone) / 2, MOTOR_Y, MOTOR_Z) * along_x(bd.Cone(9.0, POD_R, x_cone - x_aft)))
    inner = bd.Pos((x_cone + FIREWALL_X - FIREWALL_T) / 2, MOTOR_Y, MOTOR_Z) * along_x(
        bd.Cylinder(POD_R - POD_WALL, FIREWALL_X - FIREWALL_T - x_cone + 10.0))
    shell = outer - inner
    r_in, r_out = MOUNT_SLOT
    holes = bd.Pos(FIREWALL_X - FIREWALL_T - 2, MOTOR_Y, MOTOR_Z) * along_x(bd.Cylinder(4.8, FIREWALL_T + 5))
    for k in range(4):
        a = math.radians(45 + 90 * k)
        for rr in (r_in, r_out):
            holes += bd.Pos(FIREWALL_X - FIREWALL_T - 2, MOTOR_Y + rr * math.cos(a), MOTOR_Z + rr * math.sin(a)) * along_x(bd.Cylinder(1.65, FIREWALL_T + 5))
    esc_access = box(FIREWALL_X - 42.0, FIREWALL_X - 5.0, MOTOR_Y - 9.5, MOTOR_Y + 9.5, MOTOR_Z - 26.0, MOTOR_Z - 12.0)
    rear_exhaust = bd.Pos(x_aft - 2, MOTOR_Y, MOTOR_Z) * along_x(bd.Cylinder(4.0, 14))
    pod_raw = shell - holes - esc_access - rear_exhaust - full - spar_cutouts()
    return max(pod_raw.solids(), key=lambda s: s.volume)


# ---- caches & 24 printable part entry points ----------------------------------------------------
_BODY_CACHE = None
_WING_CACHE = None


def _get_body():
    global _BODY_CACHE
    if _BODY_CACHE is None:
        _BODY_CACHE = body_parts()
    return _BODY_CACHE


def _get_wings():
    global _WING_CACHE
    if _WING_CACHE is None:
        _WING_CACHE = wing_right_panels()
    return _WING_CACHE


def body_nose():         return named(_get_body()["nose"], "body_nose", SKIN)
def body_bay_fwd():      return named(_get_body()["bay_fwd"], "body_bay_fwd", SKIN)
def body_bay_aft():      return named(_get_body()["bay_aft"], "body_bay_aft", SKIN)
def body_tail():         return named(_get_body()["tail"], "body_tail", SKIN)
def hatch_fwd():         return named(_get_body()["hatch_fwd"], "hatch_fwd", DARK)
def hatch_aft():         return named(_get_body()["hatch_aft"], "hatch_aft", DARK)

def wing_1_right():      return named(_get_wings()[0][0], "wing_1_right", SKIN)
def wing_1_left():       return named(_get_wings()[0][0].mirror(bd.Plane.XZ), "wing_1_left", SKIN)
def wing_2_right():      return named(_get_wings()[0][1], "wing_2_right", SKIN)
def wing_2_left():       return named(_get_wings()[0][1].mirror(bd.Plane.XZ), "wing_2_left", SKIN)
def wing_3_right():      return named(_get_wings()[0][2], "wing_3_right", SKIN)
def wing_3_left():       return named(_get_wings()[0][2].mirror(bd.Plane.XZ), "wing_3_left", SKIN)
def wing_4_right():      return named(_get_wings()[0][3], "wing_4_right", SKIN)
def wing_4_left():       return named(_get_wings()[0][3].mirror(bd.Plane.XZ), "wing_4_left", SKIN)
def wing_5_right():      return named(_get_wings()[0][4], "wing_5_right", SKIN)
def wing_5_left():       return named(_get_wings()[0][4].mirror(bd.Plane.XZ), "wing_5_left", SKIN)

def elevon_right_part(): return named(elevon_right_solid(_get_wings()[1]), "elevon_right", DARK)
def elevon_left_part():  return named(elevon_right_solid(_get_wings()[1]).mirror(bd.Plane.XZ), "elevon_left", DARK)

def tail_fin_right():    return named(fin_plate(True), "tail_fin_right", DARK)
def tail_fin_left():     return named(fin_plate(True).mirror(bd.Plane.XZ), "tail_fin_left", DARK)
def ventral_fin_right(): return named(fin_plate(False), "ventral_fin_right", DARK)
def ventral_fin_left():  return named(fin_plate(False).mirror(bd.Plane.XZ), "ventral_fin_left", DARK)

def motor_pod_right():   return named(pod_right(_get_wings()[1]), "motor_pod_right", PETG)
def motor_pod_left():    return named(pod_right(_get_wings()[1]).mirror(bd.Plane.XZ), "motor_pod_left", PETG)


# ---- bought hardware (exact budget dimensions for 0.00 mm3 clash checks & Cycles renders!) ------
def hardware():
    parts = []
    vbox = box(CARRY_X - CARRY_BOX_W / 2, CARRY_X + CARRY_BOX_W / 2, -CARRY_Y + 0.5, CARRY_Y - 0.5,
               -CARRY_BOX_H / 2, CARRY_BOX_H / 2)
    parts.append(named(vbox, "carbon_root_v_box", "#0c0d10"))

    for side in (1, -1):
        mot = bd.Pos(FIREWALL_X + MOTOR_L / 2, side * MOTOR_Y, MOTOR_Z) * along_x(bd.Cylinder(MOTOR_D / 2, MOTOR_L))
        parts.append(named(mot, "motor_2810_900kv", "#272b33"))
        hub = bd.Pos(FIREWALL_X + MOTOR_L + 4.5, side * MOTOR_Y, MOTOR_Z) * along_x(bd.Cylinder(10.0, 9.0))
        blade = (bd.Pos(FIREWALL_X + MOTOR_L + 5.0, side * MOTOR_Y, MOTOR_Z)
                 * bd.Box(2.2, PROP_D, 15.0).rotate(bd.Axis.Y, 22.0 * side))
        parts.append(named(hub + blade, "prop_9x7.5E_cwccw", "#111318"))
        esc = box(FIREWALL_X - FIREWALL_T - ESC_DIM[0] - 2.0, FIREWALL_X - FIREWALL_T - 2.0,
                  side * MOTOR_Y - ESC_DIM[1] / 2, side * MOTOR_Y + ESC_DIM[1] / 2,
                  MOTOR_Z - 3.0, MOTOR_Z - 3.0 + ESC_DIM[2])
        parts.append(named(esc, "esc_45a_blheli_s", "#b45309"))
        tshaft = rod((TILT_SHAFT_X, side * 158.0, MOTOR_Z), (TILT_SHAFT_X, side * (MOTOR_Y + 18.0), MOTOR_Z), 2.45)
        parts.append(named(tshaft, "tilt_shaft_5mm", "#94a3b8"))

        for y0, y1, cw, ch, lbl in (
            (SPAR_Y0 + 1.0, Y_S1 - 1.0, CAP_W_IN - 0.4, SPAR_H_IN - 0.5, "carbon_spar_t700_inner"),
            (Y_S1 - 19.0, Y_S2 - 1.0, CAP_W_MID - 0.4, SPAR_H_MID - 0.5, "carbon_spar_t700_mid"),
            (Y_S2 - 19.0, HALF - 18.0, CAP_W_OUT - 0.4, SPAR_H_OUT - 0.5, "carbon_spar_t700_outer"),
        ):
            a, b = spar_point(SPAR_FRAC, y0), spar_point(SPAR_FRAC, y1)
            parts.append(named(rect_beam((a[0], side * a[1], a[2]), (b[0], side * b[1], b[2]), cw, ch), lbl, "#0c0d10"))
        ra, rb = spar_point(RWEB_FRAC, BLEND_Y + 6.0), spar_point(RWEB_FRAC, HALF - 22.0)
        parts.append(named(rect_beam((ra[0], side * ra[1], ra[2]), (rb[0], side * rb[1], rb[2]), 0.6, 2.9), "carbon_rear_hinge_web", "#0c0d10"))

        # PTK 7455 MG-D tilt servo (wing_1) + Emax ES9051 elevon servo
        for y_sv, dim, lbl in ((TILT_SERVO_Y, TILT_SERVO, "servo_ptk7455_tilt"),
                               (ELEV_SERVO_Y, ELEV_SERVO, "servo_es9051_elevon")):
            sx = line_x(SERVO_FRAC, y_sv)
            zl = min(surf(s, y_sv, False) for s in (SERVO_FRAC - 0.08, SERVO_FRAC, SERVO_FRAC + 0.08))
            sv = box(sx - dim[0] / 2 + 0.2, sx + dim[0] / 2 - 0.2,
                     side * y_sv - dim[1] / 2 + 0.2, side * y_sv + dim[1] / 2 - 0.2,
                     zl + 0.1, zl + dim[2] - 0.2)
            parts.append(named(sv, lbl, "#0284c7"))

    px, py, pz = BRICK_DIM
    pack = box(BATTERY_X - px / 2, BATTERY_X + px / 2, -py / 2, py / 2, -pz / 2, pz / 2)
    parts.append(named(pack, "battery_6s1p_brick", "#0f766e"))

    fc_x0 = 162.0
    parts.append(named(box(fc_x0, fc_x0 + FC_DIM[0], -FC_DIM[1] / 2, FC_DIM[1] / 2, TRAY_Z + 0.2, TRAY_Z + 0.2 + FC_DIM[2]),
                       "fc_speedybee_f405_mini", "#15803d"))
    vz = TRAY_Z - 1.2 - 0.3
    parts.append(named(box(fc_x0, fc_x0 + VTX[0], -VTX[1] / 2, VTX[1] / 2, vz - VTX[2], vz),
                       "video_transmitter", "#c2410c"))
    cam = bd.Box(9, CAM_W, 12) + bd.Pos(7.9, 0, 0) * along_x(bd.Cylinder(4.5, 6.8))
    parts.append(named(bd.Pos(*CAM) * cam.rotate(bd.Axis.Y, CAM_TILT), "fpv_camera_ascent", "#9333ea"))
    sz = -3.0
    parts.append(named(box(-144.0, -144.0 + GPS_DIM[0], -GPS_DIM[1] / 2, GPS_DIM[1] / 2, sz, sz + GPS_DIM[2]), "gps_m10", "#2563eb"))
    parts.append(named(box(-140.0, -140.0 + RX_DIM[0], -28.0, -28.0 + RX_DIM[1], sz, sz + RX_DIM[2]), "elrs_rx", "#7c3aed"))
    parts.append(named(box(-150.0, -150.0 + RID_DIM[0], 14.0, 14.0 + RID_DIM[1], sz, sz + RID_DIM[2]), "remote_id", "#db2777"))
    return parts


def kestrel_x_stealth():
    return bd.Compound(children=[
        body_nose(), body_bay_fwd(), body_bay_aft(), body_tail(), hatch_fwd(), hatch_aft(),
        wing_1_right(), wing_1_left(), wing_2_right(), wing_2_left(), wing_3_right(), wing_3_left(),
        wing_4_right(), wing_4_left(), wing_5_right(), wing_5_left(),
        elevon_right_part(), elevon_left_part(),
        tail_fin_right(), tail_fin_left(), ventral_fin_right(), ventral_fin_left(),
        motor_pod_right(), motor_pod_left(),
        *hardware(),
    ])


kestrel_560 = kestrel_x_stealth
__all__ = [
    "kestrel_x_stealth", "kestrel_560",
    "body_nose", "body_bay_fwd", "body_bay_aft", "body_tail", "hatch_fwd", "hatch_aft",
    "wing_1_right", "wing_1_left", "wing_2_right", "wing_2_left", "wing_3_right", "wing_3_left",
    "wing_4_right", "wing_4_left", "wing_5_right", "wing_5_left",
    "elevon_right_part", "elevon_left_part",
    "tail_fin_right", "tail_fin_left", "ventral_fin_right", "ventral_fin_left",
    "motor_pod_right", "motor_pod_left",
]
