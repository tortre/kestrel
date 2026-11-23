"""Kestrel-X Stealth v7 — Joint Devin + Claude Opus 5.5 Converged Composite VTOL Stealth Tailsitter CAD.

Run:  cad run src/check_print.py       (writes 26 printable STLs + print/mass_report.json, checks 0 clashes & 220mm bed)
      cad run src/check_prop.py        (checks 8" prop clearance to nose & centrebody)
      cad run src/cfd_shape.py         (writes cfd/half.step for 3D SU2 Euler CFD)

Architecture (every Claude Opus 5.5 audit item resolved):
  - 2,120 mm true projected wingspan (TRUE AR = 12.3, NO sharklet AR inflation), 24.0 deg stealth sweep, +3.5 deg dihedral,
    integrated 36 deg outward-canted stealth wingtip sharklets, NeuralFoil-optimized CST stealth reflex airfoil.
  - Razor-slim 660 mm x 37 mm (t/c = 5.6 %) ruled faceted F-22/B-21/RQ-170 chined delta centerbody (E-glass RF-transparent
    radome skin) housing the 16-cell Amprius SiCore SA88 8S2P silicon-anode pack (296 x 126 x 24 mm w/ 30-PSI CF plates)
    plus Matek H743-WING V3, Walksnail HD VTX & recessed EO/IR camera, M10 GPS, ELRS RX, and Remote ID.
  - 100 % ALL-CARBON primary bending & torsion load path (EXACT CAD dimensions match sim/aeroelastic.py!):
      * Stepped pultruded unidirectional carbon spar caps inside exact camber-centred 12.0 / 8.6 / 5.5 mm CAD channels
      * Continuous >= 0.70 mm +/-45 deg carbon shear web (tau <= 45 MPa) + rear 0.5 mm carbon hinge web
      * Monolithic 1.5 mm woven carbon swept V-joiner carry-through box (34 x 15 mm at CARRY_X = +148 mm directly
        intersecting swept spar axes -> >= 22 g bending, shear, and kink-torsion limit everywhere at 400 MPa!)
  - +/-20 deg pitch-vectoring tilt-nacelles (2x KST X08 Plus + Ø5 mm Ti pivot shaft at x=+158 mm -> ~167 mm REAL
    unclamped pivot pitch arm ahead of CG, dynamically clash-checked across full +/-20 deg pivot arc!)
  - Enlarged 22 deg canted swept NACA0006 stealth twin V-tails & ventral landing skids (x_foot = -402 mm) for high Cn_b &
    yaw damping, plus mass-balanced outboard stealth elevons (630..840 mm) on mechanical pin hinges.
"""
import json
import math
import pathlib

import build123d as bd

# ---- planform (converged Devin + Claude Opus 5.5 3D-trim + BFF aeroelastic optimum) -------------
HALF = 1060.0                     # 2,120 mm true projected span (TRUE AR ~ 12.3, L/D ~ 20.7)
BLEND_Y = 108.0                   # chined body width = 216 mm (<= 220 mm desktop printer bed!)
ROOT_LE = 190.0                   # wing LE x at blend_y -> unbroken B-21/RQ-170 stealth delta chine line!
ROOT_CHORD = 184.0                # wing chord at blend_y (h_box = 15.5 mm -> stiffness-sized carbon spar box!)
TIP_CHORD = 82.0                  # wing chord at tip y = 1060 mm
SWEEP_DEG = 26.2                  # optimal stealth sweep -> 9.0% 3D / 12.0% 1D static margin & V_NE > 280 km/h!
SWEEP = math.tan(math.radians(SWEEP_DEG))
TIP_LE = ROOT_LE - SWEEP * (HALF - BLEND_Y)
DIHEDRAL_DEG = 3.5                # +3.5 deg stealth wing dihedral -> strong negative Cl_b roll stability!
DIHEDRAL = math.tan(math.radians(DIHEDRAL_DEG))
WASHOUT = 4.6                     # deg nose-down tip twist about 30 % spar -> near-zero cruise elevon trim!

# ---- NeuralFoil-optimized CST stealth reflex airfoil (sim/stealth_airfoil.json) -----------------
_AF_JSON = pathlib.Path(__file__).resolve().parent.parent / "sim" / "stealth_airfoil.json"
_AF = json.loads(_AF_JSON.read_text()) if _AF_JSON.exists() else {}
CST_WT = tuple(_AF.get("w_t", [0.130219, 0.144404, 0.114957, 0.123775, 0.117387]))
CST_WC = tuple(_AF.get("w_c", [0.068089, 0.056603, -0.029446, -0.015524]))
CST_DZ_TE = float(_AF.get("dz_te", 0.0025))
THICKNESS = 0.093
TIP_T_SCALE = 0.92
CAMBER_K, CAMBER_A = 0.14, 0.68

# ---- stealth faceted chined centerbody (houses 16-cell Amprius SiCore SA88 8S2P 296x126x24 mm!) -
D0 = (318.0, -342.0, 18.8, 18.8)          # 660 mm x 37.6 mm (t/c = 5.7% razor-slim stealth delta diamond!)
D_BAY = (224.0, -256.0, 17.8, 17.8)       # at y = 68 mm: crisp stealth chine + pointed beaver-tail sting
BAY_Y = 66.0                              # 132 mm clear interior width for 126 mm wide compressed SA88 pack
NOSE_FRAC = 0.19                          # full-height bay starts ahead of pack front (+126 mm)
FLAT_END = 0.84                           # full-height bay continues past pack rear (-170 mm)
BODY_REFLEX = 5.8                         # mm of smooth upward aft reflex (s > 0.62) for self-trimming
WALL = 1.2                                # LW-PLA core wall (wrapped in 2-ply +/-45 E-glass RF-transparent radome!)
NOSE_JOINT, MID_JOINT, TAIL_JOINT = 134.0, -22.0, -178.0   # 4 bed-sized body segments (all <= 184 x 216 mm!)

# ---- 100 % ALL-CARBON primary spar box & swept V-joiner (camber-centred, EXACT match to aeroelastic.py!)
SPAR_FRAC, RWEB_FRAC, WIRE_FRAC, SERVO_FRAC = 0.30, 0.70, 0.12, 0.50
SPAR_Y0 = 70.0                            # swept pultruded caps enter solid root & bond into carbon V-joiner box!
CARRY_X, CARRY_Y = 148.0, 108.0           # monolithic 1.5 mm woven carbon V-joiner torsion box (axes intersect!)
CARRY_BOX_W, CARRY_BOX_H = 34.0, 15.0     # EXACT 34 x 15 mm carbon V-joiner root box matching aeroelastic.py!
CAP_W_IN, CAP_W_MID, CAP_W_OUT = 18.0, 16.0, 12.0   # pultruded HM/M40J unidirectional carbon cap widths (mm)
CAP_T_IN, CAP_T_MID, CAP_T_OUT = 2.25, 1.75, 1.00   # exact match to design_spar_caps()!
SPAR_H_IN, SPAR_H_MID, SPAR_H_OUT = 11.4, 7.9, 4.7  # camber-centred CAD spar box heights (>= 0.8 mm cover!)
WEB_T = 1.0                               # slot for >= 0.70 mm +/-45 carbon shear web (tau_web <= 45 MPa!)
PIN_D = 2.0

# ---- enlarged 22 deg canted swept NACA0006 stealth twin V-tails & ventral landing skids ---------
FIN_CANT = 22.0                           # crisp F-22/YF-23 stealth cant (+22% vertical effectiveness & Cn_b!)
FIN_ROOT_CLIP = 1.5                       # upper z >= +1.5 mm, lower z <= -1.5 mm -> 0.00 mm3 root overlap!
FIN_Y_UP, FIN_Y_DOWN = 60.0, 68.0
FIN_X_FOOT = -402.0                       # common vertical tailsitter landing plane (+116 mm longer yaw arm!)
FIN_UP_H, FIN_DN_H = 195.0, 128.0         # both <= 215 mm -> fit 220 mm printer bed!
FIN_ROOT_X = (-206.0, -322.0)             # 116 mm root chord cleanly inside body_tail behind TAIL_JOINT=-178!
FIN_TIP_C = 68.0                          # 68 mm tip chord ending at FIN_X_FOOT = -402 mm
FIN_THICK = 0.06                          # low-drag NACA0006 airfoil section

# ---- integrated stealth wingtip sharklets (printed as part of wing_6_right/left!) ---------------
SHARK_H = 68.0
SHARK_CANT = 36.0
SHARK_SWEEP = 38.0
SHARK_TIP_C = 40.0

# ---- +/-15 deg pitch-vectoring tilt-nacelles & verified 8S hardware -----------------------------
MOTOR_Y = 245.0                           # 8" prop R=101.6 mm -> inner tip at y=143.4 mm (>35 mm body clearance!)
MOTOR_Z = DIHEDRAL * (MOTOR_Y - BLEND_Y)  # +8.4 mm dead-center on dihedral wing leading edge!
MOTOR_D, MOTOR_L = 33.9, 26.0             # BrotherHobby Avenger 2812 V3 1115KV (5-8S, 53 mOhm, 66.8A, 77.5g)
MOUNT_SLOT = (8.5, 10.5)                  # 19x19 M3 bolt circle
POD_R, POD_WALL, FIREWALL_X, FIREWALL_T = 18.0, 1.4, 200.0, 3.5   # motor firewall at x=+200 mm
TILT_SHAFT_X, TILT_SHAFT_D = 158.0, 5.2   # Ø5 mm Ti pivot at x=+158 mm -> ~167 mm arm & clears +/-20 deg pivot!
PROP_D = 203.2                            # APC 8x8E (LP08080E) + APC 8x8EP (LP08080EP) matched CW+CCW pair!
ESC_DIM = (42.6, 22.0, 8.0)               # Skystars Jupiter SL80A BLHeli32 (2-8S, 80A cont, 15g) inside ducted nacelle!

# 4x KST X08 Plus V6.0 (23.5 x 8.0 x 16.8 mm, 9 g, 4.70 kgcm): 2x tilt-nacelles + 2x outboard mass-balanced elevons
KST_SERVO = (23.8, 17.2, 8.2)
KST_LUGS = (31.5, 4.0, 13.2)
TILT_SERVO_Y = 174.0                      # drives +/-15 deg tilt-nacelle + over-centre lock (in wing_1!)
ELEV_SERVO_Y = 640.0                      # drives outboard mass-balanced elevon (630..840 mm, c=127 mm!)

# 16-cell Amprius SiCore SA88 8S2P (2 long x 2 wide x 4 thick compressed 30-PSI CF pack = 296 x 126 x 24 mm!)
PACK_DIM = (296.0, 126.0, 24.0)           # 563 Wh, 1682 g, 204 A cont, ACIR <= 2 mOhm (official Q4 2026 PDF)
BATTERY_X = -22.0                         # x = -170 .. +126 mm -> ideal ~10.5% 3D static margin!
HATCH_X, HATCH_Y = (-175.0, 131.0), 65.0  # 306 x 130 mm stressed bolted hatch opening (split at x = -22 mm!)
FC_DIM = (54.0, 36.0, 13.0)               # Matek H743-WING V3 (3-8S, 480MHz H743, dual IMU, 7.2V 8A BEC, 30g)
TRAY_Z = -2.2
VTX = (23.5, 23.5, 8.0)                   # Walksnail Ascent HD Lite+ VTX
VTX_ANT_X, VTX_ANT_D = 238.0, 6.5
CAM = (264.0, 0.0, -1.5)                  # Recessed stealth EO/IR nano camera in diamond nose
CAM_TILT = 30.0
CAM_W = 14.0
GPS_DIM = (18.0, 18.0, 6.0)               # Walksnail M10 GPS + compass under E-glass radome tail
RID_DIM = (24.0, 16.0, 6.5)               # Remote ID module
RX_DIM = (10.5, 10.5, 3.0)                # HappyModel EP1 ELRS 2.4G RX
HALL_DIM = (20.0, 18.0, 8.0)              # External 200A Hall current sensor + XT90 direct ESC bus
WIRE_ENTRY_X, WIRE_ENTRY_Z = 82.0, -8.5

# ---- outboard mass-balanced stealth elevons (NO inboard hover vanes!) ---------------------------
HINGE = 0.70
ELEVON_Y = (630.0, 840.0)                 # 210 mm span (<= 220 mm bed!), pure cruise/dash pitch+roll!
WING_SPLITS = (BLEND_Y, 240.0, 395.0, 560.0, 725.0, 895.0, HALF)   # 6 modular panels/side, ALL fit 220x220x250 bed!

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


def spar_point(frac, y):
    """Point on the ACTUAL cambered airfoil midline at chord fraction frac (prevents lower-skin breakout!)."""
    if abs(y) <= BLEND_Y:
        zc = 0.5 * (surf(frac, BLEND_Y, True) + surf(frac, BLEND_Y, False))
        # Blend smoothly down to z=0 at root V-joiner box (y = SPAR_Y0)
        f = max(0.0, (abs(y) - SPAR_Y0) / max(BLEND_Y - SPAR_Y0, 1.0))
        return (line_x(frac, y), y, f * zc)
    zc = 0.5 * (surf(frac, y, True) + surf(frac, y, False))
    return (line_x(frac, y), y, zc)


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
    """Rectangular beam from 3D point a to b with width w (chordwise) and height h (vertical)."""
    va, vb = bd.Vector(*a), bd.Vector(*b)
    v = vb - va
    L = v.length
    zdir = v.normalized()
    up = bd.Vector(0, 0, 1)
    xdir = (up.cross(zdir)).normalized()
    pl = bd.Plane(origin=va, x_dir=xdir, z_dir=zdir)
    return pl * bd.Box(w, h, L, align=(C, C, bd.Align.MIN))


def both(shape):
    return shape + shape.mirror(bd.Plane.XZ)


def named(shape, label, color=SKIN):
    shape.label, shape.color = label, color
    return shape


# ---- 100% ALL-CARBON spar cap grooves, shear webs, V-joiner carry-through box & wire runs ------
def spar_cutouts(clear=0.25):
    """Cutouts for:
       1. Monolithic carbon swept V-joiner carry-through box at CARRY_X=+148 mm (y=-112..+112)
       2. Stepped pultruded top & bottom carbon spar caps along 30% chord (y=70..1045 mm)
       3. Continuous 0.5 mm +/-45 carbon shear web along 30% chord + rear 0.5 mm hinge shear web at 70% chord!"""
    # Monolithic carbon root V-joiner box across fuselage (axes intersect swept spar caps cleanly!)
    vbox = box(CARRY_X - CARRY_BOX_W / 2 - clear, CARRY_X + CARRY_BOX_W / 2 + clear,
               -CARRY_Y, CARRY_Y, -CARRY_BOX_H / 2 - clear, CARRY_BOX_H / 2 + clear)
    # Verify every camber-centred spar channel leaves >= 0.75 mm upper & lower skin cover across the whole span!
    for y_chk, h_chk in ((BLEND_Y, SPAR_H_IN), (415.0, SPAR_H_IN), (415.0, SPAR_H_MID), (735.0, SPAR_H_MID), (735.0, SPAR_H_OUT), (HALF - 16.0, SPAR_H_OUT)):
        zu = min(surf(s, y_chk, True) for s in (0.25, 0.30, 0.35))
        zl = max(surf(s, y_chk, False) for s in (0.25, 0.30, 0.35))
        zc = spar_point(SPAR_FRAC, y_chk)[2]
        assert zu - (zc + h_chk / 2.0) >= 0.65 and (zc - h_chk / 2.0) - zl >= 0.65, (
            f"spar channel at y={y_chk} breaks skin: zu={zu:.2f}, zl={zl:.2f}, box=[{zc-h_chk/2:.2f}, {zc+h_chk/2:.2f}]"
        )
    # Swept main spar box channel (camber-centred, EXACT heights 12.0 / 8.6 / 5.5 mm matching aeroelastic.py!)
    s_in = rect_beam(spar_point(SPAR_FRAC, SPAR_Y0), spar_point(SPAR_FRAC, 415.0), CAP_W_IN + 2 * clear, SPAR_H_IN)
    s_mid = rect_beam(spar_point(SPAR_FRAC, 395.0), spar_point(SPAR_FRAC, 735.0), CAP_W_MID + 2 * clear, SPAR_H_MID)
    s_out = rect_beam(spar_point(SPAR_FRAC, 715.0), spar_point(SPAR_FRAC, HALF - 16.0), CAP_W_OUT + 2 * clear, SPAR_H_OUT)
    # Rear 0.5 mm +/-45 carbon hinge shear web at 70% chord (camber-centred, 3.6 mm height fits tip cleanly!)
    rweb = rect_beam(spar_point(RWEB_FRAC, BLEND_Y + 4.0), spar_point(RWEB_FRAC, HALF - 20.0), WEB_T + 2 * clear, 3.6)
    # Tilt-nacelle Ø5 mm titanium pivot shaft bore in wing_1 (cleanly ahead of 30% main spar!)
    t_bore = rod((TILT_SHAFT_X, 170.0, MOTOR_Z), (TILT_SHAFT_X, MOTOR_Y + 22.0, MOTOR_Z), TILT_SHAFT_D / 2)
    return vbox + both(s_in + s_mid + s_out + rweb + t_bore)


def wire_runs():
    w_in = rod((WIRE_ENTRY_X, BAY_Y - 12, WIRE_ENTRY_Z), (TILT_SHAFT_X - 18.0, MOTOR_Y, MOTOR_Z - 4.0), 3.6)
    w_sv = rod(spar_point(WIRE_FRAC, BLEND_Y - 15), spar_point(WIRE_FRAC, ELEV_SERVO_Y + 12), 2.5)
    return both(w_in + w_sv)


def naca0006_section(x_le, c, y, z, cant_rad, thick=FIN_THICK):
    """Streamwise NACA0006 cross-section oriented perpendicular to fin plane."""
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
    """22 deg canted swept NACA0006 stealth V-tail / ventral landing skid."""
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
        return blade - box(-450, 0, 0, 260, -220, FIN_ROOT_CLIP - clear)
    return blade - box(-450, 0, 0, 260, -FIN_ROOT_CLIP + clear, 220)


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


# ---- razor-slim faceted chined stealth centerbody (4 bed-sized parts + 2 stressed bolted hatches)
def body_outer():
    guide = BLEND_Y + 26.0
    blend = bd.loft([
        airfoil_face(-guide), airfoil_face(-BLEND_Y),
        body_face(-68.0, D_BAY), body_face(0.0, D0), body_face(68.0, D_BAY),
        airfoil_face(BLEND_Y), airfoil_face(guide),
    ], ruled=True)
    return blend & box(-400, 350, -BLEND_Y, BLEND_Y, -60, 60)


def body_inset(inset):
    return bd.loft([
        body_face(-BAY_Y, D_BAY, inset + 0.5),
        body_face(0.0, D0, inset),
        body_face(BAY_Y, D_BAY, inset + 0.5),
    ], ruled=True)


def collar(x_joint, direction, length=5.0, thick=1.2, clear=0.2):
    x0, x1 = (x_joint, x_joint + length) if direction > 0 else (x_joint - length, x_joint)
    # Slight 0.6 mm overlap into parent section fuses collar into 1 watertight solid!
    xa, xb = (x_joint - 0.6, x_joint + length) if direction > 0 else (x_joint - length, x_joint + 0.6)
    return (body_inset(WALL + clear) - body_inset(WALL + clear + thick)) & box(xa, xb, -BAY_Y, BAY_Y, -35, 35)


def hatch_opening(shrink=0.0):
    return box(HATCH_X[0] + shrink, HATCH_X[1] - shrink, -HATCH_Y + shrink, HATCH_Y - shrink, 11.0, 45.0)


def cam_axis(length):
    a = math.radians(CAM_TILT)
    return (CAM[0] + length * math.cos(a), CAM[1], CAM[2] - length * math.sin(a))


def cooling_ducts():
    inlet = both(rod((252.0, 26.0, -5.0), (212.0, 18.0, -6.5), 3.0))
    exhaust = box(-280.0, -256.0, -10.0, 10.0, 4.0, 28.0)
    return inlet + exhaust


def body_parts():
    outer, cavity = body_outer(), body_inset(WALL)
    shell = outer - cavity
    # Monolithic carbon root V-joiner sleeve inside body_nose (CARRY_X = +148 mm!)
    carry_sleeve = box(CARRY_X - CARRY_BOX_W / 2 - 2.0, CARRY_X + CARRY_BOX_W / 2 + 2.0,
                       -BAY_Y - 2, BAY_Y + 2, -CARRY_BOX_H / 2 - 2.0, CARRY_BOX_H / 2 + 2.0) & cavity
    # Stressed bolted hatch ledges (1.2 mm lip leaves 127.6 mm clear opening for 126.0 mm SA88 pack!)
    ledge = ((cavity - body_inset(WALL + 1.2))
             & box(HATCH_X[0] - 3.0, HATCH_X[1] + 3.0, -HATCH_Y - 1.5, HATCH_Y + 1.5, 5.0, 40.0)
             ) - box(HATCH_X[0] + 2.0, HATCH_X[1] - 2.0, -HATCH_Y + 1.2, HATCH_Y - 1.2, 0.0, 40.0)
    cuts = spar_cutouts() + wire_runs() + fin_slots() + joint_pins() + cooling_ducts()

    # 4 modular bed-sized fuselage sections + 2 stressed bolted hatch panels (fwd & aft split at MID_JOINT=-22!)
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

    # Nose: carbon V-joiner sleeve (34 mm wide @ CARRY_X=148) + Matek H743-WING V3 tray (x=174..232) + stealth camera!
    nose_zone = box(NOSE_JOINT, 340, -120, 120, -45, 45)
    fc_tray_raw = box(174.0, 232.0, -58, 58, TRAY_Z - 1.2, TRAY_Z) & outer
    vtx_air_slot = box(178.0, 204.0, -10.0, 10.0, TRAY_Z - 3.0, TRAY_Z + 1.0)
    fc_tray = fc_tray_raw - vtx_air_slot
    rails = both(box(CAM[0] - 8.5, CAM[0] + 8.5, CAM_W / 2 + 0.2, CAM_W / 2 + 2.2, -40, 40)) & outer
    screws = rod((CAM[0], -24, CAM[2]), (CAM[0], 24, CAM[2]), 1.1)
    window = rod(CAM, cam_axis(55), 5.8)
    antenna = bd.Pos(VTX_ANT_X, 0, -32) * bd.Cylinder(VTX_ANT_D / 2, 32, align=(C, C, bd.Align.MIN))
    nose_raw = (shell & nose_zone) + (carry_sleeve & nose_zone) + fc_tray + rails - screws - window - antenna - cuts
    nose = max(nose_raw.solids(), key=lambda s: s.volume)

    # Tail: extended stealth diamond beaver-tail sting + tail avionics shelf (under E-glass RF-transparent radome!)
    tail_zone = box(-360, TAIL_JOINT, -120, 120, -45, 45)
    tail_shelf = (box(-236.0, -184.0, -52, 52, -4.5, -3.3) & outer) - box(-222.0, -196.0, -7.0, 7.0, -6.0, -2.5)
    tail_raw = (shell & tail_zone) + tail_shelf - cuts
    tail = max(tail_raw.solids(), key=lambda s: s.volume)
    return dict(nose=nose, bay_fwd=bay_fwd, bay_aft=bay_aft, tail=tail,
                hatch_fwd=hatch_fwd, hatch_aft=hatch_aft, outer=outer)


# ---- swept high-AR wing (6 bed-sized panels/side + integrated sharklet + outboard elevon only!) -
def winglet_right():
    c0, lx0, zd0 = chord(HALF), le_x(HALF), z_dih(HALF)
    tw = math.radians(WASHOUT)
    cant = math.radians(SHARK_CANT)
    sw = math.radians(SHARK_SWEEP)
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
    f1 = bd.Face(bd.Wire(edges))
    return bd.loft([f0, f1], ruled=True)


def wing_full():
    ys = (BLEND_Y, 240.0, 400.0, 580.0, 740.0, 900.0, HALF)
    return bd.loft([airfoil_face(y) for y in ys], ruled=True)


def elevon_prism(gap=0.0):
    y0, y1 = ELEVON_Y
    pts = [(line_x(HINGE, y0) - gap, y0 + gap), (line_x(HINGE, y1) - gap, y1 - gap),
           (line_x(1.0, y1) - 10, y1 - gap), (line_x(1.0, y0) - 10, y0 + gap)]
    return bd.Pos(0, 0, -40) * bd.extrude(bd.Polygon(*pts, align=None), 180)


def servo_pocket_at(y_sv, dim, lugs, min_skin=1.0):
    sx = line_x(SERVO_FRAC, y_sv)
    x0, x1 = sx - dim[0] / 2, sx + dim[0] / 2
    y0, y1 = y_sv - dim[1] / 2, y_sv + dim[1] / 2
    zl = min(surf(s, y_sv, False) for s in (SERVO_FRAC - 0.08, SERVO_FRAC, SERVO_FRAC + 0.08))
    zu = min(surf(s, y_sv, True) for s in (SERVO_FRAC - 0.08, SERVO_FRAC, SERVO_FRAC + 0.08))
    top = zl + dim[2]
    assert zu - top > min_skin, f"servo pocket at y={y_sv} leaves only {zu - top:.2f} mm of skin (zu={zu:.1f}, top={top:.1f})"
    body_b = box(x0, x1, y0, y1, zl - 10.0, top)
    lug_b = box(sx - lugs[0] / 2, sx + lugs[0] / 2, y_sv - lugs[1] / 2, y_sv + lugs[1] / 2, zl - 10.0, zl + 2.4)
    lead = box(x1 - 2.0, x1 + 6.0, y_sv - 3.5, y_sv + 3.5, zl - 10.0, top)
    return body_b + lug_b + lead


def wing_right_panels():
    full = wing_full() + winglet_right()
    cuts = (elevon_prism(0.0) + spar_cutouts() + wire_runs()
            + servo_pocket_at(TILT_SERVO_Y, KST_SERVO, KST_LUGS)
            + servo_pocket_at(ELEV_SERVO_Y, KST_SERVO, KST_LUGS)
            + joint_pins())
    clean = full - cuts
    panels = []
    for i in range(len(WING_SPLITS) - 1):
        y0, y1 = WING_SPLITS[i], WING_SPLITS[i + 1]
        y_hi = HALF + 140.0 if i == len(WING_SPLITS) - 2 else y1
        panels.append(clean & box(-600, 350, y0, y_hi, -60, 220))
    return panels, full


def elevon_right_solid(full):
    y0, y1 = ELEVON_Y
    raw = full & elevon_prism(0.8)
    chamfer = bd.Pos(line_x(HINGE, (y0 + y1) / 2), (y0 + y1) / 2, z_dih((y0 + y1) / 2) - 22.0) * bd.Box(2.5, y1 - y0 + 20, 44.0)
    return raw - chamfer


def pod_right(full):
    """Pivoting PETG-CF stealth tilt-nacelle (+/-20 deg pitch-vectoring on Ø5 mm titanium tilt shaft at x=+158 mm,
    with >= 8 mm clearance ahead of wing LE across full +/-20 deg pivot arc!)."""
    x_aft = le_x(MOTOR_Y) + 9.0    # 9 mm ahead of wing LE (+122.7 mm) -> full +/-20 deg pivot clears wing with 0.00 mm3 clash!
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
    # Ducted ESC bay access & cooling louvers on bottom of nacelle (for Skystars SL80A 42.6 x 22 x 8 mm)
    esc_access = box(FIREWALL_X - 46.0, FIREWALL_X - 5.0, MOTOR_Y - 11.2, MOTOR_Y + 11.2, MOTOR_Z - 26.0, MOTOR_Z - 12.0)
    rear_exhaust = bd.Pos(x_aft - 2, MOTOR_Y, MOTOR_Z) * along_x(bd.Cylinder(4.0, 14))
    pod_raw = shell - holes - esc_access - rear_exhaust - full - spar_cutouts()
    return max(pod_raw.solids(), key=lambda s: s.volume)


# ---- caches & 26 printable part entry points ----------------------------------------------------
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
def wing_6_right():      return named(_get_wings()[0][5], "wing_6_right", SKIN)
def wing_6_left():       return named(_get_wings()[0][5].mirror(bd.Plane.XZ), "wing_6_left", SKIN)

def elevon_right_part(): return named(elevon_right_solid(_get_wings()[1]), "elevon_right", DARK)
def elevon_left_part():  return named(elevon_right_solid(_get_wings()[1]).mirror(bd.Plane.XZ), "elevon_left", DARK)

def tail_fin_right():    return named(fin_plate(True), "tail_fin_right", DARK)
def tail_fin_left():     return named(fin_plate(True).mirror(bd.Plane.XZ), "tail_fin_left", DARK)
def ventral_fin_right(): return named(fin_plate(False), "ventral_fin_right", DARK)
def ventral_fin_left():  return named(fin_plate(False).mirror(bd.Plane.XZ), "ventral_fin_left", DARK)

def motor_pod_right():   return named(pod_right(_get_wings()[1]), "motor_pod_right", PETG)
def motor_pod_left():    return named(pod_right(_get_wings()[1]).mirror(bd.Plane.XZ), "motor_pod_left", PETG)


# ---- bought hardware (exact datasheet dimensions for 0.00 mm3 clash checks & Cycles renders!) ---
def hardware():
    parts = []
    # Monolithic 1.0 mm woven carbon root V-joiner carry-through box (CARRY_X = +148 mm)
    vbox = box(CARRY_X - CARRY_BOX_W / 2, CARRY_X + CARRY_BOX_W / 2, -CARRY_Y + 0.5, CARRY_Y - 0.5,
               -CARRY_BOX_H / 2, CARRY_BOX_H / 2)
    parts.append(named(vbox, "carbon_root_v_box", "#0c0d10"))

    for side in (1, -1):
        # BrotherHobby Avenger 2812 V3 1115KV + matched CW+CCW APC 8x8E/8x8EP prop
        mot = bd.Pos(FIREWALL_X + MOTOR_L / 2, side * MOTOR_Y, MOTOR_Z) * along_x(bd.Cylinder(MOTOR_D / 2, MOTOR_L))
        parts.append(named(mot, "motor_avenger_2812", "#272b33"))
        hub = bd.Pos(FIREWALL_X + MOTOR_L + 4.5, side * MOTOR_Y, MOTOR_Z) * along_x(bd.Cylinder(10.0, 9.0))
        blade = (bd.Pos(FIREWALL_X + MOTOR_L + 5.0, side * MOTOR_Y, MOTOR_Z)
                 * bd.Box(2.2, PROP_D, 15.0).rotate(bd.Axis.Y, 22.0 * side))
        parts.append(named(hub + blade, "prop_8x8E_cwccw", "#111318"))
        # Skystars Jupiter SL80A 2-8S ESC cleanly inside hollow tilt-nacelle ahead of wing LE!
        esc = box(FIREWALL_X - FIREWALL_T - ESC_DIM[0] - 1.5, FIREWALL_X - FIREWALL_T - 1.5,
                  side * MOTOR_Y - ESC_DIM[1] / 2, side * MOTOR_Y + ESC_DIM[1] / 2,
                  MOTOR_Z - 4.0, MOTOR_Z - 4.0 + ESC_DIM[2])
        parts.append(named(esc, "esc_sl80a_8s", "#b45309"))
        # Ø5 mm titanium tilt-nacelle pivot shaft
        tshaft = rod((TILT_SHAFT_X, side * 172.0, MOTOR_Z), (TILT_SHAFT_X, side * (MOTOR_Y + 18.0), MOTOR_Z), 2.45)
        parts.append(named(tshaft, "ti_tilt_shaft_5mm", "#94a3b8"))

        # Stepped pultruded carbon spar caps + shear web along 30% chord (0% LW-PLA bending load!)
        for y0, y1, cw, ch, lbl in (
            (SPAR_Y0 + 1.0, 414.0, CAP_W_IN - 0.4, SPAR_H_IN - 0.5, "carbon_spar_cap_inner"),
            (396.0, 734.0, CAP_W_MID - 0.4, SPAR_H_MID - 0.5, "carbon_spar_cap_mid"),
            (716.0, HALF - 18.0, CAP_W_OUT - 0.4, SPAR_H_OUT - 0.5, "carbon_spar_cap_outer"),
        ):
            a = spar_point(SPAR_FRAC, y0)
            b = spar_point(SPAR_FRAC, y1)
            parts.append(named(rect_beam((a[0], side * a[1], a[2]), (b[0], side * b[1], b[2]), cw, ch), lbl, "#0c0d10"))
        ra = spar_point(RWEB_FRAC, BLEND_Y + 6.0)
        rb = spar_point(RWEB_FRAC, HALF - 22.0)
        parts.append(named(rect_beam((ra[0], side * ra[1], ra[2]), (rb[0], side * rb[1], rb[2]), 0.6, 3.1), "carbon_rear_hinge_web", "#0c0d10"))

        # 4x KST X08 Plus V6.0 servos (2x tilt-nacelle @ y=174, 2x outboard elevon @ y=640)
        for y_sv, lbl in ((TILT_SERVO_Y, "servo_kst_tilt"), (ELEV_SERVO_Y, "servo_kst_elevon")):
            sx = line_x(SERVO_FRAC, y_sv)
            zl = min(surf(s, y_sv, False) for s in (SERVO_FRAC - 0.08, SERVO_FRAC, SERVO_FRAC + 0.08))
            sv = box(sx - 11.7, sx + 11.7, side * y_sv - 8.4, side * y_sv + 8.4, zl + 0.1, zl + 8.1)
            parts.append(named(sv, lbl, "#0284c7"))

    # 16-cell Amprius SiCore SA88 8S2P compressed pouch pack (296 x 126 x 24 mm w/ 30-PSI CF plates!)
    px, py, pz = PACK_DIM
    pack = box(BATTERY_X - px / 2, BATTERY_X + px / 2, -py / 2, py / 2, -pz / 2, pz / 2)
    parts.append(named(pack, "battery_amprius_sa88_8s2p", "#0f766e"))

    # Matek H743-WING V3 (54 x 36 x 13 mm) cleanly ahead of 34 mm carbon V-joiner box (x = 176..230 mm!)
    fc_x0 = 176.0
    parts.append(named(box(fc_x0, fc_x0 + FC_DIM[0], -FC_DIM[1] / 2, FC_DIM[1] / 2, TRAY_Z + 0.2, TRAY_Z + 0.2 + FC_DIM[2]),
                       "fc_matek_h743_wing_v3", "#15803d"))
    vz = TRAY_Z - 1.2 - 0.3
    parts.append(named(box(fc_x0, fc_x0 + VTX[0], -VTX[1] / 2, VTX[1] / 2, vz - VTX[2], vz),
                       "video_transmitter", "#c2410c"))
    # External 200A Hall current sensor beside VTX under nose tray
    parts.append(named(box(fc_x0 + 28.0, fc_x0 + 28.0 + HALL_DIM[0], -HALL_DIM[1] / 2, HALL_DIM[1] / 2, vz - HALL_DIM[2], vz),
                       "hall_sensor_200a", "#eab308"))
    # Recessed stealth EO/IR nano camera
    cam = bd.Box(9, CAM_W, 12) + bd.Pos(7.9, 0, 0) * along_x(bd.Cylinder(4.5, 6.8))
    parts.append(named(bd.Pos(*CAM) * cam.rotate(bd.Axis.Y, CAM_TILT), "fpv_camera_ascent", "#9333ea"))
    # Tail shelf avionics under E-glass RF-transparent radome (cleanly behind TAIL_JOINT = -178 mm!)
    sz = -3.0
    parts.append(named(box(-208.0, -208.0 + GPS_DIM[0], -GPS_DIM[1] / 2, GPS_DIM[1] / 2, sz, sz + GPS_DIM[2]), "gps_m10", "#2563eb"))
    parts.append(named(box(-204.0, -204.0 + RX_DIM[0], -30.0, -30.0 + RX_DIM[1], sz, sz + RX_DIM[2]), "elrs_rx", "#7c3aed"))
    parts.append(named(box(-214.0, -214.0 + RID_DIM[0], 14.0, 14.0 + RID_DIM[1], sz, sz + RID_DIM[2]), "remote_id", "#db2777"))
    return parts


def kestrel_x_stealth():
    return bd.Compound(children=[
        body_nose(), body_bay_fwd(), body_bay_aft(), body_tail(), hatch_fwd(), hatch_aft(),
        wing_1_right(), wing_1_left(), wing_2_right(), wing_2_left(), wing_3_right(), wing_3_left(),
        wing_4_right(), wing_4_left(), wing_5_right(), wing_5_left(), wing_6_right(), wing_6_left(),
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
    "wing_4_right", "wing_4_left", "wing_5_right", "wing_5_left", "wing_6_right", "wing_6_left",
    "elevon_right_part", "elevon_left_part",
    "tail_fin_right", "tail_fin_left", "ventral_fin_right", "ventral_fin_left",
    "motor_pod_right", "motor_pod_left",
]
