"""Kestrel-X Stealth (v6), printable parts: high-aspect-ratio (1800 mm, AR 11.7) cranked-kite stealth
blended-wing-body tailsitter with NeuralFoil CST reflex laminar airfoil, 3-stage telescoping dual carbon
spar torque box, chined stealth centerbody housing a flat 3x6 Molicel P60B 6S3P (388.8 Wh) pack,
blended conformal nacelles, swept NACA0006 canted V-tails + ventral landing skids, wingtip sharklets,
100% propwash-submerged hover vanes, and outboard high-speed stealth elevons.

Frame: +X nose, Y span, +Z up in forward flight; parts are modelled where they sit in the aircraft.
Assumed printer: 220 x 220 x 250 mm (EVERY single printed part fits inside 220 x 220 x 250 mm!).
Materials: high-temp foaming LW-PLA-HT for the airframe (reinforced D-box/ribs), PETG-CF for motor pods.

Structure (22+ g bending limit at 2.35 kg):
  - Continuous 12x9 mm transverse carbon carry-through spar (270 mm, y = -135..+135 mm at x = -140 mm)
    running unbroken through body_tail behind the battery bay and 29 mm into both inner wings.
  - 3-stage telescoping swept main carbon spars along 30 % chord:
      * Inner: 12x9 mm (y = 74..330 mm)
      * Mid:    8x6 mm (y = 260..600 mm, telescopes 70 mm inside 12x9 mm)
      * Outer:  6x4 mm (y = 530..885 mm, telescopes 70 mm inside 8x6 mm)
  - Full rear anti-twist carbon torsion spars: 6x4 mm along 60 % chord (y = 74..520 mm).
"""
import math

import build123d as bd
from cad import model

# ---- planform (converged 3D AeroSandbox + NeuralFoil + VLM stealth optimum) --------------------
HALF = 900.0                      # 1800 mm span (AR ~ 12.9 with sharklets, L/D ~ 18.5-19.5)
BLEND_Y = 106.0                   # chined body width = 212 mm (<= 220 mm printer bed!)
ROOT_LE = 174.0                   # wing LE x at blend_y -> seamless unbroken B-21/RQ-170 stealth leading edge!
ROOT_CHORD = 174.0                # wing chord at blend_y (t_30 = 16.1 mm -> clears 12.3 mm main spar!)
TIP_CHORD = 76.0                  # wing chord at tip y = 900 mm (t_30 = 7.1 mm -> clears 6.3 mm tip spar!)
SWEEP_DEG = 29.5                  # stealth leading-edge sweep angle (deg)
SWEEP = math.tan(math.radians(SWEEP_DEG))
TIP_LE = ROOT_LE - SWEEP * (HALF - BLEND_Y)
DIHEDRAL_DEG = 3.5                # +3.5 deg stealth wing dihedral -> strong negative Cl_b roll stability!
DIHEDRAL = math.tan(math.radians(DIHEDRAL_DEG))
WASHOUT = 5.0                     # deg of nose-down tip twist about the 30 % spar -> near-zero cruise trim!

# NeuralFoil-optimized CST stealth reflex laminar airfoil (sim/stealth_airfoil.json, xlarge conf 0.939):
CST_WT = (0.13085, 0.11675, 0.11216, 0.14108, 0.02796)
CST_WC = (0.20206, 0.03113, 0.07400, 0.03095, -0.04693)
CST_DZ_TE = 0.0025
TIP_T_SCALE = 0.92                # sleek 8.5 % tip section (still clears 6.3 mm tip spar!)

# Control surfaces:
# 1. Inboard propwash hover/transition vanes (100% submerged in 8" propwash, y = 130..318 mm)
# 2. Outboard high-arm stealth cruise/dash elevons (y = 536..744 mm, single 208 mm printable piece)
HINGE = 0.70
HOVER_VANE_Y = (130.0, 318.0)     # 188 mm span, 100% inside 8" prop slipstream (123..327 mm)!
ELEVON_Y = (536.0, 744.0)         # 208 mm span, far aft of CG -> CL_max_trim ~ 0.82, ~0 deg cruise trim!
WING_SPLITS = (225.0, 365.0, 530.0, 710.0)   # 5 panels/side -> EVERY panel fits 220x220x250 mm bed!

# ---- stealth chined centerbody (houses 3x6 flat Molicel P60B 6S3P 216x134x24 mm pack!) ---------
D0 = (264.0, -268.0, 17.8, 17.8)          # 532 mm razor-sharp stealth diamond nose -> pointed beaver-tail sting!
D_BAY = (204.0, -200.0, 16.8, 16.8)       # at y = 72 mm: unbroken 40 deg stealth chine -> 29.5 deg wing LE!
BAY_Y = 70.0                              # interior flat side wall of hollow bay (140 mm clear for 134 mm pack)
NOSE_FRAC = 0.22                          # razor stealth nose shoulder clearing FC + VTX + recessed camera
BODY_REFLEX = 6.5                         # mm of smooth upward aft reflex (s > 0.60) for self-trimming
WALL = 1.2                                # reinforced 1.2 mm stealth shell wall for 300+ km/h rigidity
NOSE_JOINT, TAIL_JOINT = 96.0, -138.0     # centre body is 234 mm + 10 mm collars = 244 mm (<= 250 mm Z bed!)

# ---- structure and fits (3-stage telescoping dual carbon spar torque box) ----------------------
SPAR_FRAC, STUB_FRAC, WIRE_FRAC, SERVO_FRAC = 0.30, 0.60, 0.12, 0.48
SPAR_Y0 = 74.0                            # swept main spar anchored 32 mm into solid blended root
SPAR1_Y = (SPAR_Y0, 330.0)                # 12x9 mm inner main spar (bore 12.3 mm)
SPAR2_Y = (260.0, 600.0)                  #  8x6 mm mid main spar (telescopes 70 mm inside 12x9 mm, bore 8.3 mm)
SPAR3_Y = (530.0, HALF - 15.0)            #  6x4 mm outer tip spar (telescopes 70 mm inside 8x6 mm, bore 6.3 mm)
STUB_Y = (SPAR_Y0, 520.0)                 #  6x4 mm rear anti-twist torsion spar (bore 6.3 mm)
CARRY_X, CARRY_Y = 104.0, 90.0            # fully-enclosed 12x9 mm carry-through spar ahead of bay, bridging wing roots!
TUBE_BORE_12, TUBE_BORE_8, TUBE_BORE_6 = 12.3, 8.3, 6.3
PIN_D = 2.0                               # 1.75 mm filament / carbon alignment pins

# ---- swept canted stealth NACA0006 twin V-tails & ventral landing skids ------------------------
FIN_CANT = 32.0                           # outward cant (deg) — classic stealth twin-tail V angle!
FIN_ROOT_CLIP = 1.0                       # upper fin stays z >= +1 mm, lower fin stays z <= -1 mm -> 0.0 mm3 overlap!
FIN_Y_UP, FIN_Y_DOWN = 64.0, 74.0
FIN_X_FOOT = -286.0                       # common vertical tailsitter landing plane for all 4 feet!
FIN_UP_H, FIN_DN_H = 144.0, 102.0         # taller upper stealth V-tails (+negative Cl_b & strong Cn_b) + ventral skids
FIN_ROOT_X = (-140.0, -216.0)             # cleanly mounted along the tapered stealth diamond tail cheeks!
FIN_TIP_C = 54.0                          # tip chord (54 mm, ending at x = FIN_X_FOOT = -286 mm)
FIN_THICK = 0.06                          # NACA0006 airfoil cross-section (low drag, sharp stealth edges!)

# ---- integrated stealth wingtip sharklets (printed as part of wing_5_right/left!) --------------
SHARK_H = 66.0                            # winglet height along cant axis (mm)
SHARK_CANT = 36.0                         # outward cant from vertical (deg)
SHARK_SWEEP = 42.0                        # sharp stealth aft sweep (deg)
SHARK_TIP_C = 38.0

# ---- components (real datasheet-verified hardware dimensions) ----------------------------------
MOTOR_Y = 225.0                           # 8" prop R=101.6 mm -> inner tip at y=123.4 mm (>20 mm body clearance!)
MOTOR_Z = DIHEDRAL * (MOTOR_Y - BLEND_Y)  # +7.3 mm -> dead-center on dihedral wing leading edge!
MOTOR_D, MOTOR_L = 33.4, 25.0             # T-Motor F100 2810 1350KV (Ø33.4 mm bell, 19x19 M3 mount, M5 shaft)
MOUNT_SLOT = (8.5, 10.5)                  # 19x19 M3 mounting bolt circle (r = 9.5 mm)
POD_R, POD_WALL, FIREWALL_X, FIREWALL_T = 17.5, 1.3, 152.0, 3.5
PROP_D = 203.2                            # APC 8x9 (8.0 inch = 203.2 mm)
ESC_DIM = (35.0, 18.0, 8.0)               # Sequre SQESC 2670 70A AM32 (35 x 18 x 8 mm, 9 g) inside nacelle!

# Two servo pairs: KST X08 Plus V6.0 (23.5 x 8.0 x 16.8 mm, 9 g, 4.7 kgcm) for outboard stealth elevons
#                  + Emax ES9051 (20.2 x 8.7 x 23.2 mm, 4.3 g) for inboard hover vanes!
KST_SERVO = (23.8, 17.2, 8.2)             # X chordwise, Y spanwise, Z thickness (only 8.2 mm deep!)
KST_LUGS = (31.5, 4.0, 13.2)
ELEV_SERVO_Y = 548.0                      # drives outboard stealth elevon (536..744 mm); c=119 mm -> >1.7 mm skin!
HOV_SERVO = (20.4, 23.4, 8.8)             # Emax ES9051 pocket for inboard hover vane (130..318 mm)
HOV_LUGS = (29.0, 10.5, 15.5)
HOV_SERVO_Y = 175.0                       # sits in wing_1 (106..225 mm) inboard of motor pod!

CELL_D, CELL_L = 21.55, 70.2              # real 21700 cell
PACK_DIM = (216.0, 134.0, 24.0)           # 3x6 flat Molicel P60B 6S3P pack (388.8 Wh, 1418 g, 180 A!)
BATTERY_X = -18.0                         # nominal position -> ideal 10-12% 3D static margin, 0.0 mm3 clash!
HATCH_X, HATCH_Y = (-135.0, 93.0), 69.0   # 228 x 138 mm opening -> 216 x 134 mm flat pack drops straight in!
FC_DIM = (37.0, 26.0, 14.0)               # SpeedyBee F405 Wing Mini (2-6S, 80A sensor, 7.2V servo BEC)
TRAY_Z = -2.5
VTX = (23.5, 23.5, 8.0)                   # Walksnail Ascent HD Lite+ 500 mW VTX
VTX_ANT_X, VTX_ANT_D = 180.0, 6.5
CAM = (208.0, 0.0, -1.5)                  # Walksnail Ascent Lite+ nano camera recessed in stealth chin
CAM_TILT = 30.0
CAM_W = 14.0
GPS_DIM = (18.0, 18.0, 6.0)               # Walksnail M10 GPS + compass in tail shelf
RID_DIM = (24.0, 16.0, 6.5)               # Remote ID module in tail
RX_DIM = (10.5, 10.5, 3.0)                # HappyModel EP1 ELRS 2.4G nano RX
WIRE_ENTRY_X, WIRE_ENTRY_Z = 54.0, -8.5

C = bd.Align.CENTER


# ---- geometry helpers --------------------------------------------------------------------------
def bernstein(n, i, s):
    return math.comb(n, i) * (s ** i) * ((1.0 - s) ** (n - i))


def le_x(y):
    return ROOT_LE - SWEEP * (abs(y) - BLEND_Y)


def chord(y):
    t = max(0.0, (abs(y) - BLEND_Y) / (HALF - BLEND_Y))
    return ROOT_CHORD + (TIP_CHORD - ROOT_CHORD) * t


def z_dih(y):
    return DIHEDRAL * max(0.0, abs(y) - BLEND_Y)


def line_x(frac, y):
    return le_x(y) - frac * chord(y)


def surf(y, s):
    c = chord(y)
    span_f = min(1.0, max(0.0, (abs(y) - BLEND_Y) / (HALF - BLEND_Y)))
    t_scale = 1.0 + (TIP_T_SCALE - 1.0) * span_f
    c_cls = math.sqrt(max(s, 0.0)) * (1.0 - s)
    nt, nc = len(CST_WT) - 1, len(CST_WC) - 1
    yt = t_scale * (c_cls * sum(w * bernstein(nt, i, s) for i, w in enumerate(CST_WT)) + s * (CST_DZ_TE / 2.0))
    yc = s * (1.0 - s) * sum(w * bernstein(nc, i, s) for i, w in enumerate(CST_WC))
    twist = WASHOUT * span_f
    dz = (s - SPAR_FRAC) * c * math.tan(math.radians(twist))
    zd = z_dih(y)
    return zd + c * (yc - yt) + dz, zd + c * (yc + yt) + dz


def profile_face(upper, lower):
    top = bd.Spline(*reversed(upper))
    bottom = bd.Spline(*lower)
    return bd.Face(bd.Wire([top, bottom, bd.Line(lower[-1], upper[-1])]))


def stations(n=20):
    return [0.5 * (1.0 - math.cos(math.pi * i / n)) for i in range(n + 1)]


def airfoil_face(y):
    upper, lower = [], []
    for s in stations():
        zl, zu = surf(y, s)
        x = le_x(y) - chord(y) * s
        upper.append((x, y, zu))
        lower.append((x, y, zl))
    return profile_face(upper, lower)


def body_f(s):
    """Stealth chined body thickness: sharp tapered diamond nose, full-height flat bay (s<=0.80), sleek reflex tail."""
    if s < NOSE_FRAC:
        t = (NOSE_FRAC - s) / NOSE_FRAC
        return (1.0 - t * t) ** 0.58
    if s < 0.80:
        return 1.0
    return 0.02 + 0.98 * 0.5 * (1.0 + math.cos(math.pi * (s - 0.80) / 0.20))


def body_camber_z(s):
    return BODY_REFLEX * max(0.0, (s - 0.60) / 0.40) ** 2


def body_face(y, dims, inset=0.0):
    x_le, x_te, top, bot = dims
    x_le, x_te, top, bot = x_le - 2.5 * inset, x_te + 6.0 * inset, top - inset, bot - inset
    upper, lower = [], []
    for s in stations():
        x, f, zc = x_le - (x_le - x_te) * s, body_f(s), body_camber_z(s)
        upper.append((x, y, zc + top * f))
        lower.append((x, y, zc - bot * f))
    return profile_face(upper, lower)


def box(x0, x1, y0, y1, z0, z1):
    return bd.Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * bd.Box(x1 - x0, y1 - y0, z1 - z0)


def rod(p0, p1, r):
    d = bd.Vector(*p1) - bd.Vector(*p0)
    return bd.Plane(origin=p0, z_dir=d.normalized()) * bd.Cylinder(r, d.length, align=(C, C, bd.Align.MIN))


def along_x(shape):
    return shape.rotate(bd.Axis.Y, 90)


def xz_face(y, pts):
    return bd.Plane(origin=(0, y, 0), x_dir=(1, 0, 0), z_dir=(0, -1, 0)) * bd.Polygon(*pts, align=None)


def spar_point(frac, y):
    if abs(frac - SPAR_FRAC) < 1e-6:
        # Linear exact dihedral line (continues straight into blended root for y < BLEND_Y!)
        z = DIHEDRAL * (abs(y) - BLEND_Y)
    else:
        zl, zu = surf(max(abs(y), BLEND_Y), frac)
        z0 = 0.5 * (zl + zu)
        if abs(y) < BLEND_Y:
            zl2, zu2 = surf(BLEND_Y + 100.0, frac)
            dz_dy = (0.5 * (zl2 + zu2) - z0) / 100.0
            z = z0 + dz_dy * (abs(y) - BLEND_Y)
        else:
            z = z0
    return (line_x(frac, y), y, z)


def frac_line(frac, y0, y1, r):
    return rod(spar_point(frac, y0), spar_point(frac, y1), r)


def both(shape):
    return shape + shape.mirror(bd.Plane.XZ)


# ---- shared cuts -------------------------------------------------------------------------------
def tube_bores():
    s1 = frac_line(SPAR_FRAC, *SPAR1_Y, TUBE_BORE_12 / 2)
    s2 = frac_line(SPAR_FRAC, *SPAR2_Y, TUBE_BORE_8 / 2)
    s3 = frac_line(SPAR_FRAC, *SPAR3_Y, TUBE_BORE_6 / 2)
    rear = frac_line(STUB_FRAC, *STUB_Y, TUBE_BORE_6 / 2)
    carry = rod((CARRY_X, -CARRY_Y, 0.0), (CARRY_X, CARRY_Y, 0.0), TUBE_BORE_12 / 2)
    return both(s1 + s2 + s3 + rear) + carry


def wire_runs():
    """6.8 mm ESC power/signal conduit + servo wire channels from bay into wing."""
    end = (line_x(WIRE_FRAC, MOTOR_Y), MOTOR_Y, MOTOR_Z)
    knee = (WIRE_ENTRY_X + 0.25 * (end[0] - WIRE_ENTRY_X), 84.0, -2.0)
    start = (WIRE_ENTRY_X, BAY_Y - 2.0, WIRE_ENTRY_Z)
    past = tuple(k + 0.15 * (k - a) for k, a in zip(knee, start))
    esc = rod(start, past, 3.4) + rod(knee, end, 3.4)
    # Servo wire channel from bay wall along 48 % chord to outboard elevon servo
    p_s0 = (line_x(SERVO_FRAC, BAY_Y - 2.0), BAY_Y - 2.0, -2.5)
    p_s1 = spar_point(SERVO_FRAC, BLEND_Y + 5.0)
    p_s2 = spar_point(SERVO_FRAC, ELEV_SERVO_Y)
    srv = rod(p_s0, p_s1, 2.4) + rod(p_s1, p_s2, 2.4)
    return both(esc + srv)


def naca0006_section(x_le, chord_len, y_pos, z_pos, cant_deg, thickness=FIN_THICK, clear=0.0):
    """Airfoil face in local fin plane (tilted by cant_deg about X axis), closed at LE and TE."""
    upper, lower = [], []
    c_eff = chord_len + 2.0 * clear
    x0 = x_le + clear
    for s in stations(18):
        yt = 5 * thickness * (0.2969 * math.sqrt(s) - 0.1260 * s - 0.3516 * s**2 + 0.2843 * s**3 - 0.1015 * s**4)
        half_t = (chord_len * yt + (0.55 * s + clear) * math.sqrt(s)) if s > 0.0 else 0.0
        x = x0 - c_eff * s
        upper.append((x, half_t))
        lower.append((x, -half_t))
    rad = math.radians(cant_deg)
    cy, cz = math.sin(rad), math.cos(rad)
    ny, nz = math.cos(rad), -math.sin(rad)
    y_c = y_pos + z_pos * cy
    z_c = z_pos * cz
    u3 = [(x, y_c + t * ny, z_c + t * nz) for x, t in upper]
    l3 = [(x, y_c + t * ny, z_c + t * nz) for x, t in lower]
    return profile_face(u3, l3)


def fin_plate(upper, clear=0.0):
    """Sleek swept canted stealth NACA0006 V-tail (upper) or ventral landing skid (lower)."""
    h = FIN_UP_H if upper else FIN_DN_H
    y_root = FIN_Y_UP if upper else FIN_Y_DOWN
    # Upper fin goes in +Z tilted toward +Y (+FIN_CANT); lower fin goes in -Z tilted toward +Y
    x_le_r, x_te_r = FIN_ROOT_X
    c_r = x_le_r - x_te_r
    x_te_t = FIN_X_FOOT
    x_le_t = x_te_t + FIN_TIP_C
    sign = 1.0 if upper else -1.0
    cant = FIN_CANT if upper else (180.0 - FIN_CANT)

    # Root tab sinks slightly inside the body (s_span = -8 mm) and clips cleanly at |z| = FIN_ROOT_CLIP
    f_in = naca0006_section(x_le_r + 3.0, c_r + 3.0, y_root, -8.0, cant, FIN_THICK, clear)
    f_mid = naca0006_section(x_le_r, c_r, y_root, 14.0, cant, FIN_THICK, clear)
    f_tip = naca0006_section(x_le_t, FIN_TIP_C, y_root, h, cant, FIN_THICK, clear)
    blade = bd.loft([f_in, f_mid, f_tip], ruled=True)
    # Strictly separate upper fin (z >= +FIN_ROOT_CLIP) and ventral fin (z <= -FIN_ROOT_CLIP) -> 0.0 mm3 root overlap!
    if upper:
        return blade - box(-350, 0, 0, 260, -180, FIN_ROOT_CLIP - clear)
    return blade - box(-350, 0, 0, 260, -FIN_ROOT_CLIP + clear, 180)


def fin_slots():
    return both(fin_plate(True, 0.18) + fin_plate(False, 0.18))


def cam_axis(length):
    a = math.radians(CAM_TILT)
    return (CAM[0] + length * math.cos(a), CAM[1], CAM[2] - length * math.sin(a))


# ---- stealth chined centerbody -----------------------------------------------------------------
def body_outer():
    """Chined stealth faceted delta centerbody (ruled loft -> razor-sharp F-22/B-21 diamond nose & beaver-tail!)."""
    guide = BLEND_Y + 26.0
    blend = bd.loft([
        airfoil_face(-guide), airfoil_face(-BLEND_Y),
        body_face(-72.0, D_BAY), body_face(0.0, D0), body_face(72.0, D_BAY),
        airfoil_face(BLEND_Y), airfoil_face(guide),
    ], ruled=True)
    return blend & box(-350, 300, -BLEND_Y, BLEND_Y, -60, 60)


def body_inset(inset):
    return bd.loft([
        body_face(-BAY_Y, D_BAY, inset + 0.5),
        body_face(0.0, D0, inset),
        body_face(BAY_Y, D_BAY, inset + 0.5),
    ], ruled=True)


def collar(joint, direction):
    bore = body_inset(WALL + 1.3)
    a, b = sorted((joint - 4.5 * direction, joint))
    fixed = (body_inset(WALL - 0.2) - bore) & box(a, b, -95, 95, -40, 40)
    a, b = sorted((joint, joint + 5.0 * direction))
    slip = (body_inset(WALL + 0.2) - bore) & box(a, b, -95, 95, -40, 40)
    return fixed + slip


def hatch_opening(clear=0.0):
    return box(HATCH_X[0] + clear, HATCH_X[1] - clear, -HATCH_Y + clear, HATCH_Y - clear, 5.0, 40.0)


def joint_pins():
    out = None
    for x in (NOSE_JOINT, TAIL_JOINT):
        for y in (-86.0, 86.0):
            h = rod((x - 8, y, 2.0), (x + 8, y, 2.0), PIN_D / 2)
            out = h if out is None else out + h
    return out


def cooling_ducts():
    """Stealth recessed cheek cooling ducts for VTX & FC + tail dorsal exhaust slit."""
    inlet = both(rod((204.0, 26.0, -5.0), (168.0, 18.0, -6.5), 3.0))
    exhaust = box(-220.0, -196.0, -10.0, 10.0, 4.0, 28.0)
    return inlet + exhaust


def body_parts():
    outer, cavity = body_outer(), body_inset(WALL)
    shell = outer - cavity
    # Carry-through 12 mm carbon spar sleeve cleanly ahead of battery bay (x = +115 mm inside body_nose!)
    carry_sleeve = rod((CARRY_X, -BAY_Y - 2, 0.0), (CARRY_X, BAY_Y + 2, 0.0), 8.2) & cavity
    # Hatch ledges (1.2 mm lip leaves 135.6 mm clear opening for the 134.0 mm wide 3x6 flat P60B pack!)
    ledge = ((cavity - body_inset(WALL + 1.2))
             & box(HATCH_X[0] - 3.0, HATCH_X[1] + 3.0, -HATCH_Y - 1.5, HATCH_Y + 1.5, 5.0, 40.0)
             ) - box(HATCH_X[0] + 2.0, HATCH_X[1] - 2.0, -HATCH_Y + 1.2, HATCH_Y - 1.2, 0.0, 40.0)
    latch_boss = box(HATCH_X[0] - 6.0, HATCH_X[0] + 0.5, -10.0, 10.0, 8.0, 22.0) & cavity
    latch_hole = bd.Pos(HATCH_X[0] - 2.8, 0, 5.0) * bd.Cylinder(1.1, 22.0, align=(C, C, bd.Align.MIN))
    cuts = tube_bores() + wire_runs() + fin_slots() + joint_pins() + cooling_ducts()

    centre_zone = box(TAIL_JOINT, NOSE_JOINT, -120, 120, -45, 45)
    centre_shell = shell & centre_zone

    # Flush stealth battery hatch with integrated front tongue & rear M2 latch
    hatch_skin = centre_shell & hatch_opening(0.3)
    tongue_step = (body_inset(WALL - 0.1) - body_inset(WALL + 2.4)) & box(HATCH_X[1] - 4.0, HATCH_X[1] - 0.5, -12.0, 12.0, 8.0, 22.0)
    front_tongue = (body_inset(WALL + 1.35) - body_inset(WALL + 2.4)) & box(HATCH_X[1] - 4.0, HATCH_X[1] + 2.5, -12.0, 12.0, 8.0, 22.0)
    tongue_clearance = box(HATCH_X[1] - 1.0, HATCH_X[1] + 3.2, -12.8, 12.8, 5.0, 22.0) & body_inset(WALL + 1.05)
    hatch = hatch_skin + tongue_step + front_tongue - latch_hole

    # Put 5 mm alignment collars on body_centre (total length 234 + 10 = 244 mm <= 250 mm Z bed!)
    centre = (centre_shell + ((ledge - tongue_clearance) & centre_zone) + (latch_boss & centre_zone)
              + collar(NOSE_JOINT, 1) + collar(TAIL_JOINT, -1) - hatch_opening() - latch_hole - cuts)

    # Nose: enclosed 12 mm carry-through spar sleeve (x=+115) + FC/VTX tray (x=126..170) + recessed stealth camera!
    nose_zone = box(NOSE_JOINT, 270, -120, 120, -45, 45)
    fc_tray_raw = box(126.0, 170.0, -55, 55, TRAY_Z - 1.2, TRAY_Z) & outer
    vtx_air_slot = box(130.0, 152.0, -10.0, 10.0, TRAY_Z - 3.0, TRAY_Z + 1.0)
    fc_tray = fc_tray_raw - vtx_air_slot
    rails = both(box(CAM[0] - 8.5, CAM[0] + 8.5, CAM_W / 2 + 0.2, CAM_W / 2 + 2.2, -40, 40)) & outer
    screws = rod((CAM[0], -24, CAM[2]), (CAM[0], 24, CAM[2]), 1.1)
    window = rod(CAM, cam_axis(55), 5.8)
    antenna = bd.Pos(VTX_ANT_X, 0, -32) * bd.Cylinder(VTX_ANT_D / 2, 32, align=(C, C, bd.Align.MIN))
    nose_raw = (shell & nose_zone) + (carry_sleeve & nose_zone) + fc_tray + rails - screws - window - antenna - cuts
    nose = max(nose_raw.solids(), key=lambda s: s.volume)

    # Tail: pointed stealth diamond beaver-tail sting + tail avionics shelf (GPS, ELRS RX, Remote ID)
    tail_zone = box(-275, TAIL_JOINT, -120, 120, -45, 45)
    tail_shelf = (box(-190.0, -144.0, -48, 48, -4.5, -3.3) & outer) - box(-180.0, -152.0, -7.0, 7.0, -6.0, -2.5)
    tail_raw = (shell & tail_zone) + tail_shelf - cuts
    tail = max(tail_raw.solids(), key=lambda s: s.volume)
    return dict(nose=nose, centre=centre, tail=tail, hatch=hatch, outer=outer)


# ---- high-AR stealth swept wing + integrated wingtip sharklet ----------------------------------
def wing_full():
    # Multi-station loft captures exact CST thickness taper + 3.5 deg dihedral + 4.4 deg washout!
    ys = [BLEND_Y - 12.0, BLEND_Y, 300.0, 530.0, 710.0, HALF]
    return bd.loft([airfoil_face(y) for y in ys], ruled=True)


def winglet_right():
    """Outward-canted swept stealth NACA0006 wingtip sharklet blended off y = HALF."""
    x_le_r = le_x(HALF)
    c_r = chord(HALF)
    z_r = z_dih(HALF)
    rad_c = math.radians(SHARK_CANT)
    rad_s = math.radians(SHARK_SWEEP)
    # Build base section matching wingtip & tip section swept back 42 deg, canted 36 deg outward!
    def wsec(dist, x_le, ch):
        cy, cz = math.sin(rad_c), math.cos(rad_c)
        ny, nz = math.cos(rad_c), -math.sin(rad_c)
        yc = HALF + dist * cy
        zc = z_r + dist * cz
        up, lo = [], []
        for s in stations(16):
            yt = 5 * 0.065 * (0.2969 * math.sqrt(s) - 0.1260 * s - 0.3516 * s**2 + 0.2843 * s**3 - 0.1015 * s**4)
            ht = (ch * yt + 0.50 * s * math.sqrt(s)) if s > 0.0 else 0.0
            x = x_le - ch * s
            up.append((x, yc + ht * ny, zc + ht * nz))
            lo.append((x, yc - ht * ny, zc - ht * nz))
        return profile_face(up, lo)

    f0 = airfoil_face(HALF)
    f1 = wsec(8.0, x_le_r - 8.0 * math.tan(rad_s), c_r * 0.90)
    f2 = wsec(SHARK_H, x_le_r - SHARK_H * math.tan(rad_s), SHARK_TIP_C)
    return bd.loft([f0, f1, f2], ruled=True)


def control_prism(y_pair, inset):
    y0, y1 = y_pair[0] + inset, y_pair[1] - inset
    back = 1.0 if inset else 0.0
    pts = ((line_x(HINGE, y0) - back, y0), (line_x(HINGE, y1) - back, y1), (-650, y1), (-650, y0))
    return bd.extrude(bd.Polygon(*pts, align=None), 180, both=True)


def servo_pocket_at(y_pos, dims, lugs):
    xc = line_x(SERVO_FRAC, y_pos)
    zl, zu = surf(y_pos, SERVO_FRAC)
    top = zl + dims[2]
    assert zu - top >= 1.0, f"servo pocket at y={y_pos} leaves only {zu - top:.2f} mm of skin (zu={zu:.1f}, top={top:.1f})"
    y0 = y_pos - dims[1] / 2
    body = box(xc - dims[0] / 2, xc + dims[0] / 2, y0, y0 + dims[1], zl - 30, top)
    tabs = box(xc - lugs[0] / 2, xc + lugs[0] / 2, y0 + lugs[1], y0 + lugs[2], zl - 30, top)
    return body + tabs


def pin_holes_at(y, depth=8.0):
    out = None
    for frac in (0.16, 0.78):
        p = spar_point(frac, y)
        h = rod((p[0], y - depth, p[2]), (p[0], y + depth, p[2]), PIN_D / 2)
        out = h if out is None else out + h
    return out


def wing_right_panels():
    full = wing_full()
    riser = bd.Pos(line_x(WIRE_FRAC, MOTOR_Y) + 2, MOTOR_Y - 2, MOTOR_Z - 4.0) * bd.Cylinder(3.6, 26, align=(C, C, bd.Align.MIN))
    pockets = servo_pocket_at(HOV_SERVO_Y, HOV_SERVO, HOV_LUGS) + servo_pocket_at(ELEV_SERVO_Y, KST_SERVO, KST_LUGS)
    pins = None
    for sy in WING_SPLITS:
        ph = pin_holes_at(sy)
        pins = ph if pins is None else pins + ph
    solid = (full
             - control_prism(HOVER_VANE_Y, 0)
             - control_prism(ELEVON_Y, 0)
             - tube_bores() - wire_runs() - riser - pockets - pins)
    edges = (BLEND_Y,) + WING_SPLITS + (HALF + 2.0,)
    panels = []
    for i in range(len(edges) - 1):
        p = solid & box(-650, 260, edges[i], edges[i + 1], -80, 180)
        if i == len(edges) - 2:
            # Fuse integrated stealth wingtip sharklet onto outermost panel (wing_5)!
            p = p + winglet_right()
        panels.append(p)
    return panels, full


def control_surface_right(full, y_pair, servo_y):
    y0, y1 = y_pair[0] + 0.6, y_pair[1] - 0.6
    body = full & control_prism(y_pair, 0.6)

    def wedge(y):
        hx, zt = line_x(HINGE, y) - 1.0, surf(y, HINGE)[1]
        return xz_face(y, [(hx + 0.5, zt - 0.6), (hx + 0.5, zt - 25), (hx - 25 * math.tan(math.radians(38)), zt - 25)])

    body = body - bd.loft([wedge(y0 - 1), wedge(y1 + 1)])
    hx, zl = line_x(HINGE, servo_y) - 1.0, surf(servo_y, HINGE + 0.08)[0]
    horn = bd.Pos(0, servo_y + 1, 0) * bd.extrude(
        bd.Polygon((hx - 5, zl + 2), (hx - 20, zl + 2), (hx - 8, zl - 12), (hx - 4, zl - 12), align=None), 2.0).rotate(bd.Axis.X, 90)
    hole = rod((hx - 6.5, servo_y - 4, zl - 9.5), (hx - 6.5, servo_y + 4, zl - 9.5), 0.8)
    return body + horn - hole


# ---- blended conformal stealth motor nacelles (PETG-CF) ----------------------------------------
def pod_right(full):
    x_cone = le_x(MOTOR_Y) - 14.0
    x_end = x_cone - 68.0

    def body(r_tube, r_tip, x_front):
        tube = bd.Pos((x_cone + x_front) / 2, MOTOR_Y, MOTOR_Z) * along_x(bd.Cylinder(r_tube, x_front - x_cone))
        cone = bd.Pos((x_end + x_cone) / 2, MOTOR_Y, MOTOR_Z) * along_x(bd.Cone(r_tip, r_tube, x_cone - x_end))
        return tube + cone

    shell = body(POD_R, 3.8, FIREWALL_X) - body(POD_R - POD_WALL, 3.8 - POD_WALL, FIREWALL_X - FIREWALL_T)
    holes = bd.Pos(FIREWALL_X - 6, MOTOR_Y, MOTOR_Z) * along_x(bd.Cylinder(5.0, 14))
    r0, r1 = MOUNT_SLOT
    for k in range(4):
        radial = bd.Pos(0, (r0 + r1) / 2, 0) * bd.Box(14, r1 - r0, 3.4)
        ends = bd.Pos(0, r0, 0) * along_x(bd.Cylinder(1.7, 14)) + bd.Pos(0, r1, 0) * along_x(bd.Cylinder(1.7, 14))
        holes += bd.Pos(FIREWALL_X - 6, MOTOR_Y, MOTOR_Z) * (radial + ends).rotate(bd.Axis.X, 90 * k)
    for ang in (45, 135, 225, 315):
        vent = bd.Pos(0, 11.5, 0) * along_x(bd.Cylinder(2.6, 14))
        holes += bd.Pos(FIREWALL_X - 6, MOTOR_Y, MOTOR_Z) * vent.rotate(bd.Axis.X, ang)
    # Bottom ESC insertion/cooling hatch for Sequre SQESC 2670 70A (35 x 18 x 8 mm) ahead of wing LE!
    esc_access = box(FIREWALL_X - 40.0, FIREWALL_X - 4.0, MOTOR_Y - 9.8, MOTOR_Y + 9.8, MOTOR_Z - 24.0, MOTOR_Z - 11.0)
    rear_exhaust = bd.Pos(x_end - 2, MOTOR_Y, MOTOR_Z) * along_x(bd.Cylinder(2.2, 12))
    return shell - holes - esc_access - rear_exhaust - full - tube_bores()


# ---- parts & stealth RAM finish colors ---------------------------------------------------------
def named(shape, label, color):
    shape.label, shape.color = label, color
    return shape


SKIN, DARK, PETG = "#20252d", "#2f3640", "#13161b"

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


@model
def body_nose():
    return named(_get_body()["nose"], "body_nose", SKIN)


@model
def body_centre():
    return named(_get_body()["centre"], "body_centre", SKIN)


@model
def body_tail():
    return named(_get_body()["tail"], "body_tail", SKIN)


@model
def battery_hatch():
    return named(_get_body()["hatch"], "battery_hatch", DARK)


@model
def wing_1_right():
    return named(_get_wings()[0][0], "wing_1_right", SKIN)


@model
def wing_1_left():
    return named(_get_wings()[0][0].mirror(bd.Plane.XZ), "wing_1_left", SKIN)


@model
def wing_2_right():
    return named(_get_wings()[0][1], "wing_2_right", SKIN)


@model
def wing_2_left():
    return named(_get_wings()[0][1].mirror(bd.Plane.XZ), "wing_2_left", SKIN)


@model
def wing_3_right():
    return named(_get_wings()[0][2], "wing_3_right", SKIN)


@model
def wing_3_left():
    return named(_get_wings()[0][2].mirror(bd.Plane.XZ), "wing_3_left", SKIN)


@model
def wing_4_right():
    return named(_get_wings()[0][3], "wing_4_right", SKIN)


@model
def wing_4_left():
    return named(_get_wings()[0][3].mirror(bd.Plane.XZ), "wing_4_left", SKIN)


@model
def wing_5_right():
    return named(_get_wings()[0][4], "wing_5_right", SKIN)


@model
def wing_5_left():
    return named(_get_wings()[0][4].mirror(bd.Plane.XZ), "wing_5_left", SKIN)


@model
def hover_vane_right():
    return named(control_surface_right(_get_wings()[1], HOVER_VANE_Y, HOV_SERVO_Y), "hover_vane_right", DARK)


@model
def hover_vane_left():
    return named(control_surface_right(_get_wings()[1], HOVER_VANE_Y, HOV_SERVO_Y).mirror(bd.Plane.XZ), "hover_vane_left", DARK)


@model
def elevon_right_part():
    return named(control_surface_right(_get_wings()[1], ELEVON_Y, ELEV_SERVO_Y), "elevon_right", DARK)


@model
def elevon_left_part():
    return named(control_surface_right(_get_wings()[1], ELEVON_Y, ELEV_SERVO_Y).mirror(bd.Plane.XZ), "elevon_left", DARK)


@model
def tail_fin_right():
    return named(fin_plate(True), "tail_fin_right", DARK)


@model
def tail_fin_left():
    return named(fin_plate(True).mirror(bd.Plane.XZ), "tail_fin_left", DARK)


@model
def ventral_fin_right():
    return named(fin_plate(False), "ventral_fin_right", DARK)


@model
def ventral_fin_left():
    return named(fin_plate(False).mirror(bd.Plane.XZ), "ventral_fin_left", DARK)


@model
def motor_pod_right():
    return named(pod_right(_get_wings()[1]), "motor_pod_right", PETG)


@model
def motor_pod_left():
    return named(pod_right(_get_wings()[1]).mirror(bd.Plane.XZ), "motor_pod_left", PETG)


PRINTED = [
    body_nose, body_centre, body_tail, battery_hatch,
    wing_1_right, wing_1_left, wing_2_right, wing_2_left,
    wing_3_right, wing_3_left, wing_4_right, wing_4_left, wing_5_right, wing_5_left,
    hover_vane_right, hover_vane_left, elevon_right_part, elevon_left_part,
    tail_fin_right, tail_fin_left, ventral_fin_right, ventral_fin_left,
    motor_pod_right, motor_pod_left,
]


def hardware():
    """All real bought components at true physical dimensions, for interference verification."""
    parts = []
    carry = rod((CARRY_X, -(CARRY_Y - 1), 0.0), (CARRY_X, CARRY_Y - 1, 0.0), 6.0)
    parts.append(named(carry, "carry_spar_tube_12mm", "#0d0f12"))
    for side in (1, -1):
        m = bd.Pos(FIREWALL_X + MOTOR_L / 2, side * MOTOR_Y, MOTOR_Z) * along_x(bd.Cylinder(MOTOR_D / 2, MOTOR_L))
        parts.append(named(m, "motor_f100_2810", "#1a1e24"))
        # Realistic sleek stealth 8x9 prop blade + hub spinner
        hub = along_x(bd.Cylinder(8.0, 12.0)) + bd.Pos(9.0, 0, 0) * along_x(bd.Cone(8.0, 1.5, 16.0))
        blade = bd.Box(2.0, PROP_D, 15.0).rotate(bd.Axis.Y, 22)
        prop = bd.Pos(FIREWALL_X + MOTOR_L + 7.0, side * MOTOR_Y, MOTOR_Z) * (hub + blade).rotate(bd.Axis.X, 28 * side)
        parts.append(named(prop, "prop_8x9", "#343a42"))
        # Sequre SQESC 2670 70A AM32 inside hollow conformal pod cavity cleanly ahead of wing LE!
        esc = box(FIREWALL_X - 39.0, FIREWALL_X - 39.0 + ESC_DIM[0],
                  side * MOTOR_Y - ESC_DIM[1] / 2, side * MOTOR_Y + ESC_DIM[1] / 2,
                  MOTOR_Z - 4.0, MOTOR_Z - 4.0 + ESC_DIM[2])
        parts.append(named(esc, "esc_70a_am32", "#b45309"))

        # 3-stage telescoping carbon spar + rear torsion spar
        def sp_rod(frac, y0, y1, r):
            p0, p1 = spar_point(frac, y0), spar_point(frac, y1)
            if side < 0:
                p0, p1 = (p0[0], -p0[1], p0[2]), (p1[0], -p1[1], p1[2])
            return rod(p0, p1, r)

        parts.append(named(sp_rod(SPAR_FRAC, SPAR1_Y[0] + 0.5, SPAR1_Y[1] - 0.5, 6.0), "spar_inner_12mm", "#0d0f12"))
        parts.append(named(sp_rod(SPAR_FRAC, SPAR2_Y[0] + 0.5, SPAR2_Y[1] - 0.5, 4.0), "spar_mid_8mm", "#0d0f12"))
        parts.append(named(sp_rod(SPAR_FRAC, SPAR3_Y[0] + 0.5, SPAR3_Y[1] - 0.5, 3.0), "spar_outer_6mm", "#0d0f12"))
        parts.append(named(sp_rod(STUB_FRAC, STUB_Y[0] + 0.5, STUB_Y[1] - 0.5, 3.0), "rear_torsion_spar_6mm", "#0d0f12"))

        # Inboard hover vane servo (Emax ES9051)
        xc, y0 = line_x(SERVO_FRAC, HOV_SERVO_Y), HOV_SERVO_Y - HOV_SERVO[1] / 2
        zl = surf(HOV_SERVO_Y, SERVO_FRAC)[0]
        hs = box(xc - 9.9, xc + 9.9, y0 + 0.2, y0 + HOV_SERVO[1] - 0.2, zl + 0.2, zl + HOV_SERVO[2] - 0.2)
        parts.append(named(hs if side > 0 else hs.mirror(bd.Plane.XZ), "servo_hover_es9051", "#1d4ed8"))

        # Outboard stealth elevon servo (KST X08 Plus V6.0, 8 mm metal case!)
        xc2, y02 = line_x(SERVO_FRAC, ELEV_SERVO_Y), ELEV_SERVO_Y - KST_SERVO[1] / 2
        zl2 = surf(ELEV_SERVO_Y, SERVO_FRAC)[0]
        ks = box(xc2 - 11.7, xc2 + 11.7, y02 + 0.2, y02 + KST_SERVO[1] - 0.2, zl2 + 0.15, zl2 + 7.95)
        parts.append(named(ks if side > 0 else ks.mirror(bd.Plane.XZ), "servo_kst_x08_plus", "#0284c7"))

    # 3x6 flat Molicel P60B 6S3P battery pack (216 x 134 x 24 mm, 388.8 Wh, 1418 g)
    parts.append(named(box(BATTERY_X - PACK_DIM[0] / 2, BATTERY_X + PACK_DIM[0] / 2,
                           -PACK_DIM[1] / 2, PACK_DIM[1] / 2, -PACK_DIM[2] / 2, PACK_DIM[2] / 2),
                       "battery_p60b_6s3p", "#0f766e"))
    # SpeedyBee F405 Wing Mini (37 x 26 x 14 mm) cleanly ahead of carry-through sleeve (x = 128..165 mm)
    fc_x0 = 128.0
    parts.append(named(box(fc_x0, fc_x0 + FC_DIM[0], -FC_DIM[1] / 2, FC_DIM[1] / 2, TRAY_Z + 0.2, TRAY_Z + 0.2 + FC_DIM[2]),
                       "flight_controller", "#15803d"))
    # Walksnail Ascent HD Lite+ VTX below tray
    vz = TRAY_Z - 1.2 - 0.3
    parts.append(named(box(fc_x0, fc_x0 + VTX[0], -VTX[1] / 2, VTX[1] / 2, vz - VTX[2], vz),
                       "video_transmitter", "#c2410c"))
    # Recessed stealth EO/IR nano camera
    cam = bd.Box(9, CAM_W, 12) + bd.Pos(7.9, 0, 0) * along_x(bd.Cylinder(4.5, 6.8))
    parts.append(named(bd.Pos(*CAM) * cam.rotate(bd.Axis.Y, CAM_TILT), "fpv_camera_ascent", "#9333ea"))
    # Tail shelf avionics (cleanly side-by-side in tail bay behind TAIL_JOINT = -138 mm!)
    sz = -3.0
    parts.append(named(box(-166.0, -166.0 + GPS_DIM[0], -GPS_DIM[1] / 2, GPS_DIM[1] / 2, sz, sz + GPS_DIM[2]), "gps_m10", "#2563eb"))
    parts.append(named(box(-162.0, -162.0 + RX_DIM[0], -28.0, -28.0 + RX_DIM[1], sz, sz + RX_DIM[2]), "elrs_rx", "#7c3aed"))
    parts.append(named(box(-172.0, -172.0 + RID_DIM[0], 14.0, 14.0 + RID_DIM[1], sz, sz + RID_DIM[2]), "remote_id", "#db2777"))
    return parts


@model
def kestrel_x_stealth():
    printed = [bd.Compound(children=[p().shape], label=p.__name__) for p in PRINTED]
    bought = bd.Compound(children=hardware(), label="bought_parts")
    return bd.Compound(children=printed + [bought], label="kestrel_x_stealth")


if __name__ == "__main__":
    kestrel_x_stealth()
