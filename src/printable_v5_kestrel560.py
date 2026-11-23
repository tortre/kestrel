"""Kestrel-560, printable parts, v5: blended wing-body with continuous carry-through spar,
cooled avionics & ESC pods, real hardware clearances, reflexed aft body, and symmetric landing fins.

Frame: +X nose, Y span, +Z up in forward flight; parts are modelled where they sit in the aircraft.
Assumed printer: 220 x 220 x 250 mm. Everything in foaming LW-PLA at one wall and ~4 % infill,
except the motor pods (PETG or PLA+, because LW-PLA softens next to a warm motor).

Structure:
  - Continuous 6 mm transverse carbon carry-through spar (190 mm, y = -95..+95 mm at x = -52 mm)
    running unbroken through the closed fuselage ring behind the hatch and 15 mm into both inner wings.
  - Two swept 6 mm main wing spars (~251 mm each, 30 % chord, y = 41.5..277 mm).
  - Two 6 mm aft anti-twist stub tubes (~88 mm each along the swept line, 64 % chord, y = 41.5..126 mm).
  All 5 carbon tubes (251 + 190 = 441 mm; 251 + 88 + 88 = 427 mm) cut from two 500 mm tubes.
"""
import math

import build123d as bd
from cad import model

# ---- planform --------------------------------------------------------------------------------
HALF = 280.0                      # 560 mm span
WING_SHIFT = 30.0                 # wing aft offset for balance with blended forebody
ROOT_LE, ROOT_CHORD = 70.0 - WING_SHIFT, 160.0
SWEEP = 130.0 / 305.0             # leading-edge sweep, mm of x per mm of span
TIP_LE, TIP_CHORD = ROOT_LE - SWEEP * HALF, 100.0
THICKNESS = 0.11
CAMBER_K, CAMBER_A = 0.14, 0.68   # reflexed flying-wing section (zeroes cruise trim drag)
WASHOUT = 3.8                     # degrees of nose-down twist at the tip about the spar line
HINGE = 0.65
ELEVON_Y = (95.0, HALF - 7.0)
BLEND_Y = 80.0                    # body ends, wing begins
SPLIT_Y = 180.0                   # inner / outer wing segment joint

# ---- body: x of nose, x of tail, height above and below the wing plane -------------------------
D0 = (165.0, -175.0, 16.2, 16.2)          # centreline section: clears 37x26x14 mm FC + 8 mm VTX and 25 mm thick Nexus pack
D38 = (118.0, -150.0, 15.2, 15.2)         # section at y = 41, outside the widened 79 mm hollow battery bay
BAY_Y = 39.5                              # interior flat side wall of the hollow bay (79 mm total interior width)
NOSE_FRAC = 0.20                          # fuller nose shoulder so the 37x26x14 mm FC clears the roof
BODY_REFLEX = 3.8                         # mm of upward aft camber behind the battery (s > 0.58) for pitch trim
WALL = 1.0
NOSE_JOINT, TAIL_JOINT = 68.0, -62.0

# ---- structure and fits ------------------------------------------------------------------------
SPAR_FRAC, STUB_FRAC, WIRE_FRAC, SERVO_FRAC = 0.30, 0.64, 0.10, 0.47
TUBE_BORE = 6.3
SPAR_Y0 = 41.5                    # swept main spars start outside the bay wall (38.5 mm anchored in solid blend)
CARRY_X, CARRY_Y = -52.0, 95.0    # continuous 190 mm carry-through carbon spar across y = -95..+95 mm
STUB_Y = (41.5, 126.0)            # 92 mm anti-twist stub tubes across the wing-body joint
PIN_D = 2.0                       # 1.75 mm filament alignment pins

# ---- symmetric canted tails & landing gear -----------------------------------------------------
FIN_T, FIN_CANT = 2.4, 30.0
FIN_ROOT_CLIP = 4.0               # root tabs end at |z| = 4 mm, short of where upper and ventral plates cross
FIN_Y_UP, FIN_Y_DOWN = 56.0, 64.0
# Symmetric +-88 mm plates at 30 deg cant give +-76.2 mm belly/back stance (+15 % blow-over resistance)
FIN_UP = [(-90, -10), (-90, 8), (-150, 88), (-205, 88), (-150, 12), (-128, 5), (-128, -10)]
FIN_DOWN = [(-90, 10), (-128, 10), (-128, -5), (-150, -12), (-205, -88), (-150, -88), (-90, -8)]

# ---- components (real verified dimensions) -----------------------------------------------------
MOTOR_Y, MOTOR_Z = 160.0, 0.0
MOTOR_D, MOTOR_L = 27.7, 20.0     # Emax ECO II 2306 (1700KV / 1900KV), M5 shaft, 16x16 M3 mount
MOUNT_SLOT = (7.5, 10.0)          # radial M3 slots: bolt circles from 15 to 20 mm (16x16, 16x19, 19x19)
POD_R, POD_WALL, FIREWALL_X, FIREWALL_T = 14.2, 1.1, 43.0, 3.0
PROP_D = 177.8
ESC_DIM = (26.0, 13.0, 5.0)       # 30A BLHeli_S/Bluejay or AM32 mini single ESC inside cooled pod
SERVO = (20.2, 23.2, 8.7)         # EMAX ES9051 4.3 g digital servo pocket
SERVO_LUGS = (29.0, 10.5, 15.5)
SERVO_Y = 118.0
CELL_D, CELL_L = 21.55, 70.2      # real 21700 cell core
PACK_DIM = (71.5, 72.5, 25.0)     # factory-built Nexus 3S1P Reliance RS50 5000mAh 70A flat pack (70x72x25 mm)
BATTERY_X = 12.0                  # nominal first-flight position; slides between -0.5 and 27.5 mm under the hatch
HATCH_X, HATCH_Y = (-39.0, 66.0), 38.5    # 105 mm long x 74.6 mm clear width -> factory 72 mm Nexus pack drops straight in!
FC_DIM = (37.0, 26.0, 14.0)       # real SpeedyBee F405 Wing Mini (37 x 26 x 14 mm)
TRAY_Z = -3.0                     # top of FC tray; leaves 9.8 mm below for 8 mm VTX and >14.5 mm above for FC
VTX = (23.5, 23.5, 8.0)           # Walksnail Ascent HD Lite+ 500 mW VTX
VTX_ANT_X, VTX_ANT_D = 104.0, 6.5
CAM = (132.0, 0.0, 0.0)           # Walksnail Ascent Lite+ nano camera (12 x 14 x 15.8 mm)
CAM_TILT = 30.0
CAM_W = 14.0
GPS_DIM = (18.0, 18.0, 6.0)       # M10 GPS + compass in tail shelf
RID_DIM = (24.0, 16.0, 6.5)       # Remote ID broadcast module in tail
RX_DIM = (10.5, 10.5, 3.0)        # ELRS 2.4 GHz nano receiver
WIRE_ENTRY_X, WIRE_ENTRY_Z = -18.0, -9.0

C = bd.Align.CENTER


# ---- geometry helpers --------------------------------------------------------------------------
def le_x(y):
    return ROOT_LE + (TIP_LE - ROOT_LE) * abs(y) / HALF


def chord(y):
    return ROOT_CHORD + (TIP_CHORD - ROOT_CHORD) * abs(y) / HALF


def line_x(frac, y):
    return le_x(y) - frac * chord(y)


def surf(y, s):
    c = chord(y)
    yt = 5 * THICKNESS * (0.2969 * math.sqrt(s) - 0.1260 * s - 0.3516 * s**2 + 0.2843 * s**3 - 0.1015 * s**4)
    yc = CAMBER_K * s * (1 - s) * (CAMBER_A - s)
    twist = WASHOUT * max(0.0, abs(y) - BLEND_Y) / (HALF - BLEND_Y)
    dz = (s - SPAR_FRAC) * c * math.tan(math.radians(twist))        # trailing edge up, leading edge down
    return c * (yc - yt) + dz, c * (yc + yt) + dz


def profile_face(upper, lower):
    top = bd.Spline(*reversed(upper))
    bottom = bd.Spline(*lower)
    return bd.Face(bd.Wire([top, bottom, bd.Line(lower[-1], upper[-1])]))


def stations(n=24):
    return [0.5 * (1 - math.cos(math.pi * i / n)) for i in range(n + 1)]


def airfoil_face(y):
    upper, lower = [], []
    for s in stations():
        zl, zu = surf(y, s)
        upper.append((le_x(y) - chord(y) * s, y, zu))
        lower.append((le_x(y) - chord(y) * s, y, zl))
    return profile_face(upper, lower)


def body_f(s):
    """Body thickness along its length: rounded nose, full height over FC and battery, tapered tail."""
    if s < NOSE_FRAC:
        t = (NOSE_FRAC - s) / NOSE_FRAC
        return (1 - t * t) ** 0.58
    if s < 0.66:
        return 1.0
    return 0.02 + 0.98 * 0.5 * (1 + math.cos(math.pi * (s - 0.66) / 0.34))


def body_camber_z(s):
    """Smooth upward reflex behind the battery bay (s > 0.58) so cruise pitching moment self-trims."""
    return BODY_REFLEX * max(0.0, (s - 0.58) / 0.42) ** 2


def body_face(y, dims, inset=0.0):
    x_le, x_te, top, bot = dims
    x_le, x_te, top, bot = x_le - 2.5 * inset, x_te + 6 * inset, top - inset, bot - inset
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


def frac_line(frac, y0, y1, z=0.0, r=TUBE_BORE / 2):
    return rod((line_x(frac, y0), y0, z), (line_x(frac, y1), y1, z), r)


def both(shape):
    return shape + shape.mirror(bd.Plane.XZ)


# ---- shared cuts -------------------------------------------------------------------------------
def tube_bores():
    swept_and_stub = both(frac_line(SPAR_FRAC, SPAR_Y0, HALF - 3) + frac_line(STUB_FRAC, *STUB_Y))
    carry_through = rod((CARRY_X, -CARRY_Y, 0.0), (CARRY_X, CARRY_Y, 0.0), TUBE_BORE / 2)
    return swept_and_stub + carry_through


def wire_runs():
    """Enlarged 6.2 mm ESC wire channels + servo wire channels into the widened 79 mm bay."""
    sx = line_x(SERVO_FRAC, BLEND_Y)
    end = (line_x(WIRE_FRAC, MOTOR_Y), MOTOR_Y, 0.5)
    t = (50.0 - 38.0) / (MOTOR_Y - 38.0)
    knee = (WIRE_ENTRY_X + t * (end[0] - WIRE_ENTRY_X), 50.0, 0.5)
    start = (WIRE_ENTRY_X, 38.0, -10.5)
    past = tuple(k + 0.15 * (k - a) for k, a in zip(knee, start))
    esc = rod(start, past, 3.1) + rod(knee, end, 3.1)
    return both(esc + rod((sx, 37.5, -4.0), (sx, BLEND_Y + 2, -4.0), 2.6))


def fin_plate(upper, clear=0.0):
    """Symmetric canted upper tail or ventral fin / landing leg."""
    t = FIN_T + 2 * clear
    pts, y, cant = (FIN_UP, FIN_Y_UP, -FIN_CANT) if upper else (FIN_DOWN, FIN_Y_DOWN, FIN_CANT)
    plate = bd.Pos(0, 0, -t / 2) * bd.extrude(bd.Polygon(*pts, align=None), t, dir=(0, 0, 1))
    plate = bd.Pos(0, y, 0) * plate.rotate(bd.Axis.X, 90).rotate(bd.Axis.X, cant)
    # The root tabs of the upper and ventral plates cross each other at z = +4.5..9.3 mm (y ~ 60 mm),
    # mostly outside the skin: as drawn the two fins could not both be fitted. Stop each tab 0.5 mm
    # short of the crossing (symmetric, so the slots it cuts in the body shrink to match).
    if upper:
        return plate - box(-300, 0, 0, 200, -100, -FIN_ROOT_CLIP - clear)
    return plate - box(-300, 0, 0, 200, FIN_ROOT_CLIP + clear, 100)


def fin_slots():
    return both(fin_plate(True, 0.15) + fin_plate(False, 0.15))


def cam_axis(length):
    a = math.radians(CAM_TILT)
    return (CAM[0] + length * math.cos(a), CAM[1], CAM[2] - length * math.sin(a))


# ---- body --------------------------------------------------------------------------------------
def body_outer():
    guide = BLEND_Y + 22
    blend = bd.loft([airfoil_face(-guide), airfoil_face(-BLEND_Y), body_face(-41, D38), body_face(0, D0),
                     body_face(41, D38), airfoil_face(BLEND_Y), airfoil_face(guide)])
    return blend & box(-300, 300, -BLEND_Y, BLEND_Y, -60, 60)


def body_inset(inset):
    """The hollow bay: 79 mm wide interior ending in flat side walls at y = +-BAY_Y (39.5 mm)."""
    return bd.loft([body_face(-BAY_Y, D38, inset + 0.5), body_face(0, D0, inset), body_face(BAY_Y, D38, inset + 0.5)])


def collar(joint, direction):
    bore = body_inset(WALL + 1.2)
    a, b = sorted((joint - 5 * direction, joint))
    fixed = (body_inset(WALL - 0.2) - bore) & box(a, b, -60, 60, -40, 40)
    a, b = sorted((joint, joint + 6 * direction))
    slip = (body_inset(WALL + 0.2) - bore) & box(a, b, -60, 60, -40, 40)
    return fixed + slip


def hatch_opening(clear=0.0):
    return box(HATCH_X[0] + clear, HATCH_X[1] - clear, -HATCH_Y + clear, HATCH_Y - clear, 5, 40)


def joint_pins():
    out = None
    for x in (NOSE_JOINT, TAIL_JOINT):
        for y in (-58, 58):
            h = rod((x - 8, y, 3.0), (x + 8, y, 3.0), PIN_D / 2)
            out = h if out is None else out + h
    return out


def cooling_ducts():
    """Ram-air cheek inlets in the nose blowing across the 500 mW Walksnail VTX & FC, plus tail exhaust."""
    inlet = both(rod((128.0, 15.0, -5.5), (102.0, 11.0, -6.5), 2.8))
    exhaust = box(-152.0, -134.0, -7.0, 7.0, 4.0, 25.0)
    return inlet + exhaust


def body_parts():
    outer, cavity = body_outer(), body_inset(WALL)
    shell = outer - cavity
    # Structural carbon carry-through sleeve across the hollow bay behind the hatch (x = -52 mm)
    carry_sleeve = rod((CARRY_X, -BAY_Y - 2, 0.0), (CARRY_X, BAY_Y + 2, 0.0), 5.0) & cavity
    # Side ledges for battery hatch: 1.2 mm lip leaves a 74.6 mm clear opening for the 72.5 mm wide flat 3S1P pack
    ledge = ((cavity - body_inset(WALL + 1.2)) & box(HATCH_X[0] - 3.0, HATCH_X[1] + 3.0, -HATCH_Y - 1.5, HATCH_Y + 1.5, 5, 40)
             ) - box(HATCH_X[0] + 2.0, HATCH_X[1] - 2.0, -HATCH_Y + 1.2, HATCH_Y - 1.2, 0, 40)
    # Rear hatch M2 latch boss & Velcro strap tie-down anchors on the bay floor
    latch_boss = box(HATCH_X[0] - 6.0, HATCH_X[0] + 0.5, -8.0, 8.0, 8.0, 20.0) & cavity
    latch_hole = bd.Pos(HATCH_X[0] - 2.8, 0, 5.0) * bd.Cylinder(1.1, 20.0, align=(C, C, bd.Align.MIN))
    strap_slots = None
    cuts = tube_bores() + wire_runs() + fin_slots() + joint_pins() + cooling_ducts()

    centre_zone = box(TAIL_JOINT, NOSE_JOINT, -100, 100, -40, 40)
    centre_shell = shell & centre_zone

    # Hatch with integrated front tongue tab (1 single solid, 0.35 mm slip clearance under roof) and rear screw hole
    hatch_skin = centre_shell & hatch_opening(0.3)
    tongue_step = (body_inset(WALL - 0.1) - body_inset(WALL + 2.4)) & box(HATCH_X[1] - 4.0, HATCH_X[1] - 0.5, -8.5, 8.5, 8.0, 20.0)
    front_tongue = (body_inset(WALL + 1.35) - body_inset(WALL + 2.4)) & box(HATCH_X[1] - 4.0, HATCH_X[1] + 2.5, -8.5, 8.5, 8.0, 20.0)
    tongue_clearance = box(HATCH_X[1] - 1.0, HATCH_X[1] + 3.2, -9.2, 9.2, 5.0, 20.0) & body_inset(WALL + 1.05)
    hatch = hatch_skin + tongue_step + front_tongue - latch_hole

    centre = (centre_shell + (carry_sleeve & centre_zone) + ((ledge - tongue_clearance) & centre_zone) + (latch_boss & centre_zone)
              + collar(NOSE_JOINT, 1) + collar(TAIL_JOINT, -1) - hatch_opening() - latch_hole - cuts)

    # Nose: slotted flight-controller tray (clears real 37x26x14 mm FC above and 23.5x23.5x8 mm VTX below)
    nose_zone = box(NOSE_JOINT, 200, -100, 100, -40, 40)
    fc_tray_raw = box(NOSE_JOINT + 6.5, NOSE_JOINT + 47.5, -45, 45, TRAY_Z - 1.2, TRAY_Z) & cavity
    vtx_air_slot = box(NOSE_JOINT + 10.0, NOSE_JOINT + 30.0, -8.0, 8.0, TRAY_Z - 3.0, TRAY_Z + 1.0)
    fc_tray = fc_tray_raw - vtx_air_slot
    rails = both(box(CAM[0] - 8.5, CAM[0] + 8.5, CAM_W / 2 + 0.2, CAM_W / 2 + 2.2, -40, 40)) & cavity
    screws = rod((CAM[0], -20, CAM[2]), (CAM[0], 20, CAM[2]), 1.1)
    window = rod(CAM, cam_axis(45), 5.5)
    antenna = bd.Pos(VTX_ANT_X, 0, -30) * bd.Cylinder(VTX_ANT_D / 2, 30, align=(C, C, bd.Align.MIN))
    nose = (shell & nose_zone) + fc_tray + rails - screws - window - antenna - cuts

    # Tail: horizontal shelf for M10 GPS + compass, ELRS RX, and Remote ID module
    tail_zone = box(-200, TAIL_JOINT, -100, 100, -40, 40)
    tail_shelf = (box(-128.0, TAIL_JOINT - 6.5, -35, 35, -4.8, -3.8) & cavity) - box(-118.0, -72.0, -6.0, 6.0, -6.0, -3.0)
    tail = (shell & tail_zone) + tail_shelf - cuts
    return dict(nose=nose, centre=centre, tail=tail, hatch=hatch, outer=outer)


# ---- wing --------------------------------------------------------------------------------------
def wing_full():
    return bd.loft([airfoil_face(0), airfoil_face(HALF)])


def elevon_prism(inset):
    y0, y1 = ELEVON_Y[0] + inset, ELEVON_Y[1] - inset
    back = 1.0 if inset else 0.0
    pts = ((line_x(HINGE, y0) - back, y0), (line_x(HINGE, y1) - back, y1), (-260, y1), (-260, y0))
    return bd.extrude(bd.Polygon(*pts, align=None), 40, both=True)


def servo_pocket():
    xc = line_x(SERVO_FRAC, SERVO_Y)
    zl, zu = surf(SERVO_Y, SERVO_FRAC)
    top = zl + SERVO[2]
    assert zu - top >= 1.2, f"servo pocket leaves only {zu - top:.1f} mm of skin"
    y0 = SERVO_Y - SERVO[1] / 2
    body = box(xc - SERVO[0] / 2, xc + SERVO[0] / 2, y0, y0 + SERVO[1], -30, top)
    tabs = box(xc - SERVO_LUGS[0] / 2, xc + SERVO_LUGS[0] / 2, y0 + SERVO_LUGS[1], y0 + SERVO_LUGS[2], -30, top)
    z_groove = surf(92, SERVO_FRAC)[0] + 3.0
    ya = BLEND_Y - 2
    strip = bd.Polygon((line_x(SERVO_FRAC, ya) + 1.75, ya), (line_x(SERVO_FRAC, y0 + 1) + 1.75, y0 + 1),
                       (line_x(SERVO_FRAC, y0 + 1) - 1.75, y0 + 1), (line_x(SERVO_FRAC, ya) - 1.75, ya), align=None)
    groove = bd.Pos(0, 0, -30) * bd.extrude(strip, 30 + z_groove, dir=(0, 0, 1))
    return body + tabs + groove


def pin_holes(y, depth=8.0):
    out = None
    for frac in (0.15, 0.55):
        z = sum(surf(y, frac)) / 2
        h = rod((line_x(frac, y), y - depth, z), (line_x(frac, y), y + depth, z), PIN_D / 2)
        out = h if out is None else out + h
    return out


def wing_right():
    full = wing_full()
    riser = bd.Pos(line_x(WIRE_FRAC, MOTOR_Y) + 2, MOTOR_Y - 2, -4.0) * bd.Cylinder(3.4, 24, align=(C, C, bd.Align.MIN))
    solid = full - elevon_prism(0) - tube_bores() - wire_runs() - riser - servo_pocket() - pin_holes(SPLIT_Y)
    inner = solid & box(-300, 200, BLEND_Y, SPLIT_Y, -50, 50)
    outer = solid & box(-300, 200, SPLIT_Y, HALF + 5, -50, 50)
    return inner, outer, full


def elevon_right(full):
    y0, y1 = ELEVON_Y[0] + 0.6, ELEVON_Y[1] - 0.6
    body = full & elevon_prism(0.6)

    def wedge(y):                      # clearance under the tape hinge for 40 degrees of down elevon
        hx, zt = line_x(HINGE, y) - 1.0, surf(y, HINGE)[1]
        return xz_face(y, [(hx + 0.5, zt - 0.6), (hx + 0.5, zt - 25), (hx - 25 * math.tan(math.radians(40)), zt - 25)])

    body = body - bd.loft([wedge(y0 - 1), wedge(y1 + 1)])
    hx, zl = line_x(HINGE, SERVO_Y) - 1.0, surf(SERVO_Y, HINGE + 0.08)[0]
    horn = bd.Pos(0, SERVO_Y + 1, 0) * bd.extrude(
        bd.Polygon((hx - 6, zl + 2), (hx - 22, zl + 2), (hx - 9, zl - 13), (hx - 5, zl - 13), align=None), 2.0).rotate(bd.Axis.X, 90)
    hole = rod((hx - 7.5, SERVO_Y - 4, zl - 10), (hx - 7.5, SERVO_Y + 4, zl - 10), 0.8)
    return body + horn - hole


# ---- serviceable & cooled motor pods -----------------------------------------------------------
def pod_right(full):
    x_cone = le_x(MOTOR_Y) - 8.0
    x_end = x_cone - 53.0

    def body(r_tube, r_tip, x_front):
        tube = bd.Pos((x_cone + x_front) / 2, MOTOR_Y, MOTOR_Z) * along_x(bd.Cylinder(r_tube, x_front - x_cone))
        cone = bd.Pos((x_end + x_cone) / 2, MOTOR_Y, MOTOR_Z) * along_x(bd.Cone(r_tip, r_tube, x_cone - x_end))
        return tube + cone

    shell = body(POD_R, 3.2, FIREWALL_X) - body(POD_R - POD_WALL, 3.2 - POD_WALL, FIREWALL_X - FIREWALL_T)
    holes = bd.Pos(FIREWALL_X - 5, MOTOR_Y, MOTOR_Z) * along_x(bd.Cylinder(4.5, 12))
    r0, r1 = MOUNT_SLOT
    for k in range(4):
        radial = bd.Pos(0, (r0 + r1) / 2, 0) * bd.Box(12, r1 - r0, 3.3)
        ends = bd.Pos(0, r0, 0) * along_x(bd.Cylinder(1.65, 12)) + bd.Pos(0, r1, 0) * along_x(bd.Cylinder(1.65, 12))
        holes += bd.Pos(FIREWALL_X - 5, MOTOR_Y, MOTOR_Z) * (radial + ends).rotate(bd.Axis.X, 90 * k)
    # Firewall cooling vents at 45/135/225/315 deg + bottom ESC insertion/cooling slot + rear cone exhaust
    for ang in (45, 135, 225, 315):
        vent = bd.Pos(0, 9.2, 0) * along_x(bd.Cylinder(2.4, 12))
        holes += bd.Pos(FIREWALL_X - 5, MOTOR_Y, MOTOR_Z) * vent.rotate(bd.Axis.X, ang)
    esc_access = box(-4.0, 29.0, MOTOR_Y - 7.2, MOTOR_Y + 7.2, -20.0, -9.5)
    rear_exhaust = bd.Pos(x_end - 2, MOTOR_Y, MOTOR_Z) * along_x(bd.Cylinder(1.8, 10))
    return shell - holes - esc_access - rear_exhaust - full


# ---- parts -------------------------------------------------------------------------------------
def named(shape, label, color):
    shape.label, shape.color = label, color
    return shape


SKIN, DARK, PETG = "#8d949b", "#5c636b", "#2b2f34"


@model
def body_nose():
    return named(body_parts()["nose"], "body_nose", SKIN)


@model
def body_centre():
    return named(body_parts()["centre"], "body_centre", SKIN)


@model
def body_tail():
    return named(body_parts()["tail"], "body_tail", SKIN)


@model
def battery_hatch():
    return named(body_parts()["hatch"], "battery_hatch", DARK)


@model
def wing_inner_right():
    return named(wing_right()[0], "wing_inner_right", SKIN)


@model
def wing_inner_left():
    return named(wing_right()[0].mirror(bd.Plane.XZ), "wing_inner_left", SKIN)


@model
def wing_outer_right():
    return named(wing_right()[1], "wing_outer_right", SKIN)


@model
def wing_outer_left():
    return named(wing_right()[1].mirror(bd.Plane.XZ), "wing_outer_left", SKIN)


@model
def elevon_right_part():
    return named(elevon_right(wing_full()), "elevon_right", DARK)


@model
def elevon_left_part():
    return named(elevon_right(wing_full()).mirror(bd.Plane.XZ), "elevon_left", DARK)


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
    return named(pod_right(wing_full()), "motor_pod_right", PETG)


@model
def motor_pod_left():
    return named(pod_right(wing_full()).mirror(bd.Plane.XZ), "motor_pod_left", PETG)


PRINTED = [body_nose, body_centre, body_tail, battery_hatch, wing_inner_right, wing_inner_left,
           wing_outer_right, wing_outer_left, elevon_right_part, elevon_left_part, tail_fin_right, tail_fin_left,
           ventral_fin_right, ventral_fin_left, motor_pod_right, motor_pod_left]


def hardware():
    """All real bought components at true physical dimensions, for interference verification."""
    parts = []
    carry = rod((CARRY_X, -(CARRY_Y - 1), 0.0), (CARRY_X, CARRY_Y - 1, 0.0), 3.0)
    parts.append(named(carry, "carry_spar_tube", "#101010"))
    for side in (1, -1):
        m = bd.Pos(FIREWALL_X + MOTOR_L / 2, side * MOTOR_Y, MOTOR_Z) * along_x(bd.Cylinder(MOTOR_D / 2, MOTOR_L))
        parts.append(named(m, "motor", "#15171a"))
        prop = bd.Pos(FIREWALL_X + MOTOR_L + 6, side * MOTOR_Y, MOTOR_Z) * (
            along_x(bd.Cylinder(6, 8)) + bd.Box(1.6, PROP_D, 12).rotate(bd.Axis.Y, 14)).rotate(bd.Axis.X, 25 * side)
        parts.append(named(prop, "prop_7x4.5", "#c8ccd0"))
        esc = box(0.0, ESC_DIM[0], side * MOTOR_Y - ESC_DIM[1] / 2, side * MOTOR_Y + ESC_DIM[1] / 2, -6.5, -6.5 + ESC_DIM[2])
        parts.append(named(esc, "esc_30a", "#d97706"))
        y_tip = HALF - 4.5
        spar = rod((line_x(SPAR_FRAC, SPAR_Y0 + 0.5), side * (SPAR_Y0 + 0.5), 0), (line_x(SPAR_FRAC, y_tip), side * y_tip, 0), 3.0)
        stub = rod((line_x(STUB_FRAC, STUB_Y[0] + 0.5), side * (STUB_Y[0] + 0.5), 0), (line_x(STUB_FRAC, STUB_Y[1] - 0.5), side * (STUB_Y[1] - 0.5), 0), 3.0)
        parts += [named(spar, "spar_tube", "#101010"), named(stub, "stub_tube", "#101010")]
        xc, y0 = line_x(SERVO_FRAC, SERVO_Y), SERVO_Y - SERVO[1] / 2
        zl = surf(SERVO_Y, SERVO_FRAC)[0]
        servo = box(xc - 9.88, xc + 9.88, y0 + 0.2, y0 + 23.05, zl + 0.15, zl + 8.49)
        parts.append(named(servo if side > 0 else servo.mirror(bd.Plane.XZ), "servo_es9051", "#1f4e9c"))
    # Factory-wrapped Nexus 3S1P 21700 Reliance RS50 flat battery pack envelope (71.5 x 72.5 x 25.0 mm)
    parts.append(named(box(BATTERY_X - PACK_DIM[0] / 2, BATTERY_X + PACK_DIM[0] / 2,
                           -PACK_DIM[1] / 2, PACK_DIM[1] / 2, -PACK_DIM[2] / 2, PACK_DIM[2] / 2),
                       "battery_nexus_rs50_3s1p", "#2a9d8f"))
    # Real SpeedyBee F405 Wing Mini (37 x 26 x 14 mm)
    fc_x0 = NOSE_JOINT + 8.0
    parts.append(named(box(fc_x0, fc_x0 + FC_DIM[0], -FC_DIM[1] / 2, FC_DIM[1] / 2, TRAY_Z + 0.2, TRAY_Z + 0.2 + FC_DIM[2]),
                       "flight_controller", "#2d6a4f"))
    # Walksnail Ascent HD Lite+ VTX (23.5 x 23.5 x 8.0 mm) below tray
    vz = TRAY_Z - 1.2 - 0.3
    parts.append(named(box(NOSE_JOINT + 8.0, NOSE_JOINT + 8.0 + VTX[0], -VTX[1] / 2, VTX[1] / 2, vz - VTX[2], vz),
                       "video_transmitter", "#e76f51"))
    # Walksnail Ascent Lite+ nano camera
    cam = bd.Box(9, CAM_W, 12) + bd.Pos(7.9, 0, 0) * along_x(bd.Cylinder(4.5, 6.8))
    parts.append(named(bd.Pos(*CAM) * cam.rotate(bd.Axis.Y, CAM_TILT), "fpv_camera_ascent", "#b5179e"))
    # Tail avionics on shelf (z = -3.8 mm top): M10 GPS + Remote ID above shelf, ELRS RX below shelf
    sz = -3.5
    parts.append(named(box(-90.0, -90.0 + GPS_DIM[0], -GPS_DIM[1] / 2, GPS_DIM[1] / 2, sz, sz + GPS_DIM[2]), "gps_m10", "#3a86ff"))
    parts.append(named(box(-86.0, -86.0 + RX_DIM[0], -RX_DIM[1] / 2, RX_DIM[1] / 2, -8.2, -8.2 + RX_DIM[2]), "elrs_rx", "#8338ec"))
    parts.append(named(box(-116.0, -116.0 + RID_DIM[0], -RID_DIM[1] / 2, RID_DIM[1] / 2, sz, sz + RID_DIM[2]), "remote_id", "#ff006e"))
    return parts


@model
def kestrel_600_printable():
    printed = [bd.Compound(children=[p().shape], label=p.__name__) for p in PRINTED]
    bought = bd.Compound(children=hardware(), label="bought_parts")
    uav = bd.Compound(children=printed + [bought], label="kestrel_600_printable")
    size = uav.bounding_box().size
    assert size.Y <= 610.01, f"span {size.Y:.1f} mm exceeds 2 ft"
    return uav


if __name__ == "__main__":
    kestrel_600_printable()
