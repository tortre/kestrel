"""Kestrel-600, printable parts, v2: blended wing-body with canted twin tails and a chin camera.

Frame: +X nose, Y span, +Z up in forward flight; parts are modelled where they sit in the aircraft.
Assumed printer: 220 x 220 x 250 mm. Everything in foaming LW-PLA at one wall and ~4 % infill,
except the motor pods (PETG or PLA+, because LW-PLA softens next to a warm motor).

Shape: the body is lofted spanwise through airfoil-like sections, so it flows into the wing with no
junction. The centre 72 mm is hollow (battery over the spars, electronics and camera in the nose);
the blend out to y = 80 prints as sparse-infill solid. Two canted plates pass through slots in the
body: their upper halves are the tails, their lower halves the ventral fins, and their four rear
corners are the landing feet.

Structure: one 6 mm carbon tube per wing along the 30 % chord line, running 44 mm into the solid
blend of the body, plus a 6 mm stub at 62 % chord against twist. Two 500 mm tubes give 2 x 282 + 2 x 105 mm.
"""
import math

import build123d as bd
from cad import model

# ---- planform --------------------------------------------------------------------------------
HALF = 305.0
WING_SHIFT = 30.0                 # wing moved aft so the aircraft balances with the wide body ahead of it
ROOT_LE, ROOT_CHORD = 70.0 - WING_SHIFT, 160.0
TIP_LE, TIP_CHORD = -60.0 - WING_SHIFT, 100.0
THICKNESS = 0.11
WASHOUT = 2.0                     # degrees of nose-down twist at the tip, about the spar line (gentler stall, less trim)
HINGE = 0.65
ELEVON_Y = (95.0, 298.0)
BLEND_Y = 80.0                    # body ends, wing begins
SPLIT_Y = 190.0                   # inner / outer wing segment joint

# ---- body: x of nose, x of tail, height above and below the wing plane -------------------------
D0 = (165.0, -175.0, 14.5, 14.5)          # centreline section: thin and symmetric, battery in the wing plane
D36 = (120.0, -150.0, 13.5, 13.5)         # section at y = 36, the edge of the hollow bay
NOSE_FRAC = 0.28                          # fraction of body length taken by the rounded nose
WALL = 1.0
NOSE_JOINT, TAIL_JOINT = 76.0, -55.0

# ---- structure and fits ------------------------------------------------------------------------
SPAR_FRAC, STUB_FRAC, WIRE_FRAC, SERVO_FRAC = 0.30, 0.62, 0.10, 0.47
TUBE_BORE = 6.3
STUB_Y = (18.0, 120.0)
SPAR_Y0 = 36.0                    # spars end at the bay wall: 44 mm of each sits in the solid blend
PIN_D = 2.0                       # 1.75 mm filament alignment pins

# ---- tails -------------------------------------------------------------------------------------
FIN_T, FIN_CANT = 2.4, 28.0
FIN_Y_UP, FIN_Y_DOWN = 56.0, 68.0          # where each plate crosses the wing plane
# outlines: x, height along the canted plate. Each plate drops into its own slot through the body and
# pokes 10 mm out the far side for a glue fillet. The slots sit behind the stub spar (x < -88).
FIN_UP = [(-88, -10), (-88, 8), (-150, 88), (-205, 88), (-150, 12), (-128, 5), (-128, -10)]
FIN_DOWN = [(-88, 10), (-128, 10), (-128, -4), (-150, -10), (-205, -75), (-165, -75), (-88, -6)]

# ---- components --------------------------------------------------------------------------------
MOTOR_Y, MOTOR_Z = 160.0, 0.0     # outboard for prop-to-body clearance; thrust line through the CG
MOTOR_D, MOTOR_L = 27.0, 22.5     # SunnySky X2206
MOUNT_SLOT = (7.5, 10.0)          # radial M3 slots: bolt circles from 15 to 20 mm (16x16, 16x19, 19x19)
POD_R, POD_WALL, FIREWALL_X, FIREWALL_T = 14.0, 1.0, 43.0, 3.0
PROP_D = 177.8
SERVO = (23.6, 27.0, 11.7)        # EMAX ES08MA II on its side
SERVO_Y = 118.0
CELL_D, CELL_L = 21.0, 70.0
BATTERY_X = 28.0                  # ahead of the spars; slides forward to trim (sim/blended.py)
HATCH_X, HATCH_Y = (-12.0, 72.0), 33.0
CAM = (128.0, 0.0, 0.0)           # 14 mm "nano" FPV camera, pivot point
CAM_TILT = 30.0                   # degrees below the flight path; adjustable on its two M2 screws
CAM_W = 14.0
WIRE_ENTRY_X = -14.0               # ESC leads enter the bay here; the battery must sit ahead of it
VTX_ANT_X, VTX_ANT_D = 100.0, 6.5     # antenna exit under the nose; the transmitter lies on the floor below the tray

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
    yc = 0.12 * s * (1 - s) * (0.75 - s)
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
    """Body thickness along its length: rounded nose, full height over the battery, tapered tail."""
    if s < NOSE_FRAC:
        t = (NOSE_FRAC - s) / NOSE_FRAC
        return (1 - t * t) ** 0.6
    if s < 0.66:
        return 1.0
    return 0.02 + 0.98 * 0.5 * (1 + math.cos(math.pi * (s - 0.66) / 0.34))


def body_face(y, dims, inset=0.0):
    x_le, x_te, top, bot = dims
    x_le, x_te, top, bot = x_le - 2.5 * inset, x_te + 6 * inset, top - inset, bot - inset
    upper, lower = [], []
    for s in stations():
        x, f = x_le - (x_le - x_te) * s, body_f(s)
        upper.append((x, y, top * f))
        lower.append((x, y, -bot * f))
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
    return both(frac_line(SPAR_FRAC, SPAR_Y0, HALF - 3) + frac_line(STUB_FRAC, *STUB_Y))


def wire_runs():
    """ESC leads run ahead of the spar into the bay behind the battery; the servo lead comes straight in
    from the groove under the wing. Inside the bay the leads pass the battery in the gaps between cells."""
    sx = line_x(SERVO_FRAC, BLEND_Y)
    esc = rod((WIRE_ENTRY_X, 30, 0.5), (line_x(WIRE_FRAC, MOTOR_Y), MOTOR_Y, 0.5), 2.75)
    return both(esc + rod((sx, 28, -4.0), (sx, BLEND_Y + 2, -4.0), 2.5))


def fin_plate(upper, clear=0.0):
    """Upper tail (canted outward going up) or ventral fin (canted outward going down)."""
    t = FIN_T + 2 * clear
    pts, y, cant = (FIN_UP, FIN_Y_UP, -FIN_CANT) if upper else (FIN_DOWN, FIN_Y_DOWN, FIN_CANT)
    plate = bd.Pos(0, 0, -t / 2) * bd.extrude(bd.Polygon(*pts, align=None), t, dir=(0, 0, 1))
    return bd.Pos(0, y, 0) * plate.rotate(bd.Axis.X, 90).rotate(bd.Axis.X, cant)


def fin_slots():
    return both(fin_plate(True, 0.15) + fin_plate(False, 0.15))


def cam_axis(length):
    a = math.radians(CAM_TILT)
    return (CAM[0] + length * math.cos(a), CAM[1], CAM[2] - length * math.sin(a))


# ---- body --------------------------------------------------------------------------------------
def body_outer():
    guide = BLEND_Y + 22
    blend = bd.loft([airfoil_face(-guide), airfoil_face(-BLEND_Y), body_face(-36, D36), body_face(0, D0),
                     body_face(36, D36), airfoil_face(BLEND_Y), airfoil_face(guide)])
    return blend & box(-300, 300, -BLEND_Y, BLEND_Y, -60, 60)


def body_inset(inset):
    """The hollow bay: the body's surface moved inward, ending in flat side walls at y = +-34."""
    return bd.loft([body_face(-34, D36, inset + 0.6), body_face(0, D0, inset), body_face(34, D36, inset + 0.6)])


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


def body_parts():
    outer, cavity = body_outer(), body_inset(WALL)
    shell = outer - cavity
    sockets = both(frac_line(STUB_FRAC, STUB_Y[0], 40, r=4.9)) & cavity
    ledge = ((cavity - body_inset(WALL + 1.2)) & box(HATCH_X[0] - 2.5, HATCH_X[1] + 2.5, -HATCH_Y - 1.5, HATCH_Y + 1.5, 5, 40)
             ) - box(HATCH_X[0] + 2, HATCH_X[1] - 2, -HATCH_Y + 1.5, HATCH_Y - 1.5, 0, 40)
    cuts = tube_bores() + wire_runs() + fin_slots() + joint_pins()
    centre_zone = box(TAIL_JOINT, NOSE_JOINT, -100, 100, -40, 40)
    centre_shell = shell & centre_zone
    hatch = centre_shell & hatch_opening(0.3)
    centre = (centre_shell + (sockets & centre_zone) + (ledge & centre_zone)
              + collar(NOSE_JOINT, 1) + collar(TAIL_JOINT, -1) - hatch_opening() - cuts)
    # nose: flight-controller tray (transmitter on the floor beneath it) and the chin camera cradle
    nose_zone = box(NOSE_JOINT, 200, -100, 100, -40, 40)
    fc_tray = box(NOSE_JOINT + 7, NOSE_JOINT + 38, -45, 45, -3.0, -2.0) & cavity
    rails = both(box(CAM[0] - 9, CAM[0] + 9, CAM_W / 2 + 0.2, CAM_W / 2 + 2.2, -40, 40)) & cavity
    screws = rod((CAM[0], -20, CAM[2]), (CAM[0], 20, CAM[2]), 1.1)
    window = rod(CAM, cam_axis(45), 5.5)
    antenna = bd.Pos(VTX_ANT_X, 0, -30) * bd.Cylinder(VTX_ANT_D / 2, 30, align=(C, C, bd.Align.MIN))
    nose = (shell & nose_zone) + fc_tray + rails - screws - window - antenna - cuts
    tail_zone = box(-200, TAIL_JOINT, -100, 100, -40, 40)
    tail = (shell & tail_zone) + (sockets & tail_zone) - cuts
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
    tabs = box(xc - 16.5, xc + 16.5, y0 + 15.5, y0 + 18.5, -30, top)
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
    riser = bd.Pos(line_x(WIRE_FRAC, MOTOR_Y) + 2, MOTOR_Y - 2, 0.5) * bd.Cylinder(3.0, 20, align=(C, C, bd.Align.MIN))
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


# ---- pods --------------------------------------------------------------------------------------
def pod_right(full):
    x_cone = le_x(MOTOR_Y) - 8.0           # the tube runs back to just behind the leading edge
    x_end = x_cone - 53.0

    def body(r_tube, r_tip, x_front):
        tube = bd.Pos((x_cone + x_front) / 2, MOTOR_Y, MOTOR_Z) * along_x(bd.Cylinder(r_tube, x_front - x_cone))
        cone = bd.Pos((x_end + x_cone) / 2, MOTOR_Y, MOTOR_Z) * along_x(bd.Cone(r_tip, r_tube, x_cone - x_end))
        return tube + cone

    shell = body(POD_R, 3.0, FIREWALL_X) - body(POD_R - POD_WALL, 3.0 - POD_WALL, FIREWALL_X - FIREWALL_T)
    holes = bd.Pos(FIREWALL_X - 5, MOTOR_Y, MOTOR_Z) * along_x(bd.Cylinder(4.5, 12))
    r0, r1 = MOUNT_SLOT
    slot = along_x(bd.Box(r1 - r0, 3.3, 12)) + along_x(bd.Pos(0, 0, 0) * bd.Cylinder(1.65, 12))
    for k in range(4):
        radial = bd.Pos(0, (r0 + r1) / 2, 0) * bd.Box(12, r1 - r0, 3.3)
        ends = bd.Pos(0, r0, 0) * along_x(bd.Cylinder(1.65, 12)) + bd.Pos(0, r1, 0) * along_x(bd.Cylinder(1.65, 12))
        holes += bd.Pos(FIREWALL_X - 5, MOTOR_Y, MOTOR_Z) * (radial + ends).rotate(bd.Axis.X, 90 * k)
    lead = bd.Pos(0, 9.0, 0) * along_x(bd.Cylinder(2.3, 12))
    holes += bd.Pos(FIREWALL_X - 5, MOTOR_Y, MOTOR_Z) * lead.rotate(bd.Axis.X, 225)
    return shell - holes - full


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
    """Bought parts, for checking fits: not printed."""
    parts = []
    for side in (1, -1):
        m = bd.Pos(FIREWALL_X + MOTOR_L / 2, side * MOTOR_Y, MOTOR_Z) * along_x(bd.Cylinder(MOTOR_D / 2, MOTOR_L))
        parts.append(named(m, "motor", "#15171a"))
        prop = bd.Pos(FIREWALL_X + MOTOR_L + 6, side * MOTOR_Y, MOTOR_Z) * (
            along_x(bd.Cylinder(6, 8)) + bd.Box(1.6, PROP_D, 12).rotate(bd.Axis.Y, 14)).rotate(bd.Axis.X, 25 * side)
        parts.append(named(prop, "prop_7x6", "#c8ccd0"))
        y_tip = HALF - 4.5
        spar = rod((line_x(SPAR_FRAC, SPAR_Y0 + 1), side * (SPAR_Y0 + 1), 0), (line_x(SPAR_FRAC, y_tip), side * y_tip, 0), 3.0)
        stub = rod((line_x(STUB_FRAC, STUB_Y[0]), side * STUB_Y[0], 0), (line_x(STUB_FRAC, STUB_Y[1]), side * STUB_Y[1], 0), 3.0)
        parts += [named(spar, "spar_tube", "#101010"), named(stub, "stub_tube", "#101010")]
        xc, y0 = line_x(SERVO_FRAC, SERVO_Y), SERVO_Y - SERVO[1] / 2
        zl = surf(SERVO_Y, SERVO_FRAC)[0]
        servo = box(xc - 11.5, xc + 11.5, y0 + 0.3, y0 + 24.3, zl + 0.1, zl + 11.6)
        parts.append(named(servo if side > 0 else servo.mirror(bd.Plane.XZ), "servo_es08ma", "#1f4e9c"))
    for dy in (-CELL_D, 0, CELL_D):
        cell = bd.Pos(BATTERY_X, dy, 0) * along_x(bd.Cylinder(CELL_D / 2, CELL_L))
        parts.append(named(cell, "cell_21700", "#2a9d8f"))
    parts.append(named(box(NOSE_JOINT + 8, NOSE_JOINT + 37, -15, 15, -2, 6), "flight_controller", "#2d6a4f"))
    parts.append(named(box(NOSE_JOINT + 9, NOSE_JOINT + 33, -11, 11, -10.5, -5.5), "video_transmitter", "#e76f51"))
    cam = bd.Box(10, CAM_W, CAM_W) + bd.Pos(9, 0, 0) * along_x(bd.Cylinder(4.5, 8))
    parts.append(named(bd.Pos(*CAM) * cam.rotate(bd.Axis.Y, CAM_TILT), "fpv_camera_14mm", "#b5179e"))
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
