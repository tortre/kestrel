"""Kestrel-600, printable parts (v1, untested prints).

Frame: +X nose, Y span, +Z up in forward flight; parts are modelled where they sit in the aircraft.
Assumed printer: 220 x 220 x 250 mm. Wings, elevons, fuselage and fins in foaming LW-PLA;
motor pods in PETG or PLA+ (LW-PLA softens next to a warm motor).

Structure: one 6 mm carbon tube per wing along the 30 % chord line, the two meeting in a V block in the
fuselage under the battery tray, plus a short 6 mm stub at 62 % chord to stop the wing twisting.
Both come from two 500 mm tubes: 2 x 325 mm spars and 2 x 105 mm stubs.
"""
import math

import build123d as bd
from cad import model

# ---- planform (from sim/optimize.py) -------------------------------------------------------
HALF = 302.0                      # wing tip station
FIN_T = 2.0
ROOT_LE, ROOT_CHORD = 70.0, 160.0
TIP_LE, TIP_CHORD = -60.0, 100.0
THICKNESS = 0.11
HINGE = 0.65
ELEVON_Y = (62.0, 292.0)
SPLIT_Y = 170.0                   # inner / outer wing segment joint

# ---- structure and fits ----------------------------------------------------------------------
SPAR_FRAC, STUB_FRAC, WIRE_FRAC, SERVO_FRAC = 0.30, 0.62, 0.10, 0.47
TUBE_BORE = 6.3                   # 6 mm carbon tube, slip fit
STUB_Y = (18.0, 120.0)
PIN_D = 2.0                       # 1.75 mm filament alignment pins
WALL = 0.8                        # fuselage shell: two 0.4 mm perimeters
HULL_Y = 43.0                     # fuselage half width at the wing

# ---- components ------------------------------------------------------------------------------
MOTOR_Y, MOTOR_Z = 150.0, 6.0     # pods raised so the thrust line passes through the (battery-high) CG
MOTOR_D, MOTOR_L = 27.0, 22.5     # SunnySky X2206
MOUNT_HOLES = ((8.0, 0.0), (-8.0, 0.0), (0.0, 9.5), (0.0, -9.5))   # 16 x 19 mm M3 cross: MEASURE YOUR MOTOR
POD_R, POD_WALL, FIREWALL_X, FIREWALL_T = 14.0, 1.0, 43.0, 3.0
PROP_D = 177.8                    # 7 inch
SERVO = (23.6, 27.0, 11.7)        # EMAX ES08MA II on its side: length (x), height + boss (y), thickness (z)
SERVO_Y = 100.0
CELL_D, CELL_L = 21.0, 70.0       # 21700
TRAY_Z, TRAY_T = 4.5, 1.0         # battery tray underside and thickness; spars pass below it
BATTERY_X = -17.0
HATCH_X, HATCH_Y = (-60.0, 26.0), 34.0

# fuselage stations: x, width, height above and below the wing plane. Sections are rounded rectangles,
# lofted smoothly through the nose and tail; the centre section is straight so the hatch and joints stay true.
NOSE = [(172, 6, 3, 3), (158, 26, 10, 7), (130, 50, 18, 11), (78, 85, 30.6, 15.8), (60, 86, 31, 16)]
TAIL = [(-65, 86, 31, 16), (-80, 85, 30.6, 15.8), (-120, 54, 18, 10), (-165, 22, 6, 5), (-180, 7, 2.5, 2.5)]
HULL_TOP = 31.0
NOSE_JOINT, TAIL_JOINT = 60.0, -65.0

C = bd.Align.CENTER


# ---- geometry helpers ------------------------------------------------------------------------
def le_x(y):
    return ROOT_LE + (TIP_LE - ROOT_LE) * abs(y) / HALF


def chord(y):
    return ROOT_CHORD + (TIP_CHORD - ROOT_CHORD) * abs(y) / HALF


def line_x(frac, y):
    return le_x(y) - frac * chord(y)


def surf(y, s):
    """Lower and upper surface z at span station y and chord fraction s."""
    c = chord(y)
    yt = 5 * THICKNESS * (0.2969 * math.sqrt(s) - 0.1260 * s - 0.3516 * s**2 + 0.2843 * s**3 - 0.1015 * s**4)
    yc = 0.12 * s * (1 - s) * (0.75 - s)
    return c * (yc - yt), c * (yc + yt)


def airfoil_face(y):
    x0, n = le_x(y), 24
    upper, lower = [], []
    for i in range(n + 1):
        s = 0.5 * (1 - math.cos(math.pi * i / n))
        zl, zu = surf(y, s)
        upper.append((x0 - chord(y) * s, y, zu))
        lower.append((x0 - chord(y) * s, y, zl))
    top = bd.Spline(*reversed(upper))
    bottom = bd.Spline(*lower)
    return bd.Face(bd.Wire([top, bottom, bd.Line(lower[-1], upper[-1])]))


def box(x0, x1, y0, y1, z0, z1):
    return bd.Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * bd.Box(x1 - x0, y1 - y0, z1 - z0)


def rod(p0, p1, r):
    d = bd.Vector(*p1) - bd.Vector(*p0)
    return bd.Plane(origin=p0, z_dir=d.normalized()) * bd.Cylinder(r, d.length, align=(C, C, bd.Align.MIN))


def along_x(shape):
    return shape.rotate(bd.Axis.Y, 90)


def xz_face(y, pts):
    """A polygon given as (x, z) points, placed in the plane at span station y."""
    return bd.Plane(origin=(0, y, 0), x_dir=(1, 0, 0), z_dir=(0, -1, 0)) * bd.Polygon(*pts, align=None)


def frac_line(frac, y0, y1, z=0.0, r=TUBE_BORE / 2):
    return rod((line_x(frac, y0), y0, z), (line_x(frac, y1), y1, z), r)


def both(shape):
    return shape + shape.mirror(bd.Plane.XZ)


# ---- fuselage --------------------------------------------------------------------------------
def hull_face(station, inset=0.0):
    x, w, top, bot = station
    h = top + bot
    r = max(0.26 * min(w, h) - inset, 0.4)
    return bd.Plane.YZ.offset(x) * bd.Pos(0, (top - bot) / 2) * bd.RectangleRounded(w - 2 * inset, h - 2 * inset, r)


def hull_loft(inset=0.0):
    """The hull's outer surface moved inward by `inset` (0 is the outside)."""
    ok = lambda st: min(st[1], st[2] + st[3]) - 2 * inset > 3.0
    nose = bd.loft([hull_face(st, inset) for st in NOSE if ok(st)])
    tail = bd.loft([hull_face(st, inset) for st in TAIL if ok(st)])
    centre = bd.loft([hull_face(NOSE[-1], inset), hull_face(TAIL[0], inset)], ruled=True)
    return nose + centre + tail


def hull_outer():
    return hull_loft(0.0)


def collar(joint, direction):
    """Slip ring fixed inside the centre section, poking 6 mm into the neighbouring section."""
    bore = hull_loft(WALL + 1.2)
    a, b = sorted((joint - 5 * direction, joint))
    fixed = (hull_loft(WALL - 0.2) - bore) & box(a, b, -60, 60, -40, 40)
    a, b = sorted((joint, joint + 6 * direction))
    slip = (hull_loft(WALL + 0.2) - bore) & box(a, b, -60, 60, -40, 40)
    return fixed + slip


def hatch_opening(clear=0.0):
    return box(HATCH_X[0] + clear, HATCH_X[1] - clear, -HATCH_Y + clear, HATCH_Y - clear, 20, 36)


def wire_holes():
    """Where the wing's wire channel and servo groove meet the fuselage side."""
    esc = frac_line(WIRE_FRAC, 30, HULL_Y + 3, z=0.5, r=2.75)
    servo = rod((line_x(SERVO_FRAC, 40), 34, -5.0), (line_x(SERVO_FRAC, 40), HULL_Y + 3, -5.0), 2.5)
    return both(esc + servo)


def tube_bores():
    return both(frac_line(SPAR_FRAC, 0, HALF + 10) + frac_line(STUB_FRAC, *STUB_Y))


def fuselage_parts():
    outer, cavity = hull_outer(), hull_loft(WALL)
    shell = outer - cavity
    # centre section internals: V block for the spars, stub sockets, battery tray, hatch ledge
    sockets = both(frac_line(SPAR_FRAC, 0, HULL_Y + 3, r=4.9) + frac_line(STUB_FRAC, STUB_Y[0], HULL_Y + 3, r=4.9))
    tray = box(BATTERY_X - 45, BATTERY_X + 45, -45, 45, TRAY_Z, TRAY_Z + TRAY_T)
    ledge = (box(HATCH_X[0] - 2.5, HATCH_X[1] + 2.5, -HATCH_Y - 1.4, HATCH_Y + 1.4, HULL_TOP - WALL - 1.6, HULL_TOP - WALL - 0.4)
             - box(HATCH_X[0] + 2, HATCH_X[1] - 2, -HATCH_Y + 1.8, HATCH_Y - 1.8, 0, 40))
    internals = (sockets + tray + ledge) & cavity
    centre_zone = box(TAIL_JOINT, NOSE_JOINT, -60, 60, -40, 40)
    centre = (shell & centre_zone) + internals + collar(NOSE_JOINT, 1) + collar(TAIL_JOINT, -1)
    hatch = (shell & centre_zone) & hatch_opening(0.3)
    centre = centre - hatch_opening() - tube_bores() - wire_holes()
    # nose: electronics tray (flight controller, receiver, GPS on foam tape)
    fc_tray = box(NOSE_JOINT + 7, 125, -45, 45, -1.0, 0) & cavity
    nose = (shell & box(NOSE_JOINT, 200, -60, 60, -40, 40)) + fc_tray
    tail = shell & box(-200, TAIL_JOINT, -60, 60, -40, 40)
    return dict(nose=nose, centre=centre, tail=tail, hatch=hatch, outer=outer)


# ---- wing ------------------------------------------------------------------------------------
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
    assert zu - top >= 1.5, f"servo pocket leaves only {zu - top:.1f} mm of skin"
    y0 = SERVO_Y - SERVO[1] / 2
    body = box(xc - SERVO[0] / 2, xc + SERVO[0] / 2, y0, y0 + SERVO[1], -30, top)
    tabs = box(xc - 16.5, xc + 16.5, y0 + 15.5, y0 + 18.5, -30, top)
    # lead groove in the lower surface from the servo to the fuselage, taped over after fitting
    z_groove = surf(65, SERVO_FRAC)[0] + 3.0
    strip = bd.Polygon((line_x(SERVO_FRAC, 38) + 1.75, 38), (line_x(SERVO_FRAC, y0 + 1) + 1.75, y0 + 1),
                       (line_x(SERVO_FRAC, y0 + 1) - 1.75, y0 + 1), (line_x(SERVO_FRAC, 38) - 1.75, 38), align=None)
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
    wires = frac_line(WIRE_FRAC, 36, MOTOR_Y, z=0.5, r=2.75)
    riser = bd.Pos(line_x(WIRE_FRAC, MOTOR_Y) + 2, MOTOR_Y - 2, 0.5) * bd.Cylinder(3.0, 20, align=(C, C, bd.Align.MIN))
    spar = frac_line(SPAR_FRAC, 0, HALF + 10)
    stub = frac_line(STUB_FRAC, *STUB_Y)
    solid = (full - elevon_prism(0) - hull_outer() - spar - stub - wires - riser - servo_pocket()
             - pin_holes(SPLIT_Y) - pin_holes(HALF))
    inner = solid & box(-300, 200, 0, SPLIT_Y, -50, 50)
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


# ---- fins, pods ------------------------------------------------------------------------------
def fin_right():
    profile = bd.Polygon((-55, 0), (-150, 60), (-200, 60), (-185, 0), (-200, -60), (-150, -60), align=None)
    profile = bd.fillet(profile.vertices(), radius=9.0)
    plate = bd.Pos(0, HALF + FIN_T, 0) * bd.extrude(profile, FIN_T).rotate(bd.Axis.X, 90)
    return plate - frac_line(SPAR_FRAC, HALF - 5, HALF + 10) - pin_holes(HALF, depth=8.0)


def pod_right(full):
    """Streamlined pod: a tube the motor's own diameter, with a tail cone that fades into the wing."""
    x_cone, x_end = -5.0, -58.0

    def body(r_tube, r_tip, x_front):
        tube = bd.Pos((x_cone + x_front) / 2, MOTOR_Y, MOTOR_Z) * along_x(bd.Cylinder(r_tube, x_front - x_cone))
        cone = bd.Pos((x_end + x_cone) / 2, MOTOR_Y, MOTOR_Z) * along_x(bd.Cone(r_tip, r_tube, x_cone - x_end))
        return tube + cone

    shell = body(POD_R, 3.0, FIREWALL_X) - body(POD_R - POD_WALL, 3.0 - POD_WALL, FIREWALL_X - FIREWALL_T)
    holes = bd.Pos(FIREWALL_X - 5, MOTOR_Y, MOTOR_Z) * along_x(bd.Cylinder(4.5, 12))
    for dy, dz in MOUNT_HOLES:
        holes += bd.Pos(FIREWALL_X - 5, MOTOR_Y + dy, MOTOR_Z + dz) * along_x(bd.Cylinder(1.65, 12))
    holes += bd.Pos(FIREWALL_X - 5, MOTOR_Y, MOTOR_Z - 10.0) * along_x(bd.Cylinder(2.3, 12))     # motor leads
    return shell - holes - full


# ---- parts -----------------------------------------------------------------------------------
def named(shape, label, color):
    shape.label, shape.color = label, color
    return shape


LW, PETG = "#d9d4c7", "#3a3f45"


@model
def fuselage_nose():
    return named(fuselage_parts()["nose"], "fuselage_nose", LW)


@model
def fuselage_centre():
    return named(fuselage_parts()["centre"], "fuselage_centre", LW)


@model
def fuselage_tail():
    return named(fuselage_parts()["tail"], "fuselage_tail", LW)


@model
def battery_hatch():
    return named(fuselage_parts()["hatch"], "battery_hatch", LW)


@model
def wing_inner_right():
    return named(wing_right()[0], "wing_inner_right", LW)


@model
def wing_inner_left():
    return named(wing_right()[0].mirror(bd.Plane.XZ), "wing_inner_left", LW)


@model
def wing_outer_right():
    return named(wing_right()[1], "wing_outer_right", LW)


@model
def wing_outer_left():
    return named(wing_right()[1].mirror(bd.Plane.XZ), "wing_outer_left", LW)


@model
def elevon_right_part():
    return named(elevon_right(wing_full()), "elevon_right", "#8a9199")


@model
def elevon_left_part():
    return named(elevon_right(wing_full()).mirror(bd.Plane.XZ), "elevon_left", "#8a9199")


@model
def tip_fin_right():
    return named(fin_right(), "tip_fin_right", LW)


@model
def tip_fin_left():
    return named(fin_right().mirror(bd.Plane.XZ), "tip_fin_left", LW)


@model
def motor_pod_right():
    return named(pod_right(wing_full()), "motor_pod_right", PETG)


@model
def motor_pod_left():
    return named(pod_right(wing_full()).mirror(bd.Plane.XZ), "motor_pod_left", PETG)


PRINTED = [fuselage_nose, fuselage_centre, fuselage_tail, battery_hatch, wing_inner_right, wing_inner_left,
           wing_outer_right, wing_outer_left, elevon_right_part, elevon_left_part, tip_fin_right, tip_fin_left,
           motor_pod_right, motor_pod_left]


def hardware():
    """Bought parts, for checking fits: not printed."""
    parts = []
    for side in (1, -1):
        m = bd.Pos(FIREWALL_X + MOTOR_L / 2, side * MOTOR_Y, MOTOR_Z) * along_x(bd.Cylinder(MOTOR_D / 2, MOTOR_L))
        parts.append(named(m, "motor", "#15171a"))
        prop = bd.Pos(FIREWALL_X + MOTOR_L + 6, side * MOTOR_Y, MOTOR_Z) * (
            along_x(bd.Cylinder(6, 8)) + bd.Box(1.6, PROP_D, 12).rotate(bd.Axis.Y, 14)).rotate(bd.Axis.X, 25 * side)
        parts.append(named(prop, "prop_7x6", "#c8ccd0"))
        y_tip = HALF + FIN_T - 1.2
        spar = rod((line_x(SPAR_FRAC, 0), 0, 0), (line_x(SPAR_FRAC, y_tip), side * y_tip, 0), 3.0)
        stub = rod((line_x(STUB_FRAC, STUB_Y[0]), side * STUB_Y[0], 0), (line_x(STUB_FRAC, STUB_Y[1]), side * STUB_Y[1], 0), 3.0)
        parts += [named(spar, "spar_tube", "#101010"), named(stub, "stub_tube", "#101010")]
        xc, y0 = line_x(SERVO_FRAC, SERVO_Y), SERVO_Y - SERVO[1] / 2
        zl = surf(SERVO_Y, SERVO_FRAC)[0]
        servo = box(xc - 11.5, xc + 11.5, y0 + 0.3, y0 + 24.3, zl + 0.1, zl + 11.6)
        parts.append(named(servo if side > 0 else servo.mirror(bd.Plane.XZ), "servo_es08ma", "#1f4e9c"))
    for dy in (-CELL_D, 0, CELL_D):
        cell = bd.Pos(BATTERY_X, dy, TRAY_Z + TRAY_T + CELL_D / 2) * along_x(bd.Cylinder(CELL_D / 2, CELL_L))
        parts.append(named(cell, "cell_21700", "#2a9d8f"))
    fc = box(75, 115, -15, 15, 0, 8)
    parts.append(named(fc, "flight_controller", "#2d6a4f"))
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
