"""Kestrel-600: a 610 mm twin-tractor tailsitter VTOL flying wing (concept airframe).

Frame: +X is the nose, Y is the span, +Z is up, in forward flight. On the ground it
sits on the four fin tips with the nose pointing at the sky.
"""
import math

import build123d as bd
from cad import model

# --- planform -------------------------------------------------------------
SPAN = 610.0
FIN_T = 3.0
HALF = SPAN / 2 - FIN_T          # wing tip station; the fin closes the span
ROOT_LE, ROOT_CHORD = 70.0, 160.0   # chords from sim/optimize.py
TIP_LE, TIP_CHORD = -60.0, 100.0
THICKNESS = 0.11                 # airfoil thickness / chord
HINGE = 0.65                     # elevon hinge line: 35 % chord elevons for hover authority
ELEVON_Y = (62.0, 292.0)

# --- propulsion -----------------------------------------------------------
MOTOR_Y = 150.0
PROP_D = 177.8                   # 7 inch (APC 7x6E)
MOTOR_D, MOTOR_L = 27.0, 22.5    # SunnySky X2206
BATTERY_X = -18.0                # battery centre, set by the balance calculation

# --- fuselage sections: x, width, height above and below the wing plane ----
HULL = [
    (165, 8, 3, 3),
    (130, 44, 13, 14),
    (60, 86, 24, 24),
    (-65, 86, 24, 24),
    (-120, 50, 14, 12),
    (-175, 20, 4, 4),
]
CELL_D, CELL_L = 21.0, 70.0      # 21700 Li-ion


def le_x(y):
    return ROOT_LE + (TIP_LE - ROOT_LE) * abs(y) / HALF


def chord(y):
    return ROOT_CHORD + (TIP_CHORD - ROOT_CHORD) * abs(y) / HALF


def airfoil_face(y):
    """A reflexed section (flying wings need the reflex for pitch stability)."""
    c, x0, n = chord(y), le_x(y), 24
    upper, lower = [], []
    for i in range(n + 1):
        s = 0.5 * (1 - math.cos(math.pi * i / n))            # 0 at LE, 1 at TE
        yt = 5 * THICKNESS * (0.2969 * math.sqrt(s) - 0.1260 * s - 0.3516 * s**2
                              + 0.2843 * s**3 - 0.1015 * s**4)
        yc = 0.12 * s * (1 - s) * (0.75 - s)
        upper.append((x0 - c * s, y, c * (yc + yt)))
        lower.append((x0 - c * s, y, c * (yc - yt)))
    top = bd.Spline(*reversed(upper))                         # TE -> LE
    bottom = bd.Spline(*lower)                                # LE -> TE
    close = bd.Line(lower[-1], upper[-1])
    return bd.Face(bd.Wire([top, bottom, close]))


def hull_section(x, w, top, bot):
    pts = [(w / 2, 0), (0.32 * w, top), (-0.32 * w, top),
           (-w / 2, 0), (-0.32 * w, -bot), (0.32 * w, -bot)]
    return bd.Plane.YZ.offset(x) * bd.Polygon(*pts, align=None)


def along_x(shape):
    return shape.rotate(bd.Axis.Y, 90)


def propeller(angle):
    hub = along_x(bd.Cylinder(6, 8))
    blades = bd.Box(1.6, PROP_D, 11).rotate(bd.Axis.Y, 12)
    spinner = bd.Pos(9, 0, 0) * along_x(bd.Cone(6, 1.5, 10))
    return (hub + blades + spinner).rotate(bd.Axis.X, angle)


def both_sides(shape, label, color):
    right = shape
    left = shape.mirror(bd.Plane.XZ)
    for part, side in ((right, "right"), (left, "left")):
        part.label = f"{label}_{side}"
        part.color = color
    return [right, left]


@model
def kestrel_600():
    # Fuselage: faceted, chined hull
    hull_outer = bd.loft([hull_section(*s) for s in HULL], ruled=True)
    bay = bd.Pos(BATTERY_X, 0, 0) * bd.Box(CELL_L + 4, 3 * CELL_D + 3, CELL_D + 2)
    fuselage = hull_outer - bay

    # Wing half, then the elevon cut out of it
    wing_full = bd.loft([airfoil_face(0), airfoil_face(HALF)])
    y0, y1 = ELEVON_Y

    def hinge_x(y, back=0.0):
        return le_x(y) - HINGE * chord(y) - back

    slot = bd.extrude(bd.Polygon((hinge_x(y0), y0), (hinge_x(y1), y1),
                                 (-260, y1), (-260, y0), align=None), 40, both=True)
    inset = bd.extrude(bd.Polygon((hinge_x(y0 + 1, 1), y0 + 1), (hinge_x(y1 - 1, 1), y1 - 1),
                                  (-260, y1 - 1), (-260, y0 + 1), align=None), 40, both=True)
    elevon = wing_full & inset
    wing = wing_full - slot - hull_outer

    # Tip fins double as the landing gear: the rearmost points are the feet
    fin_profile = bd.Polygon((-55, 0), (-150, 72), (-205, 72), (-188, 0),
                             (-205, -72), (-150, -72), align=None)
    fin = bd.Pos(0, SPAN / 2, 0) * bd.extrude(fin_profile, FIN_T).rotate(bd.Axis.X, 90)

    # Motor pods on the leading edge, props well ahead of the wing
    pod = bd.Pos(10, MOTOR_Y, 0) * along_x(bd.Cylinder(11, 60)) - wing_full
    motor = bd.Pos(40 + MOTOR_L / 2, MOTOR_Y, 0) * along_x(bd.Cylinder(MOTOR_D / 2, MOTOR_L))
    prop_r = bd.Pos(68, MOTOR_Y, 0) * propeller(25)
    prop_l = bd.Pos(68, -MOTOR_Y, 0) * propeller(-25)
    prop_r.label, prop_l.label = "prop_right", "prop_left"
    prop_r.color = prop_l.color = "#c8ccd0"

    # Payload and battery
    sensor = bd.Pos(100, 0, -22) * bd.Sphere(14) - hull_outer
    antenna = bd.Pos(-95, 0, 14) * bd.Cylinder(1.5, 60, align=(bd.Align.CENTER,) * 2 + (bd.Align.MIN,))
    antenna = antenna - hull_outer
    cells = [bd.Pos(BATTERY_X, dy, 0) * along_x(bd.Cylinder(CELL_D / 2, CELL_L))
             for dy in (-CELL_D, 0, CELL_D)]
    battery = bd.Compound(children=cells, label="battery_3s_21700")
    for cell in cells:
        cell.label = "cell_21700"
        cell.color = "#2a9d8f"

    fuselage.label, fuselage.color = "fuselage", "#2b2f34"
    sensor.label, sensor.color = "sensor_ball", "#0c0d0f"
    antenna.label, antenna.color = "antenna", "#111111"

    parts = [fuselage, sensor, antenna, battery, prop_r, prop_l]
    parts += both_sides(wing, "wing", "#50565d")
    parts += both_sides(elevon, "elevon", "#8a9199")
    parts += both_sides(fin, "tip_fin", "#2b2f34")
    parts += both_sides(pod, "motor_pod", "#3a3f45")
    parts += both_sides(motor, "motor", "#15171a")
    uav = bd.Compound(children=parts, label="kestrel_600")

    size = uav.bounding_box().size
    assert size.Y <= 610.01, f"span {size.Y:.1f} mm exceeds 2 ft"
    assert size.X <= 610.0, f"length {size.X:.1f} mm exceeds 2 ft"
    return uav


if __name__ == "__main__":
    kestrel_600()
