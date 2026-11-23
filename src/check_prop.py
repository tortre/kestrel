"""Propeller disc clearance against the body.  Run: cad run src/check_prop.py"""
import build123d as bd
from cad.build123d import closest_points, overlap_volume

import printable as P

if __name__ == "__main__":
    disc = bd.Pos(P.FIREWALL_X + P.MOTOR_L + 6, P.MOTOR_Y, P.MOTOR_Z) * P.along_x(bd.Cylinder(P.PROP_D / 2, 14))
    pivot = bd.Axis((P.TILT_SHAFT_X, P.MOTOR_Y, P.MOTOR_Z), (0, 1, 0))
    for part in (P.body_nose, P.body_bay_fwd, P.wing_1_right):
        s = part().solids()[0]
        for ang in (0.0, P.TILT_MAX_DEG, -P.TILT_MAX_DEG):
            d = disc.rotate(pivot, ang) if ang else disc
            print(f"{part.__name__} at tilt {ang:+.0f} deg: overlap mm3 {overlap_volume(d, s):.1f}, gap mm {closest_points(d, s).distance:.1f}")
