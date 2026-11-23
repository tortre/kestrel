"""Outer shape of the right half of Kestrel-X Stealth v7 as one clean solid, for 3D SU2 Euler CFD.
Run:  cad run src/cfd_shape.py

Writes cfd/half.step (mm).
"""
import pathlib

import build123d as bd

import printable as P

OUT = pathlib.Path(__file__).resolve().parent.parent / "cfd"
OUT.mkdir(exist_ok=True)
CUT = 4.0

body = P.body_outer()
wing = P.wing_full() + P.winglet_right()
x_aft = P.le_x(P.MOTOR_Y) - 24.0
x_cone = P.le_x(P.MOTOR_Y) + 14.0
pod = (bd.Pos((x_cone + P.FIREWALL_X) / 2, P.MOTOR_Y, P.MOTOR_Z) * P.along_x(bd.Cylinder(P.POD_R, P.FIREWALL_X - x_cone))
       + bd.Pos((x_aft + x_cone) / 2, P.MOTOR_Y, P.MOTOR_Z) * P.along_x(bd.Cone(5.0, P.POD_R, x_cone - x_aft))
       + bd.Pos(P.FIREWALL_X + P.MOTOR_L / 2, P.MOTOR_Y, P.MOTOR_Z) * P.along_x(bd.Cylinder(P.MOTOR_D / 2, P.MOTOR_L)))
fins = P.fin_plate(True) + P.fin_plate(False)
shape = body + wing + pod + fins

# Keep native 0.25% CST trailing edge (ZERO artificial 4 mm blunt rear step -> eliminates Euler thrust artifact!)
half = shape & P.box(-750, 420, 0, 1200, -240, 280)
solids = half.solids()
print("solids:", len(solids), "volume mm3:", round(half.volume), "bbox:", half.bounding_box())
bd.export_step(half if len(solids) > 1 else solids[0], str(OUT / "half.step"))
print("wrote", OUT / "half.step")
