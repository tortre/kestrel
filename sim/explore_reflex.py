"""Tune tip washout (nose-down twist) so the aircraft trims hands-off at cruise.  Run: .venv/Scripts/python sim/explore_reflex.py"""
import numpy as np
import blended as B

if __name__ == "__main__":
    print("washout deg   NP x   Cm at CL 0.43 (CG at 9.5 % margin)   best L/D   CL max")
    for w in (0.0, 2.0, 3.0, 4.0, 5.0, 6.0):
        B.GEO.update(washout=w)
        plane = B.airplane()
        x_np = B.neutral_point(plane)
        pol = B.polar(plane, x_np - 0.095 * plane.c_ref)
        cm = float(np.interp(0.43, pol[:, 1], pol[:, 3]))
        print(f"  {w:5.1f}      {-x_np * 1000:6.1f}   {cm:+.3f}                              {max(pol[2:, 1] / pol[2:, 2]):5.1f}    {pol[:, 1].max():.2f}")
