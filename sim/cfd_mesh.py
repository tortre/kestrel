"""Tetrahedral mesh round the half aircraft for SU2.   .venv/Scripts/python sim/cfd_mesh.py [wall_mm]

Reads cfd/half.step (from src/cfd_shape.py), writes cfd/half.su2.
SU2 frame: x downstream, z up, metres, so the CAD shape is turned half a turn about z (the half model
then lies at y <= 0 with the symmetry plane at y = 0).
"""
import pathlib
import sys

import gmsh

ROOT = pathlib.Path(__file__).resolve().parent.parent
wall = float(sys.argv[1]) / 1000 if len(sys.argv) > 1 else 0.0045
FAR = 10.0

gmsh.initialize()
gmsh.option.setNumber("General.NumThreads", 16)
occ = gmsh.model.occ
occ.importShapes(str(ROOT / "cfd" / "half.step"))
body = occ.getEntities(3)
occ.dilate(body, 0, 0, 0, 0.001, 0.001, 0.001)
occ.rotate(body, 0, 0, 0, 0, 0, 1, 3.141592653589793)
box = occ.addBox(-FAR, -FAR, -FAR, 2.4 * FAR, FAR, 2 * FAR)
fluid, _ = occ.cut([(3, box)], body)
occ.synchronize()

walls, sym, far = [], [], []
for dim, tag in gmsh.model.getEntities(2):
    x0, y0, z0, x1, y1, z1 = gmsh.model.getBoundingBox(dim, tag)
    if max(abs(x0), abs(x1), abs(z0), abs(z1), abs(y0)) < 1.8 and not (abs(y0) < 1e-6 and abs(y1) < 1e-6 and x1 - x0 > 2.5):
        (walls if not (abs(y0) < 1e-6 and abs(y1) < 1e-6) else sym).append(tag)
    elif abs(y0) < 1e-6 and abs(y1) < 1e-6:
        sym.append(tag)
    else:
        far.append(tag)
print("faces: wall", len(walls), "symmetry", len(sym), "farfield", len(far))
gmsh.model.addPhysicalGroup(2, walls, name="wall")
gmsh.model.addPhysicalGroup(2, sym, name="symmetry")
gmsh.model.addPhysicalGroup(2, far, name="farfield")
gmsh.model.addPhysicalGroup(3, [t for _, t in fluid], name="fluid")

f = gmsh.model.mesh.field
f.add("Distance", 1)
f.setNumbers(1, "SurfacesList", walls)
f.setNumber(1, "Sampling", 80)
f.add("Threshold", 2)
f.setNumber(2, "InField", 1)
f.setNumber(2, "SizeMin", wall)
f.setNumber(2, "SizeMax", 0.90)
f.setNumber(2, "DistMin", 0.006)
f.setNumber(2, "DistMax", 4.5)
f.add("Box", 3)                                  # fine mesh around the 2.12 m aircraft & wake
for k, v in dict(VIn=0.022, VOut=0.90, XMin=-0.45, XMax=1.35, YMin=-1.20, YMax=0, ZMin=-0.20, ZMax=0.26, Thickness=1.1).items():
    f.setNumber(3, k, v)
f.add("Min", 4)
f.setNumbers(4, "FieldsList", [2, 3])
f.setAsBackgroundMesh(4)
gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 18)
gmsh.option.setNumber("Mesh.MeshSizeMin", 0.0004)
gmsh.option.setNumber("Mesh.MeshSizeMax", 0.90)
gmsh.option.setNumber("Mesh.MeshSizeExtendFromBoundary", 0)
gmsh.option.setNumber("Mesh.MeshSizeFromPoints", 0)
gmsh.option.setNumber("Mesh.Algorithm", 6)
gmsh.option.setNumber("Mesh.Algorithm3D", 1)      # Delaunay 3D (robust on sharp stealth chines & thin NACA0006 fins)
gmsh.option.setNumber("Mesh.Optimize", 1)
gmsh.model.mesh.generate(3)
types, tags, _ = gmsh.model.mesh.getElements(3)
print("tetrahedra:", sum(len(t) for t in tags), "nodes:", len(gmsh.model.mesh.getNodes()[0]))
gmsh.option.setNumber("Mesh.SaveAll", 0)
gmsh.write(str(ROOT / "cfd" / "half.su2"))
gmsh.finalize()
