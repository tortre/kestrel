"""Project the current airframe (src/printable.py) to 2D line work.

Run from the repo root:  cad run renders/project_views.py
Writes renders/views.json: visible edges of the top, front and side views as
polylines in millimetres. renders/drawings.py draws every picture from it,
so the drawings always follow the CAD.
"""
import json, pathlib, sys, time

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
import build123d as bd
import printable as P

t0 = time.time()
model = P.kestrel_x_stealth()
shape = model if isinstance(model, bd.Shape) else bd.Compound(children=list(model))
bb = shape.bounding_box()
print("built in %.0f s, bbox" % (time.time() - t0), bb.min, bb.max)

# view name: (camera position, camera up)
VIEWS = {
    "top":   ((0, 0, 5000), (1, 0, 0)),     # nose up the sheet
    "front": ((5000, 0, 0), (0, 0, 1)),
    "side":  ((0, -5000, 0), (0, 0, 1)),
}

def polyline(edge, tol=0.15):
    n = max(2, min(80, int(edge.length / 2.0) + 2))
    if edge.geom_type == bd.GeomType.LINE:
        n = 2
    pts = [edge.position_at(i / (n - 1)) for i in range(n)]
    return [[round(p.X, 2), round(p.Y, 2)] for p in pts]

out = {"bbox": [list(bb.min), list(bb.max)], "views": {}}
for name, (origin, up) in VIEWS.items():
    t = time.time()
    visible, hidden = shape.project_to_viewport(origin, viewport_up=up, look_at=(0, 0, 0))
    lines = [polyline(e) for e in visible if e.length > 0.3]
    out["views"][name] = lines
    print(name, len(lines), "edges, %.0f s" % (time.time() - t))

(ROOT / "renders" / "views.json").write_text(json.dumps(out))
print("wrote views.json")
