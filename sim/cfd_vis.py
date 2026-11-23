"""High-fidelity pictures of the 3D CFD (SU2 Euler, fine mesh, new 4-degree-washout shape) with PyVista / VTK.

Run:  .venv/Scripts/python sim/cfd_vis.py [tag] [alpha]        (defaults: vis 5)
Needs cfd/<tag>/a<alpha>/vol_solution.vtu (whole field: cfd_run.py ... --volume) and surface.vtu.

Writes
  airflow/cfd3d_*.png         rendered pictures
  airflow/cfd_vis.npz         surface Cp + streamlines in the CAD frame (metres, x forward, both halves) for Blender
  airflow/web/*.bin, web.json the same, packed for the interactive page
The CFD frame is the CAD frame turned 180 degrees about z (x aft, right half at y < 0), so CAD = (-x, -y, z).
"""
import json
import pathlib
import sys

import numpy as np
import pyvista as pv

ROOT = pathlib.Path(__file__).resolve().parent.parent
tag = sys.argv[1] if len(sys.argv) > 1 else "vis"
alpha = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
RUN = ROOT / "cfd" / tag / f"a{alpha:g}"
OUT = ROOT / "airflow"
pv.OFF_SCREEN = True

surf = pv.read(RUN / "surface.vtu").extract_surface(algorithm=None).triangulate()
vol = pv.read(RUN / "vol_solution.vtu")
vol.point_data["U"] = vol.point_data["Velocity"]
u_inf = float(np.median(np.linalg.norm(vol.point_data["U"][np.linalg.norm(vol.points, axis=1) > 1.5], axis=1)))
print("cells", vol.n_cells, "freestream speed (solver units)", round(u_inf, 4), "surface triangles", surf.n_cells)
vol.point_data["speed"] = np.linalg.norm(vol.point_data["U"], axis=1) / u_inf

# streamwise vorticity, for the wake slice
der = vol.compute_derivative(scalars="U", vorticity=True)
vol.point_data["wx"] = der.point_data["vorticity"][:, 0] / u_inf      # per metre, scaled by freestream

# ---- streamlines -------------------------------------------------------------------------------------
a = np.radians(alpha)
fs = np.array([np.cos(a), 0, np.sin(a)])                       # freestream direction in the CFD frame
seeds = []
for z in (-0.030, -0.006, 0.014):              # a rake upstream, across the half span
    for y in np.linspace(-0.315, -0.01, 26):
        seeds.append((-0.30, y, z))
for y in np.linspace(-0.300, -0.255, 10):                       # extra lines round the tip, to catch the vortex roll-up
    for z in np.linspace(-0.020, 0.018, 6):
        seeds.append((-0.30, y, z))
seeds = np.array(seeds)
seeds[:, 2] += (seeds[:, 0] + 0.30) * np.tan(a) - 0.30 * np.tan(a)    # rake lies along the oncoming flow
src = pv.PolyData(seeds)
lines = vol.streamlines_from_source(src, vectors="U", integration_direction="forward", max_length=1.8,
                                    initial_step_length=0.2, max_step_length=0.5, terminal_speed=1e-6,
                                    integrator_type=45, compute_vorticity=False)
lines.point_data["speed"] = np.linalg.norm(lines.point_data["U"], axis=1) / u_inf
print("streamlines", lines.n_lines, "points", lines.n_points)


def polylines(pd):
    """List of (points, speed) per line."""
    out, conn, i = [], pd.lines, 0
    while i < len(conn):
        n = conn[i]
        ids = conn[i + 1:i + 1 + n]
        if n > 10:
            out.append((pd.points[ids], pd.point_data["speed"][ids]))
        i += n + 1
    return out


def to_cad(p):
    return p * np.array([-1, -1, 1])


def mirror(p):
    return p * np.array([1, -1, 1])


# ---- exports for Blender and the web page ---------------------------------------------------------------
faces = surf.faces.reshape(-1, 4)[:, 1:]
pts_r = to_cad(surf.points)
cp = surf.point_data["Pressure_Coefficient"]
pts = np.r_[pts_r, mirror(pts_r)]
tri = np.r_[faces, faces[:, ::-1] + len(pts_r)]
cpp = np.r_[cp, cp]
pl = polylines(lines)
L = [to_cad(p) for p, _ in pl] + [mirror(to_cad(p)) for p, _ in pl]
S = [s for _, s in pl] * 2
np.savez_compressed(OUT / "cfd_vis.npz", pts=pts, tri=tri, cp=cpp, n=np.array([len(l) for l in L]),
                    lines=np.concatenate(L), speed=np.concatenate(S), alpha=alpha)

web = OUT / "web"
web.mkdir(exist_ok=True)
pts.astype("<f4").tofile(web / "surf_pts.bin")
tri.astype("<u4").tofile(web / "surf_tri.bin")
cpp.astype("<f4").tofile(web / "surf_cp.bin")
step = [l[::3] for l in L]
sp = [s[::3] for s in S]
np.concatenate(step).astype("<f4").tofile(web / "lines_pts.bin")
np.concatenate(sp).astype("<f4").tofile(web / "lines_speed.bin")
(web / "web.json").write_text(json.dumps(dict(alpha=alpha, n_pts=len(pts), n_tri=len(tri), line_n=[len(l) for l in step],
                                               cp_range=[float(np.percentile(cpp, 1)), float(np.percentile(cpp, 99))],
                                               speed_range=[float(np.percentile(np.concatenate(sp), 1)),
                                                            float(np.percentile(np.concatenate(sp), 99))])))

# ---- PyVista renders ------------------------------------------------------------------------------------
surf_full = pv.PolyData(pts, np.c_[np.full(len(tri), 3), tri].ravel())
surf_full.point_data["Cp"] = cpp
surf_full = surf_full.compute_normals(auto_orient_normals=False, split_vertices=False)
line_full = pv.MultipleLines if False else None
lp = []
for p, s in zip(L, S):
    poly = pv.lines_from_points(p)
    poly.point_data["speed"] = s
    lp.append(poly)
line_full = pv.merge(lp)
sp_lo, sp_hi = np.percentile(line_full.point_data["speed"], [2, 98])
half = max(abs(sp_lo - 1), abs(sp_hi - 1))
cp_lim = (-0.9, 0.6)
BG = ("#05070c", "#141b2a")


def plotter():
    p = pv.Plotter(off_screen=True, window_size=(3200, 1800), lighting="none")
    p.set_background(BG[0], top=BG[1])
    p.add_light(pv.Light(position=(1.5, -2.0, 3.0), focal_point=(0, 0, 0), intensity=0.9, light_type="scene light"))
    p.add_light(pv.Light(position=(-2.0, 1.5, 1.0), focal_point=(0, 0, 0), intensity=0.45, color="#9fc4ff", light_type="scene light"))
    p.add_light(pv.Light(position=(0.5, 0.5, -3.0), focal_point=(0, 0, 0), intensity=0.25, light_type="scene light"))
    p.enable_anti_aliasing("ssaa")
    return p


def bar(title):
    return dict(title=title, color="#d8dee9", title_font_size=40, label_font_size=32, n_labels=5,
                vertical=True, position_x=0.92, position_y=0.18, height=0.64, width=0.035, fmt="%.2f")


def surface(p):
    p.add_mesh(surf_full, scalars="Cp", cmap="RdBu_r", clim=cp_lim, smooth_shading=True, specular=0.4,
               specular_power=30, ambient=0.32, scalar_bar_args=bar("Cp\n(blue = suction)"))


def shot(p, name, pos, focal=(0.02, 0, 0), up=(0, 0, 1), zoom=1.0, text=None):
    p.camera_position = [pos, focal, up]
    p.camera.zoom(zoom)
    if text:
        p.add_text(text, position="upper_left", font_size=26, color="#d8dee9")
    p.screenshot(OUT / f"cfd3d_{name}.png")
    p.close()
    print("wrote", f"cfd3d_{name}.png")


lbl = f"Kestrel-560, 3D Euler CFD (SU2), {alpha:g} deg angle of attack, fine mesh"

# 1. surface pressure from above, front three-quarter
p = plotter()
surface(p)
shot(p, "pressure_top", (0.55, -0.55, 0.65), zoom=1.25, text=lbl + "\nSurface pressure from above")

# 2. surface pressure from below
p = plotter()
surface(p)
shot(p, "pressure_under", (0.45, -0.55, -0.6), zoom=1.25, text=lbl + "\nSurface pressure from below")

# 3. streamlines coloured by local speed, over the Cp surface
p = plotter()
surface(p)
p.add_mesh(line_full, scalars="speed", cmap="turbo", clim=(1 - half, 1 + half), line_width=3, render_lines_as_tubes=True,
           scalar_bar_args=dict(bar("local speed /\nflight speed"), position_x=0.04, position_y=0.12))
shot(p, "streamlines", (-0.75, -0.95, 0.55), focal=(-0.15, 0, 0), zoom=1.05, text=lbl + "\nStreamlines coloured by local air speed")

# 4. the wake: streamwise vorticity on a plane 120 mm behind the wing tip trailing edge, plus the lines
x_tip_te = float(surf.points[:, 0].max())
planes = []
for dx in (0.03, 0.15, 0.35):
    sl = vol.slice(normal="x", origin=(x_tip_te + dx, 0, 0))
    sl = sl.clip_box((x_tip_te + dx - 0.01, x_tip_te + dx + 0.01, -0.42, 0.0, -0.12, 0.10), invert=False)
    planes.append(sl)
wake = pv.merge(planes)
wake.points[:] = to_cad(wake.points)
wake2 = wake.copy()
wake2.points[:] = mirror(wake2.points)
wake2.point_data["wx"] = -wake2.point_data["wx"]
wk = pv.merge([wake, wake2])
wlim = float(np.percentile(abs(wk.point_data["wx"]), 99.5))
p = plotter()
surface(p)
p.add_mesh(wk, scalars="wx", cmap="coolwarm", clim=(-wlim, wlim), opacity=[1, 0.9, 0.5, 0.0, 0.5, 0.9, 1],
           scalar_bar_args=dict(bar("streamwise\nvorticity, 1/m"), position_x=0.04, position_y=0.12, fmt="%.0f"))
pass
shot(p, "wake_vortices", (-1.0, -0.55, 0.45), focal=(-0.30, 0, 0), zoom=1.1,
     text=lbl + "\nTip vortices: spin of the air in three planes behind the wing")
print("cp range", float(cpp.min()), float(cpp.max()), "vorticity 99.5 %", wlim)
