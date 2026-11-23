"""Draw the airflow cases from sim/airflow.py.   .venv/Scripts/python sim/airflow_plot.py

Writes airflow/flow_<case>.png: streamlines coloured by local air speed, and the wing coloured by how
hard the air is pushing on it (pressure difference between the lower and upper skin).
"""
import json
import pathlib

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap, Normalize
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection

ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT = ROOT / "airflow"
BG, INK, INK2 = "#0b0d12", "#f0efec", "#c3c2b7"
SPEED = LinearSegmentedColormap.from_list("speed", ["#2a78d6", "#6da7ec", "#e9e9e6", "#f0a070", "#eb6834"])
LOAD = LinearSegmentedColormap.from_list("load", ["#0d366b", "#1c5cab", "#3987e5", "#86b6ef", "#e6f0fd"])
# name: (nose-down tilt of the picture in degrees, camera elevation, camera azimuth)
VIEW = {"cruise": (0, 28, -145), "slow": (0, 24, -150), "pullout": (0, 26, -140)}
KEEP = np.r_[0:29, 98:134]            # the layer of streamlines 30 mm above the wing, and the rings round each tip


def pitch(points, deg):
    a = np.radians(deg)
    r = np.array([[np.cos(a), 0, -np.sin(a)], [0, 1, 0], [np.sin(a), 0, np.cos(a)]])
    return points @ r.T


def draw(case, summary):
    d = np.load(OUT / f"flow_{case}.npz")
    tilt, elev, azim = VIEW[case]
    lines, speed, corners = pitch(d["lines"][KEEP, :400], tilt), d["speed"][KEEP, :400], pitch(d["corners"], tilt)
    pa = d["pressure_pa"]
    fig = plt.figure(figsize=(16, 9), facecolor=BG)
    ax = fig.add_axes([0.0, 0.0, 1.0, 1.0], projection="3d", facecolor=BG)
    ax.set_axis_off()
    ax.view_init(elev=elev, azim=azim)

    top = float(np.percentile(abs(pa), 98))
    signed = pa.min() < -0.25 * top
    norm_l = Normalize(-top, top) if signed else Normalize(0, top)
    cmap_l = LinearSegmentedColormap.from_list("load2", ["#eb6834", "#383835", "#3987e5"]) if signed else LOAD
    wing = Poly3DCollection(corners, facecolors=cmap_l(norm_l(pa)), edgecolors=(1, 1, 1, 0.05), linewidths=0.2)
    ax.add_collection3d(wing)

    span = max(0.04, float(np.percentile(abs(speed - 1), 99)))
    norm_s = Normalize(1 - span, 1 + span)
    seg = np.stack([lines[:, :-1], lines[:, 1:]], axis=2).reshape(-1, 2, 3)
    col = SPEED(norm_s(0.5 * (speed[:, :-1] + speed[:, 1:]).ravel()))
    for width, alpha in ((4.5, 0.10), (1.5, 0.95)):                 # a soft halo under each line
        c = col.copy()
        c[:, 3] = alpha
        ax.add_collection3d(Line3DCollection(seg, colors=c, linewidths=width))

    pts = np.concatenate([lines.reshape(-1, 3), corners.reshape(-1, 3)])
    lo, hi = pts.min(0), pts.max(0)
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(lo[2], hi[2])
    ax.set_box_aspect(hi - lo, zoom=1.7)

    s = summary[case]
    fig.text(0.035, 0.925, s["label"], color=INK, fontsize=24, fontweight="bold")
    fig.text(0.035, 0.885, f"Kestrel-560 · wing at {s['alpha_deg']:.1f}° to the air · lift {s['lift_N']:.1f} N · "
             f"peak push on the skin {s['peak_loading_pa']:.0f} Pa", color=INK2, fontsize=13)
    fig.text(0.035, 0.035, "Vortex-lattice potential flow (AeroSandbox): no boundary layer, separation, body thickness or prop wash.",
             color="#898781", fontsize=10)
    for x, cmap, norm, title, fmt in ((0.60, SPEED, norm_s, "air speed along the streamlines, × flight speed", "%.2f"),
                                      (0.80, cmap_l, norm_l, "pressure difference across the wing, Pa", "%.0f")):
        cax = fig.add_axes([x, 0.058, 0.16, 0.012])
        cb = fig.colorbar(plt.cm.ScalarMappable(norm, cmap), cax=cax, orientation="horizontal", format=fmt)
        cb.outline.set_visible(False)
        cb.ax.tick_params(colors=INK2, labelsize=9, length=0)
        cb.set_ticks(np.linspace(norm.vmin, norm.vmax, 3))
        cax.set_title(title, color=INK2, fontsize=9, loc="left", pad=5)
    fig.savefig(OUT / f"flow_{case}.png", dpi=160, facecolor=BG)
    plt.close(fig)
    print("wrote", OUT / f"flow_{case}.png")


if __name__ == "__main__":
    summary = json.loads((OUT / "flow_summary.json").read_text())
    for case in VIEW:                    # the zero-lift dive has nothing to show in potential flow, so it is not drawn
        draw(case, summary)
