"""Pressure maps and the lift / moment comparison from the SU2 runs.   .venv/Scripts/python sim/cfd_plot.py [tag]

Writes airflow/cfd_pressure_a<alpha>.png and readiness/cfd_summary.json.
"""
import csv
import json
import pathlib
import sys

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap, TwoSlopeNorm

ROOT = pathlib.Path(__file__).resolve().parent.parent
tag = sys.argv[1] if len(sys.argv) > 1 else "fine"
BG, INK, INK2 = "#0b0d12", "#f0efec", "#c3c2b7"
CMAP = LinearSegmentedColormap.from_list("cp", ["#eb6834", "#f0a070", "#9a9da3", "#6da7ec", "#2a78d6"][::-1])
MAC = 0.187


def surface(alpha):
    rows = list(csv.reader(open(ROOT / "cfd" / tag / f"a{alpha:g}" / "surface.csv")))
    head = [h.strip().strip('"') for h in rows[0]]
    a = np.array(rows[1:], float)
    return {k: a[:, i] for i, k in enumerate(head)}


def draw(alpha, cl):
    s = surface(alpha)
    x, y, z = s["x"], s["y"], s["z"]
    # SU2 writes density, momentum and energy scaled so the free stream has density 1 and speed 1 (Mach 0.2)
    p = 0.4 * (s["Energy"] - 0.5 * (s["Momentum_x"]**2 + s["Momentum_y"]**2 + s["Momentum_z"]**2) / s["Density"])
    cp = (p - 1 / (1.4 * 0.2**2)) / 0.5
    fig, axes = plt.subplots(1, 2, figsize=(16, 8.2), facecolor=BG)
    norm = TwoSlopeNorm(vmin=-1.2, vcenter=0, vmax=0.6)
    for ax, top, title in ((axes[0], True, "upper surface"), (axes[1], False, "lower surface")):
        # pick the outermost point in z at each (x, y) spot: sort so the wanted side is drawn last
        cell = 0.0045 if tag == "coarse" else 0.003
        key = np.round(x / cell).astype(np.int64) * 100000 + np.round(y / cell).astype(np.int64)
        order = np.lexsort((z if top else -z, key))               # within each cell, the wanted side comes last
        last = np.r_[key[order][1:] != key[order][:-1], True]
        keep = order[last]
        for sign in (1, -1):                                       # mirror to show the whole aircraft
            ax.scatter(sign * y[keep], -x[keep], c=cp[keep], cmap=CMAP, norm=norm, s=(cell / 0.0015) ** 2 * 2.6, linewidths=0, marker="s")
        ax.set_aspect("equal")
        ax.set_facecolor(BG)
        ax.set_axis_off()
        ax.set_title(title, color=INK2, fontsize=13)
    fig.suptitle(f"Pressure on the skin from 3D CFD, wing at {alpha:g}° to the air (lift coefficient {cl:.2f})",
                 color=INK, fontsize=20, fontweight="bold", x=0.03, ha="left", y=0.97)
    fig.text(0.03, 0.03, "SU2 inviscid (Euler) solution, 1.1 million cells, half model mirrored. Blue: suction (pulls the skin outward). Orange: pressure. No props, no friction.",
             color="#898781", fontsize=10)
    cax = fig.add_axes([0.78, 0.045, 0.18, 0.014])
    cb = fig.colorbar(plt.cm.ScalarMappable(norm, CMAP), cax=cax, orientation="horizontal")
    cb.outline.set_visible(False)
    cb.ax.tick_params(colors=INK2, labelsize=9, length=0)
    cb.set_ticks([-1.2, -0.6, 0, 0.6])
    cax.set_title("pressure coefficient", color=INK2, fontsize=9, loc="left", pad=5)
    out = ROOT / "airflow" / f"cfd_pressure_a{alpha:g}.png"
    fig.savefig(out, dpi=150, facecolor=BG)
    plt.close(fig)
    print("wrote", out)


if __name__ == "__main__":
    res = json.loads((ROOT / "cfd" / tag / "results.json").read_text())
    a = np.array([[r["alpha"], r["CL"], r["CMy"], r["CD"]] for r in res])
    slope = np.polyfit(a[:, 0], a[:, 1], 1)
    m = np.polyfit(a[:, 1], a[:, 2], 1)                           # CMy (per metre, about the CAD origin) against CL
    x_np_cad = m[0] * 1000                                        # SU2 x runs aft, so the neutral point is at CAD x = +slope (mm)
    summary = dict(tag=tag, points=res, cl_per_deg=float(slope[0]), cl_per_deg_incompressible=float(slope[0] * (1 - 0.2**2) ** 0.5),
                   alpha_zero_lift=float(-slope[1] / slope[0]), neutral_point_cad_x_mm=float(x_np_cad), cm0=float(m[1] / MAC))
    for x_cg in (8.4,):
        summary["static_margin_at_cg_8.4"] = float((x_cg - x_np_cad) / (MAC * 1000))
    (ROOT / "readiness" / f"cfd_summary_{tag}.json").write_text(json.dumps(summary, indent=1))
    print(json.dumps({k: v for k, v in summary.items() if k != "points"}, indent=1))
    for r in res:
        if r["alpha"] in (4.0, 8.0):
            draw(r["alpha"], r["CL"])
