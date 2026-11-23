"""Draw Kestrel-X from renders/views.json, and the spec strip from readiness/headline.json.

Run from the repo root (after `cad run renders/project_views.py`):
    .venv/Scripts/python renders/drawings.py

Obsera "Paper" style: Grey 300 with a light grain, fine ink lines, corner brackets,
one Signal Blue detail (the camera), the mark only.

Writes assets/drawing-planview.jpg, assets/drawing-threeview.jpg and assets/specs.png.
"""
import json
import pathlib

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = pathlib.Path(__file__).resolve().parents[1]
VIEWS = json.loads((ROOT / "renders" / "views.json").read_text())["views"]
OUT = ROOT / "assets"
FONTS = OUT / "fonts"

GREY300 = (198, 198, 196)
INK = (20, 20, 22)
BLUE = (22, 100, 255)
SS = 2                      # supersampling factor
CAM_X = 228.0               # camera station in the CAD (src/printable.py CAM)


def extent(lines):
    xs = [p[0] for l in lines for p in l]
    ys = [p[1] for l in lines for p in l]
    return min(xs), max(xs), min(ys), max(ys)


def ground(w, h, seed=7):
    """Grey 300 with a light film grain."""
    rng = np.random.default_rng(seed)
    base = np.empty((h, w, 3), np.float32)
    base[:] = GREY300
    grain = rng.normal(0.0, 7.0, (h, w, 1)).astype(np.float32)
    return Image.fromarray(np.clip(base + grain, 0, 255).astype(np.uint8), "RGB")


class Sheet:
    def __init__(self, w, h):
        self.w, self.h = w, h
        self.img = Image.new("RGBA", (w * SS, h * SS), (0, 0, 0, 0))
        self.d = ImageDraw.Draw(self.img)

    def line(self, pts, width, fill=INK):
        self.d.line([(x * SS, y * SS) for x, y in pts], fill=fill, width=max(1, round(width * SS)), joint="curve")

    def rect(self, x0, y0, x1, y1, fill):
        self.d.rectangle([x0 * SS, y0 * SS, x1 * SS, y1 * SS], fill=fill)

    def ring(self, cx, cy, r, width, fill=INK):
        self.d.ellipse([(cx - r) * SS, (cy - r) * SS, (cx + r) * SS, (cy + r) * SS], outline=fill, width=max(1, round(width * SS)))

    def dashed(self, a, b, dash, gap, width):
        (x0, y0), (x1, y1) = a, b
        length = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
        n = int(length // (dash + gap))
        for i in range(n + 1):
            t0 = i * (dash + gap) / length
            t1 = min(1.0, t0 + dash / length)
            self.line([(x0 + (x1 - x0) * t0, y0 + (y1 - y0) * t0), (x0 + (x1 - x0) * t1, y0 + (y1 - y0) * t1)], width)

    def brackets(self, x0, y0, x1, y1, arm, width):
        for cx, cy, sx, sy in ((x0, y0, 1, 1), (x1, y0, -1, 1), (x0, y1, 1, -1), (x1, y1, -1, -1)):
            self.line([(cx + sx * arm, cy), (cx, cy), (cx, cy + sy * arm)], width)

    def dimension(self, a, b, tick, width):
        """A plain dimension line with end ticks. No figure: the numbers live in the spec table."""
        (x0, y0), (x1, y1) = a, b
        self.line([a, b], width)
        if abs(y1 - y0) < 1e-6:
            for x in (x0, x1):
                self.line([(x, y0 - tick), (x, y0 + tick)], width)
        else:
            for y in (y0, y1):
                self.line([(x0 - tick, y), (x0 + tick, y)], width)

    def view(self, name, box, width, flip_x=False):
        """Fit a view into box = (x0, y0, x1, y1). Returns the mm -> px function."""
        lines = VIEWS[name]
        xa, xb, ya, yb = extent(lines)
        bx0, by0, bx1, by1 = box
        s = min((bx1 - bx0) / (xb - xa), (by1 - by0) / (yb - ya))
        ox = (bx0 + bx1) / 2 - s * (xa + xb) / 2 * (-1 if flip_x else 1)
        oy = (by0 + by1) / 2 + s * (ya + yb) / 2

        def to_px(x, y):
            return (ox + s * x * (-1 if flip_x else 1), oy - s * y)

        for l in lines:
            self.line([to_px(*p) for p in l], width)
        return to_px, s

    def mark(self, cx, cy, r, bar=INK):
        """The Obsera mark, built to the guide: ring 0.17r drawn inside, bar 0.84r x 0.20r, top 0.14r below centre."""
        self.ring(cx, cy, r, 0.17 * r)
        self.rect(cx - 0.42 * r, cy + 0.14 * r, cx + 0.42 * r, cy + 0.34 * r, bar)

    def save(self, path, size=None):
        top = self.img.resize((self.w, self.h), Image.LANCZOS)
        out = ground(self.w, self.h)
        out.paste(top, (0, 0), top)
        if size:
            out = out.resize(size, Image.LANCZOS)
        out.save(path, quality=90, optimize=True)
        print("wrote", path.relative_to(ROOT), out.size)


def camera(sheet, to_px, s):
    cx, cy = to_px(0.0, CAM_X)
    sheet.ring(cx, cy, 15 * s, 2.0)
    sheet.rect(cx - 6 * s, cy - 6 * s, cx + 6 * s, cy + 6 * s, BLUE)


def banner():
    w, h = 2400, 1200
    sh = Sheet(w, h)
    to_px, s = sh.view("top", (260, 250, 2140, 860), 3.2)
    xa, xb, ya, yb = extent(VIEWS["top"])
    cx = to_px(0, 0)[0]
    sh.dashed((cx, to_px(0, yb)[1] - 60), (cx, to_px(0, ya)[1] + 60), 16, 14, 1.6)
    camera(sh, to_px, s)
    sh.dimension((to_px(xa, 0)[0], 960), (to_px(xb, 0)[0], 960), 14, 1.8)
    sh.brackets(110, 110, w - 110, h - 110, 36, 3.0)
    r = h / 40.0
    sh.mark(w - 110 - 3 * r, h - 110 - 3 * r, r)
    sh.save(OUT / "drawing-planview.jpg")


def poster():
    w, h = 2400, 3000
    sh = Sheet(w, h)
    left, right = 330, 2070
    to_px, s = sh.view("top", (left, 420, right, 1100), 3.2)
    xa, xb, ya, yb = extent(VIEWS["top"])
    cx = to_px(0, 0)[0]
    sh.dashed((cx, to_px(0, yb)[1] - 70), (cx, to_px(0, ya)[1] + 70), 16, 14, 1.6)
    camera(sh, to_px, s)
    sh.dimension((to_px(xa, 0)[0], 1250), (to_px(xb, 0)[0], 1250), 16, 1.8)
    # front view at the same scale, under the plan view
    fa = extent(VIEWS["front"])
    fh = s * (fa[3] - fa[2])
    sh.view("front", (left, 1480, right, 1480 + fh), 3.2)
    # side view at the same scale, nose to the left
    sa = extent(VIEWS["side"])
    sw, shh = s * (sa[1] - sa[0]), s * (sa[3] - sa[2])
    y0 = 1480 + fh + 260
    sh.view("side", (cx - sw / 2, y0, cx + sw / 2, y0 + shh), 3.2)
    sh.dimension((cx - sw / 2, y0 + shh + 110), (cx + sw / 2, y0 + shh + 110), 16, 1.8)
    sh.brackets(120, 320, w - 120, 2520, 40, 3.2)
    r = 60
    sh.mark(w / 2, 2740, r, bar=BLUE)
    sh.save(OUT / "drawing-threeview.jpg")


def font(name, size, weight):
    f = ImageFont.truetype(str(FONTS / name), size * SS)
    f.set_variation_by_axes([14, weight] if name == "Inter.ttf" else [weight])
    return f


def specs():
    """The headline figures as one strip: mono labels, Inter figures. Every figure is read from the model output."""
    h = json.loads((ROOT / "readiness" / "headline.json").read_text(encoding="utf-8"))
    items = [
        ("SPAN", "1,880", "mm"),
        ("MASS", format(round(h["mass_g"]), ","), "g"),
        ("ENDURANCE", "%.0f" % h["loiter"]["min"], "min"),
        ("RANGE", "%.0f" % h["best_range"]["km"], "km"),
        ("TOP SPEED", "%.0f" % (h["vmax"] * 3.6), "km/h"),
        ("PARTS", "$%.0f" % h["cost"], ""),
    ]
    w, hh = 2400, 420
    sh = Sheet(w, hh)
    label, big, unit = font("GeistMono.ttf", 24, 400), font("Inter.ttf", 92, 600), font("GeistMono.ttf", 30, 400)
    left, col = 120, (w - 240) / len(items)
    sh.line([(left, 96), (w - left, 96)], 2.0)
    sh.line([(left, hh - 96), (w - left, hh - 96)], 2.0)
    for i, (k, v, u) in enumerate(items):
        x = left + i * col
        sh.d.text((x * SS, 132 * SS), k, font=label, fill=(95, 100, 112), features=["-liga"])
        sh.d.text((x * SS, 176 * SS), v, font=big, fill=INK, features=["tnum"])
        if u:
            vw = sh.d.textlength(v, font=big, features=["tnum"]) / SS
            sh.d.text(((x + vw + 12) * SS, 232 * SS), u, font=unit, fill=(95, 100, 112))
    sh.save(OUT / "specs.png")


if __name__ == "__main__":
    banner()
    poster()
    specs()
