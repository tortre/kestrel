"""Studio pictures of Kestrel-X.   cad run renders/render.py <shot> [--final]

Shots:  hero | top | front | rear
Model:  renders/kestrel_x.glb
        (cad step export STEP/kestrel_x_stealth_v9_under300.step renders/kestrel_x.glb --tolerance 0.05 --angle 6)
Writes  assets/render-<shot>.jpg: the aircraft, evenly lit, on plain Grey 300 with a light grain.
"""
import math
import sys
from pathlib import Path

import bpy
import numpy as np
from mathutils import Vector
from PIL import Image

from cad.blender import load

HERE = Path(__file__).resolve().parent
OUT = HERE.parent / "assets"
args = sys.argv[1:]
shot = args[0] if args and not args[0].startswith("-") else "hero"
final = "--final" in args

# camera direction (from the model towards the camera; x is the nose, z is up), lens, fill
SHOTS = {
    "hero":  ((1.0, -0.85, 0.55), 85, 0.60),
    "top":   ((0.0001, 0.0, 1.0), 100, 0.56),
    "front": ((1.0, 0.0, 0.16), 100, 0.54),
    "rear":  ((-1.0, -0.8, 0.5), 85, 0.60),
}
direction, lens, fill = SHOTS[shot]

bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
model = load(HERE / "kestrel_x.glb")
scene.render.resolution_x, scene.render.resolution_y = (2400, 1350) if final else (1200, 675)


def link(obj):
    scene.collection.objects.link(obj)
    return obj


aim = link(bpy.data.objects.new("aim", None))
aim.location = model.center
cam = link(bpy.data.objects.new("camera", bpy.data.cameras.new("camera")))
cam.data.lens = lens
view = math.tan(cam.data.angle / 2) * min(1, scene.render.resolution_y / scene.render.resolution_x)
cam.location = model.center + model.size.length / 2 / view * fill * Vector(direction).normalized()
cam.constraints.new("TRACK_TO").target = aim
cam.data.clip_start, cam.data.clip_end = 0.01, 500
scene.camera = cam

# even studio light: a bright grey surround, one large soft key above, one fill from the camera side
scene.world = bpy.data.worlds.new("world")
scene.world.use_nodes = True
bg = scene.world.node_tree.nodes["Background"]
bg.inputs["Color"].default_value = (0.75, 0.75, 0.74, 1.0)
bg.inputs["Strength"].default_value = 0.9
for name, loc, energy, size in (("key", (0.6, -0.8, 3.0), 260.0, 4.0), ("fill", (2.5, -2.0, 0.6), 60.0, 3.0)):
    light = link(bpy.data.objects.new(name, bpy.data.lights.new(name, "AREA")))
    light.data.energy, light.data.size = energy, size
    light.location = model.center + Vector(loc)
    light.constraints.new("TRACK_TO").target = aim

scene.render.film_transparent = True
scene.view_settings.view_transform = "Standard"
scene.render.image_settings.file_format = "PNG"
scene.render.image_settings.color_mode = "RGBA"
if final:
    scene.render.engine = "CYCLES"
    scene.cycles.samples = 192
    scene.cycles.use_denoising = True
    prefs = bpy.context.preferences.addons["cycles"].preferences
    for kind in ("OPTIX", "CUDA"):
        try:
            prefs.compute_device_type = kind
            prefs.get_devices()
            if any(d.type == kind for d in prefs.devices):
                for d in prefs.devices:
                    d.use = True
                scene.cycles.device = "GPU"
                break
        except Exception:
            pass
else:
    scene.render.engine = "BLENDER_EEVEE"

tmp = HERE / ("_%s.png" % shot)
scene.render.filepath = str(tmp)
bpy.ops.render.render(write_still=True)

# onto Grey 300 with a light grain, as the drawings
top = Image.open(tmp).convert("RGBA")
w, h = top.size
rng = np.random.default_rng(7)
ground = np.empty((h, w, 3), np.float32)
ground[:] = (198, 198, 196)
ground += rng.normal(0.0, 5.0, (h, w, 1))
sheet = Image.fromarray(np.clip(ground, 0, 255).astype(np.uint8), "RGB")
sheet.paste(top, (0, 0), top)
OUT.mkdir(exist_ok=True)
name = "render-%s%s.jpg" % (shot, "" if final else "-draft")
sheet.save(OUT / name, quality=90, optimize=True)
tmp.unlink()
print("wrote assets/" + name)
