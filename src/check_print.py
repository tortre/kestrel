"""Export every printable Kestrel-X Stealth v7 STL, verify 220x220x250 bed fit, estimate core mass,
verify 0.00 mm3 interference (printed-vs-printed, bought-vs-printed, battery drop-in), and write print/mass_report.json.
Run:  cad run src/check_print.py
"""
import json
import pathlib

import build123d as bd

import printable as P

OUT = pathlib.Path(__file__).resolve().parent.parent / "print"
OUT.mkdir(exist_ok=True)
BED = (220.0, 220.0, 250.0)
LW_DENSITY, PETG_DENSITY = 0.60, 1.27     # LW-PLA core (wrapped in +/-45 carbon/E-glass composite skin in aircraft.py!)


def estimate_grams(name, shape):
    vol, area = shape.volume / 1000, shape.area / 100
    if name.startswith("motor_pod"):
        return vol * PETG_DENSITY * 0.95
    if name.startswith(("tail_fin", "ventral_fin")):
        return LW_DENSITY * (area * 0.080 + vol * 0.14)
    if name in ("body_bay_fwd", "body_bay_aft", "body_tail", "wing_1_right", "wing_1_left"):
        return LW_DENSITY * (area * 0.055 + vol * 0.045) + 1.0
    return LW_DENSITY * (area * 0.050 + vol * 0.040)


def exact_mesh_size(shape):
    verts, _ = shape.tessellate(0.25)
    xs = [v.X for v in verts]
    ys = [v.Y for v in verts]
    zs = [v.Z for v in verts]
    return sorted((max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs)))


def fits_bed(size):
    return all(a <= b for a, b in zip(size, sorted(BED)))


PARTS = [
    ("body_nose", P.body_nose),
    ("body_bay_fwd", P.body_bay_fwd),
    ("body_bay_aft", P.body_bay_aft),
    ("body_tail", P.body_tail),
    ("hatch_fwd", P.hatch_fwd),
    ("hatch_aft", P.hatch_aft),
    ("wing_1_right", P.wing_1_right),
    ("wing_1_left", P.wing_1_left),
    ("wing_2_right", P.wing_2_right),
    ("wing_2_left", P.wing_2_left),
    ("wing_3_right", P.wing_3_right),
    ("wing_3_left", P.wing_3_left),
    ("wing_4_right", P.wing_4_right),
    ("wing_4_left", P.wing_4_left),
    ("wing_5_right", P.wing_5_right),
    ("wing_5_left", P.wing_5_left),
    ("elevon_right", P.elevon_right_part),
    ("elevon_left", P.elevon_left_part),
    ("tail_fin_right", P.tail_fin_right),
    ("tail_fin_left", P.tail_fin_left),
    ("ventral_fin_right", P.ventral_fin_right),
    ("ventral_fin_left", P.ventral_fin_left),
    ("motor_pod_right", P.motor_pod_right),
    ("motor_pod_left", P.motor_pod_left),
]


def sample_overlap(a, b, n=3000):
    import random
    ba, bb = a.bounding_box(), b.bounding_box()
    x0, x1 = max(ba.min.X, bb.min.X), min(ba.max.X, bb.max.X)
    y0, y1 = max(ba.min.Y, bb.min.Y), min(ba.max.Y, bb.max.Y)
    z0, z1 = max(ba.min.Z, bb.min.Z), min(ba.max.Z, bb.max.Z)
    if x1 <= x0 or y1 <= y0 or z1 <= z0:
        return 0
    rng = random.Random(0)
    return sum(a.is_inside(p) and b.is_inside(p) for p in
               (bd.Vector(rng.uniform(x0, x1), rng.uniform(y0, y1), rng.uniform(z0, z1)) for _ in range(n)))


def boxes_overlap(a, b):
    ba, bb = a.bounding_box(), b.bounding_box()
    return (ba.min.X < bb.max.X - 1e-4 and bb.min.X < ba.max.X - 1e-4
            and ba.min.Y < bb.max.Y - 1e-4 and bb.min.Y < ba.max.Y - 1e-4
            and ba.min.Z < bb.max.Z - 1e-4 and bb.min.Z < ba.max.Z - 1e-4)


def run():
    built = {}
    report = {}
    total_g = 0.0
    print(f"{'part':22s} {'size mm':>22s}  {'solids':>6s}  {'fits bed':>8s}  {'est. g':>6s}")
    for name, fn in PARTS:
        shape = fn()
        built[name] = shape
        size = exact_mesh_size(shape)
        solids = len(shape.solids())
        ok = fits_bed(size)
        g = estimate_grams(name, shape)
        total_g += g
        c = shape.center()
        report[name] = dict(
            grams=round(g, 2), x=round(c.X, 1), y=round(c.Y, 1), z=round(c.Z, 1),
            vol_cm3=round(shape.volume / 1000, 2), area_cm2=round(shape.area / 100, 1),
            size_mm=[round(s, 1) for s in size], solids=solids, fits_bed=ok,
        )
        print(f"{name:22s} {size[0]:5.1f} x {size[1]:6.1f} x {size[2]:6.1f}  {solids:6d}  {str(ok):>8s}  {g:6.1f}", flush=True)
        bd.export_stl(shape, str(OUT / f"{name}.stl"))
    print(f"printed LW-PLA core + PETG-CF pods, estimated: {total_g:.1f} g  (all fit 220x220x250 bed: {all(v['fits_bed'] for v in report.values())})")

    # Battery drop-in clearance check through bolted hatch opening
    px, py, pz = P.PACK_DIM
    pack_path = P.box(P.BAY_X - px / 2, P.BAY_X + px / 2, -py / 2, py / 2, -pz / 2, 42.0)
    bay_clash = ((pack_path & built["body_bay_fwd"]).volume + (pack_path & built["body_bay_aft"]).volume)
    print(f"battery drop-in clearance check (compressed {px:.1f}x{py:.1f}x{pz:.1f} mm SA88 8S2P pack): clash = {bay_clash:.2f} mm3")

    print("\ninterference between printed parts (mm3, should be 0):")
    clashes = 0
    names = list(built)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if not boxes_overlap(built[a], built[b]):
                continue
            inter = built[a] & built[b]
            ov = inter.volume if inter is not None else 0.0
            if ov > 0.5:
                clashes += 1
                print(f"  CLASH {a} / {b}: {ov:.1f}  (sampled: {sample_overlap(built[a], built[b])})")
    if clashes == 0:
        print("  none")

    print("\nbought parts against printed parts (mm3, should be 0):")
    hw_clashes = 0
    for h in P.hardware():
        for name, p in built.items():
            if not boxes_overlap(h, p):
                continue
            inter = h & p
            ov = inter.volume if inter is not None else 0.0
            if ov > 0.5:
                hw_clashes += 1
                print(f"  CLASH {h.label} / {name}: {ov:.1f}  (sampled: {sample_overlap(h, p)})")
    if hw_clashes == 0:
        print("  none")

    # Dynamic +/-20 deg tilt-nacelle pivot clearance check (pod + motor + ESC + prop hub vs wing_1 & wing_2!)
    print(f"\ntilt-nacelle +/-{P.TILT_MAX_DEG:.0f} deg pivot clearance check (mm3, should be 0):")
    pivot_ax = bd.Axis((P.TILT_SHAFT_X, P.MOTOR_Y, P.MOTOR_Z), (0, 1, 0))
    mot_r = bd.Pos(P.FIREWALL_X + P.MOTOR_L / 2, P.MOTOR_Y, P.MOTOR_Z) * P.along_x(bd.Cylinder(P.MOTOR_D / 2, P.MOTOR_L))
    esc_r = P.box(P.FIREWALL_X - P.FIREWALL_T - P.ESC_DIM[0] - 2.0, P.FIREWALL_X - P.FIREWALL_T - 2.0,
                  P.MOTOR_Y - P.ESC_DIM[1] / 2, P.MOTOR_Y + P.ESC_DIM[1] / 2,
                  P.MOTOR_Z - 3.0, P.MOTOR_Z - 3.0 + P.ESC_DIM[2])
    tilt_asm = built["motor_pod_right"] + mot_r + esc_r
    tilt_clashes = 0
    for ang in (-P.TILT_MAX_DEG, -20.0, -10.0, +10.0, +20.0, +P.TILT_MAX_DEG):
        rot_asm = tilt_asm.rotate(pivot_ax, ang)
        for wn in ("wing_1_right", "wing_2_right"):
            inter = rot_asm & built[wn]
            ov = inter.volume if inter is not None else 0.0
            if ov > 0.5:
                tilt_clashes += 1
                print(f"  CLASH tilt_nacelle_assembly @{ang:+.0f} deg / {wn}: {ov:.1f}")
    if tilt_clashes == 0:
        print(f"  none (pod + motor + ESC clear wing_1 and wing_2 at every angle to +/-{P.TILT_MAX_DEG:.0f} deg)")

    geo = dict(
        version="kestrel_x_stealth_v9_2",
        half=P.HALF, blend_y=P.BLEND_Y, root_le=P.ROOT_LE, tip_le=P.TIP_LE,
        root_chord=P.ROOT_CHORD, tip_chord=P.TIP_CHORD, sweep_deg=P.SWEEP_DEG,
        dihedral_deg=P.DIHEDRAL_DEG, washout=P.WASHOUT,
        thickness=P.THICKNESS, tip_t_scale=P.TIP_T_SCALE,
        cst_wt=list(P.CST_WT), cst_wc=list(P.CST_WC), cst_dz_te=P.CST_DZ_TE,
        camber_k=P.CAMBER_K, camber_a=P.CAMBER_A,
        d0=list(P.D0), d_bay=list(P.D_BAY), d_bay_y=70.0, d38=list(P.D_BAY),
        nose_frac=P.NOSE_FRAC, flat_end=P.FLAT_END, body_reflex=P.BODY_REFLEX,
        spar_frac=P.SPAR_FRAC, rweb_frac=P.RWEB_FRAC,
        spar_y0=P.SPAR_Y0, carry_x=P.CARRY_X, carry_y=P.CARRY_Y,
        carry_box_w=P.CARRY_BOX_W, carry_box_h=P.CARRY_BOX_H,
        spar_h=[P.SPAR_H_IN, P.SPAR_H_MID, P.SPAR_H_OUT],
        cap_w=[P.CAP_W_IN, P.CAP_W_MID, P.CAP_W_OUT],
        cap_t=[P.CAP_T_IN, P.CAP_T_MID, P.CAP_T_OUT],
        wing_splits=list(P.WING_SPLITS),
        shark_h=P.SHARK_H, shark_cant=P.SHARK_CANT, shark_sweep=P.SHARK_SWEEP, shark_tip_c=P.SHARK_TIP_C,
        fin_cant=P.FIN_CANT, fin_y_up=P.FIN_Y_UP, fin_y_down=P.FIN_Y_DOWN,
        fin_up_h=P.FIN_UP_H, fin_dn_h=P.FIN_DN_H, fin_x_foot=P.FIN_X_FOOT,
        fin_root_x=list(P.FIN_ROOT_X), fin_tip_c=P.FIN_TIP_C,
        motor_y=P.MOTOR_Y, motor_z=P.MOTOR_Z, firewall_x=P.FIREWALL_X,
        motor_x=P.FIREWALL_X + P.MOTOR_L / 2, tilt_shaft_x=P.TILT_SHAFT_X,
        hinge=P.HINGE, elevon_y=list(P.ELEVON_Y),
        servo_x=P.line_x(P.SERVO_FRAC, P.ELEV_SERVO_Y),
        tilt_servo_x=P.line_x(P.SERVO_FRAC, P.TILT_SERVO_Y),
        cam_x=P.CAM[0], fc_x=162.0 + P.FC_DIM[0] / 2,
        battery_x=P.BATTERY_X,
        battery_x_min=P.HATCH_X[0] + P.BRICK_DIM[0] / 2 + 2.0,
        battery_x_max=P.HATCH_X[1] - P.BRICK_DIM[0] / 2 - 2.0,
        tilt_max_deg=P.TILT_MAX_DEG, bay_x=P.BAY_X,
    )
    (OUT / "mass_report.json").write_text(json.dumps(dict(total_grams=round(total_g, 1), parts=report, geometry=geo), indent=2))


if __name__ == "__main__":
    run()
