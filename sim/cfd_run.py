"""Run SU2 (inviscid, Euler) on cfd/half.su2 at a few angles of attack and collect lift, drag and pitching moment.

Run:  .venv/Scripts/python sim/cfd_run.py <tag> <alpha> [<alpha> ...]
Writes cfd/<tag>/a<alpha>/ (SU2 output) and cfd/<tag>/results.json.

Inviscid means: pressure forces only. Lift, pitching moment and the neutral point are meaningful;
drag is induced drag plus numerical error, and there is no stall. Run at Mach 0.2 (the real aircraft
flies at Mach 0.05 to 0.15) because the compressible solver is poor below that; this raises the lift
slope by about 2 %.
"""
import csv
import json
import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
SU2 = ROOT / "tools" / "su2" / "bin" / "SU2_CFD.exe"
S_HALF = 0.3076 / 2            # m2, half of Kestrel-X Stealth v8 Budget reference area (sim/blended.py)

CFG = """
SOLVER= INC_EULER
MATH_PROBLEM= DIRECT
RESTART_SOL= NO
INC_DENSITY_INIT= 1.225
INC_VELOCITY_INIT= ( {vx}, 0.0, {vz} )
INC_NONDIM= INITIAL_VALUES
REF_ORIGIN_MOMENT_X= 0.0
REF_ORIGIN_MOMENT_Y= 0.0
REF_ORIGIN_MOMENT_Z= 0.0
REF_LENGTH= 1.0
REF_AREA= {area}
AOA= {alpha}
SIDESLIP_ANGLE= 0.0
MARKER_EULER= ( wall )
MARKER_SYM= ( symmetry )
MARKER_FAR= ( farfield )
MARKER_PLOTTING= ( wall )
MARKER_MONITORING= ( wall )
NUM_METHOD_GRAD= GREEN_GAUSS
CFL_NUMBER= 3.0
CFL_ADAPT= YES
CFL_ADAPT_PARAM= ( 0.3, 1.2, 1.0, 25.0 )
ITER= {iters}
LINEAR_SOLVER= FGMRES
LINEAR_SOLVER_PREC= ILU
LINEAR_SOLVER_ERROR= 1E-4
LINEAR_SOLVER_ITER= 10
CONV_NUM_METHOD_FLOW= FDS
MUSCL_FLOW= YES
SLOPE_LIMITER_FLOW= VENKATAKRISHNAN
VENKAT_LIMITER_COEFF= 0.1
TIME_DISCRE_FLOW= EULER_IMPLICIT
CONV_FIELD= LIFT, DRAG
CONV_STARTITER= 50
CONV_CAUCHY_ELEMS= 80
CONV_CAUCHY_EPS= 2E-5
MESH_FILENAME= ../../half.su2
MESH_FORMAT= SU2
OUTPUT_FILES= ( SURFACE_CSV, SURFACE_PARAVIEW{volume} )
SCREEN_OUTPUT= ( INNER_ITER, RMS_PRESSURE, LIFT, DRAG, MOMENT_Y )
HISTORY_OUTPUT= ( ITER, RMS_RES, AERO_COEFF )
SCREEN_WRT_FREQ_INNER= 25
OUTPUT_WRT_FREQ= 5000
"""


def run(tag, alpha, iters=600, threads=20):
    import math
    d = ROOT / "cfd" / tag / f"a{alpha:g}"
    d.mkdir(parents=True, exist_ok=True)
    volume = ", PARAVIEW" if "--volume" in sys.argv else ""
    vx = 20.0 * math.cos(math.radians(alpha))
    vz = 20.0 * math.sin(math.radians(alpha))
    (d / "case.cfg").write_text(CFG.format(alpha=alpha, vx=vx, vz=vz, area=S_HALF, iters=iters, volume=volume))
    with open(d / "log.txt", "w") as log:
        subprocess.run([str(SU2), "-t", str(threads), "case.cfg"], cwd=d, stdout=log, stderr=subprocess.STDOUT)
    rows = list(csv.reader(open(d / "history.csv")))
    head = [h.strip().strip('"') for h in rows[0]]
    last = [float(x) for x in rows[-1]]
    g = lambda k: last[head.index(k)]
    return dict(alpha=alpha, CL=g("CL"), CD=g("CD"), CMy=g("CMy"), iters=len(rows) - 1, rms_p=g("rms[P]"))


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    tag, alphas = args[0], [float(a) for a in args[1:]]
    out = []
    for a in alphas:
        r = run(tag, a)
        out.append(r)
        print(f"alpha {a:5.1f}  CL {r['CL']:+.4f}  CD {r['CD']:.5f}  CMy(about CAD origin, per metre) {r['CMy']:+.5f}  "
              f"iterations {r['iters']}  residual {r['rms_p']:.2f}", flush=True)
    (ROOT / "cfd" / tag / "results.json").write_text(json.dumps(out, indent=1))
