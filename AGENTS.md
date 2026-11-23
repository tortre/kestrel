# Working in this repo

The current build is defined in `sim/aircraft.py` (`CURRENT_BUILD`); parts and prices are in `sim/catalog.json`; geometry is `src/printable.py`, which writes `print/mass_report.json` that every sim reads. `README.md` and `PARTS_LIST.md` describe the current build; the old 560 mm v5 aircraft is no longer in this repo.

## Commands (Windows, Git Bash or PowerShell, from the repo root)

- BOM, mass, CG: `.venv/Scripts/python sim/aircraft.py`
- Tests (about 10 s, need `readiness/*.json` from a regenerate): `.venv/Scripts/python sim/test_harness.py`
- Rebuild every published number, then test (about 6 min): `.venv/Scripts/python sim/regenerate.py --tests`
- Parts hill climb at a price cap (about 4 min; writes `sim/hillclimb_v9_results.json`, which a test compares with `CURRENT_BUILD`): `.venv/Scripts/python sim/hillclimb_v9.py`
- CAD checks (about 3 min; rewrites `print/*.stl` and `print/mass_report.json`): `cad server start`, then `cad run src/check_print.py` and `cad run src/check_prop.py`
- README pictures (after a CAD change): `cad run renders/project_views.py`, `.venv/Scripts/python renders/drawings.py`, then `cad run renders/render.py <hero|top|front|rear> --final` (needs `renders/kestrel_x.glb`, exported from the STEP); all write to `assets/`
- Full-aircraft STEP: from `src/`, `cad run -c "import build123d as bd, printable as P; bd.export_step(P.kestrel_x_stealth(), '../STEP/<name>.step')"`

## Conventions

- To swap a part: add it to `sim/catalog.json` with a `conf` saying where the price came from, set the slot in `CURRENT_BUILD`, rerun regenerate with `--tests`.
- Before a new version, copy `src/printable.py` and `print/mass_report.json` to `*_v<N>_backup.*`.
- `sim/catalog.json` and several `sim/*.py` files use CRLF line endings; `sed -i` in Git Bash strips them.
- Long heredocs in Git Bash get cut off; write long files from a short Python script instead.
- Scratch scripts must not be named after a stdlib module (`numbers.py` breaks the numpy import).
- The spar caps are sized for `AIRFRAME_DESIGN_GROSS_KG` in `sim/aircraft.py`; change it and `CAP_T_*` in the CAD together.
- After changing the CG (battery, motors), check `TRIMMED_CLMAX` in `sim/aircraft.py` against `readiness/checks.json`.
