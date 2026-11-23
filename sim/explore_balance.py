"""How far aft must the wing sit on a thin, symmetric body for the aircraft to balance?

Run: .venv/Scripts/python sim/explore_balance.py
For each wing shift it finds the neutral point, then the battery position that gives the target static
margin, and says whether a battery lying in the wing plane ahead of the spars can actually sit there.
"""
import blended as B

if __name__ == "__main__":
    base = dict(B.GEO)
    parts = {k: dict(v) for k, v in B.REPORT["parts"].items()}
    moves_with_wing = ("wing_", "elevon_")
    print("shift  NP x   CG target  battery x  spar limit  fits  margin if battery at limit")
    for shift in (0, 15, 25, 35, 45):
        B.GEO.update(d0=[165.0, -175.0, 14.5, 14.5], d36=[120.0, -150.0, 13.5, 13.5],
                     root_le=base["root_le"] - shift, tip_le=base["tip_le"] - shift, servo_x=base["servo_x"] - shift)
        for k, v in parts.items():
            B.REPORT["parts"][k] = dict(v, x=v["x"] - shift) if k.startswith(moves_with_wing) else dict(v)
        plane = B.airplane()
        x_np = B.neutral_point(plane)
        target = -(x_np - B.STATIC_MARGIN * plane.c_ref) * 1000
        table = B.mass_table(0.0)
        table["spar tubes x2"] = (table["spar tubes x2"][0], -33.0 - shift)
        table["stub tubes x2"] = (table["stub tubes x2"][0], -50.0 - shift)
        fixed_m = sum(g for k, (g, x) in table.items() if k != "battery")
        fixed_mx = sum(g * x for k, (g, x) in table.items() if k != "battery")
        batt = table["battery"][0]
        total = fixed_m + batt
        x_batt = (target * total - fixed_mx) / batt
        limit = 51.0 - shift + 2.0            # battery centre must be at least this far forward to clear the spars
        cg = (fixed_mx + batt * max(x_batt, limit)) / total
        print(f"{shift:4d} {-x_np * 1000:6.1f} {target:9.1f} {x_batt:10.1f} {limit:10.1f}   {'yes' if x_batt >= limit else 'no ':3s}"
              f"   {(cg / 1000 + x_np) / plane.c_ref * 100:5.1f} %")
