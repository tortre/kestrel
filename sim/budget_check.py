"""Budget and build-variant comparison for Kestrel-X Stealth v9.2 Under-$300.   .venv/Scripts/python sim/budget_check.py
Writes readiness/budget_check.json when run as a script (importing it has no side effects)."""
import json

import aircraft as A
import harness as h

V90 = dict(motor="Glee Hobby 2812 900KV", prop="APC 8x6E / 8x6EP CW+CCW pair (LP08060E + LP08060EP)", esc="LittleBee 30A BLHeli_S",
           battery="Molicel P45B 6S1P (6x21700, 97Wh, 45A)", receiver="Receiver (HappyModel EP1 ELRS 2.4G)")
VARIANTS = (
    ("v9.2 Under-$300, hill-climbed (Glee 2810 900KV + APC 9x7.5E/EP + 45 A ESCs + ONE RS40 6S1P)", A.CURRENT_BUILD, {}),
    ("v9.0 (2812 + 8x6E/EP + 30 A LittleBee + P45B 6S1P): its ESC is sold out and discontinued", A.CURRENT_BUILD, V90),
    ("v9.2 + metal-gear elevon servos (Emax ES9052MD, pocket 9.0 mm deep)", A.CURRENT_BUILD, dict(servo="Emax ES9052MD 5.5g metal gear")),
    ("v9.2 with Molicel P45B cells (+$6, +12 % energy, but only 45 A burst: output ceiling 75 %)", A.CURRENT_BUILD,
     dict(battery="Molicel P45B 6S1P (6x21700, 97Wh, 45A)")),
    ("v9.2 with APC 9x6E/EP props (most hover margin)", A.CURRENT_BUILD, dict(prop="APC 9x6E / 9x6EP CW+CCW pair")),
    ("v9.2 + second RS40 6S1P brick in parallel (same bay, same ESCs, +$35.94)", A.CURRENT_BUILD, A.DUAL_PACK_UPGRADE),
    ("v9.2 with analog FPV instead of Walksnail HD (not recommended: loses the HD camera)", A.CURRENT_BUILD, dict(video="Analog 5.8 GHz VTX + camera (est)")),
    ("v8 Budget reference (Cine66 925KV + APC 8x8E/EP + P45B 6S2P + pitot)", A.V8_BUDGET_BUILD, {}),
)


def table():
    rows = []
    for label, base, changes in VARIANTS:
        b = A.variant(base, **changes) if changes else base
        hd = h.headline(b)
        rows.append(dict(label=label, build=b, cost=hd["cost"], mass_g=hd["mass_g"], vmax_kmh=hd["vmax"] * 3.6,
                         endurance_min=hd["loiter_min"], range_km=hd["best_range_km"], hover_w=hd["hover_w"],
                         tw_3v6=hd["static"]["3.6"]["tw"], tw_3v3=hd["static"]["3.3"]["tw"],
                         motor_a_full_fresh=hd["i_motor_full_fresh"], esc_amps=hd["esc_amps"],
                         feasible=hd["feasible"], reasons=hd["reasons"]))
    return rows


if __name__ == "__main__":
    saved = dict(h.OPTIONS)
    h.OPTIONS["off_aliexpress"] = True
    rows = table()
    h.OPTIONS.update(saved)
    print(f"{'Variant':94s} | {'AUW':>5s} | {'Vmax':>5s} | {'Endur':>6s} | {'Range':>6s} | {'T/W':>9s} | {'Cost':>7s} | gates")
    for r in rows:
        print(f"{r['label']:94s} | {r['mass_g']:4.0f}g | {r['vmax_kmh']:3.0f}kh | {r['endurance_min']:4.0f} m | {r['range_km']:4.0f}km | "
              f"{r['tw_3v6']:4.2f}/{r['tw_3v3']:4.2f} | ${r['cost']:6.2f} | {'ok' if r['feasible'] else '; '.join(r['reasons'])}")
    (A.ROOT / "readiness" / "budget_check.json").write_text(json.dumps(rows, indent=2, default=float))
