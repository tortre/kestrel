# Kestrel-X parts list

`OBSERA AERO · KESTREL-X REV 9.2 · PRICES READ 22 NOV 2026`

**$297.71** for the aircraft. The build is defined in `sim/aircraft.py`; prices
and their sources are in `sim/catalog.json`.

| # | Part | Pick | Qty | Mass (g) | Cost ($) | Source |
|---|---|---|---:|---:|---:|---|
| 1 | Motors | Glee Hobby 2810 900KV | 2 | 138.0 | 27.98 | [gleehobby.com](https://gleehobby.com/product/2810-900kv-1100kv-1500kv-motor/) |
| 2 | Props | APC 9x7.5E + 9x7.5EP, CW + CCW | 1 pair | 35.8 | 6.44 | windcatcherrc.com, brodak.com |
| 3 | ESCs | Readytosky 45 A 2-6S BLHeli_S, flashed with Bluejay | 2 | 14.0 | 19.98 | [speedyfpv.com](https://speedyfpv.com/products/readytosky-45a-blheli_s-2-6s-lipo-esc) |
| 4 | Battery | Reliance RS40 6S1P, six 21700 cells, 86 Wh | 1 | 430.0 | 35.94 | imrbatteries.com, plus $6 of nickel, wrap and leads |
| 5 | Flight controller | SpeedyBee F405 Wing Mini | 1 | 19.0 | 44.46 | AliExpress 3256810545908426, with shipping |
| 6 | HD video | Walksnail Ascent HD Lite+ kit | 1 | 10.5 | 39.99 | [caddxfpv.com](https://www.caddxfpv.com/products/walksnail-ascent-hd-lite-plus-kit) |
| 7 | Elevon servos | Emax ES9051 | 2 | 8.6 | 9.98 | [emax-usa.com](https://shop.emax-usa.com/products/emx-sv-0272-es9051) |
| 8 | GPS + compass | Quescan 18 mm M10 + QMC5883L | 1 | 6.0 | 10.25 | AliExpress 3256809818236262 |
| 9 | Receiver | BetaFPV ELRS Nano 2.4 GHz | 1 | 1.5 | 9.99 | [betafpv.com](https://betafpv.com/products/elrs-nano-receiver) |
| 10 | Tilt kit | 2x PTK 7455 MG-D servos, 5 mm shafts, 685ZZ bearings | 1 set | 48.0 | 21.00 | Estimate. May be about $10 low |
| 11 | Skin | 2 plies of 49 g/m² E-glass, epoxy, finish | 1 | 141.9 | 18.00 | Estimate |
| 12 | Spar | T700 pultruded caps 1.0 / 0.75 / 0.5 mm, web, root box | 1 | 176.7 | 22.00 | Estimate |
| 13 | Filament | LW-PLA 1.75 mm, 1 kg | 1 | 442.6 | 16.70 | AliExpress 3256812858449300 |
| 14 | Wiring and small parts | XT60, 16 AWG, two 470 µF capacitors, hinges, screws | 1 set | 34.0 | 15.00 | Estimate |
| | **Total** | | | **1,507** | **297.71** | |

Not in the total: radio, goggles, charger, spot welder, glue, Remote ID.
Shipping is counted only on the flight controller.

## Before you build

1. **Video transmitter power.** Set the VTX jumper to 5 V. The default is battery voltage, about 25 V.
2. **Load the parameter file before any test with props on.** `firmware/kestrel_x_stealth_v9_under300.param` sets the current limits. Without them a fresh pack can push 53 A through each 45 A ESC.
3. **Fit a 470 µF 35 V capacitor at each ESC.** They ship without one.
4. **Servo rail stays on 5 V.** The ES9051 is rated 4.0 to 5.5 V.
5. **Measure the motors on arrival.** Shaft thread (M5 assumed) and bolt circle (19 mm assumed) are not in the maker's listing.
6. **Bench-test the tilt servos** against prop load before the first hover. No model covers it.
7. **No airspeed sensor.** ArduPilot recommends one for tailsitters; an MS4525DO kit is about $45 to $60.
8. **Ground wind.** It tips over on its fins at about 20 km/h.

## Upgrades

| Option | Total | Gives |
|---|---:|---|
| Metal-gear elevon servos, Emax ES9052MD (pocket depth 9.0 mm in `src/printable.py`) | $306.11 | Stronger gears |
| Second battery brick in parallel | $333.65 | 192 min, 170 km |
| Airspeed sensor, MS4525DO | about $343 | Safer transitions |
