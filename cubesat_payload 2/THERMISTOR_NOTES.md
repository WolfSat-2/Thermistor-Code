# Thermistor Notes

## Why we use ADS1115s
The ESP32-C6 can only read analog on GPIO0-6. The piezos use GPIO0-5 and the I2C SDA line uses GPIO6, so there are no analog pins left for the thermistors. The C6's ADC also maxes out around 3.1V, so with a 10k/10k divider anything below about -24 C would read as a broken sensor, and the faces can get colder than that in eclipse.

Two ADS1115 boards on the same I2C bus as the AHT30 fix both problems.

## Wiring
| | ESP32-C6 | ADS1115 #1 | ADS1115 #2 |
|---|---|---|---|
| VDD | 3V3 | VDD | VDD |
| GND | GND | GND | GND |
| SDA | GPIO6 | SDA | SDA |
| SCL | GPIO7 | SCL | SCL |
| ADDR | | GND (0x48) | VDD (0x49) |
| Inputs | | A0 +X, A1 -X, A2 +Y, A3 -Y | A0 +Z, A1 -Z, A2 supply check |

Each thermistor: GPIO10 -> 10k resistor -> ADS input -> NTC -> GND.
GPIO10 also goes straight to ADS #2 A2 so the code can measure the real supply voltage.

- Power the ADS boards from 3.3V, not 5V.
- Use 0.1% resistors for the 10k. A 5% one adds about 1 C of error.
- Install the "Adafruit ADS1X15" library from the Library Manager.

## Mounting
- Glue to the inside of each panel with low-outgassing epoxy (not hot glue).
- Put Kapton tape over it.
- Same spot on every panel (center), since the sim treats each face as one temperature.
- Use PTFE wire, not PVC, and leave some slack so launch vibration doesn't break the leads.

## Calibration
1. Ice bath (crushed ice + water, stirred) = 0 C.
2. Put each thermistor in a small plastic bag in the ice bath and wait 2 min.
3. Read the Serial Monitor and set `tempOffset[i] = 0 - reading`.

## Bench test
- [ ] Serial Monitor doesn't say "not found" for either ADS
- [ ] All 6 faces read about room temp
- [ ] Pinch one thermistor, only that face goes up (checks we didn't swap wires)
- [ ] Unplug one, it says ERROR
- [ ] Put the board in the freezer, readings still make sense

## AHT30
The one AHT30 (0x38) sits inside the CubeSat, not on a panel. It goes into `internalTemp` and `humidity`, separate from the six panel thermistors. It's read with plain Wire commands, so no extra library is needed.

## Comparing with the sim
- Panel thermistors `temperature[0..5]` = `T_faces` in quaternion_version.py. Face order matches `face_normals` (0:+X 1:-X 2:+Y 3:-Y 4:+Z 5:-Z).
- AHT30 `internalTemp` = `T_internal` in the sim.
- The sim uses Kelvin, so add 273.15 to our readings.
- After running the sim, check its "External Panel Temperature Extremes" against `TEMP_MIN_VALID` / `TEMP_MAX_VALID` at the top of the sketch (keep ~20 C margin).
