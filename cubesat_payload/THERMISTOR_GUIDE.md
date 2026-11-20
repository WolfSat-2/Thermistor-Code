# Thermistor Subsystem Guide (CubeSat SHM payload)

There are six 10 kΩ NTC thermistors, one bonded to the inside of each face. They are read every 10 s through two ADS1115 16-bit ADCs on the existing I²C bus. The face numbering matches the thermal model in `quaternion_version.py`, so flight data can be compared directly with the simulation.

```
cubesat_payload/
├── cubesat_payload.ino      sketch (scheduler, buffer, readTemperatureSensors)
├── thermistor.h             NTC math + fault detection (no Arduino deps)
├── ads1115.h                ~100-line ADS1115 driver, no library needed
├── tools/thermistor_design.py   pick R_FIXED, resolution table, Steinhart-Hart fit
└── test/test_thermistor.cpp     host unit tests
```

---

## 1. Why the thermistors don't go on the ESP32-C6 pins

On the ESP32-C6, only **GPIO0–GPIO6** can be ADC inputs. The earlier draft put the thermistors on GPIO0–5, which are the same pins as the six piezos, and GPIO6 is I²C SDA. There is no spare ADC pin.

The internal ADC also clips at about 3.1 V (11 dB attenuation). With a 10k/10k divider, that means **anything colder than about −24 °C reads as "open circuit"**. A tumbling birch-panel CubeSat in eclipse can easily get colder than that.

**Fix:** use two ADS1115 boards (≈ $3 each) on the I²C bus the AHT30 already uses. This frees the internal ADC for the piezos and gives 16-bit resolution down to −60 °C.

`USE_EXTERNAL_ADC 0` still exists for quick bench tests on a bare DevKit, but don't fly it.

---

## 2. Wiring

```
            GPIO10 (THERM_EXCITE_PIN)  ──┬───────────┬─────── ... ─────┬──────────┐
                                         │           │                 │          │
                                       [10k]       [10k]             [10k]      [10k] 0.1%
                                         │           │                 │          │
  ADS#1 AIN0 (+X) ───────────────────────┤           │                 │          ├── ADS#2 AIN3 (ref)
  ADS#1 AIN1 (-X) ───────────────────────────────────┤                 │          │
   ...                                   │           │                 │        [10k] 0.1%
                                       [NTC]       [NTC]             [NTC]        │
                                         │           │                 │          │
                                        GND         GND               GND        GND

            GPIO10 ─────────────────────────────────────────────────────── ADS#2 AIN2 (V_EXC sense)
```

| Signal | ESP32-C6 | ADS1115 #1 (0x48) | ADS1115 #2 (0x49) |
|---|---|---|---|
| VDD | 3V3 | VDD | VDD |
| GND | GND | GND | GND |
| SDA | GPIO6 | SDA | SDA |
| SCL | GPIO7 | SCL | SCL |
| ADDR | – | → GND | → VDD |
| Inputs | – | AIN0 +X, AIN1 −X, AIN2 +Y, AIN3 −Y | AIN0 +Z, AIN1 −Z, AIN2 V_EXC, AIN3 ref |

- **I²C pull-ups:** 4.7 kΩ from SDA and SCL to 3V3, fitted **once** on the bus. Most breakout boards already have 10 k pull-ups. Two in parallel is fine; four or more gets too strong.
- **Optional:** a 100 nF cap from each divider node to GND filters radio noise. The 5 ms settle delay in the code already covers it.
- Power the ADS1115 from **3.3 V, not 5 V**. Its inputs must never go above VDD + 0.3 V.

### Why the divider is powered from a GPIO (`USE_EXCITE_PIN 1`)
Current through a thermistor heats it. Datasheets give a dissipation constant of about 1.5 mW/K **in air**. In vacuum there is no convection, so the same 0.3 mW can warm the bead by about 0.5 K (see the `self-heat` column of the design tool). Powering the divider only for the ~300 ms the measurement takes cuts the duty cycle to about 3 %. That removes most of the error and also saves about 1 mA continuous.

Because the code measures V_EXC itself (ADS#2 AIN2) and works with **V_out / V_EXC**, it doesn't matter that a GPIO gives 3.25 V rather than 3.30 V, or that the rail sags while the radio transmits. This is called ratiometric measurement.

### Reference divider self-test (`HAS_REF_DIVIDER 1`)
Two 0.1 % resistors of equal value must always read a ratio of 0.500. If that check fails, an ADC or the excitation is drifting, and every temperature from that pass is suspect. `thermRefOk` reports this, so consider adding it to the housekeeping telemetry.

---

## 3. Parts

| Part | Recommendation | Notes |
|---|---|---|
| NTC | 10 kΩ @ 25 °C, B ≈ 3950 K, glass bead or epoxy chip, ±1 % | e.g. Vishay NTCLE100E3103, TDK B57861S, Murata NCP18XH103. **Put the datasheet R25 and B into `THERM_MODEL`.** |
| R_FIXED | 10 kΩ, **0.1 %, ≤ 25 ppm/K** thin film | The whole measurement relies on this resistor. A 5 % resistor alone causes about 1.1 °C of error. |
| ADC | 2 × ADS1115 breakout | |
| Wire | PTFE-insulated (e.g. 30 AWG Kapton/PTFE) | PVC outgasses in vacuum. Avoid it. |

### Picking R_FIXED for your temperature range
Run the thermal model first. `quaternion_version.py` prints "External Panel Temperature Extremes". Add about 20 °C of margin on each side, then:

```bash
python tools/thermistor_design.py --range -40 90 --pick-rfixed
```

For −40…90 °C it suggests **12 kΩ**, with a worst-case resolution of 0.019 K. The default **10 kΩ** is almost as good (0.022 K at −40 °C) and keeps one value on the BOM, so the code keeps 10 k. If the simulation shows faces colder than −60 °C, re-run the tool and consider a higher R_FIXED.

Resolution is not accuracy. Accuracy comes from the thermistor tolerance (±1 % R25 ≈ ±0.25 °C), the R_FIXED tolerance, and the Beta-model error far from 25 °C. That last error is why section 6 exists.

---

## 4. Mounting on the birch panels

- Bond the bead to the **inside** face of the panel with a low-outgassing epoxy, e.g. Arathane 5753, EPO-TEK 353ND, or Loctite Stycast 2850FT. **Avoid hot glue and standard 5-minute epoxy.** Their outgassing is too high, and hot glue softens at about 60 °C.
- Cover it with a square of Kapton tape so the bead isn't measuring radiation from the electronics.
- Mount every thermistor in the same spot on its panel (e.g. the centre). The thermal model treats each face as a single node, so a sensor near an edge reads closer to the neighbouring face.
- Leave a strain-relief loop in the wire. Vibration at launch breaks fine thermistor leads at the bead, which shows up as `OPEN`.

---

## 5. Fault codes

Each sample stores `temperature[i]` (°C, or `NAN`) **and** `tempStatus[i]`:

| Code | Name | Meaning | Typical cause |
|---|---|---|---|
| 0 | OK | good reading | |
| 1 | SHORT | ratio ≤ 0.005 (R < ~50 Ω) | wire pinched to chassis, R_FIXED open |
| 2 | OPEN | ratio ≥ 0.999 (R > ~10 MΩ) | lead broken, connector off |
| 3 | RANGE | converted, but outside −60…120 °C | real extreme, or a degraded sensor |
| 4 | ADC_ERR | no reading | ADS1115 not answering on I²C |
| 5 | NO_EXC | V_EXC < 2.8 V | excitation pin/rail fault (all six faces at once) |

**Pattern tips for ground analysis:**
- **All six faces failing at once** means a shared fault: excitation, the ADC, or the bus.
- **Four faces failing (+X, −X, +Y, −Y)** means ADS#1 is down.
- **One face that toggles between OK and OPEN** means an intermittent lead, often thermal-cycling fatigue.

---

## 6. Calibration (do this before integration)

With 0.1 % R_FIXED and a 1 % NTC you get about ±0.3 °C at 25 °C with no calibration. Further from 25 °C the Beta equation drifts, typically by 0.5–1 °C at −40 °C.

**Level 1: offsets (15 minutes)**
1. Make an ice bath: crushed ice topped up with water, stirred. It sits at 0.0 °C.
2. Put each thermistor in it, sealed in a thin plastic bag. Wait 2 minutes.
3. Read the Serial output (`THERM_DEBUG 1`). Set `THERM_OFFSET_C[i] = 0.0 − reading[i]`.

**Level 2: Steinhart–Hart (better over a wide range)**
Measure the **resistance** each sensor reports (shown in the debug output) at three known temperatures, e.g. ice bath 0 °C, room temperature against a reference thermometer, and warm water at about 60 °C. Then:

```bash
python tools/thermistor_design.py --fit-sh 0 32650  23.4 10870  61.2 2510
```

Paste the three printed coefficients into the `0.0, 0.0, 0.0` slot of `THERM_MODEL`. When `shA` isn't 0 the code uses them instead of Beta. Strictly this is per sensor; for a student CubeSat, one batch fit plus Level 1 offsets is usually enough.

---

## 7. Comparing flight data with the thermal model

| Firmware index | Face | `face_normals` row in `quaternion_version.py` | Model panel print |
|---|---|---|---|
| 0 | +X | `[1, 0, 0]` | Panel 1 |
| 1 | −X | `[-1, 0, 0]` | Panel 2 |
| 2 | +Y | `[0, 1, 0]` | Panel 3 |
| 3 | −Y | `[0, -1, 0]` | Panel 4 |
| 4 | +Z | `[0, 0, 1]` (magnet face) | Panel 5 |
| 5 | −Z | `[0, 0, -1]` | Panel 6 |

- The model works in **Kelvin**, the firmware in **°C**. `T_K = T_C + 273.15`.
- The model's `T_faces` is the panel's bulk temperature. The thermistor reads the inside surface, which on 6 mm birch lags the outside by a few minutes. Expect the measured curve to be smoother and slightly delayed.
- The body axes on the real structure **must** match the model's axes. Label the panels physically (+X etc.) before integration.
- Thermal expansion analysis (`thermal_expansion()` in the model) uses `Te_max`/`Te_min` per face. Feed it real measured extremes after flight to compare predicted and actual ΔL against the strain gauges.

---

## 8. Bench test checklist

- [ ] I²C scan finds 0x38 (AHT30), 0x48, 0x49.
- [ ] Boot message prints "ADS1115 #1 found" and "#2 found".
- [ ] All six faces read room temperature within ±1 °C of each other.
- [ ] `V_EXC` ≈ 3200–3300 mV and "ref divider OK".
- [ ] Pinch one NTC between your fingers: only that face rises (catches swapped channels).
- [ ] Unplug one NTC: `OPEN`. Short its leads: `SHORT`.
- [ ] Freezer test (≈ −18 °C): readings plausible, no `OPEN`.
- [ ] Unit tests: `cd test && g++ -std=c++17 -I.. test_thermistor.cpp -o t && ./t`

---

## 9. Other things noticed in the sketch (not thermistor-specific)

- **Buffer is exactly full at transmit time.** 360 samples × 10 s = 3600 s = `TRANSMIT_INTERVAL`. A slightly late transmit drops data ("Buffer Full"). Make `MAX_BUFFER` about 400, or transmit when the buffer is full rather than on a timer.
- **Telemetry size.** `SensorData` is about 84 bytes (≈ 30 kB for 360 samples). Packing temperatures as `int16_t` centi-degrees (see the comment in `transmitData()`) cuts the thermistor part from 24 to 12 bytes per sample.
- **Watchdog.** If an I²C device locks the bus, the driver times out after 30 ms per read instead of hanging. A hardware watchdog (`esp_task_wdt`) is still worth adding before flight.
- **Strapping pins.** On the ESP32-C6, GPIO4 and GPIO5 (piezo 5/6) are also JTAG/strapping pins. Make sure the piezo circuits don't pull them during boot.
