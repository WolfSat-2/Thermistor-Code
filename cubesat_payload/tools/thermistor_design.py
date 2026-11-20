"""
thermistor_design.py - sizing and calibration helper for the CubeSat NTC thermistors.

Usage examples
--------------
  # Table of resistance / divider voltage / resolution over the predicted range
  python thermistor_design.py --range -40 90

  # Pick the best R_FIXED for that range from the E24 series
  python thermistor_design.py --range -40 90 --pick-rfixed

  # Fit Steinhart-Hart coefficients from three calibration points (C, ohms)
  python thermistor_design.py --fit-sh 0 32650  25 10000  70 1752

Take --range from the thermal model: run quaternion_version.py and use the
printed "External Panel Temperature Extremes", then add ~20 C margin on both ends.
"""

import argparse
import math

import numpy as np

# --- defaults: must match THERM_MODEL in cubesat_payload.ino -------------------
R0 = 10_000.0      # ohms at 25 C
T0_K = 298.15
BETA = 3950.0
R_FIXED = 10_000.0
V_EXC = 3.3        # V

ADS_LSB_V = 4.096 / 32768        # ADS1115, PGA +/-4.096 V
ESP_LSB_V = 3.1 / 4096           # ESP32-C6 internal ADC, 11 dB, 12-bit (effective is worse)
ESP_CLIP_V = 3.1

# Dissipation constant of a small bead NTC. Datasheets quote it IN AIR (~1.5 mW/K);
# in vacuum there is no convection, so assume much lower unless it is bonded to the panel.
DISSIPATION_W_PER_K_VACUUM = 0.5e-3

E24 = [1.0, 1.1, 1.2, 1.3, 1.5, 1.6, 1.8, 2.0, 2.2, 2.4, 2.7, 3.0,
       3.3, 3.6, 3.9, 4.3, 4.7, 5.1, 5.6, 6.2, 6.8, 7.5, 8.2, 9.1]


def r_ntc(t_c, r0=R0, beta=BETA):
    return r0 * np.exp(beta * (1.0 / (t_c + 273.15) - 1.0 / T0_K))


def ratio(t_c, r_fixed):
    r = r_ntc(t_c)
    return r / (r + r_fixed)


def dratio_dT(t_c, r_fixed):
    """Sensitivity of the divider ratio per kelvin (absolute value)."""
    r = r_ntc(t_c)
    t_k = t_c + 273.15
    dr_dt = -r * BETA / t_k ** 2
    return abs(r_fixed / (r + r_fixed) ** 2 * dr_dt)


def table(t_lo, t_hi, r_fixed):
    print(f"\nR_FIXED = {r_fixed:,.0f} ohm, NTC R25 = {R0:,.0f} ohm, B = {BETA:.0f} K, V_EXC = {V_EXC} V")
    print(f"{'T (C)':>7} {'R_ntc (ohm)':>13} {'V_out (mV)':>11} {'mV/K':>7} "
          f"{'ADS1115 res (K)':>16} {'ESP ADC res (K)':>16} {'self-heat (K)':>14}")
    for t in np.arange(t_lo, t_hi + 0.1, 10):
        r = r_ntc(t)
        v = V_EXC * ratio(t, r_fixed)
        s = V_EXC * dratio_dT(t, r_fixed)            # V per K
        ads_res = ADS_LSB_V / s
        esp = "clipped" if v > ESP_CLIP_V else f"{ESP_LSB_V / s:.3f}"
        i = V_EXC / (r + r_fixed)
        p = i * i * r                                # continuous power in NTC
        heat = p / DISSIPATION_W_PER_K_VACUUM
        print(f"{t:7.0f} {r:13,.0f} {v * 1e3:11.1f} {s * 1e3:7.2f} {ads_res:16.4f} {esp:>16} {heat:14.3f}")
    print("\nself-heat column = error if the divider were powered continuously in vacuum;"
          "\nwith USE_EXCITE_PIN (~0.3 s on per 10 s sample, ~3 % duty) it drops ~30x.")


def pick_rfixed(t_lo, t_hi):
    """Choose R_FIXED that maximises the WORST-case sensitivity over the range."""
    candidates = [m * 10 ** d for d in range(3, 6) for m in E24]
    ts = np.linspace(t_lo, t_hi, 200)
    best = max(candidates, key=lambda rf: dratio_dT(ts, rf).min())
    geo = math.sqrt(r_ntc(t_lo) * r_ntc(t_hi))
    print(f"\nRange {t_lo} .. {t_hi} C")
    print(f"  NTC spans {r_ntc(t_hi):,.0f} .. {r_ntc(t_lo):,.0f} ohm")
    print(f"  Geometric-mean rule of thumb: {geo:,.0f} ohm")
    print(f"  Best E24 value (max worst-case sensitivity): {best:,.0f} ohm")
    worst = dratio_dT(ts, best).min() * V_EXC
    print(f"  Worst-case sensitivity {worst * 1e3:.2f} mV/K -> ADS1115 resolution "
          f"{ADS_LSB_V / worst:.3f} K")
    return best


def fit_steinhart_hart(points):
    """points: [(t_c, ohms), ...] x3 -> (A, B, C) for 1/T = A + B lnR + C lnR^3."""
    M = np.array([[1.0, math.log(r), math.log(r) ** 3] for _, r in points])
    y = np.array([1.0 / (t + 273.15) for t, _ in points])
    A, B, C = np.linalg.solve(M, y)
    print("\nSteinhart-Hart coefficients (paste into THERM_MODEL):")
    print(f"    {A:.10e}, {B:.10e}, {C:.10e},")
    print("Check:")
    for t, r in points:
        L = math.log(r)
        t_fit = 1.0 / (A + B * L + C * L ** 3) - 273.15
        print(f"  {r:>10,.1f} ohm -> {t_fit:8.3f} C (measured {t} C)")
    return A, B, C


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--range", nargs=2, type=float, metavar=("T_MIN", "T_MAX"), default=[-60, 120])
    p.add_argument("--rfixed", type=float, default=R_FIXED)
    p.add_argument("--pick-rfixed", action="store_true")
    p.add_argument("--fit-sh", nargs=6, type=float, metavar=("T1", "R1", "T2", "R2", "T3", "R3"))
    a = p.parse_args()

    if a.fit_sh:
        v = a.fit_sh
        fit_steinhart_hart([(v[0], v[1]), (v[2], v[3]), (v[4], v[5])])
        return

    rf = pick_rfixed(*a.range) if a.pick_rfixed else a.rfixed
    table(a.range[0], a.range[1], rf)


if __name__ == "__main__":
    main()
