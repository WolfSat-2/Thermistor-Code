/*
====================================================
 thermistor.h  -  NTC thermistor math for the CubeSat SHM payload
====================================================

 Pure math, no Arduino dependencies, so it can be unit-tested on a PC:
     g++ -std=c++17 -I. test/test_thermistor.cpp -o t && ./t

 CIRCUIT (one per face)
 ---------------------
     V_EXC ----[ R_FIXED ]----+----[ NTC ]---- GND
                              |
                              +---> ADC input  (V_out)

     ratio = V_out / V_EXC = R_ntc / (R_FIXED + R_ntc)
     R_ntc = R_FIXED * ratio / (1 - ratio)

 Working with the RATIO (instead of assuming V_EXC = 3.300 V) is what makes
 the measurement "ratiometric": if the 3.3 V rail sags to 3.21 V because the
 radio is transmitting, both V_out and V_EXC drop together and the computed
 resistance does not change.

 FAILURE SIGNATURES
 ------------------
     NTC shorted / wire shorted to GND   -> ratio ~ 0
     NTC open / connector fell off       -> ratio ~ 1 (node pulled up to V_EXC)
     R_FIXED open                        -> ratio ~ 0 (looks like a short)
*/

#pragma once

#include <math.h>
#include <stdint.h>

namespace therm
{

// ----------------------------------------------------
// Status codes stored next to every temperature sample
// ----------------------------------------------------
enum Status : uint8_t
{
    OK            = 0,  // good reading
    SHORTED       = 1,  // ratio below ratioShort
    OPEN          = 2,  // ratio above ratioOpen
    OUT_OF_RANGE  = 3,  // converted fine, but outside [minValidC, maxValidC]
    ADC_ERROR     = 4,  // I2C / ADC failure, no number at all
    NO_EXCITATION = 5   // divider supply missing or too low to trust
};

inline const char* statusName(uint8_t s)
{
    switch(s)
    {
        case OK:            return "OK";
        case SHORTED:       return "SHORT";
        case OPEN:          return "OPEN";
        case OUT_OF_RANGE:  return "RANGE";
        case ADC_ERROR:     return "ADC_ERR";
        case NO_EXCITATION: return "NO_EXC";
        default:            return "?";
    }
}

// ----------------------------------------------------
// Thermistor model + divider description
// ----------------------------------------------------
struct Model
{
    float  rFixed;      // ohms, top resistor of the divider
    float  r0;          // ohms, NTC nominal resistance at t0K
    float  t0K;         // K, nominal temperature (298.15 K = 25 C)
    float  beta;        // K, Beta coefficient (datasheet "B25/50" or "B25/85")

    // Optional Steinhart-Hart coefficients (1/T = A + B ln R + C ln^3 R).
    // Leave A = 0 to use the Beta equation. Generate them from three
    // calibration points with tools/thermistor_design.py --fit-sh.
    double shA, shB, shC;

    float  minValidC;   // anything outside this window is flagged OUT_OF_RANGE
    float  maxValidC;

    float  ratioShort;  // ratio <= this  -> SHORTED
    float  ratioOpen;   // ratio >= this  -> OPEN
};

struct Reading
{
    float   tempC;      // NAN unless status == OK or OUT_OF_RANGE
    float   ohms;       // computed NTC resistance (NAN on SHORT/OPEN/ADC_ERROR)
    uint8_t status;
};

// ----------------------------------------------------
// Conversions
// ----------------------------------------------------
inline float ratioToOhms(float ratio, float rFixed)
{
    return rFixed * ratio / (1.0f - ratio);
}

// Beta equation: 1/T = 1/T0 + (1/B) ln(R/R0)
inline float betaOhmsToC(float ohms, float r0, float t0K, float beta)
{
    float invT = (1.0f / t0K) + logf(ohms / r0) / beta;
    return (1.0f / invT) - 273.15f;
}

// Inverse of the Beta equation (handy for tests and for sizing R_FIXED)
inline float betaCToOhms(float tempC, float r0, float t0K, float beta)
{
    float tK = tempC + 273.15f;
    return r0 * expf(beta * (1.0f / tK - 1.0f / t0K));
}

// Steinhart-Hart: 1/T = A + B ln R + C (ln R)^3
// Done in double: the C term is ~1e-7 and float loses it.
inline float shOhmsToC(double ohms, double A, double B, double C)
{
    double L    = log(ohms);
    double invT = A + B * L + C * L * L * L;
    return (float)(1.0 / invT - 273.15);
}

// ratio      : V_out / V_EXC (0..1)
// offsetC    : per-channel calibration offset, added after conversion
inline Reading convert(float ratio, const Model& m, float offsetC = 0.0f)
{
    Reading r;
    r.tempC  = NAN;
    r.ohms   = NAN;
    r.status = ADC_ERROR;

    if(!isfinite(ratio))
    {
        return r;
    }

    if(ratio <= m.ratioShort)
    {
        r.status = SHORTED;
        return r;
    }

    if(ratio >= m.ratioOpen)
    {
        r.status = OPEN;
        return r;
    }

    r.ohms = ratioToOhms(ratio, m.rFixed);

    float t = (m.shA != 0.0)
            ? shOhmsToC(r.ohms, m.shA, m.shB, m.shC)
            : betaOhmsToC(r.ohms, m.r0, m.t0K, m.beta);

    r.tempC  = t + offsetC;
    r.status = (r.tempC < m.minValidC || r.tempC > m.maxValidC) ? OUT_OF_RANGE : OK;

    return r;
}

} // namespace therm
