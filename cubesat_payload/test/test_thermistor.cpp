// Host-side unit test for thermistor.h (no hardware needed)
//   g++ -std=c++17 -Wall -I.. test_thermistor.cpp -o test_thermistor && ./test_thermistor
#include <cstdio>
#include <cmath>
#include "../thermistor.h"

static int failures = 0;

#define CHECK(cond, ...) do { if(!(cond)) { failures++; printf("FAIL: " __VA_ARGS__); printf("\n"); } } while(0)

int main()
{
    therm::Model m = {10000.0f, 10000.0f, 298.15f, 3950.0f, 0.0, 0.0, 0.0,
                      -60.0f, 120.0f, 0.005f, 0.999f};

    // 1. Round trip: temperature -> divider ratio -> temperature
    for(float t = -60.0f; t <= 115.0f; t += 5.0f)
    {
        float r     = therm::betaCToOhms(t, m.r0, m.t0K, m.beta);
        float ratio = r / (r + m.rFixed);
        therm::Reading rd = therm::convert(ratio, m);
        CHECK(rd.status == therm::OK, "t=%.1f status=%s", t, therm::statusName(rd.status));
        CHECK(fabsf(rd.tempC - t) < 0.02f, "t=%.1f got %.4f", t, rd.tempC);
    }

    // 2. 25 C must give exactly R0 at ratio 0.5 with R_FIXED = R0
    therm::Reading mid = therm::convert(0.5f, m);
    CHECK(fabsf(mid.tempC - 25.0f) < 0.01f, "mid %.4f", mid.tempC);
    CHECK(fabsf(mid.ohms - 10000.0f) < 1.0f, "mid ohms %.2f", mid.ohms);

    // 3. Fault detection
    CHECK(therm::convert(0.0f,  m).status == therm::SHORTED, "short");
    CHECK(therm::convert(0.003f, m).status == therm::SHORTED, "near short");
    CHECK(therm::convert(1.0f,  m).status == therm::OPEN, "open");
    CHECK(therm::convert(0.9995f, m).status == therm::OPEN, "near open");
    CHECK(therm::convert(NAN,   m).status == therm::ADC_ERROR, "nan");

    // 4. Out of range: -70 C is converted but flagged
    {
        float r = therm::betaCToOhms(-70.0f, m.r0, m.t0K, m.beta);
        therm::Reading rd = therm::convert(r / (r + m.rFixed), m);
        CHECK(rd.status == therm::OUT_OF_RANGE, "-70 status %s", therm::statusName(rd.status));
    }

    // 5. Calibration offset is applied
    CHECK(fabsf(therm::convert(0.5f, m, -0.3f).tempC - 24.7f) < 0.01f, "offset");

    // 6. Steinhart-Hart path: coefficients fitted to the same Beta curve at 0/25/70 C
    //    (values from tools/thermistor_design.py --fit-sh)
    therm::Model sh = m;
    sh.shA = 1.0222791883e-03; sh.shB = 2.5316539310e-04; sh.shC = -2.8087329721e-12;
    for(float t = -20.0f; t <= 90.0f; t += 10.0f)
    {
        float r = therm::betaCToOhms(t, m.r0, m.t0K, m.beta);
        therm::Reading rd = therm::convert(r / (r + m.rFixed), sh);
        CHECK(fabsf(rd.tempC - t) < 0.5f, "SH t=%.1f got %.3f", t, rd.tempC);
    }

    // 7. Ratiometric: same resistance, rail sagging 3.30 V -> 3.10 V gives same answer
    {
        float r = therm::betaCToOhms(-15.0f, m.r0, m.t0K, m.beta);
        float a = therm::convert((3300.0f * r / (r + m.rFixed)) / 3300.0f, m).tempC;
        float b = therm::convert((3100.0f * r / (r + m.rFixed)) / 3100.0f, m).tempC;
        CHECK(fabsf(a - b) < 0.001f, "ratiometric %.4f vs %.4f", a, b);
    }

    printf(failures ? "%d FAILURE(S)\n" : "All thermistor tests passed\n", failures);
    return failures ? 1 : 0;
}
