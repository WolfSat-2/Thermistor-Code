/*
====================================================
 CubeSat Structural Health Monitoring System
 Platform: ESP32-C6 DevKitC-1
 Author: Jonathan
====================================================

 Thermistor subsystem: see THERMISTOR_GUIDE.md for wiring, part selection,
 calibration and how the face numbering lines up with the thermal model
 (quaternion_version.py / magnet_testing.py).

 Files in this sketch folder:
   cubesat_payload.ino   this file (scheduler, buffer, sensors)
   thermistor.h          NTC math + fault detection (no Arduino deps, unit-tested)
   ads1115.h             tiny driver for the external 16-bit ADC
*/

#include <Wire.h>
#include "thermistor.h"
#include "ads1115.h"

// ====================================================
// CONFIGURATION
// ====================================================

#define BAUD_RATE 115200

const unsigned long SAMPLE_INTERVAL = 10000;        // 10 seconds
const unsigned long TRANSMIT_INTERVAL = 3600000;    // 60 minutes

// 1 = print every temperature sample to Serial (bench testing)
#define THERM_DEBUG 1

// 1 = thermistors on two ADS1115 chips (flight configuration, recommended)
// 0 = thermistors straight on ESP32-C6 ADC pins (bench only: those pins are
//     shared with the piezos and the internal ADC saturates below about -24 C)
#define USE_EXTERNAL_ADC 1

// 1 = divider is powered from a GPIO only while measuring (recommended:
//     removes self-heating, which is much worse in vacuum, and saves power)
// 0 = divider tied permanently to 3V3
#define USE_EXCITE_PIN 1

// 1 = ADS #2 AIN3 has a 10k/10k 0.1% divider for an ADC self-test
#define HAS_REF_DIVIDER 1

// ====================================================
// GPIO DEFINITIONS
// (Replace with actual pin numbers later)
// ====================================================

// AHT30 I2C (also shared by both ADS1115 chips)
#define SDA_PIN 6
#define SCL_PIN 7

// Piezo Sensors
#define PIEZO1_PIN 0
#define PIEZO2_PIN 1
#define PIEZO3_PIN 2
#define PIEZO4_PIN 3
#define PIEZO5_PIN 4
#define PIEZO6_PIN 5

// Thermistor divider excitation (top of all six R_FIXED resistors).
// Any free GPIO that is NOT a strapping/USB/UART pin. Avoid 8, 9, 15 (strapping),
// 12/13 (USB), 16/17 (UART0). GPIO10 is free on the DevKitC-1 header.
#define THERM_EXCITE_PIN 10

// Bench-only fallback pins (USE_EXTERNAL_ADC 0). Conflict with the piezos!
#define THERM1_PIN 0
#define THERM2_PIN 1
#define THERM3_PIN 2
#define THERM4_PIN 3
#define THERM5_PIN 4
#define THERM6_PIN 5

// Strain Gauge Amplifier
// TODO: Add HX711 or INA125 pin definitions

// ====================================================
// DATA STRUCTURE
// ====================================================

struct SensorData
{
    unsigned long timestamp;

    float temperature[6];       // deg C, NAN if the sensor is faulty

    uint8_t tempStatus[6];      // therm::Status code per face (why it is NAN)

    float strain[6];

    int vibration[6];
};

const int MAX_BUFFER = 360;

SensorData dataBuffer[MAX_BUFFER];

int bufferIndex = 0;

// ====================================================
// TIMING
// ====================================================

unsigned long previousSample = 0;
unsigned long previousTransmit = 0;

bool transmitting = false;

// ====================================================
// THERMISTOR CONFIGURATION
// ====================================================

// Face order matches the thermal model (quaternion_version.py face_normals):
//   0:+X  1:-X  2:+Y  3:-Y  4:+Z  5:-Z
const char* FACE_NAMES[6] = { "+X", "-X", "+Y", "-Y", "+Z", "-Z" };

// ASSUMED part: 10k NTC, B = 3950 (e.g. generic MF52 / Vishay NTCLE100E3103).
// Replace with the datasheet values of the part you actually fly.
therm::Model THERM_MODEL =
{
    10000.0f,   // rFixed     : 10k 0.1% 25 ppm/K metal film
    10000.0f,   // r0         : R25
    298.15f,    // t0K        : 25 C
    3950.0f,    // beta
    0.0, 0.0, 0.0,              // Steinhart-Hart A,B,C (0 = use Beta)
    -60.0f,     // minValidC
    120.0f,     // maxValidC
    0.005f,     // ratioShort : R < ~50 ohm
#if USE_EXTERNAL_ADC
    0.999f      // ratioOpen  : R > ~10 Mohm (about -78 C, far below minValidC)
#else
    0.935f      // ratioOpen  : internal ADC clips ~3.1 V -> anything colder than ~-24 C reads OPEN
#endif
};

// Per-face offsets from the ice-bath / room calibration (deg C, added to result).
// See THERMISTOR_GUIDE.md section 6.
float THERM_OFFSET_C[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

const int   THERM_OVERSAMPLE = 4;        // conversions averaged per channel
const float EXC_MIN_MV       = 2800.0f;  // below this the excitation is broken
const float SUPPLY_MV        = 3300.0f;  // only used when V_EXC cannot be measured

#if USE_EXTERNAL_ADC

ADS1115 ads[2];
const uint8_t ADS_ADDR[2] = { 0x48, 0x49 };

struct ThermChannel { uint8_t chip; uint8_t ain; };

const ThermChannel THERM_MAP[6] =
{
    {0, 0},  // +X
    {0, 1},  // -X
    {0, 2},  // +Y
    {0, 3},  // -Y
    {1, 0},  // +Z
    {1, 1},  // -Z
};

const ThermChannel EXC_SENSE = {1, 2};   // V_EXC measured directly
const ThermChannel REF_SENSE = {1, 3};   // fixed 10k/10k divider, expect 0.500

#else

const uint8_t THERM_PINS[6] =
{
    THERM1_PIN, THERM2_PIN, THERM3_PIN,
    THERM4_PIN, THERM5_PIN, THERM6_PIN
};

#endif

// Health flag for the reference divider self-test (true if not fitted)
bool thermRefOk = true;

// ====================================================
// SENSOR FUNCTIONS
// ====================================================

void thermExcitation(bool on)
{
#if USE_EXCITE_PIN
    digitalWrite(THERM_EXCITE_PIN, on ? HIGH : LOW);

    if(on)
    {
        delay(5);   // let the divider (and any filter cap) settle
    }
#else
    (void)on;
#endif
}

// Averaged reading in millivolts, NAN if any conversion fails
#if USE_EXTERNAL_ADC
float readChannelMillivolts(const ThermChannel& ch)
{
    float sum = 0.0f;

    for(int i = 0; i < THERM_OVERSAMPLE; i++)
    {
        float mv = ads[ch.chip].readMillivolts(ch.ain);

        if(isnan(mv))
        {
            return NAN;
        }

        sum += mv;
    }

    return sum / THERM_OVERSAMPLE;
}
#else
float readPinMillivolts(uint8_t pin)
{
    uint32_t sum = 0;

    for(int i = 0; i < THERM_OVERSAMPLE * 4; i++)
    {
        sum += analogReadMilliVolts(pin);   // uses eFuse calibration
        delayMicroseconds(200);
    }

    return (float)sum / (THERM_OVERSAMPLE * 4);
}
#endif

void readTemperatureSensors()
{
    SensorData& d = dataBuffer[bufferIndex];

    float vOut[6];
    float vExc;

    thermExcitation(true);

#if USE_EXTERNAL_ADC
    vExc = readChannelMillivolts(EXC_SENSE);

    for(int i = 0; i < 6; i++)
    {
        vOut[i] = readChannelMillivolts(THERM_MAP[i]);
    }

  #if HAS_REF_DIVIDER
    float vRef = readChannelMillivolts(REF_SENSE);
    float refRatio = vRef / vExc;
    thermRefOk = isfinite(refRatio) && fabsf(refRatio - 0.5f) < 0.0025f;  // +/-0.5 %
  #endif
#else
    vExc = SUPPLY_MV;   // internal ADC cannot see the rail directly

    for(int i = 0; i < 6; i++)
    {
        vOut[i] = readPinMillivolts(THERM_PINS[i]);
    }
#endif

    thermExcitation(false);

    bool excOk = isfinite(vExc) && vExc >= EXC_MIN_MV;

    for(int i = 0; i < 6; i++)
    {
        therm::Reading r;

        if(!excOk)
        {
            r.tempC  = NAN;
            r.ohms   = NAN;
            r.status = isfinite(vExc) ? therm::NO_EXCITATION : therm::ADC_ERROR;
        }
        else
        {
            r = therm::convert(vOut[i] / vExc, THERM_MODEL, THERM_OFFSET_C[i]);
        }

        // Only clean readings go into the buffer as numbers
        d.temperature[i] = (r.status == therm::OK) ? r.tempC : NAN;
        d.tempStatus[i]  = r.status;

#if THERM_DEBUG
        Serial.printf("  %s: ", FACE_NAMES[i]);

        if(isfinite(r.tempC))
        {
            Serial.printf("%7.2f C  %9.0f ohm  %s\n", r.tempC, r.ohms, therm::statusName(r.status));
        }
        else
        {
            Serial.printf("    ---          ---      %s\n", therm::statusName(r.status));
        }
#endif
    }

#if THERM_DEBUG
    Serial.printf("  V_EXC = %.1f mV  ref divider %s\n", vExc, thermRefOk ? "OK" : "FAIL");
#endif
}

bool setupThermistors()
{
#if USE_EXCITE_PIN
    pinMode(THERM_EXCITE_PIN, OUTPUT);
    digitalWrite(THERM_EXCITE_PIN, LOW);
#endif

#if USE_EXTERNAL_ADC
    bool ok = true;

    for(int i = 0; i < 2; i++)
    {
        if(ads[i].begin(ADS_ADDR[i], Wire))
        {
            Serial.printf("ADS1115 #%d found at 0x%02X\n", i + 1, ADS_ADDR[i]);
        }
        else
        {
            Serial.printf("ADS1115 #%d NOT found at 0x%02X - those faces will read ADC_ERR\n",
                          i + 1, ADS_ADDR[i]);
            ok = false;
        }
    }

    return ok;
#else
    analogReadResolution(12);

    for(int i = 0; i < 6; i++)
    {
        pinMode(THERM_PINS[i], INPUT);
        analogSetPinAttenuation(THERM_PINS[i], ADC_11db);   // ~0..3.1 V
    }

    Serial.println("WARNING: thermistors on internal ADC (bench mode), < -24 C reads OPEN");
    return true;
#endif
}

void readStrainSensors()
{
    // TODO
    // Read BF350 strain gauges through HX711/INA125
}

void readPiezoSensors()
{
    // Placeholder

    for(int i=0;i<6;i++)
    {
        dataBuffer[bufferIndex].vibration[i] = 0;
    }
}

// ====================================================
// BUFFER
// ====================================================

void saveSensorData()
{
    if(bufferIndex >= MAX_BUFFER)
    {
        Serial.println("Buffer Full");

        // TODO
        // Save overflow to flash or SD card

        return;
    }

    dataBuffer[bufferIndex].timestamp = millis();

    readTemperatureSensors();

    readStrainSensors();

    readPiezoSensors();

    bufferIndex++;

    Serial.println("Sensor data stored");
}

// ====================================================
// TRANSMISSION
// ====================================================

void transmitData()
{
    transmitting = true;

    Serial.println("Beginning transmission...");

    for(int i=0;i<bufferIndex;i++)
    {
        // TODO
        // Send through iSTAR S4 radio
        //
        // Thermistor packing suggestion (saves ~60% vs float):
        //   int16_t centiC = isnan(t) ? INT16_MIN : (int16_t)lroundf(t * 100);
        // gives 0.01 C steps over -327..+327 C, INT16_MIN = "no data".
        // Send tempStatus[] as 6 x 3-bit codes (fits in 3 bytes) or only when != OK.
    }

    Serial.println("Transmission complete");

    bufferIndex = 0;

    transmitting = false;
}

// ====================================================
// SETUP
// ====================================================

void setup()
{
    Serial.begin(BAUD_RATE);

    Wire.begin(SDA_PIN,SCL_PIN);

    setupThermistors();

    Serial.println("CubeSat Payload Started");
}

// ====================================================
// MAIN LOOP
// ====================================================

void loop()
{
    unsigned long currentTime = millis();

    // Sample sensors
    if(!transmitting &&
       currentTime - previousSample >= SAMPLE_INTERVAL)
    {
        previousSample = currentTime;

        saveSensorData();
    }

    // Downlink
    if(!transmitting &&
       currentTime - previousTransmit >= TRANSMIT_INTERVAL)
    {
        previousTransmit = currentTime;

        transmitData();
    }

    // Future:
    // Light sleep
    // Watchdog
    // Error handling
}
