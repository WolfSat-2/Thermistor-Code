/*
====================================================
 CubeSat Structural Health Monitoring System
 Platform: ESP32-C6 DevKitC-1
 Author: Jonathan
====================================================
*/

#include <Wire.h>
#include <Adafruit_ADS1X15.h>   // Library Manager: "Adafruit ADS1X15"

// ====================================================
// CONFIGURATION
// ====================================================

#define BAUD_RATE 115200

const unsigned long SAMPLE_INTERVAL = 10000;        // 10 seconds
const unsigned long TRANSMIT_INTERVAL = 3600000;    // 60 minutes

// ====================================================
// GPIO DEFINITIONS
// (Replace with actual pin numbers later)
// ====================================================

// AHT30 I2C (the two ADS1115s are on this bus too)
#define SDA_PIN 6
#define SCL_PIN 7
#define AHT30_ADDR 0x38

// Piezo Sensors
#define PIEZO1_PIN 0
#define PIEZO2_PIN 1
#define PIEZO3_PIN 2
#define PIEZO4_PIN 3
#define PIEZO5_PIN 4
#define PIEZO6_PIN 5

// Thermistors
// The C6 only has ADC on GPIO0-6 and those are used by the piezos + SDA,
// so the thermistors go through 2 ADS1115 boards on I2C instead.
//   ADS 1 (0x48, ADDR to GND): A0 = +X, A1 = -X, A2 = +Y, A3 = -Y
//   ADS 2 (0x49, ADDR to VDD): A0 = +Z, A1 = -Z, A2 = divider supply (to check it)
// Face order is the same as the thermal sim (0:+X 1:-X 2:+Y 3:-Y 4:+Z 5:-Z)

// Turns the thermistor dividers on only while reading
// (stops them heating themselves up, which is worse in vacuum)
#define THERM_POWER_PIN 10

// Strain Gauge Amplifier
// TODO: Add HX711 or INA125 pin definitions

// ====================================================
// DATA STRUCTURE
// ====================================================

struct SensorData
{
    unsigned long timestamp;

    float temperature[6];   // C, panel thermistors, NAN if that sensor is broken

    float internalTemp;     // C, AHT30 inside the CubeSat (T_internal in the sim)

    float humidity;         // %RH, AHT30

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
// THERMISTOR SETTINGS
// ====================================================

// Divider: THERM_POWER_PIN -- 10k resistor --+-- NTC -- GND
//                                            |
//                                         ADS input
//
// Using 10k NTC with B = 3950 for now, CHANGE THESE to the datasheet of
// whatever thermistor we actually buy
const float R_FIXED = 10000.0;   // use a 0.1% resistor
const float R_25 = 10000.0;      // NTC resistance at 25 C
const float BETA = 3950.0;

// Anything outside this range is treated as a bad reading.
// Check these against the "External Panel Temperature Extremes" that
// quaternion_version.py prints and keep ~20 C of margin on both sides.
const float TEMP_MIN_VALID = -60.0;
const float TEMP_MAX_VALID = 120.0;

// Calibration offsets from the ice bath test (0 C), one per face
float tempOffset[6] = {0, 0, 0, 0, 0, 0};

Adafruit_ADS1115 ads1;
Adafruit_ADS1115 ads2;

bool ads1OK = false;
bool ads2OK = false;

String faceNames[6] = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"};

// ====================================================
// SENSOR FUNCTIONS
// ====================================================

// Reads the voltage of one face's thermistor (in volts)
// Returns -1 if that ADS1115 isn't working
float readThermistorVolts(int face)
{
    float total = 0;

    // average 4 readings to cut down noise
    for(int n=0;n<4;n++)
    {
        if(face < 4)
        {
            if(!ads1OK) return -1;
            total += ads1.computeVolts(ads1.readADC_SingleEnded(face));
        }
        else
        {
            if(!ads2OK) return -1;
            total += ads2.computeVolts(ads2.readADC_SingleEnded(face - 4));
        }
    }

    return total / 4;
}

// Turns the divider voltage into a temperature using the Beta equation
// Returns NAN if the sensor looks broken
float voltsToCelsius(float vOut, float vSupply)
{
    float ratio = vOut / vSupply;

    // ~0 means shorted, ~1 means the thermistor is disconnected
    if(ratio < 0.005 || ratio > 0.999)
    {
        return NAN;
    }

    // Rt = Rf * V / (Vs - V)
    float rThermistor = R_FIXED * ratio / (1.0 - ratio);

    // 1/T = 1/T0 + (1/B) * ln(R/R0)
    float invT = (1.0 / 298.15) + (1.0 / BETA) * log(rThermistor / R_25);
    float tempC = (1.0 / invT) - 273.15;

    // anything outside this is probably a bad reading
    if(tempC < TEMP_MIN_VALID || tempC > TEMP_MAX_VALID)
    {
        return NAN;
    }

    return tempC;
}

void readTemperatureSensors()
{
    digitalWrite(THERM_POWER_PIN, HIGH);
    delay(5);   // let it settle

    // Measure the actual supply instead of assuming 3.3V.
    // If the voltage drops (like when the radio is on) both readings drop
    // together so the temperature stays right.
    float vSupply = 3.3;
    if(ads2OK)
    {
        vSupply = ads2.computeVolts(ads2.readADC_SingleEnded(2));
    }

    for(int i=0;i<6;i++)
    {
        float vOut = readThermistorVolts(i);

        float tempC = NAN;

        if(vOut >= 0 && vSupply > 2.8)
        {
            tempC = voltsToCelsius(vOut, vSupply) + tempOffset[i];
        }

        dataBuffer[bufferIndex].temperature[i] = tempC;

        // Debug
        Serial.print(faceNames[i]);
        Serial.print(": ");
        if(isnan(tempC))
        {
            Serial.println("ERROR");
        }
        else
        {
            Serial.print(tempC);
            Serial.println(" C");
        }
    }

    digitalWrite(THERM_POWER_PIN, LOW);
}

// Reads the ONE AHT30 (inside the CubeSat, not on a panel)
// Talks to it directly over I2C so we don't need another library
void readAHT30()
{
    dataBuffer[bufferIndex].internalTemp = NAN;
    dataBuffer[bufferIndex].humidity = NAN;

    // start a measurement
    Wire.beginTransmission(AHT30_ADDR);
    Wire.write(0xAC);
    Wire.write(0x33);
    Wire.write(0x00);
    if(Wire.endTransmission() != 0)
    {
        Serial.println("AHT30: ERROR");
        return;
    }

    delay(80);   // datasheet says it takes ~80 ms

    // 7 bytes back: status, 5 data bytes, CRC
    if(Wire.requestFrom(AHT30_ADDR, 7) != 7)
    {
        Serial.println("AHT30: ERROR");
        return;
    }

    uint8_t data[7];
    for(int i=0;i<7;i++)
    {
        data[i] = Wire.read();
    }

    // bit 7 of the status byte = still busy
    if(data[0] & 0x80)
    {
        Serial.println("AHT30: busy");
        return;
    }

    // humidity and temperature are 20 bits each, packed together
    uint32_t rawHum = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4);
    uint32_t rawTemp = ((uint32_t)(data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | data[5];

    float hum = rawHum / 1048576.0 * 100.0;          // 2^20 = 1048576
    float temp = rawTemp / 1048576.0 * 200.0 - 50.0;

    dataBuffer[bufferIndex].internalTemp = temp;
    dataBuffer[bufferIndex].humidity = hum;

    Serial.print("Internal: ");
    Serial.print(temp);
    Serial.print(" C, ");
    Serial.print(hum);
    Serial.println(" %RH");
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

    readAHT30();

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

    pinMode(THERM_POWER_PIN, OUTPUT);
    digitalWrite(THERM_POWER_PIN, LOW);

    ads1OK = ads1.begin(0x48);
    ads2OK = ads2.begin(0x49);

    if(!ads1OK) Serial.println("ADS1115 #1 (0x48) not found!");
    if(!ads2OK) Serial.println("ADS1115 #2 (0x49) not found!");

    // +-4.096V range so we can read up to 3.3V
    ads1.setGain(GAIN_ONE);
    ads2.setGain(GAIN_ONE);

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
