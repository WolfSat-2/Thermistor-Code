/*
====================================================
 ads1115.h  -  Minimal ADS1115 driver (single-shot, single-ended)
====================================================

 Why an external ADC?
   On the ESP32-C6 only GPIO0..GPIO6 are ADC inputs. GPIO0..5 are taken by the
   six piezos and GPIO6 is I2C SDA, so there is no room for six thermistors.
   Two ADS1115 chips on the existing I2C bus give 8 extra 16-bit channels:

     ADS #1  addr 0x48 (ADDR -> GND)   AIN0 +X  AIN1 -X  AIN2 +Y  AIN3 -Y
     ADS #2  addr 0x49 (ADDR -> VDD)   AIN0 +Z  AIN1 -Z  AIN2 V_EXC sense
                                       AIN3 reference divider (self-test)

   The AHT30 sits at 0x38, so there is no address clash.

 Settings used
   PGA  = +/-4.096 V  -> 0.125 mV per LSB (inputs must still stay below VDD + 0.3 V)
   Rate = 128 SPS     -> ~7.8 ms per conversion, good noise rejection
   Mode = single-shot -> chip sleeps (~0.5 uA) between conversions

 No library needed. If you prefer the Adafruit_ADS1X15 library, it is a drop-in
 replacement: readADC_SingleEnded(ch) + computeVolts().
*/

#pragma once

#include <Arduino.h>
#include <Wire.h>

class ADS1115
{
public:
    static constexpr uint8_t REG_CONVERSION = 0x00;
    static constexpr uint8_t REG_CONFIG     = 0x01;

    // Returns true if the chip ACKs on the bus
    bool begin(uint8_t address, TwoWire& wire = Wire)
    {
        _addr = address;
        _wire = &wire;

        _wire->beginTransmission(_addr);
        _present = (_wire->endTransmission() == 0);

        return _present;
    }

    bool present() const { return _present; }
    uint8_t address() const { return _addr; }

    // Single-ended read of AIN0..AIN3 against GND.
    // Returns millivolts, or NAN on I2C error / timeout.
    float readMillivolts(uint8_t channel)
    {
        if(!_present || channel > 3)
        {
            return NAN;
        }

        uint16_t config =
              0x8000                              // OS: start a single conversion
            | ((uint16_t)(0x4 + channel) << 12)   // MUX: AINx vs GND
            | 0x0200                              // PGA: +/-4.096 V
            | 0x0100                              // MODE: single-shot
            | 0x0080                              // DR: 128 SPS
            | 0x0003;                             // comparator disabled

        if(!writeRegister(REG_CONFIG, config))
        {
            return NAN;
        }

        // Wait for OS bit to go back to 1 (= conversion finished)
        delay(8);

        uint16_t status = 0;
        unsigned long start = millis();

        while(true)
        {
            if(!readRegister(REG_CONFIG, status))
            {
                return NAN;
            }

            if(status & 0x8000)
            {
                break;
            }

            if(millis() - start > 30)
            {
                return NAN;     // chip hung, should never happen
            }

            delay(1);
        }

        uint16_t raw = 0;

        if(!readRegister(REG_CONVERSION, raw))
        {
            return NAN;
        }

        return (int16_t)raw * 0.125f;   // LSB = 4.096 V / 32768
    }

private:
    TwoWire* _wire = nullptr;
    uint8_t  _addr = 0x48;
    bool     _present = false;

    bool writeRegister(uint8_t reg, uint16_t value)
    {
        _wire->beginTransmission(_addr);
        _wire->write(reg);
        _wire->write((uint8_t)(value >> 8));
        _wire->write((uint8_t)(value & 0xFF));

        return _wire->endTransmission() == 0;
    }

    bool readRegister(uint8_t reg, uint16_t& value)
    {
        _wire->beginTransmission(_addr);
        _wire->write(reg);

        if(_wire->endTransmission() != 0)
        {
            return false;
        }

        if(_wire->requestFrom(_addr, (uint8_t)2) != 2)
        {
            return false;
        }

        uint8_t hi = _wire->read();
        uint8_t lo = _wire->read();
        value = ((uint16_t)hi << 8) | lo;

        return true;
    }
};
