#ifndef CURRENT_SENSOR_SERVICE_H
#define CURRENT_SENSOR_SERVICE_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA228.h>

namespace CurrentSensorService {

struct Config {
    uint8_t address;
    float shunt_ohms;
    float max_current_a;

    Config()
    : address(0x40),
      shunt_ohms(0.02f),
      max_current_a(5.0f)
    {}
};

bool begin(TwoWire* wire = &Wire1, const Config& cfg = Config());

bool isReady();

float getCurrent_mA();
float getCurrent_A();
float getBusVoltage_V();
float getPower_mW();
float getDieTemp_C();

void update();

} // namespace CurrentSensorService

#endif