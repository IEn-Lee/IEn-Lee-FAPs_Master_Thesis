#include "CurrentSensorService.h"

namespace CurrentSensorService {

namespace {

Adafruit_INA228 ina228;
bool ready = false;

float current_mA = 0.0f;
float voltage_V = 0.0f;
float power_mW = 0.0f;
float temp_C = 0.0f;

} // anonymous namespace

bool begin(TwoWire* wire, const Config& cfg)
{
    ready = false;

    if (!wire) {
        return false;
    }

    if (!ina228.begin(cfg.address, wire)) {
        return false;
    }

    ina228.setShunt(cfg.shunt_ohms, cfg.max_current_a);

    ina228.setAveragingCount(INA228_COUNT_16);
    ina228.setVoltageConversionTime(INA228_TIME_150_us);
    ina228.setCurrentConversionTime(INA228_TIME_280_us);

    ready = true;
    update();

    return true;
}

bool isReady()
{
    return ready;
}

void update()
{
    if (!ready) return;

    current_mA = ina228.getCurrent_mA();
    voltage_V  = ina228.getBusVoltage_V();
    power_mW   = ina228.getPower_mW();
    temp_C     = ina228.readDieTemp();
}

float getCurrent_mA()
{
    if (!ready) return 0.0f;
    update();
    return current_mA;
}

float getCurrent_A()
{
    return getCurrent_mA() / 1000.0f;
}

float getBusVoltage_V()
{
    if (!ready) return 0.0f;
    update();
    return voltage_V;
}

float getPower_mW()
{
    if (!ready) return 0.0f;
    update();
    return power_mW;
}

float getDieTemp_C()
{
    if (!ready) return 0.0f;
    update();
    return temp_C;
}

} // namespace CurrentSensorService