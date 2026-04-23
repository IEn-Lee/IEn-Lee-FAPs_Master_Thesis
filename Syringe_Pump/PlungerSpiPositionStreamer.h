#ifndef PLUNGER_SPI_POSITION_STREAMER_H
#define PLUNGER_SPI_POSITION_STREAMER_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <TMC51X0.hpp>
#include <Adafruit_INA228.h>

#include "PlungerMotionPlanner.h"

namespace PlungerSpiPositionStreamerNS {

struct HardwareConfig {
    uint8_t pin_oe = 4;
    size_t enable_hardware_pin = 7;
    uint8_t chip_select_pin = 10;
    uint8_t ina228_addr = 0x40;

    bool use_spi1 = true;
    uint32_t spi_clock_hz = 5000000;

    bool use_wire1_for_ina = true;
};

struct KinematicsConfig {
    // 若 converter 的 real unit = mm
    float microsteps_per_mm = 3507.0f;

    // planner 的 plungerPos 單位是 m，所以要乘 1000 變成 mm
    float planner_meter_to_real_unit = 1000.0f;
};

struct DriverConfig {
    uint8_t run_current_percent = 80;
    uint8_t pwm_offset_percent = 40;
    uint8_t pwm_gradient_percent = 15;
    bool reverse_direction = false;
    uint32_t stealth_chop_threshold = 50;
};

struct TrackingConfig {
    // 這些不是軌跡生成器，而是讓 TMC 能夠追得上你 streaming 的 setpoint
    float max_velocity = 20.0f;
    float max_acceleration = 20.0f;
    float start_velocity = 0.5f;
    float stop_velocity = 0.5f;
    float first_velocity = 10.0f;
    float first_acceleration = 10.0f;
    float max_deceleration = 20.0f;
    float first_deceleration = 10.0f;

    // 幾 ms 更新一次 setpoint
    uint32_t stream_period_ms = 10;

    // 結束時再送一次 final target
    bool force_final_target = true;
};

struct InaConfig {
    float shunt_ohms = 0.02f;
    float max_current_a = 5.0f;
    INA228_AveragingCount averaging = INA228_COUNT_16;
    INA228_ConversionTime voltage_conv = INA228_TIME_150_us;
    INA228_ConversionTime current_conv = INA228_TIME_280_us;
};

struct Status {
    bool initialized = false;
    bool planner_attached = false;
    bool running = false;
    bool finished = false;
    bool fault = false;

    uint32_t last_stream_ms = 0;
    float elapsed_s = 0.0f;
    float commanded_pos_m = 0.0f;
    float commanded_pos_real = 0.0f;
    int32_t commanded_pos_chip = 0;

    float actual_pos_real = 0.0f;
    int32_t actual_pos_chip = 0;
    int32_t actual_velocity_chip = 0;
    float actual_velocity_real = 0.0f;

    float measured_current_mA = 0.0f;
    float measured_voltage_V = 0.0f;
    float measured_temp_C = 0.0f;

    char last_error[128] = {0};
};

class PlungerSpiPositionStreamer {
public:
    PlungerSpiPositionStreamer();

    bool begin(
        const HardwareConfig& hw_cfg,
        const KinematicsConfig& kin_cfg,
        const DriverConfig& drv_cfg,
        const TrackingConfig& track_cfg,
        const InaConfig& ina_cfg
    );

    bool attachPlanner(const PlungerMotionPlanner* planner);

    bool start(bool reset_actual_position_to_zero = true);
    void update();
    void stop(bool ramp_to_zero = true);
    void emergencyStop();

    bool isInitialized() const;
    bool isPlannerAttached() const;
    bool isRunning() const;
    bool isFinished() const;
    bool hasFault() const;

    const Status& getStatus() const;

    TMC51X0& driver();
    Adafruit_INA228& currentSensor();

private:
    bool initIna228_();
    bool initTmc5160_();
    void applyStreamingTarget_(float t_s);
    void refreshMeasurements_();
    void setError_(const char* msg);

private:
    HardwareConfig m_hw_cfg{};
    KinematicsConfig m_kin_cfg{};
    DriverConfig m_drv_cfg{};
    TrackingConfig m_track_cfg{};
    InaConfig m_ina_cfg{};

    const PlungerMotionPlanner* m_planner = nullptr;
    Status m_status{};

#if defined(ARDUINO_ARCH_RP2040)
    SPIClassRP2040* m_spi = &SPI;
#else
    SPIClass* m_spi = nullptr;
#endif

    TwoWire* m_wire = nullptr;

    TMC51X0 m_stepper;
    Adafruit_INA228 m_ina228;

    uint32_t m_run_start_ms = 0;
};

} // namespace PlungerSpiPositionStreamerNS

#endif