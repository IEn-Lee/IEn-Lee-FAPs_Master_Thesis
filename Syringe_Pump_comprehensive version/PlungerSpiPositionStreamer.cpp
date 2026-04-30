#include "PlungerSpiPositionStreamer.h"

namespace PlungerSpiPositionStreamerNS {

PlungerSpiPositionStreamer::PlungerSpiPositionStreamer()
{
#if !defined(ARDUINO_ARCH_RP2040)
    m_spi = nullptr;
#endif
    m_wire = nullptr;
}

bool PlungerSpiPositionStreamer::begin(
    const HardwareConfig& hw_cfg,
    const KinematicsConfig& kin_cfg,
    const DriverConfig& drv_cfg,
    const TrackingConfig& track_cfg,
    const InaConfig& ina_cfg
)
{
    m_hw_cfg = hw_cfg;
    m_kin_cfg = kin_cfg;
    m_drv_cfg = drv_cfg;
    m_track_cfg = track_cfg;
    m_ina_cfg = ina_cfg;

    m_status = Status{};

#if defined(ARDUINO_ARCH_RP2040)
    m_spi = &SPI;
#else
    m_spi = hw_cfg.use_spi1 ? &SPI1 : &SPI;
#endif

    m_wire = hw_cfg.use_wire1_for_ina ? &Wire1 : &Wire;

    if (!initIna228_()) {
        return false;
    }

    if (!initTmc5160_()) {
        return false;
    }

    m_status.initialized = true;
    return true;
}

bool PlungerSpiPositionStreamer::attachPlanner(const PlungerMotionPlanner* planner)
{
    if (!planner) {
        setError_("Planner pointer is null.");
        return false;
    }

    if (!planner->valid()) {
        setError_("Planner is not valid.");
        return false;
    }

    m_planner = planner;
    m_status.planner_attached = true;
    m_status.finished = false;
    m_status.fault = false;
    m_status.last_error[0] = '\0';
    return true;
}

bool PlungerSpiPositionStreamer::start(bool reset_actual_position_to_zero)
{
    if (!m_status.initialized) {
        setError_("Streamer not initialized.");
        return false;
    }

    if (!m_status.planner_attached || !m_planner) {
        setError_("No planner attached.");
        return false;
    }

    if (reset_actual_position_to_zero) {
        m_stepper.controller.beginRampToZeroVelocity();

        unsigned long t0 = millis();
        while (!m_stepper.controller.zeroVelocity()) {
            if (millis() - t0 > 3000) {
                setError_("Timeout waiting zero velocity before start.");
                return false;
            }
            delay(1);
        }

        m_stepper.controller.endRampToZeroVelocity();
        m_stepper.controller.zeroActualPosition();
    }

    m_run_start_ms = millis();
    m_status.running = true;
    m_status.finished = false;
    m_status.fault = false;
    m_status.last_error[0] = '\0';
    m_status.last_stream_ms = 0;

    applyStreamingTarget_(0.0f);
    refreshMeasurements_();

    return true;
}

void PlungerSpiPositionStreamer::update()
{
    if (!m_status.running || m_status.fault || !m_planner) {
        return;
    }

    const uint32_t now_ms = millis();

    if ((now_ms - m_status.last_stream_ms) < m_track_cfg.stream_period_ms) {
        return;
    }

    m_status.last_stream_ms = now_ms;

    const float t_s = (now_ms - m_run_start_ms) / 1000.0f;
    m_status.elapsed_s = t_s;

    if (t_s >= m_planner->totalDuration()) {
        if (m_track_cfg.force_final_target) {
            applyStreamingTarget_(m_planner->totalDuration());
        }

        refreshMeasurements_();
        m_status.running = false;
        m_status.finished = true;
        return;
    }

    applyStreamingTarget_(t_s);
    refreshMeasurements_();
}

void PlungerSpiPositionStreamer::stop(bool ramp_to_zero)
{
    if (ramp_to_zero) {
        m_stepper.controller.beginRampToZeroVelocity();
    }

    m_status.running = false;
}

void PlungerSpiPositionStreamer::emergencyStop()
{
    m_status.running = false;
    m_status.fault = true;
    strncpy(m_status.last_error, "Emergency stop triggered.", sizeof(m_status.last_error) - 1);
    m_status.last_error[sizeof(m_status.last_error) - 1] = '\0';
}

bool PlungerSpiPositionStreamer::isInitialized() const
{
    return m_status.initialized;
}

bool PlungerSpiPositionStreamer::isPlannerAttached() const
{
    return m_status.planner_attached;
}

bool PlungerSpiPositionStreamer::isRunning() const
{
    return m_status.running;
}

bool PlungerSpiPositionStreamer::isFinished() const
{
    return m_status.finished;
}

bool PlungerSpiPositionStreamer::hasFault() const
{
    return m_status.fault;
}

const Status& PlungerSpiPositionStreamer::getStatus() const
{
    return m_status;
}

TMC51X0& PlungerSpiPositionStreamer::driver()
{
    return m_stepper;
}

Adafruit_INA228& PlungerSpiPositionStreamer::currentSensor()
{
    return m_ina228;
}

bool PlungerSpiPositionStreamer::initIna228_()
{
    if (!m_wire) {
        setError_("Wire interface not assigned.");
        return false;
    }

    m_wire->begin();

    if (!m_ina228.begin(m_hw_cfg.ina228_addr, m_wire)) {
        setError_("INA228 not found.");
        return false;
    }

    m_ina228.setShunt(m_ina_cfg.shunt_ohms, m_ina_cfg.max_current_a);
    m_ina228.setAveragingCount(m_ina_cfg.averaging);
    m_ina228.setVoltageConversionTime(m_ina_cfg.voltage_conv);
    m_ina228.setCurrentConversionTime(m_ina_cfg.current_conv);

    return true;
}

bool PlungerSpiPositionStreamer::initTmc5160_()
{
    if (!m_spi) {
        setError_("SPI interface not assigned.");
        return false;
    }

#if defined(ARDUINO_ARCH_RP2040)
    // 若你要 RP2040 自訂腳位，可在這裡打開
    // m_spi->setSCK(...);
    // m_spi->setTX(...);
    // m_spi->setRX(...);
#endif

    pinMode(m_hw_cfg.pin_oe, OUTPUT);
    digitalWrite(m_hw_cfg.pin_oe, HIGH);
    delay(10);

    m_spi->begin();

    const auto spi_parameters =
        tmc51x0::SpiParameters{}
            .withSpi(m_spi)
            .withChipSelectPin(m_hw_cfg.chip_select_pin)
            .withClockRate(m_hw_cfg.spi_clock_hz);

    const auto converter_parameters =
        tmc51x0::ConverterParameters{}
            .withMicrostepsPerRealPositionUnit(m_kin_cfg.microsteps_per_mm);

    const auto driver_parameters_real =
        tmc51x0::DriverParameters{}
            .withRunCurrent(m_drv_cfg.run_current_percent)
            .withPwmOffset(m_drv_cfg.pwm_offset_percent)
            .withPwmGradient(m_drv_cfg.pwm_gradient_percent)
            .withMotorDirection(
                m_drv_cfg.reverse_direction
                    ? tmc51x0::ReverseDirection
                    : tmc51x0::ForwardDirection
            )
            .withStealthChopThreshold(m_drv_cfg.stealth_chop_threshold);

    const auto controller_parameters_real =
        tmc51x0::ControllerParameters{}
            .withRampMode(tmc51x0::PositionMode)
            .withMaxVelocity(m_track_cfg.max_velocity)
            .withMaxAcceleration(m_track_cfg.max_acceleration)
            .withStartVelocity(m_track_cfg.start_velocity)
            .withStopVelocity(m_track_cfg.stop_velocity)
            .withFirstVelocity(m_track_cfg.first_velocity)
            .withFirstAcceleration(m_track_cfg.first_acceleration)
            .withMaxDeceleration(m_track_cfg.max_deceleration)
            .withFirstDeceleration(m_track_cfg.first_deceleration);

    m_stepper.setupSpi(spi_parameters);
    m_stepper.converter.setup(converter_parameters);

    const auto driver_chip =
        m_stepper.converter.driverParametersRealToChip(driver_parameters_real);
    m_stepper.driver.setup(driver_chip);
    m_stepper.driver.setEnableHardwarePin(m_hw_cfg.enable_hardware_pin);

    const auto controller_chip =
        m_stepper.converter.controllerParametersRealToChip(controller_parameters_real);
    m_stepper.controller.setup(controller_chip);

    if (!m_stepper.communicating()) {
        setError_("No TMC5160 SPI communication.");
        return false;
    }

    if (m_stepper.controller.stepAndDirectionMode()) {
        setError_("Step/Dir mode enabled. SPI motion cannot run.");
        return false;
    }

    m_stepper.driver.enable();

    m_stepper.controller.beginRampToZeroVelocity();

    unsigned long t0 = millis();
    while (!m_stepper.controller.zeroVelocity()) {
        if (millis() - t0 > 3000) {
            setError_("Timeout waiting TMC zero velocity.");
            return false;
        }
        delay(1);
    }

    m_stepper.controller.endRampToZeroVelocity();
    m_stepper.controller.zeroActualPosition();

    return true;
}

void PlungerSpiPositionStreamer::applyStreamingTarget_(float t_s)
{
    if (!m_planner) return;

    PlungerMotionPlanner::Sample s = m_planner->evaluate(t_s);

    // planner.plungerPos 單位是 m
    const float pos_real = s.plungerPos * m_kin_cfg.planner_meter_to_real_unit;

    const int32_t pos_chip =
        m_stepper.converter.positionRealToChip(pos_real);

    m_stepper.controller.writeTargetPosition(pos_chip);

    m_status.commanded_pos_m = s.plungerPos;
    m_status.commanded_pos_real = pos_real;
    m_status.commanded_pos_chip = pos_chip;
}

void PlungerSpiPositionStreamer::refreshMeasurements_()
{
    m_status.actual_pos_chip = m_stepper.controller.readActualPosition();
    m_status.actual_pos_real =
        m_stepper.converter.positionChipToReal(m_status.actual_pos_chip);

    m_status.actual_velocity_chip = m_stepper.controller.readActualVelocity();
    m_status.actual_velocity_real =
        m_stepper.converter.velocityChipToReal(m_status.actual_velocity_chip);

    m_status.measured_current_mA = m_ina228.getCurrent_mA();
    m_status.measured_voltage_V = m_ina228.getBusVoltage_V();
    m_status.measured_temp_C = m_ina228.readDieTemp();
}

void PlungerSpiPositionStreamer::setError_(const char* msg)
{
    m_status.fault = true;
    m_status.running = false;

    if (!msg) {
        m_status.last_error[0] = '\0';
        return;
    }

    strncpy(m_status.last_error, msg, sizeof(m_status.last_error) - 1);
    m_status.last_error[sizeof(m_status.last_error) - 1] = '\0';
}

} // namespace PlungerSpiPositionStreamerNS