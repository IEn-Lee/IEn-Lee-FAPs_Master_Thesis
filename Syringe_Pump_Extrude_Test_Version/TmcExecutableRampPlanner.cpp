#include "TmcExecutableRampPlanner.h"

#include <math.h>
#define DEBUG_TMC_EXECUTABLE_PLANNER 1

TmcExecutableRampPlanner::TmcExecutableRampPlanner()
: m_valid(false),
  m_cmd(),
  m_config(),
  m_total_duration_s(0.0f),
  m_target_distance_mm(0.0f),
  m_stroke_per_ml_mm(0.0f),
  m_t_accel_s(0.0f),
  m_t_const_s(0.0f),
  m_t_decel_s(0.0f),
  m_v_max_mm_s(0.0f),
  m_a_mm_s2(0.0f),
  m_d_mm_s2(0.0f),
  m_distance_accel_mm(0.0f),
  m_distance_const_mm(0.0f),
  m_distance_decel_mm(0.0f)
{
}

bool TmcExecutableRampPlanner::is_valid_positive(float x)
{
    return isfinite(x) && x > 0.0f;
}

float TmcExecutableRampPlanner::min_positive(float a, float b)
{
    const bool a_ok = is_valid_positive(a);
    const bool b_ok = is_valid_positive(b);

    if (a_ok && b_ok) {
        return (a < b) ? a : b;
    }

    if (a_ok) return a;
    if (b_ok) return b;

    return 0.0f;
}

float TmcExecutableRampPlanner::select_acceleration_scale(
    float viscosity_mPa_s,
    const Config& config
)
{
    if (!isfinite(viscosity_mPa_s) || viscosity_mPa_s <= 0.0f) {
        return config.high_viscosity_accel_scale;
    }

    if (viscosity_mPa_s <= config.low_viscosity_mPa_s) {
        return config.low_viscosity_accel_scale;
    }

    if (viscosity_mPa_s >= config.high_viscosity_mPa_s) {
        return config.high_viscosity_accel_scale;
    }

    return config.medium_viscosity_accel_scale;
}

bool TmcExecutableRampPlanner::build(
    const PlungerMotionPlanner& ideal_planner,
    const FlowConstraintModel::Params& flow_params,
    const FlowConstraintModel::Result& flow_result,
    const Config& config
)
{
    m_valid = false;
    m_cmd = TmcRampCommand();
    m_config = config;

    m_total_duration_s = 0.0f;
    m_target_distance_mm = 0.0f;
    m_stroke_per_ml_mm = 0.0f;

    m_t_accel_s = 0.0f;
    m_t_const_s = 0.0f;
    m_t_decel_s = 0.0f;

    m_v_max_mm_s = 0.0f;
    m_a_mm_s2 = 0.0f;
    m_d_mm_s2 = 0.0f;

    m_distance_accel_mm = 0.0f;
    m_distance_const_mm = 0.0f;
    m_distance_decel_mm = 0.0f;

    if (!flow_result.valid) {
        return false;
    }

    if (!is_valid_positive(flow_result.Q_allow)) {
        return false;
    }

    if (!is_valid_positive(flow_params.stroke_per_ml_mm)) {
        return false;
    }

    if (!is_valid_positive(flow_result.A)) {
        return false;
    }

    const float target_volume_m3 = ideal_planner.getTargetVolume_m3();

    if (!is_valid_positive(target_volume_m3)) {
        return false;
    }

    const float quantity_uL = target_volume_m3 * 1.0e9f;

    if (!is_valid_positive(quantity_uL)) {
        return false;
    }

    // Final motor distance should use the calibrated syringe conversion.
    // quantity_uL / 1000 = mL
    const float target_distance_mm =
        (quantity_uL / 1000.0f) * flow_params.stroke_per_ml_mm;

    if (!is_valid_positive(target_distance_mm)) {
        return false;
    }

    if (!is_valid_positive(quantity_uL)) {
        return false;
    }

    if (!is_valid_positive(target_distance_mm)) {
        return false;
    }

    // ------------------------------------------------------------
    // Velocity limit
    // ------------------------------------------------------------
    // flow_result.Q_allow is m^3/s.
    // flow_result.A is m^2.
    //
    // v = Q / A [m/s]
    // Convert to [mm/s].
    // ------------------------------------------------------------
    const float v_from_flow_mm_s =
        (flow_result.Q_allow / flow_result.A) * 1.0e3f;

    const float v_from_ideal_mm_s =
        ideal_planner.getPeakVelocity_mps() * 1.0e3f;

    const float v_from_user_mm_s =
        flow_params.max_linear_speed_mm_s;

    float v_limit_mm_s = min_positive(v_from_flow_mm_s, v_from_ideal_mm_s);
    v_limit_mm_s = min_positive(v_limit_mm_s, v_from_user_mm_s);

    // ===== Planner Inner Value =====
    #if DEBUG_TMC_EXECUTABLE_PLANNER
    Serial.println("========== TMC PLANNER VELOCITY ==========");
    Serial.print("v_from_flow_mm_s = ");
    Serial.println(v_from_flow_mm_s, 6);

    Serial.print("v_from_ideal_mm_s = ");
    Serial.println(v_from_ideal_mm_s, 6);

    Serial.print("v_from_user_mm_s = ");
    Serial.println(v_from_user_mm_s, 6);

    Serial.print("v_limit_mm_s = ");
    Serial.println(v_limit_mm_s, 6);
    Serial.println("==========================================");
    #endif
    // ===============================

    if (!is_valid_positive(v_limit_mm_s)) {
        return false;
    }

    float velocity_safety_scale = config.velocity_safety_scale;
    if (!isfinite(velocity_safety_scale) || velocity_safety_scale <= 0.0f) {
        velocity_safety_scale = 1.0f;
    }
    if (velocity_safety_scale > 1.0f) {
        velocity_safety_scale = 1.0f;
    }

    float v_max_mm_s = v_limit_mm_s * velocity_safety_scale;

    if (v_max_mm_s < config.min_velocity_mm_s) {
        v_max_mm_s = config.min_velocity_mm_s;
    }

    // Never allow the lower bound to exceed the physical cap.
    if (v_max_mm_s > v_limit_mm_s) {
        v_max_mm_s = v_limit_mm_s;
    }

    if (!is_valid_positive(v_max_mm_s)) {
        return false;
    }

    // ------------------------------------------------------------
    // Acceleration / deceleration strategy
    // ------------------------------------------------------------
    // This does not create separate low/high viscosity paths.
    // It only changes ramp aggressiveness.
    // ------------------------------------------------------------
    const float ideal_duration_s = ideal_planner.totalDuration();

    if (!is_valid_positive(ideal_duration_s)) {
        return false;
    }

    float accel_scale =
        select_acceleration_scale(flow_params.viscosity, config);

    if (!isfinite(accel_scale) || accel_scale <= 0.0f) {
        accel_scale = config.high_viscosity_accel_scale;
    }

    // Use the ideal duration as timing reference.
    // 15% of ideal duration is used as a starting acceleration time.
    float accel_time_ref_s = ideal_duration_s * 0.15f;

    if (accel_time_ref_s < 0.05f) {
        accel_time_ref_s = 0.05f;
    }

    float base_accel_mm_s2 = v_max_mm_s / accel_time_ref_s;

    if (!is_valid_positive(base_accel_mm_s2)) {
        return false;
    }

    float a_mm_s2 = base_accel_mm_s2 * accel_scale;
    float d_mm_s2 = base_accel_mm_s2 * accel_scale;

    // ===== Planner Inner Value =====
    #if DEBUG_TMC_EXECUTABLE_PLANNER
    Serial.println("========== TMC PLANNER ACCEL ==========");
    Serial.print("ideal_duration_s = ");
    Serial.println(ideal_duration_s, 6);

    Serial.print("accel_time_ref_s = ");
    Serial.println(accel_time_ref_s, 6);

    Serial.print("base_accel_mm_s2 = ");
    Serial.println(base_accel_mm_s2, 6);

    Serial.print("accel_scale = ");
    Serial.println(accel_scale, 6);

    Serial.print("final_a_mm_s2 = ");
    Serial.println(a_mm_s2, 6);

    Serial.print("final_d_mm_s2 = ");
    Serial.println(d_mm_s2, 6);
    Serial.println("=======================================");
    #endif
    // ===============================

    if (a_mm_s2 < config.min_acceleration_mm_s2) {
        a_mm_s2 = config.min_acceleration_mm_s2;
    }

    if (d_mm_s2 < config.min_deceleration_mm_s2) {
        d_mm_s2 = config.min_deceleration_mm_s2;
    }

    if (is_valid_positive(config.max_acceleration_mm_s2) &&
        a_mm_s2 > config.max_acceleration_mm_s2) {
        a_mm_s2 = config.max_acceleration_mm_s2;
    }

    if (is_valid_positive(config.max_deceleration_mm_s2) &&
        d_mm_s2 > config.max_deceleration_mm_s2) {
        d_mm_s2 = config.max_deceleration_mm_s2;
    }

    if (!is_valid_positive(a_mm_s2) || !is_valid_positive(d_mm_s2)) {
        return false;
    }

    // ------------------------------------------------------------
    // Convert to trapezoidal / triangular executable approximation
    // ------------------------------------------------------------
    float distance_accel_mm =
        (v_max_mm_s * v_max_mm_s) / (2.0f * a_mm_s2);

    float distance_decel_mm =
        (v_max_mm_s * v_max_mm_s) / (2.0f * d_mm_s2);

    float distance_const_mm = 0.0f;

    // If target distance is too short, reduce peak velocity and use
    // a triangular profile.
    if ((distance_accel_mm + distance_decel_mm) > target_distance_mm) {
        const float numerator =
            2.0f * target_distance_mm * a_mm_s2 * d_mm_s2;

        const float denominator =
            a_mm_s2 + d_mm_s2;

        if (!is_valid_positive(numerator) ||
            !is_valid_positive(denominator)) {
            return false;
        }

        v_max_mm_s = sqrtf(numerator / denominator);

        if (!is_valid_positive(v_max_mm_s)) {
            return false;
        }

        distance_accel_mm =
            (v_max_mm_s * v_max_mm_s) / (2.0f * a_mm_s2);

        distance_decel_mm =
            (v_max_mm_s * v_max_mm_s) / (2.0f * d_mm_s2);

        distance_const_mm = 0.0f;
    }
    else {
        distance_const_mm =
            target_distance_mm - distance_accel_mm - distance_decel_mm;
    }

    const float t_accel_s = v_max_mm_s / a_mm_s2;
    const float t_decel_s = v_max_mm_s / d_mm_s2;

    float t_const_s = 0.0f;
    if (distance_const_mm > 0.0f) {
        t_const_s = distance_const_mm / v_max_mm_s;
    }

    const float executable_duration_s =
        t_accel_s + t_const_s + t_decel_s;

    // ===== Planner Inner Value =====
    #if DEBUG_TMC_EXECUTABLE_PLANNER
    Serial.println("========== TMC PLANNER PROFILE ==========");
    Serial.print("target_distance_mm = ");
    Serial.println(target_distance_mm, 6);

    Serial.print("distance_accel_mm = ");
    Serial.println(distance_accel_mm, 6);

    Serial.print("distance_const_mm = ");
    Serial.println(distance_const_mm, 6);

    Serial.print("distance_decel_mm = ");
    Serial.println(distance_decel_mm, 6);

    Serial.print("t_accel_s = ");
    Serial.println(t_accel_s, 6);

    Serial.print("t_const_s = ");
    Serial.println(t_const_s, 6);

    Serial.print("t_decel_s = ");
    Serial.println(t_decel_s, 6);

    Serial.print("executable_duration_s = ");
    Serial.println(executable_duration_s, 6);
    Serial.println("=========================================");
    #endif
    // ===============================

    if (!is_valid_positive(executable_duration_s)) {
        return false;
    }

    // ------------------------------------------------------------
    // Fill TmcRampCommand
    // ------------------------------------------------------------
    m_cmd.quantity_uL = quantity_uL;
    m_cmd.stroke_per_ml_mm = flow_params.stroke_per_ml_mm;
    m_cmd.target_distance_mm = target_distance_mm;

    m_cmd.max_velocity_mm_s = v_max_mm_s;
    m_cmd.max_acceleration_mm_s2 = a_mm_s2;
    m_cmd.max_deceleration_mm_s2 = d_mm_s2;

    float start_velocity =
        (isfinite(config.start_velocity_mm_s) && config.start_velocity_mm_s >= 0.0f)
        ? config.start_velocity_mm_s
        : 0.0f;

    float stop_velocity =
        (isfinite(config.stop_velocity_mm_s) && config.stop_velocity_mm_s >= 0.0f)
        ? config.stop_velocity_mm_s
        : 0.0f;

    if (start_velocity > v_max_mm_s) {
        start_velocity = v_max_mm_s;
    }

    if (stop_velocity > v_max_mm_s) {
        stop_velocity = v_max_mm_s;
    }

    m_cmd.start_velocity_mm_s = start_velocity;
    m_cmd.stop_velocity_mm_s = stop_velocity;

    float first_velocity =
        v_max_mm_s * config.first_velocity_ratio;

    if (!isfinite(first_velocity) || first_velocity < 0.0f) {
        first_velocity = 0.0f;
    }

    if (first_velocity > v_max_mm_s) {
        first_velocity = v_max_mm_s;
    }

    m_cmd.first_velocity_mm_s = first_velocity;

    float first_accel =
        a_mm_s2 * config.first_acceleration_ratio;

    float first_decel =
        d_mm_s2 * config.first_deceleration_ratio;

    if (!is_valid_positive(first_accel)) {
        first_accel = a_mm_s2;
    }

    if (!is_valid_positive(first_decel)) {
        first_decel = d_mm_s2;
    }

    if (first_accel > a_mm_s2) {
        first_accel = a_mm_s2;
    }

    if (first_decel > d_mm_s2) {
        first_decel = d_mm_s2;
    }

    m_cmd.first_acceleration_mm_s2 = first_accel;
    m_cmd.first_deceleration_mm_s2 = first_decel;

    m_cmd.expected_duration_s = executable_duration_s;

    // Direction is still owned by MotorControlScreen / UI setting for now.
    // The planner only generates magnitude and ramp shape.
    m_cmd.reverse_direction = config.reverse_direction;

    if (!is_valid_positive(m_cmd.quantity_uL) ||
        !is_valid_positive(m_cmd.stroke_per_ml_mm) ||
        !is_valid_positive(m_cmd.target_distance_mm) ||
        !is_valid_positive(m_cmd.max_velocity_mm_s) ||
        !is_valid_positive(m_cmd.max_acceleration_mm_s2) ||
        !is_valid_positive(m_cmd.max_deceleration_mm_s2) ||
        !is_valid_positive(m_cmd.expected_duration_s)) {
        m_cmd = TmcRampCommand();
        return false;
    }

    m_cmd.valid = true;

    // ------------------------------------------------------------
    // Cache for evaluate()
    // ------------------------------------------------------------
    m_valid = true;

    m_total_duration_s = executable_duration_s;
    m_target_distance_mm = target_distance_mm;
    m_stroke_per_ml_mm = flow_params.stroke_per_ml_mm;

    m_t_accel_s = t_accel_s;
    m_t_const_s = t_const_s;
    m_t_decel_s = t_decel_s;

    m_v_max_mm_s = v_max_mm_s;
    m_a_mm_s2 = a_mm_s2;
    m_d_mm_s2 = d_mm_s2;

    m_distance_accel_mm = distance_accel_mm;
    m_distance_const_mm = distance_const_mm;
    m_distance_decel_mm = distance_decel_mm;

    return true;
}

bool TmcExecutableRampPlanner::isValid() const
{
    return m_valid;
}

const TmcRampCommand& TmcExecutableRampPlanner::command() const
{
    return m_cmd;
}

float TmcExecutableRampPlanner::totalDuration() const
{
    return m_total_duration_s;
}

TmcExecutableRampPlanner::Sample TmcExecutableRampPlanner::evaluate(float t_s) const
{
    Sample out;

    if (!m_valid || !m_cmd.valid) {
        return out;
    }

    if (!isfinite(t_s) || t_s < 0.0f) {
        t_s = 0.0f;
    }

    if (t_s > m_total_duration_s) {
        t_s = m_total_duration_s;
    }

    out.t_s = t_s;

    float distance_mm = 0.0f;
    float velocity_mm_s = 0.0f;

    if (t_s <= m_t_accel_s) {
        distance_mm =
            0.5f * m_a_mm_s2 * t_s * t_s;

        velocity_mm_s =
            m_a_mm_s2 * t_s;
    }
    else if (t_s <= (m_t_accel_s + m_t_const_s)) {
        const float tau =
            t_s - m_t_accel_s;

        distance_mm =
            m_distance_accel_mm +
            m_v_max_mm_s * tau;

        velocity_mm_s =
            m_v_max_mm_s;
    }
    else {
        const float tau =
            t_s - m_t_accel_s - m_t_const_s;

        distance_mm =
            m_distance_accel_mm +
            m_distance_const_mm +
            m_v_max_mm_s * tau -
            0.5f * m_d_mm_s2 * tau * tau;

        velocity_mm_s =
            m_v_max_mm_s - m_d_mm_s2 * tau;

        if (velocity_mm_s < 0.0f) {
            velocity_mm_s = 0.0f;
        }
    }

    if (distance_mm < 0.0f) {
        distance_mm = 0.0f;
    }

    if (distance_mm > m_target_distance_mm) {
        distance_mm = m_target_distance_mm;
    }

    out.distance_mm = distance_mm;
    out.velocity_mm_s = velocity_mm_s;

    if (m_stroke_per_ml_mm > 0.0f) {
        out.volume_uL =
            (distance_mm / m_stroke_per_ml_mm) * 1000.0f;
    }
    else {
        out.volume_uL = 0.0f;
    }

    if (out.volume_uL > m_cmd.quantity_uL) {
        out.volume_uL = m_cmd.quantity_uL;
    }

    if (out.volume_uL < 0.0f) {
        out.volume_uL = 0.0f;
    }

    return out;
}