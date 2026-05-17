#include "FlowConstraintModel.h"

// =========================
// Utility
// =========================
bool FlowConstraintModel::is_valid_positive(float x)
{
    return isfinite(x) && x > 0.0f;
}

float FlowConstraintModel::safe_positive(float x)
{
    return (isfinite(x) && x > 0.0f) ? x : 0.0f;
}

float FlowConstraintModel::plunger_area_from_R(float R)
{
    return PI * R * R;
}

float FlowConstraintModel::viscosity_Pa_s_from_mPa_s(float viscosity)
{
    return viscosity / 1000.0f;
}

float FlowConstraintModel::linear_velocity_mps_from_flow(
    const Params& p,
    float Q_m3_s
)
{
    if (!is_valid_positive(p.stroke_per_ml_mm) ||
        !is_valid_positive(Q_m3_s)) {
        return 0.0f;
    }

    // Q [m^3/s] * 1e6 = mL/s
    // v [mm/s] = flow [mL/s] * stroke_per_ml_mm
    // v [m/s] = v [mm/s] / 1000
    const float flow_mL_s = Q_m3_s * 1.0e6f;
    const float v_mm_s = flow_mL_s * p.stroke_per_ml_mm;

    return v_mm_s * 1.0e-3f;
}

float FlowConstraintModel::flow_from_linear_velocity_mps(
    const Params& p,
    float v_m_s
)
{
    if (!is_valid_positive(p.stroke_per_ml_mm) ||
        !is_valid_positive(v_m_s)) {
        return 0.0f;
    }

    // v [m/s] -> v [mm/s]
    // flow [mL/s] = v [mm/s] / stroke_per_ml_mm
    // Q [m^3/s] = flow [mL/s] * 1e-6
    const float v_mm_s = v_m_s * 1000.0f;
    const float flow_mL_s = v_mm_s / p.stroke_per_ml_mm;

    return flow_mL_s * 1.0e-6f;
}

float FlowConstraintModel::estimate_plunger_second_moment(const Params& p)
{
    const float a = p.shaft;       // plunger shaft outer width [m]
    const float d = p.shaft_walls; // plunger shaft wall thickness [m]

    if (!is_valid_positive(a) || !is_valid_positive(d)) {
        return 0.0f;
    }

    if (d >= a) {
        return 0.0f;
    }

    // Conservative fallback estimate based on plunger shaft wall geometry.
    // This follows the simplified model:
    //
    // I_est = (d * a^3 + (a - d) * d^3) / 12
    //
    // Unit:
    // m * m^3 = m^4
    const float I_est =
        (d * a * a * a + (a - d) * d * d * d) / 12.0f;

    if (!is_valid_positive(I_est)) {
        return 0.0f;
    }

    return I_est;
}

// =========================
// Structure limit
// =========================
void FlowConstraintModel::calculate_struct_flow_limit(
    const Params& p,
    float& Q_struct,
    float& v_struct,
    float& F_crit,
    float& F_allow,
    float& I_used,
    bool& I_auto_estimated
)
{
    Q_struct = 0.0f;
    v_struct = 0.0f;
    F_crit   = 0.0f;
    F_allow  = 0.0f;
    I_used   = 0.0f;
    I_auto_estimated = false;

    const float mu = viscosity_Pa_s_from_mPa_s(p.viscosity); // Pa.s
    const float A  = plunger_area_from_R(p.R);               // m^2

    if (mu <= 0.0f ||
        A <= 0.0f ||
        p.r <= 0.0f ||
        p.l <= 0.0f ||
        p.R <= 0.0f ||
        p.S <= 0.0f ||
        p.L <= 0.0f ||
        p.K_buckling <= 0.0f) {
        return;
    }

    // I_plunger selection rule:
    // - If user provided a valid positive I_plunger, use it directly.
    // - If I_plunger is zero, empty, negative, NaN, or invalid, estimate it from shaft geometry.
    if (is_valid_positive(p.I_plunger)) {
        I_used = p.I_plunger;
        I_auto_estimated = false;
    } else {
        I_used = estimate_plunger_second_moment(p);
        I_auto_estimated = true;
    }

    if (!is_valid_positive(I_used)) {
        return;
    }

    F_crit  = (PI * PI * p.E * I_used) / ((p.K_buckling * p.L) * (p.K_buckling * p.L));
    F_allow = F_crit / p.S;

    // Q_max = (F_allow * r^4) / (8 * mu * l * R^2)
    const float r4 = p.r * p.r * p.r * p.r;
    Q_struct = (F_allow * r4) / (8.0f * mu * p.l * p.R * p.R);
    v_struct = linear_velocity_mps_from_flow(p, Q_struct);

    Q_struct = safe_positive(Q_struct);
    v_struct = safe_positive(v_struct);
    F_crit   = safe_positive(F_crit);
    F_allow  = safe_positive(F_allow);
}

// =========================
// Motor-current limit
// =========================
void FlowConstraintModel::flow_limit_from_current(
    const Params& p,
    float& Q_motor,
    float& v_motor,
    float& dP_motor,
    float& F_motor
)
{
    Q_motor  = 0.0f;
    v_motor  = 0.0f;
    dP_motor = 0.0f;
    F_motor  = 0.0f;

    const float mu = viscosity_Pa_s_from_mPa_s(p.viscosity); // Pa.s
    const float A  = plunger_area_from_R(p.R);               // m^2

    if (mu <= 0.0f ||
        A <= 0.0f ||
        p.lead_pitch <= 0.0f ||
        p.eta <= 0.0f ||
        p.Kt <= 0.0f ||
        p.I_limit <= 0.0f ||
        p.r <= 0.0f ||
        p.l <= 0.0f) {
        return;
    }

    const float tau_max = p.I_limit * p.Kt;                          // Nm
    F_motor             = (2.0f * PI * p.eta * tau_max) / p.lead_pitch; // N
    dP_motor            = F_motor / A;                               // Pa

    const float r4 = p.r * p.r * p.r * p.r;
    Q_motor = (dP_motor * PI * r4) / (8.0f * mu * p.l);              // m^3/s
    v_motor = linear_velocity_mps_from_flow(p, Q_motor);              // m/s

    Q_motor  = safe_positive(Q_motor);
    v_motor  = safe_positive(v_motor);
    dP_motor = safe_positive(dP_motor);
    F_motor  = safe_positive(F_motor);
}

// =========================
// RPM limit
// =========================
void FlowConstraintModel::flow_limit_from_rpm(
    const Params& p,
    float& Q_rpm,
    float& v_rpm
)
{
    Q_rpm = 0.0f;
    v_rpm = 0.0f;

    if (!is_valid_positive(p.max_rpm) ||
        !is_valid_positive(p.lead_pitch) ||
        !is_valid_positive(p.stroke_per_ml_mm)) {
        return;
    }

    const float rev_per_sec = p.max_rpm / 60.0f;

    v_rpm = rev_per_sec * p.lead_pitch;          // m/s
    Q_rpm = flow_from_linear_velocity_mps(p, v_rpm);

    Q_rpm = safe_positive(Q_rpm);
    v_rpm = safe_positive(v_rpm);
}

// =========================
// Step-frequency limit
// =========================
void FlowConstraintModel::flow_limit_from_step_frequency(
    const Params& p,
    float& Q_step,
    float& v_step
)
{
    Q_step = 0.0f;
    v_step = 0.0f;

    if (!is_valid_positive(p.steps_per_rev) ||
        !is_valid_positive(p.max_step_freq) ||
        !is_valid_positive(p.lead_pitch) ||
        !is_valid_positive(p.stroke_per_ml_mm)) {
        return;
    }

    const float rev_per_sec = p.max_step_freq / p.steps_per_rev;

    v_step = rev_per_sec * p.lead_pitch;         // m/s
    Q_step = flow_from_linear_velocity_mps(p, v_step);

    Q_step = safe_positive(Q_step);
    v_step = safe_positive(v_step);
}

void FlowConstraintModel::calculate_acceleration_limit(
    const Params& p,
    Result& out
)
{
    out.F_capacity = 0.0f;
    out.F_pressure_at_Q_allow = 0.0f;
    out.F_friction_used = 0.0f;
    out.F_accel_available = 0.0f;

    out.moving_mass_kg = 0.0f;
    out.a_allow_m_s2 = 0.0f;
    out.a_allow_mm_s2 = 0.0f;

    if (!is_valid_positive(out.F_allow) ||
        !is_valid_positive(out.F_motor)) {
        return;
    }

    out.F_capacity = fminf(out.F_allow, out.F_motor);

    // Calculate pressure force at Q_allow for debug/reference.
    // Default model does not subtract this unless use_load_terms_for_accel is true.
    if (is_valid_positive(out.Q_allow) &&
        is_valid_positive(out.mu) &&
        is_valid_positive(p.l) &&
        is_valid_positive(p.r) &&
        is_valid_positive(out.A)) {

        const float r4 = p.r * p.r * p.r * p.r;

        if (is_valid_positive(r4)) {
            const float dP_at_Q_allow =
                (8.0f * out.mu * p.l * out.Q_allow) / (PI * r4);

            out.F_pressure_at_Q_allow =
                dP_at_Q_allow * out.A;
        }
    }

    out.F_friction_used =
        is_valid_positive(p.friction_force_N)
        ? p.friction_force_N
        : 0.0f;

    out.moving_mass_kg =
        is_valid_positive(p.moving_mass_kg)
        ? p.moving_mass_kg
        : 0.5f;

    if (p.use_load_terms_for_accel) {
        out.F_accel_available =
            out.F_capacity
            - out.F_pressure_at_Q_allow
            - out.F_friction_used;
    }
    else {
        // First-order model:
        // F_allow already includes the structural safety factor.
        // Pressure and friction are monitored but not subtracted.
        out.F_accel_available = out.F_capacity;
    }

    if (out.F_accel_available < 0.0f) {
        out.F_accel_available = 0.0f;
    }

    float util =
        (isfinite(p.acceleration_utilization) &&
         p.acceleration_utilization > 0.0f &&
         p.acceleration_utilization <= 1.0f)
        ? p.acceleration_utilization
        : 1.0f;

    if (is_valid_positive(out.moving_mass_kg) &&
        is_valid_positive(out.F_accel_available)) {

        out.a_allow_m_s2 =
            out.F_accel_available / out.moving_mass_kg;

        out.a_allow_mm_s2 =
            out.a_allow_m_s2 * 1000.0f * util;
    }

    out.F_capacity = safe_positive(out.F_capacity);
    out.F_pressure_at_Q_allow = safe_positive(out.F_pressure_at_Q_allow);
    out.F_friction_used = safe_positive(out.F_friction_used);
    out.F_accel_available = safe_positive(out.F_accel_available);
    out.moving_mass_kg = safe_positive(out.moving_mass_kg);
    out.a_allow_m_s2 = safe_positive(out.a_allow_m_s2);
    out.a_allow_mm_s2 = safe_positive(out.a_allow_mm_s2);
}

// =========================
// Main compute
// =========================
// FlowConstraintModel::Result ==> 這個 function 回傳一個 Result(return type)，而這個 Result 是定義在 FlowConstraintModel 裡面
// FlowConstraintModel::compute ==> 這個 compute 是屬於 FlowConstraintModel 這個 class 的函式
FlowConstraintModel::Result FlowConstraintModel::compute(const Params& p)
{
    Result out;

    out.mu = viscosity_Pa_s_from_mPa_s(p.viscosity);
    out.A  = plunger_area_from_R(p.R);

    // Basic sanity
    if (out.mu <= 0.0f || out.A <= 0.0f || p.r <= 0.0f || p.l <= 0.0f || p.lead_pitch <= 0.0f) {
        out.valid = false;
        return out;
    }

    // ----- 1) Structure side -----
    calculate_struct_flow_limit(
        p,
        out.Q_struct,
        out.v_struct,
        out.F_crit,
        out.F_allow,
        out.I_used,
        out.I_auto_estimated
    );

    // ----- 2) Motor-current side -----
    flow_limit_from_current(
        p,
        out.Q_motor,
        out.v_motor,
        out.dP_motor,
        out.F_motor
    );

    // ----- 3) RPM side -----
    flow_limit_from_rpm(
        p,
        out.Q_rpm,
        out.v_rpm
    );

    // ----- 4) Step-frequency side -----
    flow_limit_from_step_frequency(
        p,
        out.Q_step,
        out.v_step
    );

    // ----- 5) User-defined linear speed limit -----
    // max_linear_speed_mm_s is defined in customized parameters.
    // It limits the maximum plunger linear velocity directly.
    //
    // Unit conversion:
    // max_linear_speed_mm_s [mm/s] -> v_user_speed [m/s]
    // Q is converted using stroke_per_ml_mm, not plunger area.
    float v_user_speed = 0.0f;   // m/s
    float Q_user_speed = 0.0f;   // m^3/s

    if (p.max_linear_speed_mm_s > 0.0f) {
        v_user_speed = p.max_linear_speed_mm_s * 1.0e-3f;
        Q_user_speed = flow_from_linear_velocity_mps(p, v_user_speed);
    }

    v_user_speed = safe_positive(v_user_speed);
    Q_user_speed = safe_positive(Q_user_speed);

    // ----- 6) Combined safe limits -----
    // Force-side safety:
    // limited by structural buckling and motor current.
    out.Q_force_safe = fminf(out.Q_struct, out.Q_motor);

    // Hardware speed safety:
    // limited by motor rpm and step frequency.
    // beta is applied here as a hardware-side safety factor.
    float Q_hardware_speed_safe =
        safe_positive(p.beta) * fminf(out.Q_rpm, out.Q_step);

    // User-defined speed limit:
    // This is treated as an absolute operator-defined maximum.
    // Therefore it is NOT multiplied by beta again.
    if (Q_user_speed > 0.0f) {
        out.Q_speed_safe = fminf(Q_hardware_speed_safe, Q_user_speed);
    } else {
        out.Q_speed_safe = Q_hardware_speed_safe;
    }

    // Final allowed flow:
    // must satisfy both force-side and speed-side limits.
    out.Q_allow = fminf(out.Q_force_safe, out.Q_speed_safe);

    out.Q_force_safe = safe_positive(out.Q_force_safe);
    out.Q_speed_safe = safe_positive(out.Q_speed_safe);
    out.Q_allow      = safe_positive(out.Q_allow);

    out.v_allow =
        linear_velocity_mps_from_flow(p, out.Q_allow);

    out.v_allow = safe_positive(out.v_allow);

    // ----- 7) Acceleration upper bound -----
    // This is a physical safety ceiling.
    // It does not solve the TMC low-acceleration execution issue.
    calculate_acceleration_limit(p, out);

    out.valid =
        is_valid_positive(out.Q_allow) &&
        is_valid_positive(out.Q_force_safe) &&
        is_valid_positive(out.Q_speed_safe) &&
        is_valid_positive(out.v_allow) &&
        is_valid_positive(out.A) &&
        is_valid_positive(out.mu);

    return out;
}



