#include "FlowConstraintModel.h"

// =========================
// Utility
// =========================
float FlowConstraintModel::safe_positive(float x)
{
    return (x > 0.0f) ? x : 0.0f;
}

float FlowConstraintModel::plunger_area_from_R(float R)
{
    return PI * R * R;
}

float FlowConstraintModel::viscosity_Pa_s_from_mPa_s(float viscosity)
{
    return viscosity / 1000.0f;
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
    float& I_used
)
{
    Q_struct = 0.0f;
    v_struct = 0.0f;
    F_crit   = 0.0f;
    F_allow  = 0.0f;
    I_used   = 0.0f;

    const float mu = viscosity_Pa_s_from_mPa_s(p.viscosity); // Pa.s
    const float A  = plunger_area_from_R(p.R);               // m^2

    if (mu <= 0.0f || A <= 0.0f || p.l <= 0.0f || p.R <= 0.0f || p.S <= 0.0f || p.L <= 0.0f || p.K_buckling <= 0.0f) {
        return;
    }

    // If user does not provide I_plunger, estimate a very small fallback value
    if (p.I_plunger > 0.0f) {
        I_used = p.I_plunger;
    } else {
        // Keep naming consistent with your original code logic
        // NOTE: this is only a placeholder estimate for I_plunger
        float I_est = (p.shaft_walls / (p.shaft * p.shaft * p.shaft)) / 12.0f * 1e-12f;
        if (I_est < 1e-18f) I_est = 1e-18f;
        I_used = I_est;
    }

    F_crit  = (PI * PI * p.E * I_used) / ((p.K_buckling * p.L) * (p.K_buckling * p.L));
    F_allow = F_crit / p.S;

    // Q_max = (F_allow * r^4) / (8 * mu * l * R^2)
    const float r4 = p.r * p.r * p.r * p.r;
    Q_struct = (F_allow * r4) / (8.0f * mu * p.l * p.R * p.R);
    v_struct = Q_struct / A;

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

    if (mu <= 0.0f || A <= 0.0f || p.lead_pitch <= 0.0f || p.eta <= 0.0f || p.r <= 0.0f || p.l <= 0.0f) {
        return;
    }

    const float tau_max = p.I_limit * p.Kt;                          // Nm
    F_motor             = (2.0f * PI * p.eta * tau_max) / p.lead_pitch; // N
    dP_motor            = F_motor / A;                               // Pa

    const float r4 = p.r * p.r * p.r * p.r;
    Q_motor = (dP_motor * PI * r4) / (8.0f * mu * p.l);              // m^3/s
    v_motor = Q_motor / A;                                            // m/s

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

    const float A = plunger_area_from_R(p.R);
    if (A <= 0.0f) return;

    const float rev_per_sec = p.max_rpm / 60.0f;
    v_rpm = rev_per_sec * p.lead_pitch;
    Q_rpm = A * v_rpm;

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

    const float A = plunger_area_from_R(p.R);
    if (A <= 0.0f || p.steps_per_rev <= 0.0f) return;

    const float rev_per_sec = p.max_step_freq / p.steps_per_rev;
    v_step = rev_per_sec * p.lead_pitch;
    Q_step = A * v_step;

    Q_step = safe_positive(Q_step);
    v_step = safe_positive(v_step);
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
        out.I_used
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

    // ----- 5) Combined safe limits -----
    out.Q_force_safe = min(out.Q_struct, out.Q_motor);
    out.Q_speed_safe = safe_positive(p.beta) * min(out.Q_rpm, out.Q_step);
    out.Q_allow      = min(out.Q_force_safe, out.Q_speed_safe);

    out.Q_force_safe = safe_positive(out.Q_force_safe);
    out.Q_speed_safe = safe_positive(out.Q_speed_safe);
    out.Q_allow      = safe_positive(out.Q_allow);

    out.valid = true;
    return out;
}