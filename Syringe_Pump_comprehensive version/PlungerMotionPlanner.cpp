#include "PlungerMotionPlanner.h"

// =========================
// Utility
// =========================
float PlungerMotionPlanner::safe_positive(float x)
{
    return (x > 0.0f) ? x : 0.0f;
}

// =========================
// Closed-form propagation
// =========================
void PlungerMotionPlanner::propagate_segment(
    float x0, float v0, float a0, float j, float tau,
    float& x1, float& v1, float& a1
)
{
    x1 = x0 + v0 * tau + 0.5f * a0 * tau * tau + (1.0f / 6.0f) * j * tau * tau * tau;
    v1 = v0 + a0 * tau + 0.5f * j * tau * tau;
    a1 = a0 + j * tau;
}

void PlungerMotionPlanner::segment_state(
    float x0, float v0, float a0, float j, float t_local,
    float& x, float& v, float& a
)
{
    x = x0 + v0 * t_local + 0.5f * a0 * t_local * t_local + (1.0f / 6.0f) * j * t_local * t_local * t_local;
    v = v0 + a0 * t_local + 0.5f * j * t_local * t_local;
    a = a0 + j * t_local;
}

// =========================
// Flow-side estimation
// =========================
float PlungerMotionPlanner::pressure_from_flow(
    float mu, float l, float r, float Q
)
{
    if (mu <= 0.0f || l <= 0.0f || r <= 0.0f) return 0.0f;
    const float r4 = r * r * r * r;
    if (r4 <= 0.0f) return 0.0f;

    // ΔP = (8 μ l Q) / (π r^4)
    return (8.0f * mu * l * Q) / (PI * r4);
}

float PlungerMotionPlanner::estimate_current_from_flow(float Q) const
{
    if (m_params.r <= 0.0f || m_params.eta <= 0.0f || m_params.Kt <= 0.0f) return 0.0f;

    // Kp = (8 * mu * l) / (pi * r^4)
    const float r4 = m_params.r * m_params.r * m_params.r * m_params.r;
    if (r4 <= 0.0f) return 0.0f;

    const float Kp = (8.0f * m_mu * m_params.l) / (PI * r4); // Pa / (m^3/s)

    // F = (Kp * Q) * A
    // tau = F * lead_pitch / (2*pi*eta)
    // I = tau / Kt
    return ((Kp * Q * m_A) * m_params.lead_pitch) /
           (2.0f * PI * m_params.eta * m_params.Kt);
}

void PlungerMotionPlanner::fill_derived_outputs(Sample& s) const
{
    // x [m] -> dispensed volume [m^3]
    const float volume_m3 = s.plungerPos * m_A;

    // v [m/s] -> Q [m^3/s]
    const float Q = s.plungerVel * m_A;

    const float dP = pressure_from_flow(m_mu, m_params.l, m_params.r, Q);
    const float F  = dP * m_A;
    const float I  = estimate_current_from_flow(Q);

    s.volume       = volume_m3 * 1.0e9f;  // uL
    s.flow         = Q;
    s.flow_uL_s    = Q * 1.0e9f;          // uL/s
    s.pressure     = dP;
    s.pressure_bar = dP / 1.0e5f;
    s.force        = F;
    s.currentEst   = I;
}

// =========================
// Build
// =========================
bool PlungerMotionPlanner::build(
    const Command& cmd,
    const FlowConstraintModel::Params& params,
    const FlowConstraintModel::Result& limits
)
{
    m_params = params;
    m_limits = limits;
    m_cmd    = cmd;

    m_A            = FlowConstraintModel::plunger_area_from_R(params.R);                // m^2
    m_mu           = FlowConstraintModel::viscosity_Pa_s_from_mPa_s(params.viscosity);  // Pa.s
    m_targetVolume = cmd.volume * 1.0e-9f;                                              // uL -> m^3
    m_targetStroke = 0.0f;
    m_v_allow      = 0.0f;
    m_v_peak       = 0.0f;
    m_a_max        = 0.0f;
    m_j_max        = 0.0f;
    m_T_total      = 0.0f;
    m_valid        = false;

    for (int i = 0; i < 7; i++) {
        m_durations[i] = 0.0f;
        m_jerks[i]     = 0.0f;
        m_edges[i]     = 0.0f;
        m_starts[i].t  = 0.0f;
        m_starts[i].x  = 0.0f;
        m_starts[i].v  = 0.0f;
        m_starts[i].a  = 0.0f;
    }

    // Basic checks
    if (!limits.valid) return false;
    if (m_A <= 0.0f) return false;
    if (m_targetVolume <= 0.0f) return false;
    if (!(cmd.ramp_ratio > 0.0f && cmd.ramp_ratio < (1.0f / 3.0f))) return false;
    if (cmd.min_ramp_time <= 0.0f) return false;
    if (limits.Q_allow <= 0.0f) return false;

    // Domain mapping
    m_targetStroke = m_targetVolume / m_A; // m
    m_v_allow      = limits.Q_allow / m_A; // m/s

    if (m_targetStroke <= 0.0f || m_v_allow <= 0.0f) return false;

    // Initial guess from pure ramp_ratio design
    const float T_total_guess = m_targetStroke / (m_v_allow * (1.0f - 3.0f * cmd.ramp_ratio));
    const float Tj_candidate  = cmd.ramp_ratio * T_total_guess;

    float Tj = (Tj_candidate > cmd.min_ramp_time) ? Tj_candidate : cmd.min_ramp_time;
    float Ta = Tj;

    // For symmetric 7-segment position profile with Ta=Tj:
    // X = v_peak * (3*Tj + Tv)
    float Tv = m_targetStroke / m_v_allow - 3.0f * Tj;

    if (Tv < 0.0f) {
        // Stroke too small to support chosen v_allow and min_ramp_time
        Tv = 0.0f;
        m_v_peak = m_targetStroke / (3.0f * Tj);
    } else {
        m_v_peak = m_v_allow;
    }

    m_T_total = 6.0f * Tj + Tv;

    // For Ta = Tj:
    // v_peak = a_max * (Tj + Ta) = 2*a_max*Tj
    m_a_max = m_v_peak / (2.0f * Tj);
    m_j_max = m_a_max / Tj;

    m_durations[0] = Tj;
    m_durations[1] = Ta;
    m_durations[2] = Tj;
    m_durations[3] = Tv;
    m_durations[4] = Tj;
    m_durations[5] = Ta;
    m_durations[6] = Tj;

    m_jerks[0] =  m_j_max;
    m_jerks[1] =  0.0f;
    m_jerks[2] = -m_j_max;
    m_jerks[3] =  0.0f;
    m_jerks[4] = -m_j_max;
    m_jerks[5] =  0.0f;
    m_jerks[6] =  m_j_max;

    // Segment end times
    float acc = 0.0f;
    for (int i = 0; i < 7; i++) {
        acc += m_durations[i];
        m_edges[i] = acc;
    }

    // Segment start states
    float x0 = 0.0f;
    float v0 = 0.0f;
    float a0 = 0.0f;

    m_starts[0].t = 0.0f;
    m_starts[0].x = x0;
    m_starts[0].v = v0;
    m_starts[0].a = a0;

    for (int i = 0; i < 6; i++) {
        float x1, v1, a1;
        propagate_segment(x0, v0, a0, m_jerks[i], m_durations[i], x1, v1, a1);

        m_starts[i + 1].t = m_edges[i];
        m_starts[i + 1].x = x1;
        m_starts[i + 1].v = v1;
        m_starts[i + 1].a = a1;

        x0 = x1;
        v0 = v1;
        a0 = a1;
    }

    m_valid = true;
    return true;
}

// =========================
// Evaluate
// =========================
PlungerMotionPlanner::Sample PlungerMotionPlanner::evaluate(float t) const
{
    Sample s;
    s.t = t;

    if (!m_valid) {
        s.finished = true;
        return s;
    }

    if (t <= 0.0f) {
        s.plungerPos  = 0.0f;
        s.plungerVel  = 0.0f;
        s.plungerAcc  = 0.0f;
        s.plungerJerk = m_jerks[0];
        s.finished    = false;
        fill_derived_outputs(s);
        return s;
    }

    if (t >= m_T_total) {
        s.t           = m_T_total;
        s.plungerPos  = m_targetStroke;
        s.plungerVel  = 0.0f;
        s.plungerAcc  = 0.0f;
        s.plungerJerk = 0.0f;
        s.finished    = true;
        fill_derived_outputs(s);
        s.volume = m_targetVolume * 1.0e9f; // exact final boundary in uL
        return s;
    }

    int seg = 0;
    float t_local = t;

    if (t < m_edges[0]) {
        seg = 0;
        t_local = t;
    } else if (t < m_edges[1]) {
        seg = 1;
        t_local = t - m_edges[0];
    } else if (t < m_edges[2]) {
        seg = 2;
        t_local = t - m_edges[1];
    } else if (t < m_edges[3]) {
        seg = 3;
        t_local = t - m_edges[2];
    } else if (t < m_edges[4]) {
        seg = 4;
        t_local = t - m_edges[3];
    } else if (t < m_edges[5]) {
        seg = 5;
        t_local = t - m_edges[4];
    } else {
        seg = 6;
        t_local = t - m_edges[5];
    }

    float x, v, a;
    segment_state(
        m_starts[seg].x,
        m_starts[seg].v,
        m_starts[seg].a,
        m_jerks[seg],
        t_local,
        x, v, a
    );

    s.plungerPos  = x;
    s.plungerVel  = v;
    s.plungerAcc  = a;
    s.plungerJerk = m_jerks[seg];
    s.finished    = false;

    fill_derived_outputs(s);
    return s;
}

// =========================
// Getters
// =========================
bool PlungerMotionPlanner::valid() const
{
    return m_valid;
}

float PlungerMotionPlanner::totalDuration() const
{
    return m_T_total;
}

float PlungerMotionPlanner::getTargetVolume_m3() const
{
    return m_targetVolume;
}

float PlungerMotionPlanner::getTargetStroke_m() const
{
    return m_targetStroke;
}

float PlungerMotionPlanner::getPeakVelocity_mps() const
{
    return m_v_peak;
}

float PlungerMotionPlanner::getArea_m2() const
{
    return m_A;
}