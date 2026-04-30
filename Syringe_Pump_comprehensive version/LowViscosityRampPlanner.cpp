#include "LowViscosityRampPlanner.h"

bool LowViscosityRampPlanner::build(const Params& params)
{
    m_params = params;
    m_valid = false;

    if (params.quantity_uL <= 0.0f) return false;
    if (params.stroke_per_ml_mm <= 0.0f) return false;
    if (params.max_velocity_mm_s <= 0.0f) return false;
    if (params.max_acceleration_mm_s2 <= 0.0f) return false;
    if (params.max_deceleration_mm_s2 <= 0.0f) return false;

    const float quantity_ml = params.quantity_uL / 1000.0f;
    m_target_mm = quantity_ml * params.stroke_per_ml_mm;

    if (m_target_mm <= 0.0f) return false;

    m_vmax = params.max_velocity_mm_s;
    m_acc  = params.max_acceleration_mm_s2;
    m_dec  = params.max_deceleration_mm_s2;

    float t_acc_candidate = m_vmax / m_acc;
    float x_acc_candidate = 0.5f * m_acc * t_acc_candidate * t_acc_candidate;

    float t_dec_candidate = m_vmax / m_dec;
    float x_dec_candidate = 0.5f * m_dec * t_dec_candidate * t_dec_candidate;

    if ((x_acc_candidate + x_dec_candidate) <= m_target_mm) {
        // Trapezoidal profile
        m_t_acc = t_acc_candidate;
        m_t_dec = t_dec_candidate;

        m_x_acc = x_acc_candidate;
        m_x_dec = x_dec_candidate;

        m_x_const = m_target_mm - m_x_acc - m_x_dec;
        m_t_const = m_x_const / m_vmax;
    }
    else {
        // Triangular profile
        m_vmax = sqrtf(
            (2.0f * m_target_mm * m_acc * m_dec) /
            (m_acc + m_dec)
        );

        m_t_acc = m_vmax / m_acc;
        m_t_dec = m_vmax / m_dec;
        m_t_const = 0.0f;

        m_x_acc = 0.5f * m_acc * m_t_acc * m_t_acc;
        m_x_dec = 0.5f * m_dec * m_t_dec * m_t_dec;
        m_x_const = 0.0f;
    }

    m_T_total = m_t_acc + m_t_const + m_t_dec;

    if (m_T_total <= 0.0f) return false;

    m_valid = true;
    return true;
}

LowViscosityRampPlanner::Sample LowViscosityRampPlanner::evaluate(float t) const
{
    Sample s;
    s.t = t;

    if (!m_valid) {
        s.finished = true;
        return s;
    }

    if (t <= 0.0f) {
        s.position_mm = 0.0f;
        s.velocity_mm_s = 0.0f;
        s.acceleration_mm_s2 = 0.0f;
    }
    else if (t < m_t_acc) {
        // Acceleration phase
        s.acceleration_mm_s2 = m_acc;
        s.velocity_mm_s = m_acc * t;
        s.position_mm = 0.5f * m_acc * t * t;
    }
    else if (t < (m_t_acc + m_t_const)) {
        // Constant velocity phase
        float tc = t - m_t_acc;

        s.acceleration_mm_s2 = 0.0f;
        s.velocity_mm_s = m_vmax;
        s.position_mm = m_x_acc + m_vmax * tc;
    }
    else if (t < m_T_total) {
        // Deceleration phase
        float td = t - m_t_acc - m_t_const;

        s.acceleration_mm_s2 = -m_dec;
        s.velocity_mm_s = m_vmax - m_dec * td;

        if (s.velocity_mm_s < 0.0f) {
            s.velocity_mm_s = 0.0f;
        }

        s.position_mm =
            m_x_acc +
            m_x_const +
            m_vmax * td -
            0.5f * m_dec * td * td;
    }
    else {
        s.t = m_T_total;
        s.position_mm = m_target_mm;
        s.velocity_mm_s = 0.0f;
        s.acceleration_mm_s2 = 0.0f;
        s.finished = true;
    }

    if (s.position_mm > m_target_mm) {
        s.position_mm = m_target_mm;
    }

    s.volume_uL =
        (s.position_mm / m_params.stroke_per_ml_mm) * 1000.0f;

    s.flow_uL_s =
        (s.velocity_mm_s / m_params.stroke_per_ml_mm) * 1000.0f;

    if (s.volume_uL > m_params.quantity_uL) {
        s.volume_uL = m_params.quantity_uL;
    }

    if (t >= m_T_total) {
        s.finished = true;
    }

    return s;
}

bool LowViscosityRampPlanner::valid() const
{
    return m_valid;
}

float LowViscosityRampPlanner::totalDuration() const
{
    return m_T_total;
}

float LowViscosityRampPlanner::targetDistanceMm() const
{
    return m_target_mm;
}