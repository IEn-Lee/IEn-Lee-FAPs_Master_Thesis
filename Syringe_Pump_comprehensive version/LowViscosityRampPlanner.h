#ifndef LOW_VISCOSITY_RAMP_PLANNER_H
#define LOW_VISCOSITY_RAMP_PLANNER_H

#include <Arduino.h>

class LowViscosityRampPlanner {
public:
    struct Params {
        float quantity_uL;
        float stroke_per_ml_mm;

        float max_velocity_mm_s;
        float max_acceleration_mm_s2;
        float max_deceleration_mm_s2;

        float start_velocity_mm_s;
        float stop_velocity_mm_s;
        float first_velocity_mm_s;
        float first_acceleration_mm_s2;

        Params()
        : quantity_uL(1000.0f),              // Default target volume for one extrusion cycle [uL]
          stroke_per_ml_mm(13.0f),           // Syringe-specific plunger displacement per milliliter [mm/mL]
          max_velocity_mm_s(0.3f),           // Upper limit for linear plunger velocity during low-viscosity extrusion [mm/s]
          max_acceleration_mm_s2(0.08f),     // Maximum acceleration used for the main ramp-up phase [mm/s^2]
          max_deceleration_mm_s2(0.08f),     // Maximum deceleration used for the main ramp-down phase [mm/s^2]
          start_velocity_mm_s(0.0f),         // Motion starts from zero velocity to prevent an abrupt initial push [mm/s]
          stop_velocity_mm_s(0.0f),          // Motion ends at zero velocity for a controlled final stop [mm/s]
          first_velocity_mm_s(0.03f),        // Low initial transition velocity before entering the main acceleration phase [mm/s]
          first_acceleration_mm_s2(0.02f)    // Reduced initial acceleration for smoother plunger engagement [mm/s^2]
        {}
    };

    struct Sample {
        float t;

        float position_mm;
        float velocity_mm_s;
        float acceleration_mm_s2;

        float volume_uL;
        float flow_uL_s;

        bool finished;

        Sample()
        : t(0.0f),
          position_mm(0.0f),
          velocity_mm_s(0.0f),
          acceleration_mm_s2(0.0f),
          volume_uL(0.0f),
          flow_uL_s(0.0f),
          finished(false)
        {}
    };

    bool build(const Params& params);
    Sample evaluate(float t) const;

    bool valid() const;
    float totalDuration() const;
    float targetDistanceMm() const;

private:
    Params m_params;

    float m_target_mm;
    float m_vmax;
    float m_acc;
    float m_dec;

    float m_t_acc;
    float m_t_const;
    float m_t_dec;
    float m_T_total;

    float m_x_acc;
    float m_x_const;
    float m_x_dec;

    bool m_valid;
};

#endif