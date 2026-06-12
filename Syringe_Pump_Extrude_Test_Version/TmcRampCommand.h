#ifndef TMC_RAMP_COMMAND_H
#define TMC_RAMP_COMMAND_H

struct TmcRampCommand {
    // =========================
    // Target definition
    // =========================
    float quantity_uL;
    float stroke_per_ml_mm;
    float target_distance_mm;

    // =========================
    // TMC ramp parameters
    // =========================
    float max_velocity_mm_s;
    float max_acceleration_mm_s2;
    float max_deceleration_mm_s2;

    float start_velocity_mm_s;
    float stop_velocity_mm_s;

    float first_velocity_mm_s;
    float first_acceleration_mm_s2;
    float first_deceleration_mm_s2;

    // =========================
    // Prediction / UI
    // =========================
    float expected_duration_s;

    // =========================
    // Direction / validity
    // =========================
    bool reverse_direction;
    bool valid;

    TmcRampCommand()
    : quantity_uL(0.0f),
      stroke_per_ml_mm(0.0f),
      target_distance_mm(0.0f),
      max_velocity_mm_s(0.0f),
      max_acceleration_mm_s2(0.0f),
      max_deceleration_mm_s2(0.0f),
      start_velocity_mm_s(0.0f),
      stop_velocity_mm_s(0.0f),
      first_velocity_mm_s(0.0f),
      first_acceleration_mm_s2(0.0f),
      first_deceleration_mm_s2(0.0f),
      expected_duration_s(0.0f),
      reverse_direction(false),
      valid(false)
    {}
};

#endif // TMC_RAMP_COMMAND_H