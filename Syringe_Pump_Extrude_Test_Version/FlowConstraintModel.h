#ifndef FLOW_CONSTRAINT_MODEL_H
#define FLOW_CONSTRAINT_MODEL_H

#include <Arduino.h>
#include <math.h>

class FlowConstraintModel {
public:
    // =========================
    // Input parameter set
    // =========================
    struct Params {
        // ----- Fluid / syringe / cannula -----
        float viscosity;     // Fluid dynamic viscosity (mPa.s)
        float R;             // Plunger tip radius (m)
        float L;             // Plunger length (m)
        float shaft;         // Plunger shaft outer width a (m)
        float shaft_walls;   // Plunger shaft wall thickness d (m)
        float E;             // Elastic modulus (Pa)
        float S;             // Factor of safety (-)
        float r;             // Cannula inner radius (m)
        float l;             // Cannula length (m)

        // ----- Screw / motor -----
        float lead_pitch;    // Lead screw lead (m/rev)
        float eta;           // Screw efficiency (-)
        float Kt;            // Effective motor torque constant (Nm/A)
        float I_limit;       // Stepper phase current limit (A)

        // ----- Speed / pulse limits -----
        float max_rpm;       // Maximum allowed screw speed (rpm)
        float steps_per_rev; // Effective pulses per revolution
        float max_step_freq; // Maximum stable step frequency (Hz)
        float beta;          // Safety factor on usable speed-side flow rate (0..1)

        float stroke_per_ml_mm;        // Linear plunger travel per 1 mL [mm/mL]
        float max_linear_speed_mm_s;   // Maximum linear plunger speed [mm/s]

        // ----- Acceleration-limit model -----
        float moving_mass_kg;              // Equivalent moving mass on plunger axis [kg]
        float friction_force_N;            // Optional estimated friction force [N]
        float acceleration_utilization;    // 0..1, utilization of calculated acceleration ceiling
        bool use_load_terms_for_accel;     // false: monitor pressure/friction but do not subtract them

        // ----- Buckling assumptions -----
        float K_buckling;    // Euler effective length factor K (-)
        float I_plunger;     // Second moment of area of plunger shaft (m^4). >0: user input, <=0 or invalid: estimate from shaft/shaft_walls

        // member initializer list
        // equal to Params() { viscosity = 60000.0f; R = 0.0049f; }
        Params()
        : viscosity(60000.0f),
        R(0.0049f),
        L(0.062f),
        shaft(0.0093f),
        shaft_walls(0.0008f),
        E(1.0e9f),
        S(3.0f), // Safety factor
        r(0.00013f),
        l(0.012f),
        lead_pitch(0.0015f),
        eta(0.30f),
        Kt(0.45f),
        I_limit(2.5f),
        max_rpm(300.0f),
        steps_per_rev(3200.0f),
        max_step_freq(50000.0f),
        beta(0.30f),
        stroke_per_ml_mm(13.0f),
        max_linear_speed_mm_s(0.3f),
        moving_mass_kg(0.5f),
        friction_force_N(0.0f),
        acceleration_utilization(1.0f),
        use_load_terms_for_accel(true),
        K_buckling(1.0f),
        I_plunger(8.0927e-11f)
        {}
    };

    // =========================
    // Output result set
    // =========================
    struct Result {
        // ----- Flow limits -----
        float Q_struct;      // Structure-limited flow rate (m^3/s)
        float Q_motor;       // Motor-current-limited flow rate (m^3/s)
        float Q_rpm;         // RPM-limited flow rate (m^3/s)
        float Q_step;        // Step-frequency-limited flow rate (m^3/s)
        float Q_force_safe;  // min(Q_struct, Q_motor)
        float Q_speed_safe;  // beta * min(Q_rpm, Q_step)
        float Q_allow;       // Final usable flow rate (m^3/s)

        // ----- Linear plunger speed limits -----
        float v_struct;      // Structure-limited plunger speed (m/s)
        float v_motor;       // Motor-current-limited plunger speed (m/s)
        float v_rpm;         // RPM-limited plunger speed (m/s)
        float v_step;        // Step-frequency-limited plunger speed (m/s)
        float v_allow;       // Final allowed plunger speed (m/s)

        // ----- Force / pressure -----
        float F_crit;        // Euler critical buckling force (N)
        float F_allow;       // Allowable structural force (N)
        float F_motor;       // Maximum motor-generated axial force (N)
        float dP_motor;      // Maximum motor-generated pressure (Pa)

        float F_capacity;              // min(F_allow, F_motor) [N]
        float F_pressure_at_Q_allow;   // Pressure force at Q_allow [N], debug/reference
        float F_friction_used;         // Friction force used in acceleration calculation [N]
        float F_accel_available;       // Force available for acceleration [N]

        float moving_mass_kg;          // Equivalent moving mass used [kg]
        float a_allow_m_s2;            // Maximum allowed acceleration [m/s^2]
        float a_allow_mm_s2;           // Maximum allowed acceleration [mm/s^2]

        // ----- Geometry / derived values -----
        float A;             // Plunger tip area (m^2)
        float mu;            // Dynamic viscosity (Pa.s)
        float I_used;        // Actually used second moment of area (m^4)
        bool I_auto_estimated; // true if I_plunger was estimated from shaft geometry

        // ----- Status -----
        bool valid;          // true if result is numerically valid

        Result()
        : Q_struct(0.0f),
        Q_motor(0.0f),
        Q_rpm(0.0f),
        Q_step(0.0f),
        Q_force_safe(0.0f),
        Q_speed_safe(0.0f),
        Q_allow(0.0f),
        v_struct(0.0f),
        v_motor(0.0f),
        v_rpm(0.0f),
        v_step(0.0f),
        v_allow(0.0f),
        F_crit(0.0f),
        F_allow(0.0f),
        F_motor(0.0f),
        dP_motor(0.0f),
        F_capacity(0.0f),
        F_pressure_at_Q_allow(0.0f),
        F_friction_used(0.0f),
        F_accel_available(0.0f),
        moving_mass_kg(0.0f),
        a_allow_m_s2(0.0f),
        a_allow_mm_s2(0.0f),
        A(0.0f),
        mu(0.0f),
        I_used(0.0f),
        I_auto_estimated(false),
        valid(false)
        {}
    };

    // =========================
    // Main API
    // =========================
    // 定義一個屬於 FlowConstraintModel 的計算函式
    // 它不需要物件就能用 (static)
    // 輸入 Params
    // 回傳 Result
    static Result compute(const Params& p);

    // =========================
    // Utility helpers
    // =========================
    static float plunger_area_from_R(float R);   // m^2
    static float viscosity_Pa_s_from_mPa_s(float viscosity); // Pa.s

private:
    static void calculate_struct_flow_limit(
        const Params& p,
        float& Q_struct,
        float& v_struct,
        float& F_crit,
        float& F_allow,
        float& I_used,
        bool& I_auto_estimated
    );

    static void flow_limit_from_current(
        const Params& p,
        float& Q_motor,
        float& v_motor,
        float& dP_motor,
        float& F_motor
    );

    static void flow_limit_from_rpm(
        const Params& p,
        float& Q_rpm,
        float& v_rpm
    );

    static void flow_limit_from_step_frequency(
        const Params& p,
        float& Q_step,
        float& v_step
    );

    static float safe_positive(float x);
    static bool is_valid_positive(float x);
    static float estimate_plunger_second_moment(const Params& p);

    static float linear_velocity_mps_from_flow(
        const Params& p,
        float Q_m3_s
    );

    static float flow_from_linear_velocity_mps(
        const Params& p,
        float v_m_s
    );

    static void calculate_acceleration_limit(
        const Params& p,
        Result& out
    );
};

#endif