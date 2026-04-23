#ifndef PLUNGER_MOTION_PLANNER_H
#define PLUNGER_MOTION_PLANNER_H

#include <Arduino.h>
#include <math.h>
#include "FlowConstraintModel.h"

class PlungerMotionPlanner {
public:
    // =========================
    // Command
    // =========================
    struct Command {
        float volume;         // Target volume per shot (uL)
        float ramp_ratio;     // Each ramp subsegment time as fraction of total motion time; must be < 1/3
        float min_ramp_time;  // Absolute minimum time for each ramp subsegment (s)

        Command()
        : volume(1000.0f),
          ramp_ratio(0.05f),
          min_ramp_time(0.05f)
        {}
    };

    // =========================
    // Motion sample at time t
    // =========================
    struct Sample {
        float t; // Current time since motion start (s)

        // ----- Plunger domain -----
        float plungerPos;   // Plunger position (m)
        float plungerVel;   // Plunger velocity (m/s)
        float plungerAcc;   // Plunger acceleration (m/s^2)
        float plungerJerk;  // Plunger jerk (m/s^3)

        // ----- Flow domain (derived) -----
        float volume;       // Dispensed volume (uL)
        float flow;         // Flow rate (m^3/s)
        float flow_uL_s;    // Flow rate (uL/s)
        float pressure;     // Estimated pressure drop (Pa)
        float pressure_bar; // Estimated pressure drop (bar)
        float force;        // Estimated plunger axial force (N)
        float currentEst;   // Estimated motor current (A)

        bool finished;      // true if t >= total duration

        Sample()
        : t(0.0f),
          plungerPos(0.0f),
          plungerVel(0.0f),
          plungerAcc(0.0f),
          plungerJerk(0.0f),
          volume(0.0f),
          flow(0.0f),
          flow_uL_s(0.0f),
          pressure(0.0f),
          pressure_bar(0.0f),
          force(0.0f),
          currentEst(0.0f),
          finished(false)
        {}
    };

    // =========================
    // Build / query
    // =========================
    bool build(
        const Command& cmd,
        const FlowConstraintModel::Params& params,
        const FlowConstraintModel::Result& limits
    );

    Sample evaluate(float t) const;

    bool valid() const;
    float totalDuration() const;

    // Optional getters
    float getTargetVolume_m3() const; // m^3
    float getTargetStroke_m() const;  // m
    float getPeakVelocity_mps() const;
    float getArea_m2() const;

private:
    // =========================
    // Internal segment start state
    // =========================
    struct SegmentStart {
        float t; // Segment start time (s)
        float x; // Segment start position (m)
        float v; // Segment start velocity (m/s)
        float a; // Segment start acceleration (m/s^2)
    };

    // ----- copied input -----
    FlowConstraintModel::Params m_params;
    FlowConstraintModel::Result m_limits;
    Command m_cmd;

    // ----- derived constants -----
    float m_A;             // Plunger area (m^2)
    float m_mu;            // Dynamic viscosity (Pa.s)
    float m_targetVolume;  // Target volume (m^3)
    float m_targetStroke;  // Target plunger stroke (m)
    float m_v_allow;       // Allowed plunger velocity (m/s)
    float m_v_peak;        // Actual peak plunger velocity (m/s)
    float m_a_max;         // Peak plunger acceleration (m/s^2)
    float m_j_max;         // Peak plunger jerk (m/s^3)

    // ----- 7 segments -----
    float m_durations[7];  // Segment durations (s)
    float m_jerks[7];      // Segment jerks (m/s^3)
    float m_edges[7];      // Segment end times (s)
    SegmentStart m_starts[7];

    float m_T_total;       // Total motion time (s)
    bool m_valid;

    // =========================
    // Helpers
    // =========================
    static float safe_positive(float x);

    static void propagate_segment(
        float x0, float v0, float a0, float j, float tau,
        float& x1, float& v1, float& a1
    );

    static void segment_state(
        float x0, float v0, float a0, float j, float t_local,
        float& x, float& v, float& a
    );

    static float pressure_from_flow(
        float mu, float l, float r, float Q
    );

    float estimate_current_from_flow(float Q) const;
    void fill_derived_outputs(Sample& s) const;
};

#endif