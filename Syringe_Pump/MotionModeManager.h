#ifndef MOTION_MODE_MANAGER_H
#define MOTION_MODE_MANAGER_H

#include <Arduino.h>

namespace MotionModeManager {

// =========================
// High-level motion mode
// =========================
enum MotionMode {
    MODE_IDLE = 0,

    // Precision-critical motion:
    // external planner + SPI streaming
    MODE_PRECISION_EXTRUSION,

    // Non-critical service motions:
    // TMC internal ramp generator
    MODE_SERVICE_MOTION
};

// =========================
// Concrete usage scenarios
// =========================
enum MotionScenario {
    SCENARIO_NONE = 0,

    // Precision scenarios
    SCENARIO_EXTRUSION_STARTUP,
    SCENARIO_EXTRUSION_ACTIVE,
    SCENARIO_EXTRUSION_STOPPING,

    // Service scenarios
    SCENARIO_HOMING,
    SCENARIO_MANUAL_JOG,
    SCENARIO_MANUAL_DISTANCE_MOVE,
    SCENARIO_PLUNGER_DETECTED_RECOVERY,
    SCENARIO_SERVICE_TEST_MOVE,
    SCENARIO_REPOSITION
};

// =========================
// Backend type
// =========================
enum MotionBackend {
    BACKEND_NONE = 0,
    BACKEND_EXTERNAL_SPI_STREAMING,
    BACKEND_TMC_INTERNAL_RAMP
};

// =========================
// State snapshot
// =========================
struct MotionState {
    MotionMode mode;
    MotionScenario scenario;
    MotionBackend backend;
    bool busy;
    bool precisionCritical;
};

// =========================
// Mode transition result
// =========================
struct TransitionResult {
    bool success;
    MotionMode newMode;
    MotionScenario newScenario;
    MotionBackend newBackend;
    const char* message;
};

// =========================
// Lifecycle
// =========================
void begin();
void reset();

// =========================
// State read
// =========================
MotionState getState();

MotionMode getMode();
MotionScenario getScenario();
MotionBackend getBackend();

bool isBusy();
bool isIdle();

bool isPrecisionMode();
bool isServiceMode();

bool usesExternalStreaming();
bool usesInternalRamp();

// =========================
// Scenario mapping
// =========================
MotionMode modeForScenario(MotionScenario scenario);
MotionBackend backendForScenario(MotionScenario scenario);
bool isPrecisionCriticalScenario(MotionScenario scenario);

// =========================
// Transition control
// =========================
TransitionResult requestScenario(MotionScenario scenario);
TransitionResult requestMode(MotionMode mode);

TransitionResult finishCurrentScenario();
TransitionResult forceIdle(const char* reason = "Forced idle");

// =========================
// Permission checks
// =========================
bool canStartScenario(MotionScenario scenario);
bool canInterruptWith(MotionScenario scenario);

// =========================
// Helpers for UI / debug
// =========================
const char* modeToString(MotionMode mode);
const char* scenarioToString(MotionScenario scenario);
const char* backendToString(MotionBackend backend);

} // namespace MotionModeManager

#endif