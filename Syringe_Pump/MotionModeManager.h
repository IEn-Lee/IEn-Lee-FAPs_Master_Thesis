#ifndef MOTION_MODE_MANAGER_H
#define MOTION_MODE_MANAGER_H

#include <Arduino.h>

namespace MotionModeManager {

enum MotionMode {
    MODE_IDLE = 0,

    // Extrusion motion:
    // TMC internal ramp generator with fluid-aware ramp parameters
    MODE_EXTRUSION,

    // Service motions:
    // Homing, manual jog, reposition
    MODE_SERVICE_MOTION,

    // Fault state:
    // Current overload, stall, timeout, driver fault, etc.
    MODE_FAULT
};

// =========================
// Concrete usage scenarios
// =========================
enum MotionScenario {
    SCENARIO_NONE = 0,

    // extrusion
    SCENARIO_EXTRUSION_ACTIVE,
    SCENARIO_EXTRUSION_STOPPING,

    // service
    SCENARIO_HOMING,
    SCENARIO_MANUAL_JOG,
    SCENARIO_REPOSITION,

    // fault
    SCENARIO_FAULT
};

// =========================
// Backend type
// =========================
enum MotionBackend {
    BACKEND_NONE = 0,
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

bool isExtrusionMode();
bool isPrecisionMode();   // legacy alias for extrusion mode
bool isServiceMode();
bool isFaultMode();

bool usesExternalStreaming(); // legacy compatibility, always false
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