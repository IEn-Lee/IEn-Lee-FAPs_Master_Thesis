#include "MotionModeManager.h"

namespace MotionModeManager {

namespace {

// =========================
// Internal state
// =========================
MotionState g_state = {
    MODE_IDLE,
    SCENARIO_NONE,
    BACKEND_NONE,
    false,
    false
};

TransitionResult make_result(
    bool success,
    MotionMode mode,
    MotionScenario scenario,
    MotionBackend backend,
    const char* message
) {
    TransitionResult r;
    r.success = success;
    r.newMode = mode;
    r.newScenario = scenario;
    r.newBackend = backend;
    r.message = message;
    return r;
}

void apply_state(MotionScenario scenario) {
    g_state.scenario = scenario;
    g_state.mode = modeForScenario(scenario);
    g_state.backend = backendForScenario(scenario);
    g_state.precisionCritical = isPrecisionCriticalScenario(scenario);
    g_state.busy = (scenario != SCENARIO_NONE);
}

bool is_precision_scenario_internal(MotionScenario scenario) {
    switch (scenario) {
        case SCENARIO_EXTRUSION_STARTUP:
        case SCENARIO_EXTRUSION_ACTIVE:
        case SCENARIO_EXTRUSION_STOPPING:
            return true;

        default:
            return false;
    }
}

bool is_service_scenario_internal(MotionScenario scenario) {
    switch (scenario) {
        case SCENARIO_HOMING:
        case SCENARIO_MANUAL_JOG:
        case SCENARIO_MANUAL_DISTANCE_MOVE:
        case SCENARIO_PLUNGER_DETECTED_RECOVERY:
        case SCENARIO_SERVICE_TEST_MOVE:
        case SCENARIO_REPOSITION:
            return true;

        default:
            return false;
    }
}

} // anonymous namespace

// =========================
// Lifecycle
// =========================
void begin() {
    reset();
}

void reset() {
    g_state.mode = MODE_IDLE;
    g_state.scenario = SCENARIO_NONE;
    g_state.backend = BACKEND_NONE;
    g_state.busy = false;
    g_state.precisionCritical = false;
}

// =========================
// State read
// =========================
MotionState getState() {
    return g_state;
}

MotionMode getMode() {
    return g_state.mode;
}

MotionScenario getScenario() {
    return g_state.scenario;
}

MotionBackend getBackend() {
    return g_state.backend;
}

bool isBusy() {
    return g_state.busy;
}

bool isIdle() {
    return g_state.mode == MODE_IDLE;
}

bool isPrecisionMode() {
    return g_state.mode == MODE_PRECISION_EXTRUSION;
}

bool isServiceMode() {
    return g_state.mode == MODE_SERVICE_MOTION;
}

bool usesExternalStreaming() {
    return g_state.backend == BACKEND_EXTERNAL_SPI_STREAMING;
}

bool usesInternalRamp() {
    return g_state.backend == BACKEND_TMC_INTERNAL_RAMP;
}

// =========================
// Scenario mapping
// =========================
MotionMode modeForScenario(MotionScenario scenario) {
    if (is_precision_scenario_internal(scenario)) {
        return MODE_PRECISION_EXTRUSION;
    }

    if (is_service_scenario_internal(scenario)) {
        return MODE_SERVICE_MOTION;
    }

    return MODE_IDLE;
}

MotionBackend backendForScenario(MotionScenario scenario) {
    if (is_precision_scenario_internal(scenario)) {
        return BACKEND_EXTERNAL_SPI_STREAMING;
    }

    if (is_service_scenario_internal(scenario)) {
        return BACKEND_TMC_INTERNAL_RAMP;
    }

    return BACKEND_NONE;
}

bool isPrecisionCriticalScenario(MotionScenario scenario) {
    return is_precision_scenario_internal(scenario);
}

// =========================
// Permission checks
// =========================
bool canStartScenario(MotionScenario scenario) {
    if (scenario == SCENARIO_NONE) {
        return true;
    }

    if (!g_state.busy) {
        return true;
    }

    return false;
}

bool canInterruptWith(MotionScenario scenario) {
    if (!g_state.busy) {
        return true;
    }

    // During precision extrusion, do not allow service actions to interrupt.
    if (g_state.mode == MODE_PRECISION_EXTRUSION) {
        switch (scenario) {
            case SCENARIO_NONE:
                return true;

            case SCENARIO_EXTRUSION_STOPPING:
                return true;

            default:
                return false;
        }
    }

    // During service motion, allow forceful transitions only to idle or homing-like flows
    if (g_state.mode == MODE_SERVICE_MOTION) {
        switch (scenario) {
            case SCENARIO_NONE:
            case SCENARIO_HOMING:
                return true;

            default:
                return false;
        }
    }

    return false;
}

// =========================
// Transition control
// =========================
TransitionResult requestScenario(MotionScenario scenario) {
    if (scenario == SCENARIO_NONE) {
        reset();
        return make_result(
            true,
            g_state.mode,
            g_state.scenario,
            g_state.backend,
            "Returned to idle"
        );
    }

    if (!g_state.busy) {
        apply_state(scenario);
        return make_result(
            true,
            g_state.mode,
            g_state.scenario,
            g_state.backend,
            "Scenario started"
        );
    }

    if (!canInterruptWith(scenario)) {
        return make_result(
            false,
            g_state.mode,
            g_state.scenario,
            g_state.backend,
            "Transition denied: current scenario is active"
        );
    }

    apply_state(scenario);
    return make_result(
        true,
        g_state.mode,
        g_state.scenario,
        g_state.backend,
        "Scenario switched"
    );
}

TransitionResult requestMode(MotionMode mode) {
    switch (mode) {
        case MODE_IDLE:
            reset();
            return make_result(
                true,
                g_state.mode,
                g_state.scenario,
                g_state.backend,
                "Mode set to idle"
            );

        case MODE_PRECISION_EXTRUSION:
            return requestScenario(SCENARIO_EXTRUSION_ACTIVE);

        case MODE_SERVICE_MOTION:
            return requestScenario(SCENARIO_SERVICE_TEST_MOVE);

        default:
            return make_result(
                false,
                g_state.mode,
                g_state.scenario,
                g_state.backend,
                "Unknown mode"
            );
    }
}

TransitionResult finishCurrentScenario() {
    reset();
    return make_result(
        true,
        g_state.mode,
        g_state.scenario,
        g_state.backend,
        "Scenario finished"
    );
}

TransitionResult forceIdle(const char* reason) {
    reset();
    return make_result(
        true,
        g_state.mode,
        g_state.scenario,
        g_state.backend,
        reason ? reason : "Forced idle"
    );
}

// =========================
// String helpers
// =========================
const char* modeToString(MotionMode mode) {
    switch (mode) {
        case MODE_IDLE:
            return "IDLE";
        case MODE_PRECISION_EXTRUSION:
            return "PRECISION_EXTRUSION";
        case MODE_SERVICE_MOTION:
            return "SERVICE_MOTION";
        default:
            return "UNKNOWN_MODE";
    }
}

const char* scenarioToString(MotionScenario scenario) {
    switch (scenario) {
        case SCENARIO_NONE:
            return "NONE";
        case SCENARIO_EXTRUSION_STARTUP:
            return "EXTRUSION_STARTUP";
        case SCENARIO_EXTRUSION_ACTIVE:
            return "EXTRUSION_ACTIVE";
        case SCENARIO_EXTRUSION_STOPPING:
            return "EXTRUSION_STOPPING";
        case SCENARIO_HOMING:
            return "HOMING";
        case SCENARIO_MANUAL_JOG:
            return "MANUAL_JOG";
        case SCENARIO_MANUAL_DISTANCE_MOVE:
            return "MANUAL_DISTANCE_MOVE";
        case SCENARIO_PLUNGER_DETECTED_RECOVERY:
            return "PLUNGER_DETECTED_RECOVERY";
        case SCENARIO_SERVICE_TEST_MOVE:
            return "SERVICE_TEST_MOVE";
        case SCENARIO_REPOSITION:
            return "REPOSITION";
        default:
            return "UNKNOWN_SCENARIO";
    }
}

const char* backendToString(MotionBackend backend) {
    switch (backend) {
        case BACKEND_NONE:
            return "NONE";
        case BACKEND_EXTERNAL_SPI_STREAMING:
            return "EXTERNAL_SPI_STREAMING";
        case BACKEND_TMC_INTERNAL_RAMP:
            return "TMC_INTERNAL_RAMP";
        default:
            return "UNKNOWN_BACKEND";
    }
}

} // namespace MotionModeManager