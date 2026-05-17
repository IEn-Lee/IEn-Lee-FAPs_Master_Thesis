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

bool is_extrusion_scenario_internal(MotionScenario scenario)
{
    switch (scenario)
    {
        case SCENARIO_EXTRUSION_ACTIVE:
        case SCENARIO_EXTRUSION_STOPPING:
            return true;

        default:
            return false;
    }
}

bool is_service_scenario_internal(MotionScenario scenario)
{
    switch (scenario) {

        case SCENARIO_HOMING:
        case SCENARIO_MANUAL_JOG:
        case SCENARIO_REPOSITION:
            return true;

        default:
            return false;
    }
}

bool is_fault_scenario_internal(MotionScenario scenario)
{
    return scenario == SCENARIO_FAULT;
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

bool isExtrusionMode()
{
    return g_state.mode == MODE_EXTRUSION;
}

bool isPrecisionMode()
{
    return isExtrusionMode();
}

bool isServiceMode()
{
    return g_state.mode == MODE_SERVICE_MOTION;
}

bool isFaultMode()
{
    return g_state.mode == MODE_FAULT;
}

bool usesExternalStreaming()
{
    return false;
}

bool usesInternalRamp() {
    return g_state.backend == BACKEND_TMC_INTERNAL_RAMP;
}

// =========================
// Scenario mapping
// =========================
MotionMode modeForScenario(MotionScenario scenario)
{
    if (is_extrusion_scenario_internal(scenario)) {
        return MODE_EXTRUSION;
    }

    if (is_service_scenario_internal(scenario)) {
        return MODE_SERVICE_MOTION;
    }

    if (is_fault_scenario_internal(scenario)) {
        return MODE_FAULT;
    }

    return MODE_IDLE;
}

MotionBackend backendForScenario(MotionScenario scenario)
{
    if (is_extrusion_scenario_internal(scenario)) {
        return BACKEND_TMC_INTERNAL_RAMP;
    }

    if (is_service_scenario_internal(scenario)) {
        return BACKEND_TMC_INTERNAL_RAMP;
    }

    return BACKEND_NONE;
}

bool isPrecisionCriticalScenario(MotionScenario scenario)
{
    return is_extrusion_scenario_internal(scenario);
}

// =========================
// Permission checks
// =========================
bool canStartScenario(MotionScenario scenario)
{
    if (scenario == SCENARIO_NONE) {
        return true;
    }

    if (scenario == SCENARIO_FAULT) {
        return true;
    }

    if (scenario == SCENARIO_EXTRUSION_STOPPING) {
        return g_state.scenario == SCENARIO_EXTRUSION_ACTIVE;
    }

    if (!g_state.busy) {
        return true;
    }

    return false;
}

bool canInterruptWith(MotionScenario scenario)
{
    if (!g_state.busy) {
        return true;
    }

    if (scenario == SCENARIO_FAULT) {
        return true;
    }

    if (scenario == SCENARIO_NONE) {
        return true;
    }

    // During extrusion, only controlled stopping is allowed.
    if (g_state.mode == MODE_EXTRUSION) {
        switch (scenario)
        {
            case SCENARIO_EXTRUSION_STOPPING:
                return true;

            default:
                return false;
        }
    }

    // During service motion, allow fault, idle, or homing transition.
    if (g_state.mode == MODE_SERVICE_MOTION) {
        switch (scenario)
        {
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
TransitionResult requestScenario(MotionScenario scenario)
{
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

    if (!canStartScenario(scenario) && !canInterruptWith(scenario)) {
        return make_result(
            false,
            g_state.mode,
            g_state.scenario,
            g_state.backend,
            "Transition denied"
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

TransitionResult requestMode(MotionMode mode)
{
    switch (mode)
    {
        case MODE_IDLE:
            reset();
            return make_result(
                true,
                g_state.mode,
                g_state.scenario,
                g_state.backend,
                "Mode set to idle"
            );

        default:
            return make_result(
                false,
                g_state.mode,
                g_state.scenario,
                g_state.backend,
                "Direct mode request denied: request a specific scenario"
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

TransitionResult enterFault(const char* reason)
{
    apply_state(SCENARIO_FAULT);

    return make_result(
        true,
        g_state.mode,
        g_state.scenario,
        g_state.backend,
        reason ? reason : "Fault"
    );
}

// =========================
// String helpers
// =========================
const char* modeToString(MotionMode mode)
{
    switch (mode)
    {
        case MODE_IDLE:
            return "IDLE";

        case MODE_EXTRUSION:
            return "EXTRUSION";

        case MODE_SERVICE_MOTION:
            return "SERVICE_MOTION";

        case MODE_FAULT:
            return "FAULT";

        default:
            return "UNKNOWN_MODE";
    }
}

const char* scenarioToString(MotionScenario scenario)
{
    switch (scenario)
    {
        case SCENARIO_NONE:
            return "NONE";

        case SCENARIO_HOMING:
            return "HOMING";

        case SCENARIO_MANUAL_JOG:
            return "MANUAL_JOG";

        case SCENARIO_REPOSITION:
            return "REPOSITION";

        case SCENARIO_FAULT:
            return "FAULT";

        case SCENARIO_EXTRUSION_ACTIVE:
            return "EXTRUSION_ACTIVE";

        case SCENARIO_EXTRUSION_STOPPING:
            return "EXTRUSION_STOPPING";

        default:
            return "UNKNOWN_SCENARIO";
    }
}

const char* backendToString(MotionBackend backend)
{
    switch (backend)
    {
        case BACKEND_NONE:
            return "NONE";

        case BACKEND_TMC_INTERNAL_RAMP:
            return "TMC_INTERNAL_RAMP";

        default:
            return "UNKNOWN_BACKEND";
    }
}

} // namespace MotionModeManager