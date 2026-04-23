#ifndef MOTOR_CONTROL_SCREEN_H
#define MOTOR_CONTROL_SCREEN_H

#include <Arduino.h>
#include "lvgl.h"

namespace MotorControlScreen {

// =========================
// Lifecycle
// =========================
void setMainScreen(lv_obj_t* screen);

void build();
lv_obj_t* getScreen();
lv_obj_t** getScreenHandle();

void update();

void destroy();
void destroyAsync(void* user_data);

// =========================
// Motion input mode
// =========================
enum MotionInputMode {
    MODE_REVOLUTIONS = 0,
    MODE_DISTANCE_MM = 1
};

// =========================
// Runtime settings model
// =========================
struct MotorSpiSettings {
    // ----- Driver parameters -----
    float runCurrentPercent;        // %
    float pwmOffsetPercent;         // %
    float pwmGradientPercent;       // %
    bool  reverseDirection;         // false = Forward, true = reverse
    float stealthChopThreshold;     // mm/s

    // ----- Controller parameters -----
    float maxVelocity;              // mm/s
    float maxAcceleration;          // mm/s^2
    float startVelocity;            // mm/s
    float stopVelocity;             // mm/s
    float firstVelocity;            // mm/s
    float firstAcceleration;        // mm/s^2
    float maxDeceleration;          // mm/s^2
    float firstDeceleration;        // mm/s^2

    // ----- Motion command -----
    MotionInputMode mode;           // revolutions / distance(mm)
    float forwardValue;             // rev or mm
    float backwardValue;            // rev or mm
};

// =========================
// Settings access
// =========================
const MotorSpiSettings& getSettings();
void setSettings(const MotorSpiSettings& settings);
void resetSettingsToDefault();

// Pull current values from UI into settings model
bool pullSettingsFromUi();

// Push current settings model into UI
void pushSettingsToUi();

// =========================
// SPI / motion control
// =========================
bool initMotorSpiSystem();
void stopMotion();

// Convert motion value to chip target position
int32_t computeTargetChipFromMode(float value, MotionInputMode mode);

// Optional status helpers
bool isReady();
bool isRunning();

} // namespace MotorControlScreen

#endif