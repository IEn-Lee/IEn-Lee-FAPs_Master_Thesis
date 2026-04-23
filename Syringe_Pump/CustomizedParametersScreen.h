#ifndef CUSTOMIZED_PARAMETERS_SCREEN_H
#define CUSTOMIZED_PARAMETERS_SCREEN_H

#include <Arduino.h>
#include "lvgl.h"

namespace CustomizedParametersScreen {

// ===== Parameter indices =====
enum ParamIndex {
    DEV_P_VISCOSITY = 0,
    DEV_P_R,
    DEV_P_L,
    DEV_P_SHAFT,
    DEV_P_SHAFT_WALL,
    DEV_P_E,
    DEV_P_S,
    DEV_P_r,
    DEV_P_l,
    //DEV_P_VOLUME,
    DEV_P_LEAD_PITCH,
    DEV_P_I_LIMIT,
    DEV_P_STROKE_PER_ML_MM,
    DEV_P_MAX_LINEAR_SPEED_MM_S,
    DEV_P_COUNT
};

// ===== Parameter storage =====
typedef struct {
    const char* key;
    const char* unit;
    const char* default_str;
    char saved_str[32];
} ParamValue;

// ===== Public API =====
void setMainScreen(lv_obj_t* screen);

void initStore();
void build();

lv_obj_t* getScreen();
lv_obj_t** getScreenHandle();

void destroy();
void destroyAsync(void* user_data);

// save / reset
void saveParameters();
void resetParametersToDefault();

// access
const char* getSavedValue(int idx);
float getSavedFloat(int idx);
ParamValue* getStore();

// dialogs
void showSaveDoneDialog();
void showDefaultConfirmDialog();

} // namespace CustomizedParametersScreen

#endif