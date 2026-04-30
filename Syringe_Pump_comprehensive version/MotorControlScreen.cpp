#include "MotorControlScreen.h"
#include "CustomizedParametersScreen.h"

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <TMC51X0.hpp>
#include <Adafruit_INA228.h>

#include "lvgl.h"
#include "fonts.h"

namespace {

// =========================
// Hardware config
// =========================
#define PIN_OE 4

const size_t ENABLE_HARDWARE_PIN = 7;
const uint8_t CHIP_SELECT_PIN    = 10;
const uint8_t INA228_ADDR        = 0x40;

#if defined(ARDUINO_ARCH_RP2040)
SPIClassRP2040 &spi = SPI;
size_t SCK_PIN = 13;
size_t TX_PIN  = 11;
size_t RX_PIN  = 12;
#else
SPIClass &spi = SPI1;
#endif

// leadscrew: 1.5 mm / rev
static constexpr float LEADSCREW_MM_PER_REV = 1.5f;

// 200 fullsteps/rev * (32 microsteps / 1.5 mm) ≈ 2133.33(Theoretical value)
static constexpr float MICROSTEPS_PER_REAL_POSITION_UNIT = 3505.0f; // new coefficient = theoretical movement / actual movement

// =========================
// Runtime objects
// =========================
TMC51X0 stepper;
Adafruit_INA228 ina228;

lv_obj_t* main_screen_obj = NULL;
lv_obj_t* screen_obj      = NULL;

bool motor_spi_ready  = false;
bool motor_is_running = false;
bool moving_forward_phase = true;

int32_t current_target_chip = 0;
int32_t current_home_chip   = 0;

// =========================
// Settings model
// =========================
MotorControlScreen::MotorSpiSettings g_settings = {
    80.0f,  // runCurrentPercent
    40.0f,  // pwmOffsetPercent
    15.0f,  // pwmGradientPercent
    true,   // reverseDirection
    50.0f,  // stealthChopThreshold

    5.0f,  // maxVelocity
    5.0f,   // maxAcceleration
    0.5f,   // startVelocity
    0.5f,   // stopVelocity
    1.0f,  // firstVelocity
    2.0f,   // firstAcceleration
    5.0f,   // maxDeceleration
    2.0f,   // firstDeceleration

    MotorControlScreen::MODE_REVOLUTIONS, // mode
    1.0f,   // forwardValue
    1.0f    // backwardValue
};

// =========================
// UI objects
// =========================
lv_obj_t* ta_runCurrent         = NULL;
lv_obj_t* ta_pwmOffset          = NULL;
lv_obj_t* ta_pwmGradient        = NULL;
lv_obj_t* dd_direction          = NULL;
lv_obj_t* ta_stealthThreshold   = NULL;

lv_obj_t* ta_maxVelocity        = NULL;
lv_obj_t* ta_maxAcceleration    = NULL;
lv_obj_t* ta_startVelocity      = NULL;
lv_obj_t* ta_stopVelocity       = NULL;
lv_obj_t* ta_firstVelocity      = NULL;
lv_obj_t* ta_firstAcceleration  = NULL;
lv_obj_t* ta_maxDeceleration    = NULL;
lv_obj_t* ta_firstDeceleration  = NULL;

lv_obj_t* dd_mode               = NULL;
lv_obj_t* ta_forwardValue       = NULL;
lv_obj_t* ta_backwardValue      = NULL;

lv_obj_t* lbl_status            = NULL;
lv_obj_t* btn_start             = NULL;
lv_obj_t* btn_stop              = NULL;

// shared keypad
lv_obj_t* kb_numeric            = NULL;
lv_obj_t* kb_target_ta          = NULL;

// =========================
// Keypad map
// =========================
static const char * num_kb_map[] = {
    "1", "2", "3", "\n",
    "4", "5", "6", "\n",
    "7", "8", "9", "\n",
    "0", "00", ".", "\n",
    LV_SYMBOL_BACKSPACE, LV_SYMBOL_NEW_LINE, ""
};

// =========================
// Helpers
// =========================
float ta_to_float(lv_obj_t* ta, float fallback)
{
    if (!ta) return fallback;
    const char* txt = lv_textarea_get_text(ta);
    if (!txt || txt[0] == '\0') return fallback;
    return atof(txt);
}

void set_ta_float(lv_obj_t* ta, float value, uint8_t decimals = 2)
{
    if (!ta) return;

    char buf[32];
    snprintf(buf, sizeof(buf), "%.*f", decimals, value);
    lv_textarea_set_text(ta, buf);
}

int dd_get_selected(lv_obj_t* dd)
{
    if (!dd) return 0;
    return (int)lv_dropdown_get_selected(dd);
}

void set_status(const char* text)
{
    if (lbl_status) {
        lv_label_set_text(lbl_status, text);
    }
}

void set_status_fmt(const char* fmt, ...)
{
    if (!lbl_status) return;

    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    lv_label_set_text(lbl_status, buf);
}

// =========================
// UI helper builders
// =========================
lv_obj_t* create_title(lv_obj_t* parent, const char* text, lv_coord_t x, lv_coord_t y)
{
    lv_obj_t* lbl = lv_label_create(parent);
    lv_label_set_text(lbl, text);
    lv_obj_set_style_text_font(lbl, &montserrat_24, 0);
    lv_obj_align(lbl, LV_ALIGN_TOP_LEFT, x, y);
    return lbl;
}

lv_obj_t* create_textarea(lv_obj_t* parent, lv_coord_t x, lv_coord_t y, const char* placeholder)
{
    lv_obj_t* ta = lv_textarea_create(parent);
    lv_obj_set_size(ta, 180, 55);
    lv_obj_align(ta, LV_ALIGN_TOP_LEFT, x, y);
    lv_textarea_set_one_line(ta, true);
    lv_textarea_set_placeholder_text(ta, placeholder);
    lv_obj_set_style_text_font(ta, &montserrat_20, 0);
    lv_obj_clear_flag(ta, LV_OBJ_FLAG_SCROLLABLE);
    return ta;
}

lv_obj_t* create_dropdown(lv_obj_t* parent, lv_coord_t x, lv_coord_t y, const char* options)
{
    lv_obj_t* dd = lv_dropdown_create(parent);
    lv_obj_set_size(dd, 180, 45);
    lv_obj_align(dd, LV_ALIGN_TOP_LEFT, x, y);
    lv_dropdown_set_options(dd, options);
    lv_obj_set_style_text_font(dd, &montserrat_18, 0);
    return dd;
}

// =========================
// Keyboard
// =========================
void keyboard_attach_to(lv_obj_t* ta)
{
    if (!kb_numeric) return;
    kb_target_ta = ta;
    lv_obj_clear_flag(kb_numeric, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(kb_numeric);
}

void textarea_focus_cb(lv_event_t* e)
{
    lv_obj_t* ta = lv_event_get_target(e);
    keyboard_attach_to(ta);
}

void keyboard_event_cb(lv_event_t* e)
{
    lv_obj_t* kb = lv_event_get_target(e);
    const char* txt = lv_btnmatrix_get_btn_text(kb, lv_btnmatrix_get_selected_btn(kb));

    if (!txt || !kb_target_ta) return;

    if (strcmp(txt, LV_SYMBOL_BACKSPACE) == 0) {
        lv_textarea_del_char(kb_target_ta);
        return;
    }

    if (strcmp(txt, LV_SYMBOL_NEW_LINE) == 0) {
        lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
        kb_target_ta = NULL;
        return;
    }

    // avoid multiple '.'
    if (strcmp(txt, ".") == 0) {
        const char* cur = lv_textarea_get_text(kb_target_ta);
        if (strchr(cur, '.') != NULL) return;
    }

    // avoid '-' except first char
    if (strcmp(txt, "-") == 0) {
        const char* cur = lv_textarea_get_text(kb_target_ta);
        if (strlen(cur) > 0 || strchr(cur, '-') != NULL) return;
    }

    lv_textarea_add_text(kb_target_ta, txt);
}

void create_keyboard(lv_obj_t* parent)
{
    if (kb_numeric) return;

    kb_numeric = lv_btnmatrix_create(parent);
    lv_obj_set_size(kb_numeric, 250, 300);
    lv_obj_align(kb_numeric, LV_ALIGN_TOP_RIGHT, -20, 90);
    lv_btnmatrix_set_map(kb_numeric, num_kb_map);
    lv_obj_add_flag(kb_numeric, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(kb_numeric, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_add_event_cb(kb_numeric, keyboard_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
}

// =========================
// Settings <-> UI
// =========================
void pull_settings_from_ui_internal()
{
    g_settings.runCurrentPercent    = ta_to_float(ta_runCurrent, 80.0f);
    g_settings.pwmOffsetPercent     = ta_to_float(ta_pwmOffset, 40.0f);
    g_settings.pwmGradientPercent   = ta_to_float(ta_pwmGradient, 15.0f);
    g_settings.reverseDirection     = (dd_get_selected(dd_direction) == 1);
    g_settings.stealthChopThreshold = ta_to_float(ta_stealthThreshold, 50.0f);

    g_settings.maxVelocity          = ta_to_float(ta_maxVelocity, 8.0f);
    g_settings.maxAcceleration      = ta_to_float(ta_maxAcceleration, 5.0f);
    g_settings.startVelocity        = ta_to_float(ta_startVelocity, 0.5f);
    g_settings.stopVelocity         = ta_to_float(ta_stopVelocity, 0.5f);
    g_settings.firstVelocity        = ta_to_float(ta_firstVelocity, 2.0f);
    g_settings.firstAcceleration    = ta_to_float(ta_firstAcceleration, 2.0f);
    g_settings.maxDeceleration      = ta_to_float(ta_maxDeceleration, 5.0f);
    g_settings.firstDeceleration    = ta_to_float(ta_firstDeceleration, 2.0f);

    g_settings.mode                 = (dd_get_selected(dd_mode) == 0)
                                        ? MotorControlScreen::MODE_REVOLUTIONS
                                        : MotorControlScreen::MODE_DISTANCE_MM;

    g_settings.forwardValue         = ta_to_float(ta_forwardValue, 1.0f);
    g_settings.backwardValue        = 0.0f;
}

void push_settings_to_ui_internal()
{
    set_ta_float(ta_runCurrent,        g_settings.runCurrentPercent, 0);
    set_ta_float(ta_pwmOffset,         g_settings.pwmOffsetPercent, 0);
    set_ta_float(ta_pwmGradient,       g_settings.pwmGradientPercent, 0);
    set_ta_float(ta_stealthThreshold,  g_settings.stealthChopThreshold, 1);

    set_ta_float(ta_maxVelocity,       g_settings.maxVelocity, 2);
    set_ta_float(ta_maxAcceleration,   g_settings.maxAcceleration, 2);
    set_ta_float(ta_startVelocity,     g_settings.startVelocity, 2);
    set_ta_float(ta_stopVelocity,      g_settings.stopVelocity, 2);
    set_ta_float(ta_firstVelocity,     g_settings.firstVelocity, 2);
    set_ta_float(ta_firstAcceleration, g_settings.firstAcceleration, 2);
    set_ta_float(ta_maxDeceleration,   g_settings.maxDeceleration, 2);
    set_ta_float(ta_firstDeceleration, g_settings.firstDeceleration, 2);

    set_ta_float(ta_forwardValue,      g_settings.forwardValue, 2);
    set_ta_float(ta_backwardValue,     g_settings.backwardValue, 2);

    if (dd_direction) {
        lv_dropdown_set_selected(dd_direction, g_settings.reverseDirection ? 1 : 0);
    }

    if (dd_mode) {
        lv_dropdown_set_selected(
            dd_mode,
            (g_settings.mode == MotorControlScreen::MODE_REVOLUTIONS) ? 0 : 1
        );
    }
}

// =========================
// SPI initialization
// =========================
bool init_motor_spi_system_internal()
{
    motor_spi_ready = false;

    if (!ina228.begin(INA228_ADDR, &Wire1)) {
        set_status("INA228 not found");
        return false;
    }

    ina228.setShunt(0.02f, 5.0f);
    ina228.setAveragingCount(INA228_COUNT_16);
    ina228.setVoltageConversionTime(INA228_TIME_150_us);
    ina228.setCurrentConversionTime(INA228_TIME_280_us);

#if defined(ARDUINO_ARCH_RP2040)
    spi.setSCK(SCK_PIN);
    spi.setTX(TX_PIN);
    spi.setRX(RX_PIN);
#endif

    pinMode(PIN_OE, OUTPUT);
    digitalWrite(PIN_OE, HIGH);
    delay(10);

    auto spi_parameters =
        tmc51x0::SpiParameters{}
            .withSpi(&spi)
            .withChipSelectPin(CHIP_SELECT_PIN)
            .withClockRate(5000000);

    auto converter_parameters =
        tmc51x0::ConverterParameters{}
            .withMicrostepsPerRealPositionUnit(MICROSTEPS_PER_REAL_POSITION_UNIT);

    auto driver_parameters_real =
        tmc51x0::DriverParameters{}
            .withRunCurrent((uint8_t)g_settings.runCurrentPercent)
            .withPwmOffset((uint8_t)g_settings.pwmOffsetPercent)
            .withPwmGradient((uint8_t)g_settings.pwmGradientPercent)
            .withMotorDirection(tmc51x0::ForwardDirection)
            .withStealthChopThreshold(g_settings.stealthChopThreshold);

    auto controller_parameters_real =
        tmc51x0::ControllerParameters{}
            .withRampMode(tmc51x0::PositionMode)
            .withMaxVelocity(g_settings.maxVelocity)
            .withMaxAcceleration(g_settings.maxAcceleration)
            .withStartVelocity(g_settings.startVelocity)
            .withStopVelocity(g_settings.stopVelocity)
            .withFirstVelocity(g_settings.firstVelocity)
            .withFirstAcceleration(g_settings.firstAcceleration)
            .withMaxDeceleration(g_settings.maxDeceleration)
            .withFirstDeceleration(g_settings.firstDeceleration);

    spi.begin();

    stepper.setupSpi(spi_parameters);
    stepper.converter.setup(converter_parameters);

    auto driver_chip =
        stepper.converter.driverParametersRealToChip(driver_parameters_real);
    stepper.driver.setup(driver_chip);
    stepper.driver.setEnableHardwarePin(ENABLE_HARDWARE_PIN);

    auto controller_chip =
        stepper.converter.controllerParametersRealToChip(controller_parameters_real);
    stepper.controller.setup(controller_chip);

    if (!stepper.communicating()) {
        set_status("SPI comm failed");
        return false;
    }

    if (stepper.controller.stepAndDirectionMode()) {
        set_status("Step/Dir mode enabled");
        return false;
    }

    stepper.driver.enable();

    stepper.controller.beginRampToZeroVelocity();

    unsigned long t0 = millis();
    while (!stepper.controller.zeroVelocity()) {
        if (millis() - t0 > 3000) {
            set_status("Zero velocity timeout");
            return false;
        }
        delay(10);
    }

    stepper.controller.endRampToZeroVelocity();
    stepper.controller.zeroActualPosition();

    current_home_chip = 0;
    moving_forward_phase = true;
    motor_spi_ready = true;

    set_status("SPI ready");
    return true;
}

// =========================
// Motion helpers
// =========================
int32_t compute_target_chip_from_mode_internal(
    float value,
    MotorControlScreen::MotionInputMode mode
)
{
    float target_mm = 0.0f;
    // for motor calibration
    float scale_factor = 10.0f;

    if (mode == MotorControlScreen::MODE_DISTANCE_MM) {
        target_mm = value * scale_factor;
    } else {
        target_mm = value * LEADSCREW_MM_PER_REV * scale_factor;
    }

    return stepper.converter.positionRealToChip(target_mm);
}

void start_motion_internal()
{
    pull_settings_from_ui_internal();

    if (!init_motor_spi_system_internal()) {
        return;
    }

    int32_t target = compute_target_chip_from_mode_internal(
        g_settings.forwardValue,
        g_settings.mode
    );

    // Direction dropdown:
    // 0 = Forward
    // 1 = Reverse
    if (dd_get_selected(dd_direction) == 1) {
        target = -target;
    }

    current_target_chip = target;
    stepper.controller.writeTargetPosition(current_target_chip);

    motor_is_running = true;
    moving_forward_phase = false; // 不再使用來回模式

    set_status("Moving");
}

void stop_motion_internal()
{
    motor_is_running = false;

    if (motor_spi_ready) {
        stepper.controller.beginRampToZeroVelocity();
    }

    set_status("Stopped");
}

void update_spi_motion_internal()
{
    if (!motor_is_running || !motor_spi_ready) return;

    if (stepper.controller.positionReached()) {
        motor_is_running = false;
        set_status("Done");
        return;
    }

    float current_mA = ina228.getCurrent_mA();
    float voltage_V  = ina228.getBusVoltage_V();
    float temp_C     = ina228.readDieTemp();

    int32_t actual_pos_chip = stepper.controller.readActualPosition();
    float actual_pos_mm = stepper.converter.positionChipToReal(actual_pos_chip);

    set_status_fmt(
        "Run \nPos = %.2f mm \nI = %.2f mA \nV = %.2f V \nT = %.2f C",
        actual_pos_mm / 10,
        current_mA,
        voltage_V,
        temp_C
    );
}

// =========================
// Button callbacks
// =========================
void start_btn_event_cb(lv_event_t* e)
{
    LV_UNUSED(e);
    start_motion_internal();
}

void stop_btn_event_cb(lv_event_t* e)
{
    LV_UNUSED(e);
    stop_motion_internal();
}

void back_btn_event_cb(lv_event_t* e)
{
    LV_UNUSED(e);

    if (main_screen_obj) {
        lv_scr_load(main_screen_obj);
    }

    lv_async_call(MotorControlScreen::destroyAsync, NULL);
}

} // anonymous namespace

// =========================
// Public namespace impl
// =========================
namespace MotorControlScreen {

void setMainScreen(lv_obj_t* screen)
{
    main_screen_obj = screen;
}

void build()
{
    if (screen_obj) return;

    screen_obj = lv_obj_create(NULL);
    //lv_obj_clear_flag(screen_obj, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* title = lv_label_create(screen_obj);
    lv_label_set_text(title, "Motor Control (SPI)");
    lv_obj_set_style_text_font(title, &montserrat_40, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

    // ===== Motion command =====
    create_title(screen_obj, "Mode", 20, 65);
    dd_mode = create_dropdown(screen_obj, 20, 95, "Revolutions\nDistance(mm)");

    create_title(screen_obj, "Direction", 210, 65);
    dd_direction = create_dropdown(screen_obj, 210, 95, "Forward\nReverse");

    create_title(screen_obj, "Quantity", 400, 65);
    ta_forwardValue = create_textarea(screen_obj, 400, 95, "1.0");

    // create_title(screen_obj, "Forward", 210, 60);
    // ta_forwardValue = create_textarea(screen_obj, 210, 90, "1.0");

    // create_title(screen_obj, "Backward", 380, 60);
    // ta_backwardValue = create_textarea(screen_obj, 380, 90, "1.0");

    // ===== Driver params =====
    create_title(screen_obj, "RunCurrent %", 20, 145);
    ta_runCurrent = create_textarea(screen_obj, 20, 175, "80");

    create_title(screen_obj, "PwmOffset %", 210, 145);
    ta_pwmOffset = create_textarea(screen_obj, 210, 175, "40");

    create_title(screen_obj, "PwmGradient %", 400, 145);
    ta_pwmGradient = create_textarea(screen_obj, 400, 175, "15");

    create_title(screen_obj, "StealthThr", 20, 225);
    ta_stealthThreshold = create_textarea(screen_obj, 20, 255, "50");

    // ===== Controller params =====
    create_title(screen_obj, "MaxVel", 20, 390);
    ta_maxVelocity = create_textarea(screen_obj, 20, 420, "10");

    create_title(screen_obj, "MaxAcc", 210, 390);
    ta_maxAcceleration = create_textarea(screen_obj, 210, 420, "5");

    create_title(screen_obj, "StartVel", 210, 225);
    ta_startVelocity = create_textarea(screen_obj, 210, 255, "0.5");

    create_title(screen_obj, "StopVel", 400, 225);
    ta_stopVelocity = create_textarea(screen_obj, 400, 255, "0.5");

    create_title(screen_obj, "FirstVel", 20, 310);
    ta_firstVelocity = create_textarea(screen_obj, 20, 340, "1.0");

    create_title(screen_obj, "FirstAcc", 210, 310);
    ta_firstAcceleration = create_textarea(screen_obj, 210, 340, "2");

    create_title(screen_obj, "MaxDec", 400, 390);
    ta_maxDeceleration = create_textarea(screen_obj, 400, 420, "5");

    create_title(screen_obj, "FirstDec", 400, 310);
    ta_firstDeceleration = create_textarea(screen_obj, 400, 340, "2");

    // ===== Buttons =====
    btn_start = lv_btn_create(screen_obj);
    lv_obj_set_size(btn_start, 135, 60);
    lv_obj_align(btn_start, LV_ALIGN_TOP_RIGHT, -20, 80);
    lv_obj_add_event_cb(btn_start, start_btn_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t* lbl_start = lv_label_create(btn_start);
    lv_label_set_text(lbl_start, "START");
    lv_obj_set_style_text_font(lbl_start, &montserrat_24, 0);
    lv_obj_center(lbl_start);

    btn_stop = lv_btn_create(screen_obj);
    lv_obj_set_size(btn_stop, 135, 60);
    lv_obj_align(btn_stop, LV_ALIGN_TOP_RIGHT, -20, 155);
    lv_obj_set_style_bg_color(btn_stop, lv_palette_main(LV_PALETTE_RED), LV_PART_MAIN);
    lv_obj_add_event_cb(btn_stop, stop_btn_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t* lbl_stop = lv_label_create(btn_stop);
    lv_label_set_text(lbl_stop, "STOP");
    lv_obj_set_style_text_font(lbl_stop, &montserrat_24, 0);
    lv_obj_center(lbl_stop);

    lv_obj_t* btn_back = lv_btn_create(screen_obj);
    lv_obj_set_size(btn_back, 135, 60);
    lv_obj_align(btn_back, LV_ALIGN_TOP_RIGHT, -20, 230);
    lv_obj_set_style_bg_color(btn_back, lv_palette_main(LV_PALETTE_GREY), LV_PART_MAIN);
    lv_obj_add_event_cb(btn_back, back_btn_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t* lbl_back = lv_label_create(btn_back);
    lv_label_set_text(lbl_back, "BACK");
    lv_obj_set_style_text_color(lbl_back, lv_color_hex(0xFFFFFF),  LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_back, &montserrat_24, 0);
    lv_obj_center(lbl_back);

    // ===== Status =====
    lbl_status = lv_label_create(screen_obj);
    lv_label_set_text(lbl_status, "Idle");
    lv_obj_set_style_text_font(lbl_status, &montserrat_18, 0);
    lv_label_set_long_mode(lbl_status, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_status, 160);
    lv_obj_align(lbl_status, LV_ALIGN_TOP_RIGHT, -13.5, 310);

    // keyboard for textareas
    create_keyboard(screen_obj);

    lv_obj_t* textareas[] = {
        ta_forwardValue,
        ta_backwardValue,
        ta_runCurrent,
        ta_pwmOffset,
        ta_pwmGradient,
        ta_stealthThreshold,
        ta_maxVelocity,
        ta_maxAcceleration,
        ta_startVelocity,
        ta_stopVelocity,
        ta_firstVelocity,
        ta_firstAcceleration,
        ta_maxDeceleration,
        ta_firstDeceleration
    };

    for (size_t i = 0; i < sizeof(textareas)/sizeof(textareas[0]); i++) {
        if (textareas[i]) {
            lv_obj_add_event_cb(textareas[i], textarea_focus_cb, LV_EVENT_FOCUSED, NULL);
            lv_obj_add_event_cb(textareas[i], textarea_focus_cb, LV_EVENT_CLICKED, NULL);
        }
    }

    push_settings_to_ui_internal();
}

lv_obj_t* getScreen()
{
    return screen_obj;
}

lv_obj_t** getScreenHandle()
{
    return &screen_obj;
}

void update()
{
    update_spi_motion_internal();
}

void destroy()
{
    if (screen_obj) {
        lv_obj_del(screen_obj);
        screen_obj = NULL;
    }

    ta_runCurrent = NULL;
    ta_pwmOffset = NULL;
    ta_pwmGradient = NULL;
    dd_direction = NULL;
    ta_stealthThreshold = NULL;

    ta_maxVelocity = NULL;
    ta_maxAcceleration = NULL;
    ta_startVelocity = NULL;
    ta_stopVelocity = NULL;
    ta_firstVelocity = NULL;
    ta_firstAcceleration = NULL;
    ta_maxDeceleration = NULL;
    ta_firstDeceleration = NULL;

    dd_mode = NULL;
    ta_forwardValue = NULL;
    ta_backwardValue = NULL;

    lbl_status = NULL;
    btn_start = NULL;
    btn_stop = NULL;

    kb_numeric = NULL;
    kb_target_ta = NULL;

    motor_spi_ready = false;
    motor_is_running = false;
    moving_forward_phase = true;
    current_target_chip = 0;
    current_home_chip = 0;
}

void destroyAsync(void* user_data)
{
    LV_UNUSED(user_data);
    destroy();
}

const MotorSpiSettings& getSettings()
{
    return g_settings;
}

void setSettings(const MotorSpiSettings& settings)
{
    g_settings = settings;
    push_settings_to_ui_internal();
}

void resetSettingsToDefault()
{
    g_settings = {
        80.0f,
        40.0f,
        15.0f,
        true,
        50.0f,
        5.0f,
        5.0f,
        0.5f,
        0.5f,
        1.0f,
        2.0f,
        5.0f,
        2.0f,
        MODE_REVOLUTIONS,
        1.0f,
        1.0f
    };

    push_settings_to_ui_internal();
}

bool pullSettingsFromUi()
{
    pull_settings_from_ui_internal();
    return true;
}

void pushSettingsToUi()
{
    push_settings_to_ui_internal();
}

bool initMotorSpiSystem()
{
    pull_settings_from_ui_internal();
    return init_motor_spi_system_internal();
}

void stopMotion()
{
    stop_motion_internal();
}

int32_t computeTargetChipFromMode(float value, MotionInputMode mode)
{
    return compute_target_chip_from_mode_internal(value, mode);
}

bool isReady()
{
    return motor_spi_ready;
}

bool isRunning()
{
    return motor_is_running;
}

// =========================
// Start Single Move
// =========================
bool startSingleMove(float quantity_uL, float stroke_per_ml_mm)
{
    if (quantity_uL <= 0.0f || stroke_per_ml_mm <= 0.0f) {
        return false;
    }

    pull_settings_from_ui_internal();

    g_settings.maxVelocity =
        CustomizedParametersScreen::getSavedFloat(
            CustomizedParametersScreen::DEV_P_MAX_LINEAR_SPEED_MM_S
        );

    if (g_settings.maxVelocity < 0.1f) {
        g_settings.maxVelocity = 0.1f;
    }

    if (!init_motor_spi_system_internal()) {
        return false;
    }

    // uL -> mL -> mm
    const float quantity_mL = quantity_uL / 1000.0f;
    const float target_mm = quantity_mL * stroke_per_ml_mm;

    if (target_mm <= 0.0f) {
        return false;
    }

    int32_t target_chip =
        compute_target_chip_from_mode_internal(
            target_mm,
            MODE_DISTANCE_MM
        );

    // Direction dropdown:
    // 0 = Forward
    // 1 = Reverse
    if (g_settings.reverseDirection) {
        target_chip = -target_chip;
    }

    current_target_chip = target_chip;
    stepper.controller.writeTargetPosition(current_target_chip);

    motor_is_running = true;
    set_status("Low viscosity move");

    return true;
}

bool motionFinished()
{
    if (!motor_spi_ready) return true;

    if (!motor_is_running) return true;

    if (stepper.controller.positionReached()) {
        motor_is_running = false;
        set_status("Done");
        return true;
    }

    return false;
}

float getMeasuredCurrentA()
{
    if (!motor_spi_ready) return 0.0f;
    return ina228.getCurrent_mA() / 1000.0f;
}

} // namespace MotorControlScreen


