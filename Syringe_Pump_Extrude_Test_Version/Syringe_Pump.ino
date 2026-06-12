//#include "arduino_secrets.h"

#define LV_CONF_INCLUDE_SIMPLE

#include "lv_conf.h"
#include "Arduino_H7_Video.h"
#include "Arduino_GigaDisplayTouch.h"
#include "lvgl.h" //LVGL version 8.3.10
#include "fonts.h"
#include "images.h"
#include "FlowConstraintModel.h"
#include "PlungerMotionPlanner.h"
#include "TmcExecutableRampPlanner.h"
#include "RealtimeCurveRenderer.h"
#include "MotorControlScreen.h"
#include "CustomizedParametersScreen.h"
#include "MotionModeManager.h"

// ====== For Planner Value Monitor =====
#define DEBUG_PLANNER 1
// ======================================

// --- Global Variables ---
// struct put before all function
typedef struct{
  const char** keymap;          // keypad layout
  lv_align_t align;             // keypad align mode
  lv_coord_t x_ofs;             // keypad x offset
  lv_coord_t y_ofs;             // keypad y offset
  bool allow_decimal;           // allow "."
  bool allow_double_zero;       // allow "00"
  uint8_t max_chars;            // max input length, 0 = no limit
} keypad_config_t;

typedef struct{
  lv_obj_t* display_label; // area to show value
  lv_obj_t* textarea;      // user input
  lv_obj_t* options_btn;
  lv_obj_t* list;
  lv_obj_t* keyboard;      // dedicated keypad
  const keypad_config_t* keypad_cfg; // keypad config for this block
  bool has_options;
}input_block_t;

typedef struct{
  const char* title;
  const char* value;
}option_item_t;

// struct for numeric keypad
static const char * num_kb_map[] = {
    "1", "2", "3", LV_SYMBOL_BACKSPACE, "\n",
    "4", "5", "6", LV_SYMBOL_BACKSPACE, "\n",
    "7", "8", "9", LV_SYMBOL_NEW_LINE, "\n",
    "0", "00", ".", LV_SYMBOL_NEW_LINE, ""
};

static const char * viscosity_kb_map[] = {
    "1", "2", "3", LV_SYMBOL_BACKSPACE, "\n",
    "4", "5", "6", LV_SYMBOL_BACKSPACE, "\n",
    "7", "8", "9", LV_SYMBOL_NEW_LINE, "\n",
    "0", "00", "000", LV_SYMBOL_NEW_LINE, ""
};

static const char * quantity_kb_map[] = {
    "1", "2", "3", LV_SYMBOL_BACKSPACE, "\n",
    "4", "5", "6", LV_SYMBOL_BACKSPACE, "\n",
    "7", "8", "9", LV_SYMBOL_NEW_LINE, "\n",
    "0", "00", "000", LV_SYMBOL_NEW_LINE, ""
};

static const keypad_config_t viscosity_keypad_cfg = {
    viscosity_kb_map,
    LV_ALIGN_BOTTOM_MID,
    160,
    -50,
    false,   // allow decimal
    true,  // allow "00"
    5       // max chars
};

static const keypad_config_t quantity_keypad_cfg = {
    quantity_kb_map,
    LV_ALIGN_BOTTOM_MID,
    -160,
    -50,
    false,  // no decimal
    true,   // allow "00"
    4       // max chars
};

// --- Tab ---
lv_obj_t* tab1;
lv_obj_t* tab2;
lv_obj_t* tab3;
// --- Screen navigation ---
static lv_obj_t* main_screen = NULL;
static lv_obj_t* manual_screen_tab1 = NULL;
static lv_obj_t* manual_screen_tab2 = NULL;
static lv_obj_t* manual_screen_tab3 = NULL;
// --- START / STOP state ---
static bool is_running = false;
static lv_obj_t* start_btn = NULL;
static lv_obj_t* confirm_dialog = NULL;
// --- Retraction / Backdraw mode ---
static lv_obj_t* retract_switch = NULL;
static lv_obj_t* retract_switch_label = NULL;

static bool retract_mode_enabled = false;        // UI目前設定
static bool run_retract_mode_enabled = false;    // 本次運行鎖定的設定

static bool retract_move_started = false;

static const float RETRACT_DISTANCE_MM = 3.0f; // retract distance
static const float RETRACT_MAX_VELOCITY_MM_S = 0.20f;
static const float RETRACT_ACCEL_MM_S2 = 0.1f;
static const float RETRACT_DECEL_MM_S2 = 0.1f;
// --- Recorded parameters ---
static const char* recorded_viscosity = NULL;
static const char* recorded_quantity  = NULL;
// predicted duration from motion planner
static float recorded_duration_sec_exact = 0.0f; // exact predicted duration (s)
static int   recorded_duration_sec       = 30;   // rounded duration for existing countdown logic
static bool  recorded_duration_valid     = false;
// --- Timer ---
static unsigned long run_start_ms = 0;
// --- countdown overlay ---
static lv_obj_t* finish_dialog = NULL;
static lv_obj_t* countdown_dialog = NULL;
static lv_obj_t* countdown_label  = NULL;
static lv_obj_t* countdown_ok_btn = NULL;
static unsigned long last_countdown_update = 0;
// --- System Progress UI ---
static lv_obj_t* arc_progress = NULL;
static lv_obj_t* lbl_percent = NULL;

static lv_obj_t* lbl_viscosity = NULL;
static lv_obj_t* lbl_quantity = NULL;
static lv_obj_t* lbl_remaining = NULL;
static lv_obj_t* lbl_duration = NULL;
static lv_obj_t* lbl_elapsed = NULL;
static lv_obj_t* lbl_tmc_target_distance = NULL;
static lv_obj_t* lbl_tmc_actual_distance = NULL;
// TMC distance monitor cache.
// Must be declared before show_start_confirm_dialog(), because the START lambda resets it.
static float last_tmc_target_distance_mm = 0.0f;
static float last_tmc_actual_distance_mm = 0.0f;
static bool tmc_distance_frozen_after_extrusion = false;
// --- Tab3 preview panels ---
static lv_obj_t* tab3_volume_curve_panel = NULL;
static lv_obj_t* tab3_current_curve_panel = NULL;
// --- Tab3 control buttons ---
static lv_obj_t* btn_tab3_parameter_setting = NULL;
static lv_obj_t* btn_tab3_motor_control = NULL;
static lv_obj_t* btn_tab3_disable_motor = NULL;
// --- Motion planner runtime ---
static PlungerMotionPlanner realtime_motion_planner;
static bool realtime_motion_planner_valid = false;

// TMC executable ramp planner
static TmcExecutableRampPlanner tmc_executable_planner;
static bool tmc_executable_planner_valid = false;
static TmcRampCommand g_tmc_ramp_cmd;

static float realtime_curve_volume_max_uL = 1000.0f;
static float realtime_curve_current_max_A = 3.0f;

static unsigned long last_curve_update_ms = 0;
// --- Current tail recording after motor finish ---
static const float CURRENT_TAIL_SEC = 5.0f;      // motor finish 後多記錄 5 秒電流
static float chart_total_duration_sec = 0.0f;    // chart X 軸總時間
static bool current_tail_recording = false;
static unsigned long current_tail_start_ms = 0;
// --- Global Variables ---
input_block_t* viscosity_block;
input_block_t* quantity_block;
// --- Stop request state ---
static bool stop_requested = false;
static bool stop_requested_by_timeout = false;
static unsigned long stop_request_ms = 0;
// --- Display Duration Seconds ---
static float extrusion_duration_sec_exact = 0.0f;
static float retraction_duration_sec_exact = 0.0f;

// 主擠出 + 回抽，不含 current tail
static float operation_motion_duration_sec_exact = 0.0f;

// UI 顯示與倒數用，包含保守估算
static float display_duration_sec_exact = 0.0f;
static int   display_duration_sec = 0;

// 你目前 60000 mPas / 1 mL 實測：865.85 / 845.86 = 1.0236
// 先用 1.024 修正模型低估問題
static const float EXTRUSION_DURATION_CALIBRATION = 1.024f;
static const float RETRACTION_DURATION_CALIBRATION = 1.000f;

// 顯示給使用者看的保守 margin
static const float DISPLAY_MARGIN_RATIO = 0.25f;
static const float DISPLAY_MARGIN_MIN_SEC = 3.0f;

// phase timeout margin
// 一般 phase timeout 使用比例型 margin，不再使用固定 240 s 上限。
static const float TIMEOUT_MARGIN_RATIO = 0.30f;
static const float TIMEOUT_MARGIN_MIN_SEC = 30.0f;

// 開啟 retraction 時，主擠出 phase 使用更寬鬆的比例型 timeout。
// 目的：避免長時間高黏度擠出在 motionFinished() 之前被 timeout stop，導致 retraction 無法觸發。
static const float RETRACTION_EXTRUSION_TIMEOUT_RATIO = 0.50f;
static const float RETRACTION_EXTRUSION_TIMEOUT_MIN_SEC = 300.0f;

// Clear residual value of countdown
static bool system_progress_force_remaining_zero = false;
static bool system_progress_force_complete = false;
static bool system_progress_tail_ramp_active = false;
static float system_progress_tail_start_percent = 0.0f;
static unsigned long system_progress_tail_start_ms = 0;

// retarct manager
enum RunPhase {
    RUN_PHASE_IDLE,
    RUN_PHASE_EXTRUDING,
    RUN_PHASE_RETRACTING,
    RUN_PHASE_TAIL_RECORDING
};
//

static RunPhase run_phase = RUN_PHASE_IDLE;
static unsigned long phase_start_ms = 0;

typedef struct {
    const char* key;
    const char* unit;
    const char* default_str;   // 預設值（字串）
    char saved_str[32];        // 已儲存值
} dev_param_value_t;

// --- Build Screen ---
typedef struct {
    lv_obj_t** target_screen;
    void (*screen_builder)();
} nav_btn_ctx_t;
// --- Static Context Pool ---
static nav_btn_ctx_t nav_btn_ctx_pool[8];
static uint8_t nav_btn_ctx_count = 0;
static input_block_t input_block_pool[4];
static uint8_t input_block_count = 0;
// --------------------------------------

// --- Display Shield Initialization  ---
Arduino_H7_Video Display(800, 480, GigaDisplayShield);
Arduino_GigaDisplayTouch TouchDetector;
// --------------------------------------

// --- Multi-Tab Initialization ---
void lv_initial_tabview(void){
  lv_obj_t* tabview = lv_tabview_create(lv_scr_act(), LV_DIR_TOP, 80);
  lv_obj_t* tab_btns = lv_tabview_get_tab_btns(tabview);
  lv_obj_set_style_text_font(tab_btns, &montserrat_22, 0);
  (void)tab_btns;

  tab1 = lv_tabview_add_tab(tabview, "SETTING");
  lv_obj_clear_flag(tab1, LV_OBJ_FLAG_SCROLLABLE);
  tab2 = lv_tabview_add_tab(tabview, "SYSTEM PROGRESS");
  lv_obj_clear_flag(tab2, LV_OBJ_FLAG_SCROLLABLE);
  tab3 = lv_tabview_add_tab(tabview, "DEVELOPER INFO");
  lv_obj_clear_flag(tab3, LV_OBJ_FLAG_SCROLLABLE);
}
// --------------------------------------

// --- START / STOP Button ---
static void start_btn_event_cb(lv_event_t* e)
{
    lv_obj_t* btn = lv_event_get_target(e);

    if (!is_running) {
        fetch_setting_values();

        if (!recorded_viscosity) {
            show_missing_param_dialog("Missing liquid viscosity");
            return;
        }

        if (!recorded_quantity) {
            show_missing_param_dialog("Missing extrusion quantity");
            return;
        }

        float viscosity_value = atof(recorded_viscosity);
        float quantity_value  = atof(recorded_quantity);

        if (viscosity_value <= 0.0f) {
            show_missing_param_dialog("Invalid liquid viscosity");
            return;
        }

        if (quantity_value <= 0.0f) {
            show_missing_param_dialog("Invalid extrusion quantity");
            return;
        }

        auto tr = MotionModeManager::requestScenario(
            MotionModeManager::SCENARIO_EXTRUSION_ACTIVE
        );

        if (!tr.success) {
            show_missing_param_dialog(tr.message);
            return;
        }

        // 鎖定本次確認視窗使用的回抽模式
        run_retract_mode_enabled = retract_mode_enabled;

        if (!calculate_predicted_duration()) {
            MotionModeManager::forceIdle("Failed to build extrusion planner");
            show_missing_param_dialog("Failed to build extrusion planner");
            return;
        }

        // 確認視窗打開後，不允許再改回抽模式
        set_retract_switch_enabled(false);

        show_start_confirm_dialog(btn);  
    }
    else {
        show_confirm_dialog(btn, tab1);
    }
}

void create_start_button(lv_obj_t* parent, lv_coord_t x, lv_coord_t y)
{
    start_btn = lv_btn_create(parent);
    lv_obj_set_size(start_btn, 250, 60);
    lv_obj_align(start_btn, LV_ALIGN_BOTTOM_MID, x, y);
    lv_obj_set_style_bg_color(start_btn, lv_color_hex(0x0000FF), 0);

    lv_obj_add_event_cb(start_btn, start_btn_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t* label = lv_label_create(start_btn);
    lv_label_set_text(label, "START");
    lv_obj_set_style_text_font(label, &montserrat_30, 0);
    lv_obj_center(label);
}

static void update_retract_switch_label()
{
    if (!retract_switch_label) return;

    lv_label_set_text(
        retract_switch_label,
        retract_mode_enabled ? "Retraction: ON" : "Retraction: OFF"
    );
}

static void retract_switch_event_cb(lv_event_t* e)
{
    lv_obj_t* sw = lv_event_get_target(e);

    retract_mode_enabled = lv_obj_has_state(sw, LV_STATE_CHECKED);

    update_retract_switch_label();
}

static void create_retract_mode_switch(lv_obj_t* parent)
{
    retract_switch_label = lv_label_create(parent);
    lv_label_set_text(retract_switch_label, "Retraction: OFF");
    lv_obj_set_style_text_font(retract_switch_label, &montserrat_20, 0);

    // 放在 User Manual 按鈕左邊
    lv_obj_align(
        retract_switch_label,
        LV_ALIGN_BOTTOM_LEFT,
        10,
        -60
    );

    retract_switch = lv_switch_create(parent);
    lv_obj_set_size(retract_switch, 90, 45);

    // User Manual 按鈕目前在右下角，所以 switch 往左放
    lv_obj_align(
        retract_switch,
        LV_ALIGN_BOTTOM_LEFT,
        10,
        -10
    );

    // 不要求和附圖一致，只做明顯 ON/OFF 顏色
    lv_obj_set_style_bg_color(
        retract_switch,
        lv_palette_main(LV_PALETTE_BLUE),
        LV_PART_INDICATOR | LV_STATE_CHECKED
    );

    // disabled 時反灰
    lv_obj_set_style_opa(
        retract_switch,
        LV_OPA_50,
        LV_STATE_DISABLED
    );

    lv_obj_add_event_cb(
        retract_switch,
        retract_switch_event_cb,
        LV_EVENT_VALUE_CHANGED,
        NULL
    );

    update_retract_switch_label();
}

void show_confirm_dialog(lv_obj_t* btn, lv_obj_t* parent)
{
    if (confirm_dialog) return;

    confirm_dialog = lv_obj_create(parent);
    lv_obj_set_size(confirm_dialog, 420, 200);
    lv_obj_center(confirm_dialog);
    lv_obj_set_style_radius(confirm_dialog, 12, 0);
    lv_obj_set_style_bg_color(confirm_dialog, lv_palette_lighten(LV_PALETTE_GREY, 3), 0);
    lv_obj_set_style_pad_all(confirm_dialog, 20, 0);

    lv_obj_t* msg = lv_label_create(confirm_dialog);
    lv_label_set_text(msg, "Stop the operation?");
    lv_obj_set_style_text_font(msg, &montserrat_32, 0);
    lv_obj_align(msg, LV_ALIGN_TOP_MID, 0, 10);

    // YES
    lv_obj_t* btn_yes = lv_btn_create(confirm_dialog);
    lv_obj_set_size(btn_yes, 120, 50);
    lv_obj_align(btn_yes, LV_ALIGN_BOTTOM_LEFT, 30, -20);
    lv_obj_set_style_bg_color(btn_yes, lv_palette_main(LV_PALETTE_RED), 0);

    lv_obj_t* lbl_yes = lv_label_create(btn_yes);
    lv_label_set_text(lbl_yes, "YES");
    lv_obj_set_style_text_font(lbl_yes, &montserrat_24, 0);
    lv_obj_center(lbl_yes);

    // NO
    lv_obj_t* btn_no = lv_btn_create(confirm_dialog);
    lv_obj_set_size(btn_no, 120, 50);
    lv_obj_align(btn_no, LV_ALIGN_BOTTOM_RIGHT, -30, -20);

    lv_obj_t* lbl_no = lv_label_create(btn_no);
    lv_label_set_text(lbl_no, "NO");
    lv_obj_set_style_text_font(lbl_no, &montserrat_24, 0);
    lv_obj_center(lbl_no);

    // YES event
    lv_obj_add_event_cb(btn_yes, [](lv_event_t* e){

        LV_UNUSED(e);

        auto tr = MotionModeManager::requestScenario(
            MotionModeManager::SCENARIO_EXTRUSION_STOPPING
        );

        if (!tr.success) {
            MotionModeManager::forceIdle("User stop requested");
        }
        
        MotorControlScreen::forceDisableMotor();

        stop_requested = false;
        stop_requested_by_timeout = false;
        stop_request_ms = 0;

        is_running = false;
        current_tail_recording = false;
        current_tail_start_ms = 0;
        chart_total_duration_sec = 0.0f;
        run_phase = RUN_PHASE_IDLE;
        phase_start_ms = 0;

        MotionModeManager::forceIdle("User stopped and motor disabled");

        recorded_duration_valid = false;
        realtime_motion_planner_valid = false;
        tmc_executable_planner_valid = false;
        g_tmc_ramp_cmd = TmcRampCommand();

        set_tab3_control_buttons_enabled(true);
        set_retract_switch_enabled(true);
        retract_move_started = false;

        if (RealtimeCurveRenderer::isInitialized()) {
            RealtimeCurveRenderer::releaseTempData();
        }

        if (start_btn) {
            lv_obj_t* label = lv_obj_get_child(start_btn, 0);
            lv_label_set_text(label, "START");
            lv_obj_set_style_bg_color(start_btn, lv_color_hex(0x0000FF), 0);
        }

        clear_countdown_dialog();

        if (confirm_dialog) {
            lv_obj_del(confirm_dialog);
            confirm_dialog = NULL;
        }

        show_finish_dialog();

        if (confirm_dialog) {
            lv_obj_del(confirm_dialog);
            confirm_dialog = NULL;
        }

    }, LV_EVENT_CLICKED, NULL);
    // NO event
    lv_obj_add_event_cb(btn_no, [](lv_event_t*){
        lv_obj_del(confirm_dialog);
        confirm_dialog = NULL;
    }, LV_EVENT_CLICKED, NULL);
}
// --------------------------------------

// --- Setting tab data processing & error handle ---
bool fetch_setting_values()
{
    const char* viscosity = lv_label_get_text(viscosity_block->display_label);
    const char* quantity  = lv_label_get_text(quantity_block->display_label);

    // 判斷是否為未輸入狀態
    if (strcmp(viscosity, "VALUE") == 0) {
        recorded_viscosity = NULL;
    } else {
        recorded_viscosity = viscosity;
    }

    if (strcmp(quantity, "VALUE") == 0) {
        recorded_quantity = NULL;
    } else {
        recorded_quantity = quantity;
    }

    // success only if both exist
    return (recorded_viscosity && recorded_quantity);
}

static float apply_duration_calibration(float raw_duration_s, float calibration)
{
    if (raw_duration_s <= 0.0f) {
        return 0.0f;
    }

    return raw_duration_s * calibration;
}

static float calc_display_margin_s(float base_duration_s)
{
    if (base_duration_s <= 0.0f) {
        return DISPLAY_MARGIN_MIN_SEC;
    }

    float margin = base_duration_s * DISPLAY_MARGIN_RATIO;

    if (margin < DISPLAY_MARGIN_MIN_SEC) {
        margin = DISPLAY_MARGIN_MIN_SEC;
    }

    return margin;
}

static float calc_timeout_margin_s(float base_duration_s)
{
    if (base_duration_s <= 0.0f) {
        return TIMEOUT_MARGIN_MIN_SEC;
    }

    float margin = base_duration_s * TIMEOUT_MARGIN_RATIO;

    if (margin < TIMEOUT_MARGIN_MIN_SEC) {
        margin = TIMEOUT_MARGIN_MIN_SEC;
    }

    return margin;
}

static float calc_extrusion_timeout_margin_s(float base_duration_s)
{
    float margin = calc_timeout_margin_s(base_duration_s);

    if (run_retract_mode_enabled) {
        float retract_safe_margin =
            base_duration_s * RETRACTION_EXTRUSION_TIMEOUT_RATIO;

        if (retract_safe_margin < RETRACTION_EXTRUSION_TIMEOUT_MIN_SEC) {
            retract_safe_margin = RETRACTION_EXTRUSION_TIMEOUT_MIN_SEC;
        }

        if (retract_safe_margin > margin) {
            margin = retract_safe_margin;
        }
    }

    return margin;
}

static void format_seconds_2dp(char* buf, size_t buf_size, float seconds)
{
    if (!buf || buf_size == 0) return;

    if (seconds < 0.0f) {
        seconds = 0.0f;
    }

    unsigned long ms =
        (unsigned long)(seconds * 1000.0f + 0.5f);

    unsigned long sec_int  = ms / 1000;
    unsigned long sec_frac = (ms % 1000) / 10;

    snprintf(buf, buf_size, "%lu.%02lu", sec_int, sec_frac);
}

static float calc_system_progress_percent_f()
{
    unsigned long elapsed_ms = millis() - run_start_ms;

    float progress_total_sec = operation_motion_duration_sec_exact;

    if (progress_total_sec <= 0.0f) {
        progress_total_sec = recorded_duration_sec_exact;
    }

    if (progress_total_sec <= 0.0f) {
        progress_total_sec = 1.0f;
    }

    float percent_f =
        ((elapsed_ms / 1000.0f) * 100.0f) / progress_total_sec;

    if (percent_f < 0.0f) {
        percent_f = 0.0f;
    }

    if (percent_f > 99.9f) {
        percent_f = 99.9f;
    }

    return percent_f;
}

static void set_system_progress_percent(float percent_f, bool allow_100)
{
    if (percent_f < 0.0f) {
        percent_f = 0.0f;
    }

    if (allow_100) {
        if (percent_f > 100.0f) {
            percent_f = 100.0f;
        }
    }
    else {
        if (percent_f > 99.9f) {
            percent_f = 99.9f;
        }
    }

    if (arc_progress) {
        int arc_percent = allow_100
            ? (int)(percent_f + 0.5f)
            : (int)(percent_f);

        lv_arc_set_value(arc_progress, arc_percent);
    }

    if (lbl_percent) {
        int percent_x10 = (int)(percent_f * 10.0f + 0.5f);

        if (allow_100 && percent_x10 > 1000) {
            percent_x10 = 1000;
        }

        if (!allow_100 && percent_x10 > 999) {
            percent_x10 = 999;
        }

        lv_label_set_text_fmt(
            lbl_percent,
            "%d.%d%%",
            percent_x10 / 10,
            percent_x10 % 10
        );
    }
}

static void begin_system_progress_tail_ramp()
{
    system_progress_force_remaining_zero = true;
    system_progress_force_complete = false;

    system_progress_tail_ramp_active = true;
    system_progress_tail_start_ms = millis();
    system_progress_tail_start_percent = calc_system_progress_percent_f();

    if (system_progress_tail_start_percent > 99.9f) {
        system_progress_tail_start_percent = 99.9f;
    }

    if (lbl_remaining) {
        lv_label_set_text(
            lbl_remaining,
            "Remaining Time (s): \n0.00"
        );
    }
}

static void force_system_progress_operation_complete()
{
    system_progress_force_remaining_zero = true;
    system_progress_force_complete = true;

    system_progress_tail_ramp_active = false;
    system_progress_tail_start_percent = 0.0f;
    system_progress_tail_start_ms = 0;

    if (lbl_remaining) {
        lv_label_set_text(
            lbl_remaining,
            "Remaining Time (s): \n0.00"
        );
    }

    set_system_progress_percent(100.0f, true);

    if (lbl_percent && arc_progress) {
        lv_obj_align_to(
            lbl_percent,
            arc_progress,
            LV_ALIGN_CENTER,
            0,
            0
        );
    }
}

static TmcRampCommand make_retract_command_4mm();
static bool start_retract_move_4mm();

void show_start_confirm_dialog(lv_obj_t* start_btn)
{
    if (confirm_dialog) return;

    confirm_dialog = lv_obj_create(lv_scr_act());
    lv_obj_set_size(confirm_dialog, 500, 280);
    lv_obj_center(confirm_dialog);
    lv_obj_set_style_radius(confirm_dialog, 12, 0);
    lv_obj_set_style_bg_color(confirm_dialog, lv_palette_lighten(LV_PALETTE_GREY, 3), 0);
    lv_obj_set_style_pad_all(confirm_dialog, 20, 0);

    // Title
    lv_obj_t* title = lv_label_create(confirm_dialog);
    lv_label_set_text(title, "Start with following settings:");
    lv_obj_set_style_text_font(title, &montserrat_30, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 0);

    // Info text
    lv_obj_t* info = lv_label_create(confirm_dialog);

    char info_buf[256];
    //
    if (recorded_duration_valid) {
        char duration_buf[32];
        char retract_buf[32];
        char backdraw_buf[64];

        format_seconds_2dp(
            duration_buf,
            sizeof(duration_buf),
            display_duration_sec_exact
        );

        if (run_retract_mode_enabled && retraction_duration_sec_exact > 0.0f)  {
            format_seconds_2dp(
                retract_buf,
                sizeof(retract_buf),
                retraction_duration_sec_exact
            );

            snprintf(
                backdraw_buf,
                sizeof(backdraw_buf),
                "ON (+%s s)",
                retract_buf
            );
        }
        else {
            snprintf(
                backdraw_buf,
                sizeof(backdraw_buf),
                "OFF"
            );
        }

        snprintf(
            info_buf,
            sizeof(info_buf),
            "Liquid Viscosity: %s\n"
            "Extrusion Quantity: %s\n"
            "Duration: %s s\n"
            "Backdraw: %s",
            recorded_viscosity,
            recorded_quantity,
            duration_buf,
            backdraw_buf
        );
    }
    else {
        snprintf(
            info_buf,
            sizeof(info_buf),
            "Liquid Viscosity: %s\n"
            "Extrusion Quantity: %s\n"
            "Duration: N/A\n"
            "Backdraw: %s",
            recorded_viscosity,
            recorded_quantity,
            run_retract_mode_enabled ? "ON" : "OFF"
        );
    }

    lv_label_set_text(info, info_buf);
    lv_obj_set_style_text_font(info, &montserrat_28, 0);
    lv_obj_align(info, LV_ALIGN_TOP_LEFT, 10, 50);

    // YES
    lv_obj_t* btn_yes = lv_btn_create(confirm_dialog);
    lv_obj_set_size(btn_yes, 120, 50);
    lv_obj_align(btn_yes, LV_ALIGN_BOTTOM_LEFT, 30, 0);
    lv_obj_set_style_bg_color(btn_yes, lv_palette_main(LV_PALETTE_RED), 0);

    lv_obj_t* lbl_yes = lv_label_create(btn_yes);
    lv_label_set_text(lbl_yes, "YES");
    lv_obj_set_style_text_font(lbl_yes, &montserrat_24, 0);
    lv_obj_center(lbl_yes);

    // NO
    lv_obj_t* btn_no = lv_btn_create(confirm_dialog);
    lv_obj_set_size(btn_no, 120, 50);
    lv_obj_align(btn_no, LV_ALIGN_BOTTOM_RIGHT, -30, 0);

    lv_obj_t* lbl_no = lv_label_create(btn_no);
    lv_label_set_text(lbl_no, "NO");
    lv_obj_set_style_text_font(lbl_no, &montserrat_24, 0);
    lv_obj_center(lbl_no);

    // YES → really start
    lv_obj_add_event_cb(btn_yes, [](lv_event_t* e){
        lv_obj_t* btn = (lv_obj_t*)lv_event_get_user_data(e);
        lv_obj_t* label = lv_obj_get_child(btn, 0);

        // 先檢查 command
        if (!tmc_executable_planner_valid || !g_tmc_ramp_cmd.valid)
        {
            MotionModeManager::forceIdle("Invalid TMC ramp command");
            set_retract_switch_enabled(true);

            lv_obj_del(confirm_dialog);
            confirm_dialog = NULL;

            show_missing_param_dialog("Invalid TMC ramp command");
            return;
        }

        // 再啟動 motor
        if (!MotorControlScreen::startExtrusionMove(g_tmc_ramp_cmd))
        {
            MotionModeManager::forceIdle("Extrusion start failed");
            set_retract_switch_enabled(true);

            lv_obj_del(confirm_dialog);
            confirm_dialog = NULL;

            show_missing_param_dialog("Extrusion start failed");
            return;
        }

        // motor start 成功後，才進入 running state
        stop_requested = false;
        stop_requested_by_timeout = false;

        // 使用 START 時已鎖定的回抽模式
        retract_move_started = false;

        is_running = true;
        run_start_ms = millis();
        phase_start_ms = run_start_ms;
        run_phase = RUN_PHASE_EXTRUDING;

        system_progress_force_remaining_zero = false;
        system_progress_force_complete = false;
        system_progress_tail_ramp_active = false;
        system_progress_tail_start_percent = 0.0f;
        system_progress_tail_start_ms = 0;

        // Reset TMC distance monitor for the new extrusion run.
        tmc_distance_frozen_after_extrusion = false;
        last_tmc_target_distance_mm = 0.0f;
        last_tmc_actual_distance_mm = 0.0f;

        last_curve_update_ms = 0;

        set_tab3_control_buttons_enabled(false);
        set_retract_switch_enabled(false);

        // reset charts using executable motion range
        RealtimeCurveRenderer::Config curve_cfg;
        current_tail_recording = false;
        current_tail_start_ms = 0;

        chart_total_duration_sec =
            operation_motion_duration_sec_exact + CURRENT_TAIL_SEC;

        if (chart_total_duration_sec <= 0.0f) {
            chart_total_duration_sec =
                recorded_duration_sec_exact + CURRENT_TAIL_SEC;
        }

        curve_cfg.expected_duration_s = chart_total_duration_sec;
        curve_cfg.sample_interval_ms  = 100;   // one point every 100 ms
        curve_cfg.point_count         = 0;      // auto-calc
        curve_cfg.volume_max_uL       = realtime_curve_volume_max_uL * 1.1f; // chart Y axis range
        curve_cfg.current_max_A       = realtime_curve_current_max_A; // chart Y axis range

        if (curve_cfg.volume_max_uL < 1.0f) {
            curve_cfg.volume_max_uL = 1.0f;
        }
        if (curve_cfg.current_max_A < 0.1f) {
            curve_cfg.current_max_A = 0.1f;
        }

        RealtimeCurveRenderer::reset(curve_cfg);

        update_system_progress_data();
        show_countdown_dialog(display_duration_sec_exact, tab1);

        // ✅ 強制第一點
        if (tmc_executable_planner_valid && RealtimeCurveRenderer::isInitialized()) {
            TmcExecutableRampPlanner::Sample s0 =
                tmc_executable_planner.evaluate(0.0f);

            RealtimeCurveRenderer::pushSample(
                0.0f,
                s0.volume_uL,
                MotorControlScreen::getMeasuredCurrentA()
            );
        }

        lv_label_set_text(label, "STOP");
        lv_obj_set_style_bg_color(btn, lv_color_hex(0xFF0000), 0);

        lv_obj_del(confirm_dialog);
        confirm_dialog = NULL;
    }, LV_EVENT_CLICKED, start_btn);

    // NO → cancel
    lv_obj_add_event_cb(btn_no, [](lv_event_t*){
        MotionModeManager::forceIdle("User cancelled start");

        realtime_motion_planner_valid = false;
        tmc_executable_planner_valid = false;
        g_tmc_ramp_cmd = TmcRampCommand();

        recorded_duration_valid = false;

        set_retract_switch_enabled(true);

        lv_obj_del(confirm_dialog);
        confirm_dialog = NULL;
    }, LV_EVENT_CLICKED, NULL);
}

void show_missing_param_dialog(const char* msg)
{
    if (confirm_dialog) return;

    confirm_dialog = lv_obj_create(lv_scr_act());
    lv_obj_set_size(confirm_dialog, 500, 200);
    lv_obj_center(confirm_dialog);
    lv_obj_set_style_radius(confirm_dialog, 12, 0);
    lv_obj_set_style_bg_color(confirm_dialog, lv_palette_lighten(LV_PALETTE_GREY, 3), 0);
    lv_obj_set_style_pad_all(confirm_dialog, 20, 0);

    lv_obj_t* label = lv_label_create(confirm_dialog);
    lv_label_set_text(label, msg);
    lv_obj_set_style_text_font(label, &montserrat_34, 0);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 20);

    lv_obj_t* btn_ok = lv_btn_create(confirm_dialog);
    lv_obj_set_size(btn_ok, 120, 50);
    lv_obj_align(btn_ok, LV_ALIGN_BOTTOM_MID, 0, 0);

    lv_obj_t* lbl_ok = lv_label_create(btn_ok);
    lv_label_set_text(lbl_ok, "OK");
    lv_obj_set_style_text_font(lbl_ok, &montserrat_24, 0);
    lv_obj_center(lbl_ok);

    lv_obj_add_event_cb(btn_ok, [](lv_event_t*){
        lv_obj_del(confirm_dialog);
        confirm_dialog = NULL;
    }, LV_EVENT_CLICKED, NULL);
}

void show_countdown_dialog(float duration_sec, lv_obj_t* parent)
{
    if (countdown_dialog) return;

    countdown_dialog = lv_obj_create(parent);

    lv_obj_set_size(countdown_dialog, 400, 200);
    lv_obj_center(countdown_dialog);
    lv_obj_set_style_radius(countdown_dialog, 12, 0);
    lv_obj_set_style_bg_color(
        countdown_dialog,
        lv_palette_lighten(LV_PALETTE_GREY, 3),
        0
    );
    lv_obj_set_style_pad_all(countdown_dialog, 20, 0);

    // title
    lv_obj_t* title = lv_label_create(countdown_dialog);
    lv_label_set_text(title, "Remaining Time");
    lv_obj_set_style_text_font(title, &montserrat_34, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    // countdown value
    countdown_label = lv_label_create(countdown_dialog);
    char duration_buf[32];
    format_seconds_2dp(duration_buf, sizeof(duration_buf), duration_sec);

    lv_label_set_text_fmt(countdown_label, "%s s", duration_buf);
    lv_obj_set_style_text_font(countdown_label, &montserrat_48, 0);
    lv_obj_align(countdown_label, LV_ALIGN_CENTER, 0, 20);

    countdown_ok_btn = NULL;
    last_countdown_update = millis();
}

void show_finish_dialog()
{
    if (finish_dialog) return;

    finish_dialog = lv_obj_create(lv_scr_act());
    lv_obj_set_size(finish_dialog, 420, 220);
    lv_obj_center(finish_dialog);
    lv_obj_set_style_radius(finish_dialog, 12, 0);
    lv_obj_set_style_bg_color(
        finish_dialog,
        lv_palette_lighten(LV_PALETTE_GREY, 3),
        0
    );
    lv_obj_set_style_pad_all(finish_dialog, 20, 0);

    lv_obj_t* label = lv_label_create(finish_dialog);
    lv_label_set_text(label, "Finished");
    lv_obj_set_style_text_font(label, &montserrat_48, 0);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -20);

    lv_obj_t* btn_ok = lv_btn_create(finish_dialog);
    lv_obj_set_size(btn_ok, 120, 50);
    lv_obj_align(btn_ok, LV_ALIGN_BOTTOM_MID, 0, 0);

    lv_obj_t* lbl = lv_label_create(btn_ok);
    lv_label_set_text(lbl, "OK");
    lv_obj_set_style_text_font(lbl, &montserrat_24, 0);
    lv_obj_center(lbl);

    lv_obj_add_event_cb(btn_ok, [](lv_event_t*) {
        lv_obj_del(finish_dialog);
        finish_dialog = NULL;
    }, LV_EVENT_CLICKED, NULL);
}

void countdown_finish()
{
    if (!countdown_dialog) return;

    lv_label_set_text(countdown_label, "Finished");
    lv_obj_set_style_text_font(countdown_label, &montserrat_40, 0);
    lv_obj_align(countdown_label, LV_ALIGN_CENTER, 0, 0);

    countdown_ok_btn = lv_btn_create(countdown_dialog);
    lv_obj_set_size(countdown_ok_btn, 120, 50);
    lv_obj_align(countdown_ok_btn, LV_ALIGN_BOTTOM_MID, 0, 0);

    lv_obj_t* lbl = lv_label_create(countdown_ok_btn);
    lv_label_set_text(lbl, "OK");
    lv_obj_set_style_text_font(lbl, &montserrat_24, 0);
    lv_obj_center(lbl);

    lv_obj_add_event_cb(countdown_ok_btn, [](lv_event_t*){
        lv_obj_del(countdown_dialog);
        countdown_dialog = NULL;
        countdown_label  = NULL;
        countdown_ok_btn = NULL;
    }, LV_EVENT_CLICKED, NULL);
}

void clear_countdown_dialog()
{
    if (countdown_dialog) {
        lv_obj_del(countdown_dialog);
        countdown_dialog = NULL;
        countdown_label  = NULL;
        countdown_ok_btn = NULL;
    }
}

// --------------------------------------


// --- Setting tab input block ---
// numerice keypad even
static void num_kb_event_cb(lv_event_t * e)
{
    lv_obj_t * kb = lv_event_get_target(e);
    lv_obj_t * ta = (lv_obj_t *)lv_event_get_user_data(e);
    input_block_t * blk = (input_block_t *)lv_obj_get_user_data(ta);

    const char * txt = lv_btnmatrix_get_btn_text(kb, lv_btnmatrix_get_selected_btn(kb));
    if(!txt || !blk) return;

    const keypad_config_t* cfg = blk->keypad_cfg;

    // BACKSPACE
    if(strcmp(txt, LV_SYMBOL_BACKSPACE) == 0) {
        lv_textarea_del_char(ta);
        return;
    }

    // ENTER
    if(strcmp(txt, LV_SYMBOL_NEW_LINE) == 0) {
        const char * cur = lv_textarea_get_text(ta);

        if(strlen(cur) == 0) {
            lv_label_set_text(blk->display_label, "VALUE");
            lv_obj_set_style_text_color(blk->display_label, lv_color_hex(0xA0A0A0), 0);
        }
        else {
            lv_label_set_text(blk->display_label, cur);
            lv_obj_set_style_text_color(blk->display_label, lv_color_hex(0x000000), 0);
        }

        // only hide this keyboard
        lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    // filter decimal
    if (strcmp(txt, ".") == 0) {
        if (!cfg || !cfg->allow_decimal) return;

        const char* cur = lv_textarea_get_text(ta);
        if (strchr(cur, '.') != NULL) return; // only one decimal point
    }

    // filter "00"
    if (strcmp(txt, "00") == 0) {
        if (!cfg || !cfg->allow_double_zero) return;
    }

    // max length limit
    if (cfg && cfg->max_chars > 0) {
        const char* cur = lv_textarea_get_text(ta);
        size_t cur_len = strlen(cur);
        size_t add_len = strlen(txt);

        if ((cur_len + add_len) > cfg->max_chars) {
            return;
        }
    }

    // normal input
    lv_textarea_add_text(ta, txt);
}

input_block_t* lv_create_input_block(
    lv_obj_t* parent,
    const char* title,
    bool has_options,
    lv_coord_t x,
    lv_coord_t y,
    const keypad_config_t* keypad_cfg
){
  // 改用靜態 pool，避免 malloc
  if (input_block_count >= (sizeof(input_block_pool) / sizeof(input_block_pool[0]))) {
      return NULL;
  }

  input_block_t* block = &input_block_pool[input_block_count++];
  memset(block, 0, sizeof(input_block_t));

  block->has_options = has_options;
  block->options_btn = NULL;
  block->list = NULL;
  block->keyboard = NULL;
  block->keypad_cfg = keypad_cfg;

  // create container (wrap the entire UI Block)
  lv_obj_t* cont = lv_obj_create(parent);
  lv_obj_set_size(cont, 380, 280);
  lv_obj_set_style_radius(cont, 10, 0);
  lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE); // avoid scrollable
  lv_obj_align(cont, LV_ALIGN_TOP_LEFT, x, y);

  // create label title
  lv_obj_t* title_label = lv_label_create(parent);
  lv_label_set_text(title_label, title);
  lv_obj_set_style_text_font(title_label, &montserrat_30, 0);
  lv_obj_align_to(title_label, cont, LV_ALIGN_TOP_MID, 0, 0);

  // create value display box
  lv_obj_t* box = lv_obj_create(cont);
  lv_obj_set_size(box, 320, 80);
  lv_obj_set_style_radius(box, 10, 0);
  lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_align_to(box, title_label, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
  
  // display content
  lv_obj_t* value_label = lv_label_create(box);
  lv_label_set_text(value_label, "VALUE");
  lv_obj_set_style_text_font(value_label, &montserrat_32, 0);
  lv_obj_set_style_text_color(value_label, lv_color_hex(0xA0A0A0), 0);
  lv_obj_center(value_label);
  block->display_label = value_label;

  // value
  lv_obj_t* ta = lv_textarea_create(cont);
  lv_obj_set_size(ta, 200, 48);
  lv_textarea_set_placeholder_text(ta, "Enter");
  lv_obj_clear_flag(ta, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_text_font(ta, &montserrat_20, 0);
  lv_obj_align_to(ta, box, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 15);
  block->textarea = ta;
  lv_obj_set_user_data(ta, block);
  
  // create numeric keypad button
  lv_obj_t* btn_kb = lv_btn_create(cont);
  lv_obj_set_size(btn_kb, 60, 40);
  lv_obj_align_to(btn_kb, box, LV_ALIGN_OUT_BOTTOM_LEFT, 210,15);
  // show numeric keypad icon
  lv_obj_t * kb_lbl = lv_label_create(btn_kb);
  lv_label_set_text(kb_lbl, LV_SYMBOL_KEYBOARD);
  lv_obj_align_to(kb_lbl, btn_kb, LV_ALIGN_RIGHT_MID, 0, 0);
  // create numeric keypad (btnmatrix)
  lv_obj_t * num_kb = lv_btnmatrix_create(parent);
  lv_obj_set_size(num_kb, 360, 280);

  if (keypad_cfg) {
      lv_obj_align(num_kb, keypad_cfg->align, keypad_cfg->x_ofs, keypad_cfg->y_ofs);
      lv_btnmatrix_set_map(num_kb, keypad_cfg->keymap);
  } else {
    lv_obj_align(num_kb, LV_ALIGN_BOTTOM_MID, 0, 13);
  }

  lv_obj_move_foreground(num_kb);
  lv_obj_add_flag(num_kb, LV_OBJ_FLAG_HIDDEN);
  lv_obj_clear_flag(num_kb, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_event_cb(num_kb, num_kb_event_cb, LV_EVENT_VALUE_CHANGED, ta);

  block->keyboard = num_kb;
  // numeric keypad button trigger
  // [捕捉列表](參數){ 函式本體 }
  // lv_obj_add_event_cb(target_obj, callback_func, event_type, user_data);
  // 這段程式碼在 btn_kb 物件上註冊了一個事件 callback，當 LV_EVENT_CLICKED 發生時被觸發。 
  // callback 本身是一個 lambda 函式，接收 LVGL 傳入的事件參數 e。 
  // keyboard 物件透過 user_data 傳入事件中，並在 callback 內使用 lv_event_get_user_data(e) 取得。 
  // 最後 callback 會清除 keyboard 的 LV_OBJ_FLAG_HIDDEN 旗標，使鍵盤顯示出來。
  lv_obj_add_event_cb(btn_kb, [](lv_event_t* e){
      input_block_t* blk = (input_block_t*)lv_event_get_user_data(e);
      if (!blk || !blk->keyboard) return;

      // show only this block's keypad
      lv_obj_clear_flag(blk->keyboard, LV_OBJ_FLAG_HIDDEN);
      lv_obj_move_foreground(blk->keyboard); // make keypad always on top of layers
  }, LV_EVENT_CLICKED, block);

  // options list event
  // bool value to check if have options
  if(!has_options){
    return block;
  }
  // create options button
  lv_obj_t* opt_btn = lv_btn_create(cont);
  lv_obj_set_size(opt_btn, 270, 48);
  lv_obj_align_to(opt_btn, ta, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);
  block->options_btn = opt_btn;
  
  lv_obj_t* opt_lbl = lv_label_create(opt_btn);
  lv_label_set_text(opt_lbl, "Options List");
  lv_obj_set_style_text_font(opt_lbl, &montserrat_20, 0);
  lv_obj_center(opt_lbl);
  // create options list
  lv_obj_t* list = lv_list_create(parent);
  lv_obj_set_size(list, 350, 200);
  lv_obj_add_flag(list, LV_OBJ_FLAG_HIDDEN);
  //lv_obj_set_style_text_font(list, &montserrat_18, 0);
  lv_obj_align_to(list, opt_btn, LV_ALIGN_TOP_MID, 320, -80);
  block->list = list;
  
  lv_obj_add_event_cb(opt_btn, [](lv_event_t* e){
    input_block_t* blk = (input_block_t*)lv_event_get_user_data(e);
    if (lv_obj_has_flag(blk->list, LV_OBJ_FLAG_HIDDEN))
        lv_obj_clear_flag(blk->list, LV_OBJ_FLAG_HIDDEN);
    else
        lv_obj_add_flag(blk->list, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(blk->list);
  }, LV_EVENT_CLICKED, block);
  return block;
}
// --------------------------------------

static void hide_setting_keypads()
{
    if (viscosity_block && viscosity_block->keyboard) {
        lv_obj_add_flag(viscosity_block->keyboard, LV_OBJ_FLAG_HIDDEN);
    }

    if (quantity_block && quantity_block->keyboard) {
        lv_obj_add_flag(quantity_block->keyboard, LV_OBJ_FLAG_HIDDEN);
    }
}

// --- Add options item ---
void lv_add_option_item(
    input_block_t* block,
    const char* title,
    const char* value)
{
    if (!block->has_options || block->list == NULL) return;

    lv_obj_t* btn = lv_list_add_btn(block->list, LV_SYMBOL_RIGHT, title);
    // get the child of btn and only change text size
    lv_obj_t* icno_label = lv_obj_get_child(btn, 0);
    lv_obj_t* text_label = lv_obj_get_child(btn, 1);

    lv_obj_set_style_text_font(text_label, &montserrat_20, 0);

    lv_obj_add_event_cb(btn, [](lv_event_t* e){
        lv_obj_t* item = lv_event_get_target(e);
        const char* val = (const char*)lv_event_get_user_data(e);
        input_block_t* blk = (input_block_t*)lv_obj_get_user_data(item);

        lv_label_set_text(blk->display_label, val);
        lv_obj_set_style_text_color(blk->display_label, lv_color_hex(0x0000), 0);
        lv_obj_add_flag(blk->list, LV_OBJ_FLAG_HIDDEN);

    }, LV_EVENT_CLICKED, (void*)value);

    lv_obj_set_user_data(btn, block);
}
// --------------------------------------

// --- System Progress UI ---
void create_system_progress_ui(lv_obj_t* parent)
{
    /* ===== ARC Progress ===== */
    arc_progress = lv_arc_create(parent);
    lv_obj_set_size(arc_progress, 250, 250);
    lv_arc_set_rotation(arc_progress, 0);
    lv_arc_set_angles(arc_progress, 0, 360);
    lv_arc_set_bg_angles(arc_progress, 0, 360);
    lv_arc_set_range(arc_progress, 0, 100);
    lv_arc_set_value(arc_progress, 0);
    /* ✅ 關鍵：關閉 indicator 動畫 */
    lv_obj_set_style_anim_time(arc_progress, 0, LV_PART_INDICATOR);
    /* 禁止互動 */
    lv_obj_clear_flag(arc_progress, LV_OBJ_FLAG_CLICKABLE);
    //lv_obj_remove_style(arc_progress, NULL, LV_PART_KNOB); // 不要旋鈕
    lv_obj_align(arc_progress, LV_ALIGN_LEFT_MID, 10, 0);

    lbl_percent = lv_label_create(parent);
    lv_label_set_text(lbl_percent, "0.0%");
    lv_obj_set_style_text_font(lbl_percent, &montserrat_48, 0);
    lv_obj_align_to(lbl_percent, arc_progress, LV_ALIGN_CENTER, 5, 0);

    /* ===== Syringe Icon (暫用符號) ===== */
    // lv_obj_t* syringe = lv_label_create(parent);
    // lv_label_set_text(syringe, LV_SYMBOL_CHARGE); // 之後可換成圖片
    // lv_obj_set_style_text_font(syringe, &montserrat_48, 0);
    // lv_obj_align_to(syringe, arc_progress, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    /* ===== Info Text ===== */
    lv_coord_t x =300;
    lv_coord_t y = 10;

    lbl_remaining = lv_label_create(parent);
    lv_label_set_text(lbl_remaining, "Remaining Time (s): ");
    lv_obj_set_style_text_font(lbl_remaining, &montserrat_40, 0);
    lv_obj_set_pos(lbl_remaining, x, y);
    //lv_obj_align_to(lbl_remaining, lbl_quantity, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 20);

    lbl_viscosity = lv_label_create(parent);
    lv_label_set_text(lbl_viscosity, "Viscosity (mPa.s): ");
    lv_obj_set_style_text_font(lbl_viscosity, &montserrat_26, 0);
    lv_obj_align_to(lbl_viscosity, lbl_remaining, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 55);
    //lv_obj_set_pos(lbl_viscosity, x, y);

    lbl_quantity = lv_label_create(parent);
    lv_label_set_text(lbl_quantity, "Quantity (uL): ");
    lv_obj_set_style_text_font(lbl_quantity, &montserrat_26, 0);
    lv_obj_align_to(lbl_quantity, lbl_viscosity, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 15);

    lbl_duration = lv_label_create(parent);
    lv_label_set_text(lbl_duration, "Duration (s): ");
    lv_obj_set_style_text_font(lbl_duration, &montserrat_26, 0);
    lv_obj_align_to(lbl_duration, lbl_quantity, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 15);

    lbl_elapsed = lv_label_create(parent);
    lv_label_set_text(lbl_elapsed, "Elapsed Time (s): ");
    lv_obj_set_style_text_font(lbl_elapsed, &montserrat_26, 0);
    lv_obj_align_to(lbl_elapsed, lbl_duration, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 15);

    lbl_tmc_target_distance = lv_label_create(parent);
    lv_label_set_text(lbl_tmc_target_distance, "Target Distance (mm): 0.00");
    lv_obj_set_style_text_font(lbl_tmc_target_distance, &montserrat_26, 0);
    lv_obj_align_to(lbl_tmc_target_distance, lbl_elapsed, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 15);

    lbl_tmc_actual_distance = lv_label_create(parent);
    lv_label_set_text(lbl_tmc_actual_distance, "Actual Distance (mm): 0.00");
    lv_obj_set_style_text_font(lbl_tmc_actual_distance, &montserrat_26, 0);
    lv_obj_align_to(lbl_tmc_actual_distance, lbl_tmc_target_distance, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 15);

}

void create_tab3_curve_preview_blocks(lv_obj_t* parent)
{
    // ===== Layout idea =====
    // top-right area keeps the two navigation buttons on the far right
    // two preview panels are placed to the left of those buttons

    const lv_coord_t panel_w = 325;
    const lv_coord_t panel_h = 230;
    const lv_coord_t top_y   = 10;

    // ---------- Volume-t panel ----------
    tab3_volume_curve_panel = lv_obj_create(parent);
    lv_obj_set_size(tab3_volume_curve_panel, panel_w, panel_h);
    lv_obj_align(tab3_volume_curve_panel, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_clear_flag(tab3_volume_curve_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(tab3_volume_curve_panel, 10, 0);
    lv_obj_set_style_pad_all(tab3_volume_curve_panel, 10, 0);

    lv_obj_t* lbl_volume_title = lv_label_create(tab3_volume_curve_panel);
    lv_label_set_text(lbl_volume_title, "Volume (mL)");
    lv_obj_set_style_text_font(lbl_volume_title, &montserrat_18, 0);
    lv_obj_align(lbl_volume_title, LV_ALIGN_TOP_LEFT, 0, -5);

    // lv_obj_t* lbl_volume_placeholder = lv_label_create(tab3_volume_curve_panel);
    // lv_label_set_text(lbl_volume_placeholder, "Reserved for future curve display");
    // lv_obj_set_style_text_font(lbl_volume_placeholder, &montserrat_18, 0);
    // lv_obj_set_style_text_color(lbl_volume_placeholder, lv_color_hex(0x808080), 0);
    // lv_obj_align(lbl_volume_placeholder, LV_ALIGN_CENTER, 0, 10);

    // ---------- Current-t panel ----------
    tab3_current_curve_panel = lv_obj_create(parent);
    lv_obj_set_size(tab3_current_curve_panel, panel_w, panel_h);
    lv_obj_align_to(tab3_current_curve_panel, tab3_volume_curve_panel, LV_ALIGN_OUT_RIGHT_TOP, 10, 0);
    lv_obj_clear_flag(tab3_current_curve_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(tab3_current_curve_panel, 10, 0);
    lv_obj_set_style_pad_all(tab3_current_curve_panel, 10, 0);

    lv_obj_t* lbl_current_title = lv_label_create(tab3_current_curve_panel);
    lv_label_set_text(lbl_current_title, "Current (A)");
    lv_obj_set_style_text_font(lbl_current_title, &montserrat_18, 0);
    lv_obj_align(lbl_current_title, LV_ALIGN_TOP_LEFT, 0, -5);

    // lv_obj_t* lbl_current_placeholder = lv_label_create(tab3_current_curve_panel);
    // lv_label_set_text(lbl_current_placeholder, "Reserved for future curve display");
    // lv_obj_set_style_text_font(lbl_current_placeholder, &montserrat_18, 0);
    // lv_obj_set_style_text_color(lbl_current_placeholder, lv_color_hex(0x808080), 0);
    // lv_obj_align(lbl_current_placeholder, LV_ALIGN_CENTER, 0, 10);
}

static void build_manual_screen_tab1()
{
    if (manual_screen_tab1) return;

    manual_screen_tab1 = lv_obj_create(NULL);
    //lv_obj_clear_flag(manual_screen_tab1, LV_OBJ_FLAG_SCROLLABLE);

    // Title
    lv_obj_t* title = lv_label_create(manual_screen_tab1);
    lv_label_set_text(title, "User Manual for SETTING");
    lv_obj_set_style_text_font(title, &montserrat_40, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

    // Text content
    lv_obj_t* text = lv_label_create(manual_screen_tab1);
    lv_label_set_text(
        text,
        "Purpose:\n"
        "Set viscosity and extrusion quantity before starting.\n\n"

        "Input Fields:\n"
        "Viscosity(mPa.s) - Select from list or enter manually\n"
        "Quantity(uL) - Enter using keypad\n\n"

        "Steps:\n"
        "1. Set viscosity\n"
        "2. Enter quantity\n"
        "3. Press START\n"
        "4. Confirm dialog -> YES\n\n"
        "System Behavior:\n"
        "- Show confirmation dialog\n"
        "- Displays predicted duration\n"
        "- Countdown starts\n"
        "- START -> STOP(Used to stop the operation)\n\n"

        "Notes:\n"
        "Both values required\n"
        "Displayed values = used values\n\n"

        "Warnings:\n"
        "- Missing viscosity -> blocked\n"
        "- Missing quantity -> blocked\n\n"

        "Related:\n"
        "SYSTEM PROGRESS -> runtime info\n"
        "DEVELOPER INFO -> curve preview\n"
    );
    lv_obj_set_style_text_font(text, &montserrat_24, 0);
    lv_label_set_long_mode(text, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(text, 700);
    lv_obj_align(text, LV_ALIGN_TOP_LEFT, 20, 100);

    // BACK button
    lv_obj_t* btn_back = lv_btn_create(manual_screen_tab1);
    lv_obj_set_size(btn_back, 120, 60);
    lv_obj_set_style_bg_color(btn_back, lv_palette_main(LV_PALETTE_GREY), LV_PART_MAIN);
    lv_obj_align(btn_back, LV_ALIGN_TOP_RIGHT, -20, 100);

    lv_obj_t* lbl_back = lv_label_create(btn_back);
    lv_label_set_text(lbl_back, "BACK");
    lv_obj_set_style_text_font(lbl_back, &montserrat_24, 0);
    lv_obj_center(lbl_back);

    lv_obj_add_event_cb(btn_back, [](lv_event_t*){
        if (main_screen) {
            lv_scr_load(main_screen);
        }
        lv_async_call(destroy_manual_screen_tab1_async, NULL);
    }, LV_EVENT_CLICKED, NULL);
}

static void build_manual_screen_tab2()
{
    if (manual_screen_tab2) return;

    manual_screen_tab2 = lv_obj_create(NULL);
    //lv_obj_clear_flag(manual_screen_tab2, LV_OBJ_FLAG_SCROLLABLE);

    // Title
    lv_obj_t* title = lv_label_create(manual_screen_tab2);
    lv_label_set_text(title, "User Manual for SYSTEM PROGRESS");
    lv_obj_set_style_text_font(title, &montserrat_40, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

    // Text content
    lv_obj_t* text = lv_label_create(manual_screen_tab2);
    lv_label_set_text(
        text,
        "Purpose:\n"
        "Monitor real-time system status during operation.\n\n"

        "Main Display:\n"
        "Progress Circle - Shows completion(%)\n"
        "Remaining Time - Time left\n"
        "Elapsed Time - Time passed\n\n"

        "Parameters Shown:\n"
        "Viscosity(mPa.s)\n"
        "Quantity(uL)\n"
        "Duration(s)\n\n"

        "How it works:\n"
        "- Updates automatically during run\n"
        "- Progress follows operation time\n"
        "- Values based on SETTING inputs\n\n"

        "System Behavior:\n"
        "- Progress increases from 0% -> 100%\n"
        "- Remaining time decreases continuously\n"
        "- Stops updating when operation finishes\n\n"

        "Notes:\n"
        "Display is read-only\n"
        "Duration is a predicted value\n\n"

        "Warnings:\n"
        "- No update if system is not running\n"
        "- Values remain static after completion\n\n"

        "Related:\n"
        "SETTING -> defines input values\n"
        "DEVELOPER INFO -> curve data\n"
    );
    lv_obj_set_style_text_font(text, &montserrat_24, 0);
    lv_label_set_long_mode(text, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(text, 700);
    lv_obj_align(text, LV_ALIGN_TOP_LEFT, 20, 100);

    // BACK button
    lv_obj_t* btn_back = lv_btn_create(manual_screen_tab2);
    lv_obj_set_size(btn_back, 120, 60);
    lv_obj_set_style_bg_color(btn_back, lv_palette_main(LV_PALETTE_GREY), LV_PART_MAIN);
    lv_obj_align(btn_back, LV_ALIGN_TOP_RIGHT, -20, 100);

    lv_obj_t* lbl_back = lv_label_create(btn_back);
    lv_label_set_text(lbl_back, "BACK");
    lv_obj_set_style_text_font(lbl_back, &montserrat_24, 0);
    lv_obj_center(lbl_back);

    lv_obj_add_event_cb(btn_back, [](lv_event_t*){
        if (main_screen) {
            lv_scr_load(main_screen);
        }
        lv_async_call(destroy_manual_screen_tab2_async, NULL);
    }, LV_EVENT_CLICKED, NULL);
}

static void build_manual_screen_tab3()
{
    if (manual_screen_tab3) return;

    manual_screen_tab3 = lv_obj_create(NULL);
    //lv_obj_clear_flag(manual_screen_tab3, LV_OBJ_FLAG_SCROLLABLE);

    // Title
    lv_obj_t* title = lv_label_create(manual_screen_tab3);
    lv_label_set_text(title, "User Manual for DEVELOPER INFO");
    lv_obj_set_style_text_font(title, &montserrat_40, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

    // Text content
    lv_obj_t* text = lv_label_create(manual_screen_tab3);
    lv_label_set_text(
        text,
        "Purpose:\n"
        "View system data and access advanced controls.\n\n"

        "Main Display:\n"
        "Volume Curve - Predicted volume vs time\n"
        "Current Curve - Measured current vs time\n"
        "Slider Position - Estimated slider position\n\n"

        "Functions:\n"
        "Customized Parameters(Top Right):\n"
        "- Open parameter settings page\n"
        "- Adjust system parameters\n\n"
        
        "Motor Control(SPI/TMC5160)(Right Middle):\n"
        "- Open motor control interface\n"
        "- Control motor and motion settings\n"
        "- Perform SPI-based motor control test\n\n"

        "Homing(Bottom Right):\n"
        "- Press button to start homing\n"
        "- Confirm dialog required before execution\n\n"

        "How it works:\n"
        "- Volume curve is based on prediction\n"
        "- Current curve uses real sensor data\n"
        "- Slider position is system estimation\n"
        "- Customized Parameters adapts system to hardware setup\n"
        "- Motor Control enables SPI-based motor control testing\n"
        "- Homing resets slider reference position\n\n"

        "System Behavior:\n"
        "- During system running (extrusion), function buttons are disabled\n"
        "- START button changes to STOP during operation\n"
        "- Real-time curves and system data update continuously\n"
        "- During homing, all functions are disabled (system locked)\n"
        "- System shows 'Homing in progress'\n"
        "- Normal operation resumes after homing is completed\n\n"

        "Notes:\n"
        "Volume is predicted, current is measured\n"
        "Slider position may differ from actual\n"
        "Use homing if slider position deviates significantly from actual position\n"
        "Update parameters if syringe specification changes\n"
        "Press SAVE after modifying parameters\n"
        "No Arduino reprogramming required for parameter update\n"
        "Motor Control default is for Joy-iT NEMA23-03 with TMC5160A-TA\n"
        "Recalibration required if motor or driver is changed\n"
        "SPI parameter changes require Arduino reprogramming\n\n"

        "Warnings:\n"
        "- Customized Parameters, Motor Control and Homing are disabled during system running(extrusion)\n"
        "- All functions are disabled during homing process(system locked)\n\n"

        "Related:\n"
        "SETTING -> input parameters\n"
        "SYSTEM PROGRESS -> runtime status\n"
    );
    lv_obj_set_style_text_font(text, &montserrat_24, 0);
    lv_label_set_long_mode(text, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(text, 700);
    lv_obj_align(text, LV_ALIGN_TOP_LEFT, 20, 100);

    // BACK button
    lv_obj_t* btn_back = lv_btn_create(manual_screen_tab3);
    lv_obj_set_size(btn_back, 120, 60);
    lv_obj_set_style_bg_color(btn_back, lv_palette_main(LV_PALETTE_GREY), LV_PART_MAIN);
    lv_obj_align(btn_back, LV_ALIGN_TOP_RIGHT, -20, 100);

    lv_obj_t* lbl_back = lv_label_create(btn_back);
    lv_label_set_text(lbl_back, "BACK");
    lv_obj_set_style_text_font(lbl_back, &montserrat_24, 0);
    lv_obj_center(lbl_back);

    lv_obj_add_event_cb(btn_back, [](lv_event_t*){
        if (main_screen) {
            lv_scr_load(main_screen);
        }
        lv_async_call(destroy_manual_screen_tab3_async, NULL);
    }, LV_EVENT_CLICKED, NULL);
}
// 在 DEVELOPER INFO(tab3) 建一顆按鈕，用來跳轉
static void nav_button_event_cb(lv_event_t* e)
{
    lv_obj_t* btn = lv_event_get_target(e);
    nav_btn_ctx_t* ctx = (nav_btn_ctx_t*)lv_obj_get_user_data(btn);
    if (!ctx) return;

    if (ctx->target_screen && *(ctx->target_screen) == NULL) {
        if (ctx->screen_builder) {
            ctx->screen_builder();
        }
    }

    if (ctx->target_screen && *(ctx->target_screen)) {
        lv_scr_load(*(ctx->target_screen));
    }
}

static void apply_alpha_icon_style(lv_obj_t* img, lv_color_t color)
{
    if (!img) return;

    lv_obj_set_style_img_recolor(img, color, LV_PART_MAIN);
    lv_obj_set_style_img_recolor_opa(img, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_img_opa(img, LV_OPA_COVER, LV_PART_MAIN);
}

static lv_obj_t* create_nav_button(
    lv_obj_t* parent,
    lv_coord_t btn_w,
    lv_coord_t btn_h,
    lv_align_t align,
    lv_coord_t x_ofs,
    lv_coord_t y_ofs,
    const void* img_src,
    uint16_t img_zoom,
    lv_obj_t** target_screen,
    void (*screen_builder)()
)
{
    lv_obj_t* btn = lv_btn_create(parent);
    lv_obj_set_size(btn, btn_w, btn_h);
    lv_obj_align(btn, align, x_ofs, y_ofs);

    lv_obj_t* img = lv_img_create(btn);
    lv_img_set_src(img, img_src);
    const lv_img_dsc_t* dsc = (const lv_img_dsc_t*)img_src;
    lv_img_set_pivot(img, dsc->header.w / 2, dsc->header.h / 2);
    lv_img_set_zoom(img, img_zoom);

    apply_alpha_icon_style(img, lv_color_black());

    lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);
    lv_obj_clear_flag(img, LV_OBJ_FLAG_CLICKABLE);

    // 不再 malloc，改用靜態 pool
    if (nav_btn_ctx_count < (sizeof(nav_btn_ctx_pool) / sizeof(nav_btn_ctx_pool[0]))) {
        nav_btn_ctx_pool[nav_btn_ctx_count].target_screen = target_screen;
        nav_btn_ctx_pool[nav_btn_ctx_count].screen_builder = screen_builder;

        lv_obj_set_user_data(btn, &nav_btn_ctx_pool[nav_btn_ctx_count]);
        nav_btn_ctx_count++;
    } else {
        lv_obj_set_user_data(btn, NULL);
    }

    lv_obj_add_event_cb(btn, nav_button_event_cb, LV_EVENT_CLICKED, NULL);

    return btn;
}

static void disable_motor_button_event_cb(lv_event_t* e)
{
    LV_UNUSED(e);

    if (is_running || stop_requested) {
        show_missing_param_dialog("Cannot disable motor while running");
        return;
    }

    MotorControlScreen::disableMotor();
}

static lv_obj_t* create_disable_motor_button(
    lv_obj_t* parent,
    lv_coord_t btn_w,
    lv_coord_t btn_h,
    lv_align_t align,
    lv_coord_t x_ofs,
    lv_coord_t y_ofs,
    const void* img_src,
    uint16_t img_zoom
)
{
    lv_obj_t* btn = lv_btn_create(parent);
    lv_obj_set_size(btn, btn_w, btn_h);
    lv_obj_align(btn, align, x_ofs, y_ofs);

    lv_obj_set_style_bg_color(btn, lv_color_hex(0x80C000), 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
    lv_obj_set_style_border_opa(btn, LV_OPA_COVER, 0);

    lv_obj_t* img = lv_img_create(btn);
    lv_img_set_src(img, img_src);
    const lv_img_dsc_t* dsc = (const lv_img_dsc_t*)img_src;
    lv_img_set_pivot(img, dsc->header.w / 2, dsc->header.h / 2);
    lv_img_set_zoom(img, img_zoom);

    apply_alpha_icon_style(img, lv_color_black());

    lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);
    lv_obj_clear_flag(img, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_add_event_cb(
        btn,
        disable_motor_button_event_cb,
        LV_EVENT_CLICKED,
        NULL
    );

    return btn;
}

static void set_tab3_control_buttons_enabled(bool enabled)
{
    lv_obj_t* buttons[] = {
        btn_tab3_parameter_setting,
        btn_tab3_motor_control,
        btn_tab3_disable_motor
    };

    // sizeof(buttons) 取得是bytes of array not number of elements
    const int button_count = sizeof(buttons) / sizeof(buttons[0]);

    for (int i = 0; i < button_count; i++) {
        lv_obj_t* btn = buttons[i];
        if (!btn) continue;

        if (enabled) {
            lv_obj_clear_state(btn, LV_STATE_DISABLED);
            lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);

            lv_obj_set_style_bg_color(btn, lv_color_hex(0X80C000), 0);  // ✅ restore color
            lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
            lv_obj_set_style_border_opa(btn, LV_OPA_COVER, 0);

            lv_obj_t* child = lv_obj_get_child(btn, 0);
            if (child) {
                lv_obj_set_style_img_opa(child, LV_OPA_COVER, 0);
                lv_obj_set_style_text_opa(child, LV_OPA_COVER, 0);
            }
        }
        else {
            lv_obj_add_state(btn, LV_STATE_DISABLED);
            lv_obj_clear_flag(btn, LV_OBJ_FLAG_CLICKABLE);

            lv_obj_set_style_bg_color(btn, lv_color_hex(0xC8C8C8), 0);
            lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
            lv_obj_set_style_border_opa(btn, LV_OPA_COVER, 0);

            lv_obj_t* child = lv_obj_get_child(btn, 0);
            if (child) {
                lv_obj_set_style_img_opa(child, LV_OPA_50, 0);
                lv_obj_set_style_text_opa(child, LV_OPA_50, 0);
            }
        }
    }
}

static void set_retract_switch_enabled(bool enabled)
{
    if (!retract_switch) return;

    if (enabled) {
        lv_obj_clear_state(retract_switch, LV_STATE_DISABLED);
        lv_obj_add_flag(retract_switch, LV_OBJ_FLAG_CLICKABLE);

        if (retract_switch_label) {
            lv_obj_set_style_opa(retract_switch_label, LV_OPA_COVER, 0);
        }
    }
    else {
        lv_obj_add_state(retract_switch, LV_STATE_DISABLED);
        lv_obj_clear_flag(retract_switch, LV_OBJ_FLAG_CLICKABLE);

        if (retract_switch_label) {
            lv_obj_set_style_opa(retract_switch_label, LV_OPA_50, 0);
        }
    }
}


// LVGL not support %f, have to use %ld.%02ld
static void format_mm_2dp(char* buf, size_t buf_size, float value_mm)
{
    if (!buf || buf_size == 0) return;

    bool negative = false;

    if (value_mm < 0.0f) {
        negative = true;
        value_mm = -value_mm;
    }

    unsigned long scaled =
        (unsigned long)(value_mm * 100.0f + 0.5f);

    unsigned long int_part = scaled / 100;
    unsigned long frac_part = scaled % 100;

    if (negative) {
        snprintf(
            buf,
            buf_size,
            "-%lu.%02lu",
            int_part,
            frac_part
        );
    }
    else {
        snprintf(
            buf,
            buf_size,
            "%lu.%02lu",
            int_part,
            frac_part
        );
    }
}

static void render_tmc_distance_labels()
{
    char target_buf[24];
    char actual_buf[24];

    format_mm_2dp(
        target_buf,
        sizeof(target_buf),
        last_tmc_target_distance_mm
    );

    format_mm_2dp(
        actual_buf,
        sizeof(actual_buf),
        last_tmc_actual_distance_mm
    );

    if (lbl_tmc_target_distance) {
        char line_buf[80];

        snprintf(
            line_buf,
            sizeof(line_buf),
            "Target Distance (mm): %s",
            target_buf
        );

        lv_label_set_text(
            lbl_tmc_target_distance,
            line_buf
        );
    }

    if (lbl_tmc_actual_distance) {
        char line_buf[80];

        snprintf(
            line_buf,
            sizeof(line_buf),
            "Actual Distance (mm): %s",
            actual_buf
        );

        lv_label_set_text(
            lbl_tmc_actual_distance,
            line_buf
        );
    }
}

static void update_tmc_distance_labels()
{
    // During extrusion, keep reading live TMC values.
    // After extrusion has finished, keep the cached extrusion-end values.
    if (!tmc_distance_frozen_after_extrusion &&
        MotorControlScreen::isReady()) {

        last_tmc_target_distance_mm =
            MotorControlScreen::getTargetDistanceMm();

        last_tmc_actual_distance_mm =
            MotorControlScreen::getActualDistanceMm();
    }

    render_tmc_distance_labels();
}

static void freeze_tmc_distance_at_extrusion_end()
{
    // Capture extrusion-end values before retraction starts.
    // This is important because start_retract_move_4mm() will issue a new TMC target.
    if (MotorControlScreen::isReady()) {
        last_tmc_target_distance_mm =
            MotorControlScreen::getTargetDistanceMm();

        last_tmc_actual_distance_mm =
            MotorControlScreen::getActualDistanceMm();
    }

    tmc_distance_frozen_after_extrusion = true;

    render_tmc_distance_labels();
}

void update_system_progress_data()
{
    // TMC distance belongs to MotorControlScreen state.
    // Update it even when the main extrusion run flag is not active.
    update_tmc_distance_labels();

    if (!is_running) return;

    unsigned long elapsed_ms = millis() - run_start_ms;

    float total_sec =
        (display_duration_sec_exact > 0.0f)
        ? display_duration_sec_exact
        : recorded_duration_sec_exact;

    float elapsed_sec_f = elapsed_ms / 1000.0f;
    
    float remaining_sec_f = total_sec - elapsed_sec_f;

    if (remaining_sec_f < 0.0f) {
        remaining_sec_f = 0.0f;
    }

    if (system_progress_force_remaining_zero) {
        remaining_sec_f = 0.0f;
    }

    /* ===== 同步真實資料 ===== */

    // Quantity（來自 SETTING）
    if (recorded_quantity) {
        lv_label_set_text_fmt(
            lbl_quantity,
            "Quantity (uL): %s",
            recorded_quantity
        );
    }

    // Viscosity（真實 viscosity）
    if (recorded_viscosity){ 
        lv_label_set_text_fmt(
            lbl_viscosity,
            "Viscosity (mPa.s): %s",
            recorded_viscosity
        );
    }

    // Duration（真實 duration）
    char duration_buf[32];
    format_seconds_2dp(duration_buf, sizeof(duration_buf), total_sec);

    lv_label_set_text_fmt(
        lbl_duration,
        "Duration (s): %s",
        duration_buf
    );

    // Remaining time
    // 轉成 xx.xx
    char remaining_buf[32];
    format_seconds_2dp(remaining_buf, sizeof(remaining_buf), remaining_sec_f);

    lv_label_set_text_fmt(
        lbl_remaining,
        "Remaining Time (s): \n%s",
        remaining_buf
    );

    // Elapsed time
    char elapsed_buf[32];
    format_seconds_2dp(elapsed_buf, sizeof(elapsed_buf), elapsed_sec_f);

    lv_label_set_text_fmt(
        lbl_elapsed,
        "Elapsed Time (s): %s",
        elapsed_buf
    );
}
// --------------------------------------
static void destroy_manual_screen_tab1()
{
    if (manual_screen_tab1) {
        lv_obj_del(manual_screen_tab1);
        manual_screen_tab1 = NULL;
    }
}

static void destroy_manual_screen_tab2()
{
    if (manual_screen_tab2) {
        lv_obj_del(manual_screen_tab2);
        manual_screen_tab2 = NULL;
    }
}

static void destroy_manual_screen_tab3()
{
    if (manual_screen_tab3) {
        lv_obj_del(manual_screen_tab3);
        manual_screen_tab3 = NULL;
    }
}

static void destroy_manual_screen_tab1_async(void* user_data)
{
    LV_UNUSED(user_data);
    destroy_manual_screen_tab1();
}

static void destroy_manual_screen_tab2_async(void* user_data)
{
    LV_UNUSED(user_data);
    destroy_manual_screen_tab2();
}

static void destroy_manual_screen_tab3_async(void* user_data)
{
    LV_UNUSED(user_data);
    destroy_manual_screen_tab3();
}

static bool calculate_predicted_duration()
{
    recorded_duration_valid = false;

    recorded_duration_sec_exact = 0.0f;
    recorded_duration_sec = 0;

    extrusion_duration_sec_exact = 0.0f;
    retraction_duration_sec_exact = 0.0f;
    operation_motion_duration_sec_exact = 0.0f;

    display_duration_sec_exact = 0.0f;
    display_duration_sec = 0;

    if (!build_realtime_motion_planner_from_current_settings()) {
        return false;
    }

    if (!tmc_executable_planner_valid || !g_tmc_ramp_cmd.valid) {
        return false;
    }

    float raw_extrusion_duration_s =
        MotorControlScreen::estimateExecutableDurationS(g_tmc_ramp_cmd);

    if (raw_extrusion_duration_s <= 0.0f) {
        return false;
    }

    extrusion_duration_sec_exact =
        apply_duration_calibration(
            raw_extrusion_duration_s,
            EXTRUSION_DURATION_CALIBRATION
        );

    g_tmc_ramp_cmd.expected_duration_s = extrusion_duration_sec_exact;

    // recorded_duration_sec_exact 保留為主擠出時間
    recorded_duration_sec_exact = extrusion_duration_sec_exact;

    if (run_retract_mode_enabled) {
        TmcRampCommand retract_cmd = make_retract_command_4mm();

        if (!retract_cmd.valid) {
            return false;
        }

        retraction_duration_sec_exact = retract_cmd.expected_duration_s;
    }

    operation_motion_duration_sec_exact =
        extrusion_duration_sec_exact + retraction_duration_sec_exact;

    if (operation_motion_duration_sec_exact <= 0.0f) {
        return false;
    }

    float display_margin_s =
        calc_display_margin_s(operation_motion_duration_sec_exact);

    // UI 顯示與 countdown 使用同一個值
    display_duration_sec_exact =
        operation_motion_duration_sec_exact + display_margin_s;

    recorded_duration_sec =
        (int)(recorded_duration_sec_exact + 0.5f);

    display_duration_sec =
        (int)(display_duration_sec_exact + 0.999f);

    if (recorded_duration_sec < 1) {
        recorded_duration_sec = 1;
    }

    if (display_duration_sec < 1) {
        display_duration_sec = 1;
    }

    recorded_duration_valid = true;
    return true;
}

// ====== For Planner Value Monitor =====
#if DEBUG_PLANNER

static void print_flow_params(const FlowConstraintModel::Params& p)
{
    Serial.println("========== FLOW PARAMS ==========");

    Serial.print("viscosity_mPa_s = ");
    Serial.println(p.viscosity, 6);

    Serial.print("R = ");
    Serial.println(p.R, 9);

    Serial.print("L = ");
    Serial.println(p.L, 9);

    Serial.print("shaft = ");
    Serial.println(p.shaft, 9);

    Serial.print("shaft_walls = ");
    Serial.println(p.shaft_walls, 9);

    Serial.print("E = ");
    Serial.println(p.E, 6);

    Serial.print("S = ");
    Serial.println(p.S, 6);

    Serial.print("needle_r = ");
    Serial.println(p.r, 9);

    Serial.print("needle_l = ");
    Serial.println(p.l, 9);

    Serial.print("lead_pitch = ");
    Serial.println(p.lead_pitch, 9);

    Serial.print("I_limit = ");
    Serial.println(p.I_limit, 6);

    Serial.print("stroke_per_ml_mm = ");
    Serial.println(p.stroke_per_ml_mm, 6);

    Serial.print("max_linear_speed_mm_s = ");
    Serial.println(p.max_linear_speed_mm_s, 6);

    Serial.print("I_plunger = ");
    Serial.println(p.I_plunger, 12);

    Serial.print("moving_mass_kg = ");
    Serial.println(p.moving_mass_kg, 6);

    Serial.print("friction_force_N = ");
    Serial.println(p.friction_force_N, 6);

    Serial.print("acceleration_utilization = ");
    Serial.println(p.acceleration_utilization, 6);

    Serial.print("use_load_terms_for_accel = ");
    Serial.println(p.use_load_terms_for_accel ? "true" : "false");

    Serial.print("eta = ");
    Serial.println(p.eta, 6);

    Serial.print("Kt = ");
    Serial.println(p.Kt, 6);

    Serial.print("max_rpm = ");
    Serial.println(p.max_rpm, 6);

    Serial.print("steps_per_rev = ");
    Serial.println(p.steps_per_rev, 6);

    Serial.print("max_step_freq = ");
    Serial.println(p.max_step_freq, 6);

    Serial.print("beta = ");
    Serial.println(p.beta, 6);

    Serial.print("K_buckling = ");
    Serial.println(p.K_buckling, 6);

    Serial.println("=================================");
}

static void print_flow_result(const FlowConstraintModel::Result& r)
{
    Serial.println("========== FLOW RESULT ==========");

    Serial.print("valid = ");
    Serial.println(r.valid ? "true" : "false");

    Serial.print("A_m2 = ");
    Serial.println(r.A, 12);

    Serial.print("mu_Pa_s = ");
    Serial.println(r.mu, 9);

    Serial.print("I_used = ");
    Serial.println(r.I_used, 12);

    Serial.print("F_crit_N = ");
    Serial.println(r.F_crit, 6);

    Serial.print("F_allow_N = ");
    Serial.println(r.F_allow, 6);

    Serial.print("F_motor_N = ");
    Serial.println(r.F_motor, 6);

    Serial.print("dP_motor_Pa = ");
    Serial.println(r.dP_motor, 6);

    Serial.print("F_capacity_N = ");
    Serial.println(r.F_capacity, 6);

    Serial.print("F_pressure_at_Q_allow_N = ");
    Serial.println(r.F_pressure_at_Q_allow, 6);

    Serial.print("F_friction_used_N = ");
    Serial.println(r.F_friction_used, 6);

    Serial.print("F_accel_available_N = ");
    Serial.println(r.F_accel_available, 6);

    Serial.print("moving_mass_kg = ");
    Serial.println(r.moving_mass_kg, 6);

    Serial.print("a_allow_m_s2 = ");
    Serial.println(r.a_allow_m_s2, 6);

    Serial.print("a_allow_mm_s2 = ");
    Serial.println(r.a_allow_mm_s2, 6);

    Serial.print("Q_struct_m3_s = ");
    Serial.println(r.Q_struct, 12);

    Serial.print("Q_motor_m3_s = ");
    Serial.println(r.Q_motor, 12);

    Serial.print("Q_rpm_m3_s = ");
    Serial.println(r.Q_rpm, 12);

    Serial.print("Q_step_m3_s = ");
    Serial.println(r.Q_step, 12);

    Serial.print("Q_force_safe_m3_s = ");
    Serial.println(r.Q_force_safe, 12);

    Serial.print("Q_speed_safe_m3_s = ");
    Serial.println(r.Q_speed_safe, 12);

    Serial.print("Q_allow_m3_s = ");
    Serial.println(r.Q_allow, 12);

    Serial.print("Q_allow_uL_s = ");
    Serial.println(r.Q_allow * 1.0e9f, 6);

    Serial.print("v_allow_mm_s = ");
    Serial.println(r.v_allow * 1000.0f, 6);

    Serial.println("=================================");
}

static void print_ideal_planner(const PlungerMotionPlanner& planner)
{
    Serial.println("========== IDEAL PLANNER ==========");

    Serial.print("target_volume_m3 = ");
    Serial.println(planner.getTargetVolume_m3(), 12);

    Serial.print("target_volume_uL = ");
    Serial.println(planner.getTargetVolume_m3() * 1.0e9f, 6);

    Serial.print("target_stroke_m = ");
    Serial.println(planner.getTargetStroke_m(), 12);

    Serial.print("target_stroke_mm = ");
    Serial.println(planner.getTargetStroke_m() * 1000.0f, 6);

    Serial.print("ideal_total_duration_s = ");
    Serial.println(planner.totalDuration(), 6);

    Serial.print("ideal_peak_velocity_mm_s = ");
    Serial.println(planner.getPeakVelocity_mps() * 1000.0f, 6);

    Serial.print("area_m2 = ");
    Serial.println(planner.getArea_m2(), 12);

    Serial.println("===================================");
}

static void print_tmc_command(const TmcRampCommand& cmd)
{
    Serial.println("========== TMC RAMP COMMAND ==========");

    Serial.print("valid = ");
    Serial.println(cmd.valid ? "true" : "false");

    Serial.print("quantity_uL = ");
    Serial.println(cmd.quantity_uL, 6);

    Serial.print("stroke_per_ml_mm = ");
    Serial.println(cmd.stroke_per_ml_mm, 6);

    Serial.print("target_distance_mm = ");
    Serial.println(cmd.target_distance_mm, 6);

    Serial.print("max_velocity_mm_s = ");
    Serial.println(cmd.max_velocity_mm_s, 6);

    Serial.print("max_acceleration_mm_s2 = ");
    Serial.println(cmd.max_acceleration_mm_s2, 6);

    Serial.print("max_deceleration_mm_s2 = ");
    Serial.println(cmd.max_deceleration_mm_s2, 6);

    Serial.print("start_velocity_mm_s = ");
    Serial.println(cmd.start_velocity_mm_s, 6);

    Serial.print("stop_velocity_mm_s = ");
    Serial.println(cmd.stop_velocity_mm_s, 6);

    Serial.print("first_velocity_mm_s = ");
    Serial.println(cmd.first_velocity_mm_s, 6);

    Serial.print("first_acceleration_mm_s2 = ");
    Serial.println(cmd.first_acceleration_mm_s2, 6);

    Serial.print("first_deceleration_mm_s2 = ");
    Serial.println(cmd.first_deceleration_mm_s2, 6);

    Serial.print("expected_duration_s = ");
    Serial.println(cmd.expected_duration_s, 6);

    Serial.print("reverse_direction = ");
    Serial.println(cmd.reverse_direction ? "true" : "false");

    Serial.println("======================================");
}

#endif
// ======================================

static bool build_realtime_motion_planner_from_current_settings()
{
    realtime_motion_planner_valid = false;
    tmc_executable_planner_valid = false;
    g_tmc_ramp_cmd = TmcRampCommand();

    realtime_curve_volume_max_uL = 1000.0f;
    realtime_curve_current_max_A = 3.0f;

    if (!recorded_viscosity || !recorded_quantity) {
        return false;
    }

    FlowConstraintModel::Params flow_params;

    flow_params.viscosity = atof(recorded_viscosity); // mPa.s
    flow_params.R = CustomizedParametersScreen::getSavedFloat(CustomizedParametersScreen::DEV_P_R);
    flow_params.L = CustomizedParametersScreen::getSavedFloat(CustomizedParametersScreen::DEV_P_L);
    flow_params.shaft = CustomizedParametersScreen::getSavedFloat(CustomizedParametersScreen::DEV_P_SHAFT);
    flow_params.shaft_walls = CustomizedParametersScreen::getSavedFloat(CustomizedParametersScreen::DEV_P_SHAFT_WALL);
    flow_params.E = CustomizedParametersScreen::getSavedFloat(CustomizedParametersScreen::DEV_P_E);
    flow_params.r = CustomizedParametersScreen::getSavedFloat(CustomizedParametersScreen::DEV_P_r);
    flow_params.l = CustomizedParametersScreen::getSavedFloat(CustomizedParametersScreen::DEV_P_l);
    flow_params.lead_pitch = CustomizedParametersScreen::getSavedFloat(CustomizedParametersScreen::DEV_P_LEAD_PITCH);
    flow_params.I_limit = CustomizedParametersScreen::getSavedFloat(CustomizedParametersScreen::DEV_P_I_LIMIT);
    flow_params.stroke_per_ml_mm = CustomizedParametersScreen::getSavedFloat(CustomizedParametersScreen::DEV_P_STROKE_PER_ML_MM);
    flow_params.max_linear_speed_mm_s = CustomizedParametersScreen::getSavedFloat(CustomizedParametersScreen::DEV_P_MAX_LINEAR_SPEED_MM_S);
    flow_params.I_plunger = CustomizedParametersScreen::getSavedFloat(CustomizedParametersScreen::DEV_P_I_PLUNGER);

    // First-order acceleration upper-bound model.
    // This does NOT explain the current idle no-motion issue.
    // The idle no-motion issue is still treated as a TMC low-acceleration execution problem.
    flow_params.moving_mass_kg = 0.5f;
    flow_params.friction_force_N = 0.0f;
    flow_params.acceleration_utilization = 1.0f;

    // false:
    // F_pressure and F_friction are calculated for debug/reference,
    // but they are not subtracted from F_capacity.
    flow_params.use_load_terms_for_accel = false;

    // Engineering constants such as eta, Kt, max_rpm, steps_per_rev,
    // max_step_freq, beta, S, and K_buckling are kept in
    // FlowConstraintModel::Params() defaults.
    // They are intentionally not exposed to normal syringe users.

    // ====== For Planner Value Monitor =====
    // #if DEBUG_PLANNER
    // print_flow_params(flow_params);
    // #endif
    // ======================================

    FlowConstraintModel::Result flow_result = FlowConstraintModel::compute(flow_params);
    // ====== For Planner Value Monitor =====
    // #if DEBUG_PLANNER
    // print_flow_result(flow_result);
    // #endif
    // ======================================

    if (!flow_result.valid || flow_result.Q_allow <= 0.0f) {
        return false;
    }

    PlungerMotionPlanner::Command motion_cmd;
    motion_cmd.volume = atof(recorded_quantity); // uL
    motion_cmd.ramp_ratio = 0.05f;
    motion_cmd.min_ramp_time = 0.05f;

    if (!realtime_motion_planner.build(motion_cmd, flow_params, flow_result)) {
        return false;
    }

    realtime_motion_planner_valid = true;

    // ====== For Planner Value Monitor =====
    // #if DEBUG_PLANNER
    // print_ideal_planner(realtime_motion_planner);
    // #endif
    // ======================================

    // // ------------------------------------------------------------
    // // TMC executable ramp configuration
    // // ------------------------------------------------------------
    // // This configuration defines the executable lower and upper bounds
    // // used when converting the ideal fluid-safe motion curve into a
    // // TMC5160 internal-ramp command.
    // //
    // // Important:
    // // These values are not viscosity classification thresholds.
    // // They are not used to switch control paths.
    // //
    // // The purpose of this block is to ensure that the generated TMC ramp:
    // // 1. does not exceed the physical flow / force constraints calculated
    // //    by FlowConstraintModel
    // // 2. does not fall below the experimentally verified executable ramp
    // //    range of the TMC5160 internal ramp generator
    // //
    // // Current verified baseline from manual Motor Control test:
    // // MaxVel   = 0.30 mm/s
    // // MaxAcc   = 0.20 mm/s^2
    // // MaxDec   = 0.20 mm/s^2
    // // FirstVel = 0.00 mm/s
    // // FirstAcc = 0.10 mm/s^2
    // // FirstDec = 0.10 mm/s^2
    // //
    // // Note:
    // // Acceleration values lower than this baseline may generate a valid
    // // software command but may not produce reliable physical motor motion,
    // // especially after conversion into TMC internal register values.
    // //
    // // Future work:
    // // Move these verified ramp bounds into CustomizedParametersScreen
    // // or a dedicated hardware calibration profile.

    TmcExecutableRampPlanner::Config tmc_cfg;

    // Required for very low speed such as 0.007 mm/s
    tmc_cfg.min_velocity_mm_s = 0.001f;

    // Do not set acceleration too low.
    // At your current effective resolution, 0.005 to 0.01 mm/s² is safer.
    tmc_cfg.min_acceleration_mm_s2 = 0.005f;
    tmc_cfg.min_deceleration_mm_s2 = 0.005;

    // Keep upper bound moderate.
    tmc_cfg.max_acceleration_mm_s2 = 0.10f;
    tmc_cfg.max_deceleration_mm_s2 = 0.10f;

    tmc_cfg.start_velocity_mm_s = 0.0f;

    // Position mode should not use VSTOP = 0.
    // Must be smaller than 0.007 mm/s.
    tmc_cfg.stop_velocity_mm_s = 0.0022f;

    // Keep first velocity zero if this was already verified.
    tmc_cfg.first_velocity_ratio = 0.50f;

    // But keep first acceleration/deceleration nonzero.
    tmc_cfg.first_acceleration_ratio = 0.40f;
    tmc_cfg.first_deceleration_ratio = 0.40f;

    tmc_cfg.velocity_safety_scale = 1.0f;
    tmc_cfg.reverse_direction = false;

    if (!tmc_executable_planner.build(
            realtime_motion_planner,
            flow_params,
            flow_result,
            tmc_cfg
        )) {
        realtime_motion_planner_valid = false;
        tmc_executable_planner_valid = false;
        g_tmc_ramp_cmd = TmcRampCommand();
        return false;
    }

    g_tmc_ramp_cmd = tmc_executable_planner.command();
    tmc_executable_planner_valid = g_tmc_ramp_cmd.valid;

    // ====== For Planner Value Monitor =====
    // #if DEBUG_PLANNER
    // print_tmc_command(g_tmc_ramp_cmd);
    // #endif
    // ======================================

    // //////////
    // Serial.println("========== SETTING GENERATED TMC CMD ==========");

    // Serial.print("quantity_uL = ");
    // Serial.println(g_tmc_ramp_cmd.quantity_uL, 6);

    // Serial.print("stroke_per_ml_mm = ");
    // Serial.println(g_tmc_ramp_cmd.stroke_per_ml_mm, 6);

    // Serial.print("target_distance_mm = ");
    // Serial.println(g_tmc_ramp_cmd.target_distance_mm, 6);

    // Serial.print("max_velocity_mm_s = ");
    // Serial.println(g_tmc_ramp_cmd.max_velocity_mm_s, 6);

    // Serial.print("max_acceleration_mm_s2 = ");
    // Serial.println(g_tmc_ramp_cmd.max_acceleration_mm_s2, 6);

    // Serial.print("max_deceleration_mm_s2 = ");
    // Serial.println(g_tmc_ramp_cmd.max_deceleration_mm_s2, 6);

    // Serial.print("start_velocity_mm_s = ");
    // Serial.println(g_tmc_ramp_cmd.start_velocity_mm_s, 6);

    // Serial.print("stop_velocity_mm_s = ");
    // Serial.println(g_tmc_ramp_cmd.stop_velocity_mm_s, 6);

    // Serial.print("first_velocity_mm_s = ");
    // Serial.println(g_tmc_ramp_cmd.first_velocity_mm_s, 6);

    // Serial.print("first_acceleration_mm_s2 = ");
    // Serial.println(g_tmc_ramp_cmd.first_acceleration_mm_s2, 6);

    // Serial.print("first_deceleration_mm_s2 = ");
    // Serial.println(g_tmc_ramp_cmd.first_deceleration_mm_s2, 6);

    // Serial.print("expected_duration_s = ");
    // Serial.println(g_tmc_ramp_cmd.expected_duration_s, 6);

    // Serial.print("reverse_direction = ");
    // Serial.println(g_tmc_ramp_cmd.reverse_direction ? "true" : "false");

    // Serial.print("valid = ");
    // Serial.println(g_tmc_ramp_cmd.valid ? "true" : "false");

    // Serial.println("================================================");
    // //////////

    if (!tmc_executable_planner_valid) {
        realtime_motion_planner_valid = false;
        return false;
    }

    realtime_curve_volume_max_uL = g_tmc_ramp_cmd.quantity_uL;

    if (realtime_curve_volume_max_uL < 1.0f) {
        realtime_curve_volume_max_uL = 1.0f;
    }

    float peak_current_A = 0.0f;
    const int scan_count = 120;

    for (int i = 0; i <= scan_count; i++) {
        float t_s = (realtime_motion_planner.totalDuration() * i) / scan_count;
        PlungerMotionPlanner::Sample s = realtime_motion_planner.evaluate(t_s);
        if (s.currentEst > peak_current_A) {
            peak_current_A = s.currentEst;
        }
    }

    if (peak_current_A < 0.1f) {
        peak_current_A = 0.1f;
    }

    realtime_curve_current_max_A = peak_current_A * 1.1f;

    return true;
}

static TmcRampCommand make_retract_command_4mm()
{
    TmcRampCommand retract_cmd;

    if (!g_tmc_ramp_cmd.valid) {
        return retract_cmd;
    }

    retract_cmd = g_tmc_ramp_cmd;

    retract_cmd.valid = true;
    retract_cmd.target_distance_mm = RETRACT_DISTANCE_MM;
    retract_cmd.quantity_uL = 0.0f;

    // 回抽方向 = 主擠出相反
    retract_cmd.reverse_direction = !g_tmc_ramp_cmd.reverse_direction;

    // 保留你設定的回抽速度，不為了估算去改速度
    retract_cmd.max_velocity_mm_s = RETRACT_MAX_VELOCITY_MM_S;
    retract_cmd.max_acceleration_mm_s2 = RETRACT_ACCEL_MM_S2;
    retract_cmd.max_deceleration_mm_s2 = RETRACT_DECEL_MM_S2;

    retract_cmd.start_velocity_mm_s = 0.0f;
    retract_cmd.stop_velocity_mm_s = 0.0022f;

    retract_cmd.first_velocity_mm_s = 0.0f;
    retract_cmd.first_acceleration_mm_s2 = RETRACT_ACCEL_MM_S2 * 0.4f;
    retract_cmd.first_deceleration_mm_s2 = RETRACT_DECEL_MM_S2 * 0.4f;

    float raw_duration_s =
        MotorControlScreen::estimateExecutableDurationS(retract_cmd);

    if (raw_duration_s <= 0.0f) {
        retract_cmd.valid = false;
        return retract_cmd;
    }

    retract_cmd.expected_duration_s =
        apply_duration_calibration(
            raw_duration_s,
            RETRACTION_DURATION_CALIBRATION
        );

    return retract_cmd;
}

static bool start_retract_move_4mm()
{
    TmcRampCommand retract_cmd = make_retract_command_4mm();

    if (!retract_cmd.valid) {
        return false;
    }

    retraction_duration_sec_exact =
        retract_cmd.expected_duration_s;

    Serial.println("========== RETRACTION COMMAND ==========");
    Serial.print("target_distance_mm = ");
    Serial.println(retract_cmd.target_distance_mm, 6);
    Serial.print("max_velocity_mm_s = ");
    Serial.println(retract_cmd.max_velocity_mm_s, 6);
    Serial.print("max_acceleration_mm_s2 = ");
    Serial.println(retract_cmd.max_acceleration_mm_s2, 6);
    Serial.print("max_deceleration_mm_s2 = ");
    Serial.println(retract_cmd.max_deceleration_mm_s2, 6);
    Serial.print("expected_duration_s = ");
    Serial.println(retract_cmd.expected_duration_s, 6);
    Serial.print("reverse_direction = ");
    Serial.println(retract_cmd.reverse_direction ? "true" : "false");
    Serial.println("========================================");

    return MotorControlScreen::startExtrusionMove(retract_cmd);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Display.begin();
  TouchDetector.begin();
  MotionModeManager::begin();

  lv_initial_tabview();
  CustomizedParametersScreen::initStore();

  main_screen = lv_scr_act();
  MotorControlScreen::setMainScreen(main_screen);
  CustomizedParametersScreen::setMainScreen(main_screen);

  // input_block_t* lv_create_input_block(lv_obj_t* parent, const char* title, bool* has_options, lv_coord_t x, lv_coord_t y)
  viscosity_block = lv_create_input_block(tab1, "Liquid Viscosity (mPa.s)", true, -10, -10, &viscosity_keypad_cfg);
  // lv_add_option_item(parent, char* label, char* visocsity)
  lv_add_option_item(viscosity_block, "1 (Water)", "1");
  lv_add_option_item(viscosity_block, "30,000", "30000");
  lv_add_option_item(viscosity_block, "60,000", "60000");

  quantity_block = lv_create_input_block(tab1, "Extrusion Quantity (uL)", false, 390, -10, &quantity_keypad_cfg);
  create_start_button(tab1, 0, -10);
  create_retract_mode_switch(tab1);

  create_system_progress_ui(tab2);
  create_tab3_curve_preview_blocks(tab3);

  RealtimeCurveRenderer::Config curve_cfg;
  curve_cfg.point_count = 120;
  curve_cfg.expected_duration_s = 10.0f;
  curve_cfg.volume_max_uL = 1000.0f;
  curve_cfg.current_max_A = 3.0f;

  RealtimeCurveRenderer::init(
      tab3_volume_curve_panel,
      tab3_current_curve_panel,
      curve_cfg
  );

  // 在 DEVELOPER INFO(tab3) 放跳轉按鈕
  btn_tab3_parameter_setting = create_nav_button(
      tab3,                   // 放在 tab3
      85, 85,                 // 按鈕尺寸
      LV_ALIGN_TOP_RIGHT,     // 對齊方式
      0, 0,                   // x, y offset
      &parameter_setting,     // 圖片
      180,                    // 圖片縮放
      CustomizedParametersScreen::getScreenHandle(),            // 要跳轉的 screen
      CustomizedParametersScreen::build        // 若尚未建立，先呼叫 builder
  );
  btn_tab3_motor_control = create_nav_button(
      tab3,                   // 放在 tab3
      85, 85,                 // 按鈕尺寸
      LV_ALIGN_TOP_RIGHT,     // 對齊方式
      0, 95,                   // x, y offset
      &stepper_motor,     // 圖片
      256,                    // 圖片縮放
      MotorControlScreen::getScreenHandle(),            // 要跳轉的 screen
      MotorControlScreen::build        // 若尚未建立，先呼叫 builder
  );
  btn_tab3_disable_motor = create_disable_motor_button(
      tab3,
      85, 85,
      LV_ALIGN_TOP_RIGHT,
      0, 190,
      &stepper_motor_disable,
      256
  );
 create_nav_button(
      tab3,                   // 放在 tab3
      70, 70,                 // 按鈕尺寸
      LV_ALIGN_BOTTOM_RIGHT,     // 對齊方式
      0, 0,                   // x, y offset
      &user_manual,     // 圖片
      150,                    // 圖片縮放
      &manual_screen_tab3,            // 要跳轉的 screen
      build_manual_screen_tab3   // 若尚未建立，先呼叫 builder
  );
 create_nav_button(
      tab1,                   // 放在 tab3
      70, 70,                 // 按鈕尺寸
      LV_ALIGN_BOTTOM_RIGHT,     // 對齊方式
      0, 0,                   // x, y offset
      &user_manual,     // 圖片
      150,                    // 圖片縮放
      &manual_screen_tab1,            // 要跳轉的 screen
      build_manual_screen_tab1   // 若尚未建立，先呼叫 builder
  );
 create_nav_button(
      tab2,                   // 放在 tab3
      70, 70,                 // 按鈕尺寸
      LV_ALIGN_BOTTOM_RIGHT,     // 對齊方式
      0, 0,                   // x, y offset
      &user_manual,     // 圖片
      150,                    // 圖片縮放
      &manual_screen_tab2,            // 要跳轉的 screen
      build_manual_screen_tab2   // 若尚未建立，先呼叫 builder
  );

  set_tab3_control_buttons_enabled(true);
  set_retract_switch_enabled(true);
  retract_move_started = false;
}

void loop()
{
    lv_timer_handler();

    // =============================
    // PRIORITY: user stop request
    // =============================
    if (stop_requested)
    {
        bool stopped = MotorControlScreen::motionFinished();

        bool stop_wait_timeout =
            (millis() - stop_request_ms > 3000);

        if (stopped || stop_wait_timeout)
        {
            const bool was_timeout_stop = stop_requested_by_timeout;

            if (stop_wait_timeout && !stopped) {
                MotorControlScreen::forceStopState();
            }

            stop_requested = false;
            stop_requested_by_timeout = false;
            is_running = false;

            run_phase = RUN_PHASE_IDLE;
            phase_start_ms = 0;
            current_tail_recording = false;
            current_tail_start_ms = 0;
            retract_move_started = false;

            const char* stop_reason = "Stopped";

            if (was_timeout_stop) {
                stop_reason = stop_wait_timeout
                    ? "Timeout stop failed"
                    : "Extrusion timeout stopped";
            }
            else {
                stop_reason = stop_wait_timeout
                    ? "Stop timeout"
                    : "User stopped";
            }

            MotionModeManager::forceIdle(stop_reason);

            recorded_duration_valid = false;
            realtime_motion_planner_valid = false;
            tmc_executable_planner_valid = false;
            g_tmc_ramp_cmd = TmcRampCommand();

            set_tab3_control_buttons_enabled(true);
            set_retract_switch_enabled(true);
            retract_move_started = false;

            if (RealtimeCurveRenderer::isInitialized()) {
                RealtimeCurveRenderer::releaseTempData();
            }

            if (start_btn)
            {
                lv_obj_t* label = lv_obj_get_child(start_btn, 0);
                lv_label_set_text(label, "START");
                lv_obj_set_style_bg_color(
                    start_btn,
                    lv_color_hex(0x0000FF),
                    0
                );
            }

            clear_countdown_dialog();
            show_finish_dialog();
        }

        MotorControlScreen::update();
        delay(5);
        return;
    }

    // =============================
    // Normal runtime logic
    // =============================
    if (is_running) {
        float phase_elapsed_s =
            (millis() - phase_start_ms) / 1000.0f;

        bool finished_by_motor =
            MotorControlScreen::motionFinished();
        //
        if (run_phase == RUN_PHASE_EXTRUDING) {

            if (finished_by_motor) {

                // Freeze extrusion-end TMC target / actual distance.
                // Do this before retraction starts, otherwise the retraction target
                // will overwrite the displayed extrusion-end values.
                freeze_tmc_distance_at_extrusion_end();

                if (run_retract_mode_enabled) {
                    retract_move_started = true;

                    if (!start_retract_move_4mm()) {
                        MotionModeManager::forceIdle("Retraction failed");

                        is_running = false;
                        current_tail_recording = false;
                        retract_move_started = false;
                        run_phase = RUN_PHASE_IDLE;
                        phase_start_ms = 0;

                        recorded_duration_valid = false;
                        realtime_motion_planner_valid = false;
                        tmc_executable_planner_valid = false;
                        g_tmc_ramp_cmd = TmcRampCommand();

                        set_tab3_control_buttons_enabled(true);
                        set_retract_switch_enabled(true);

                        if (start_btn) {
                            lv_obj_t* label = lv_obj_get_child(start_btn, 0);
                            lv_label_set_text(label, "START");
                            lv_obj_set_style_bg_color(start_btn, lv_color_hex(0x0000FF), 0);
                        }

                        clear_countdown_dialog();
                        show_missing_param_dialog("Retraction failed");

                        MotorControlScreen::update();
                        delay(5);
                        return;
                    }

                    run_phase = RUN_PHASE_RETRACTING;
                    phase_start_ms = millis();

                    MotorControlScreen::update();
                    delay(5);
                    return;
                }
                
                begin_system_progress_tail_ramp();

                current_tail_recording = true;
                current_tail_start_ms = millis();
                run_phase = RUN_PHASE_TAIL_RECORDING;
                phase_start_ms = millis();
            }
            else {
                float extrusion_timeout_margin_s =
                    calc_extrusion_timeout_margin_s(extrusion_duration_sec_exact);

                float timeout_limit_s =
                    extrusion_duration_sec_exact + extrusion_timeout_margin_s;

                if (phase_elapsed_s > timeout_limit_s) {
                    auto tr = MotionModeManager::requestScenario(
                        MotionModeManager::SCENARIO_EXTRUSION_STOPPING
                    );

                    if (!tr.success) {
                        MotionModeManager::forceIdle("Extrusion timeout");
                    }

                    MotorControlScreen::stopMotion();

                    stop_requested = true;
                    stop_requested_by_timeout = true;
                    stop_request_ms = millis();

                    MotorControlScreen::update();
                    delay(5);
                    return;
                }
            }
        }
        else if (run_phase == RUN_PHASE_RETRACTING) {
            
            if (finished_by_motor) {
                begin_system_progress_tail_ramp();

                current_tail_recording = true;
                current_tail_start_ms = millis();
                run_phase = RUN_PHASE_TAIL_RECORDING;
                phase_start_ms = millis();
            }
            else {
                float timeout_limit_s =
                    retraction_duration_sec_exact +
                    calc_timeout_margin_s(retraction_duration_sec_exact);

                if (phase_elapsed_s > timeout_limit_s) {
                    auto tr = MotionModeManager::requestScenario(
                        MotionModeManager::SCENARIO_EXTRUSION_STOPPING
                    );

                    if (!tr.success) {
                        MotionModeManager::forceIdle("Retraction timeout");
                    }

                    MotorControlScreen::stopMotion();

                    stop_requested = true;
                    stop_requested_by_timeout = true;
                    stop_request_ms = millis();

                    MotorControlScreen::update();
                    delay(5);
                    return;
                }
            }
        }
        else if (run_phase == RUN_PHASE_TAIL_RECORDING) {

            if (millis() - current_tail_start_ms >=
                (unsigned long)(CURRENT_TAIL_SEC * 1000.0f)) {

                is_running = false;
                current_tail_recording = false;
                retract_move_started = false;
                run_phase = RUN_PHASE_IDLE;
                phase_start_ms = 0;

                force_system_progress_operation_complete();

                set_tab3_control_buttons_enabled(true);
                set_retract_switch_enabled(true);

                MotionModeManager::finishCurrentScenario();

                recorded_duration_valid = false;
                realtime_motion_planner_valid = false;
                tmc_executable_planner_valid = false;
                g_tmc_ramp_cmd = TmcRampCommand();

                if (RealtimeCurveRenderer::isInitialized()) {
                    RealtimeCurveRenderer::releaseTempData();
                }

                if (start_btn) {
                    lv_obj_t* label = lv_obj_get_child(start_btn, 0);
                    lv_label_set_text(label, "START");
                    lv_obj_set_style_bg_color(start_btn, lv_color_hex(0x0000FF), 0);
                }

                clear_countdown_dialog();
                show_finish_dialog();
            }
        }
    }

    // --- countdown update ---
    if (is_running && countdown_dialog && countdown_label) {
        if (millis() - last_countdown_update >= 50) {
            last_countdown_update = millis();

            unsigned long elapsed_ms = millis() - run_start_ms;

            float elapsed_sec_f = elapsed_ms / 1000.0f;
            float remaining_sec_f =
                display_duration_sec_exact - elapsed_sec_f;

            if (remaining_sec_f < 0.0f) {
                remaining_sec_f = 0.0f;
            }

            char remaining_buf[32];
            format_seconds_2dp(
                remaining_buf,
                sizeof(remaining_buf),
                remaining_sec_f
            );

            lv_label_set_text_fmt(
                countdown_label,
                "%s s",
                remaining_buf
            );
        }
    }

    // --- System Progress update ---
    if (is_running) {
        if (system_progress_force_complete) {
            set_system_progress_percent(100.0f, true);
        }
        else if (system_progress_tail_ramp_active &&
                run_phase == RUN_PHASE_TAIL_RECORDING) {

            float tail_elapsed_s =
                (millis() - system_progress_tail_start_ms) / 1000.0f;

            float ratio = tail_elapsed_s / CURRENT_TAIL_SEC;

            if (ratio < 0.0f) {
                ratio = 0.0f;
            }

            if (ratio > 1.0f) {
                ratio = 1.0f;
            }

            float percent_f =
                system_progress_tail_start_percent +
                (99.9f - system_progress_tail_start_percent) * ratio;

            set_system_progress_percent(percent_f, false);
        }
        else {
            float percent_f = calc_system_progress_percent_f();
            set_system_progress_percent(percent_f, false);
        }
    }

    MotorControlScreen::update();
    update_system_progress_data();

    // --- realtime curve update ---
    if ((is_running || current_tail_recording) && RealtimeCurveRenderer::isInitialized()) {
        if (millis() - last_curve_update_ms >= 50) {
            last_curve_update_ms = millis();

            float chart_t_s = (millis() - run_start_ms) / 1000.0f;

            if (recorded_duration_sec_exact <= 0.0f) {
                recorded_duration_sec_exact = 1.0f;
            }

            if (chart_total_duration_sec <= 0.0f) {
                chart_total_duration_sec = recorded_duration_sec_exact;
            }

            if (chart_t_s > chart_total_duration_sec) {
                chart_t_s = chart_total_duration_sec;
            }

            if (tmc_executable_planner_valid) {
                // motion planner 只評估到 motor duration
                // tail 區間 volume 固定在最後值
                float eval_t_s = chart_t_s;
                if (eval_t_s > recorded_duration_sec_exact) {
                    eval_t_s = recorded_duration_sec_exact;
                }

                TmcExecutableRampPlanner::Sample sample =
                    tmc_executable_planner.evaluate(eval_t_s);

                float current_A =
                    MotorControlScreen::getMeasuredCurrentA();

                RealtimeCurveRenderer::pushSample(
                    chart_t_s,
                    sample.volume_uL,
                    current_A
                );
            }
        }
    }

    delay(5);
}