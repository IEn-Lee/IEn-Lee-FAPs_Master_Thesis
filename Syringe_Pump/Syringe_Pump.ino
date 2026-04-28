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
#include "RealtimeCurveRenderer.h"
#include "MotorControlScreen.h"
#include "PlungerSpiPositionStreamer.h"
#include "CustomizedParametersScreen.h"

using namespace PlungerSpiPositionStreamerNS;

// --- Global Variables ---
// --- SPI Real Control ---
static PlungerSpiPositionStreamer realtime_streamer;
static bool realtime_streamer_valid = false;
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
    6       // max chars
};

static const keypad_config_t quantity_keypad_cfg = {
    quantity_kb_map,
    LV_ALIGN_BOTTOM_MID,
    -160,
    -50,
    false,  // no decimal
    true,   // allow "00"
    6       // max chars
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
// --- Tab3 preview panels ---
static lv_obj_t* tab3_volume_curve_panel = NULL;
static lv_obj_t* tab3_current_curve_panel = NULL;
// --- Tab3 control buttons ---
static lv_obj_t* btn_tab3_parameter_setting = NULL;
static lv_obj_t* btn_tab3_motor_control = NULL;
// --- Motion planner runtime ---
static PlungerMotionPlanner realtime_motion_planner;
static bool realtime_motion_planner_valid = false;

static float realtime_curve_volume_max_uL = 1000.0f;
static float realtime_curve_current_max_A = 3.0f;

static unsigned long last_curve_update_ms = 0;
// --- Global Variables ---
input_block_t* viscosity_block;
input_block_t* quantity_block;

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
        // 嘗試抓取設定
        fetch_setting_values();

        if (!recorded_viscosity) {
            show_missing_param_dialog("Missing liquid viscosity");
            return;
        }
        if (!recorded_quantity) {
            show_missing_param_dialog("Missing extrusion quantity");
            return;
        }

        if (!calculate_predicted_duration()) {
            show_missing_param_dialog("Failed to build motion planner");
            return;
        }

        if (!build_realtime_spi_streamer_from_current_settings()) {
            show_missing_param_dialog("Failed to initialize SPI control");
            return;
        }

        show_start_confirm_dialog(btn);
    }
    else {
        // STOP → reuse existing stop confirm
        show_confirm_dialog(btn, tab1);
    }
}

void create_start_button(lv_obj_t* parent, lv_coord_t x, lv_coord_t y)
{
    start_btn = lv_btn_create(parent);
    lv_obj_set_size(start_btn, 320, 60);
    lv_obj_align(start_btn, LV_ALIGN_BOTTOM_MID, x, y);
    lv_obj_set_style_bg_color(start_btn, lv_color_hex(0x0000FF), 0);

    lv_obj_add_event_cb(start_btn, start_btn_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t* label = lv_label_create(start_btn);
    lv_label_set_text(label, "START");
    lv_obj_set_style_text_font(label, &montserrat_30, 0);
    lv_obj_center(label);
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
        is_running = false;

        if (realtime_streamer_valid && realtime_streamer.isRunning()) {
            realtime_streamer.stop(true);
        }

        set_tab3_control_buttons_enabled(true);
        
        if (realtime_motion_planner_valid && RealtimeCurveRenderer::isInitialized()) {
            float t_s = (millis() - run_start_ms) / 1000.0f;
            if (t_s > recorded_duration_sec_exact) {
                t_s = recorded_duration_sec_exact;
            }

            PlungerMotionPlanner::Sample sample = realtime_motion_planner.evaluate(t_s);

            RealtimeCurveRenderer::pushSample(
                t_s,
                sample.volume,
                sample.currentEst
            );

            RealtimeCurveRenderer::releaseTempData();
        }

        realtime_motion_planner_valid = false;

        clear_countdown_dialog();

        lv_obj_t* btn = (lv_obj_t*)lv_event_get_user_data(e);
        lv_obj_t* label = lv_obj_get_child(btn, 0);

        lv_label_set_text(label, "START");
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x0000FF), 0);

        lv_obj_del(confirm_dialog);
        confirm_dialog = NULL;
    }, LV_EVENT_CLICKED, btn);

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

    if (recorded_duration_valid) {
        unsigned long duration_ms   = (unsigned long)(recorded_duration_sec_exact * 1000.0f + 0.5f);
        unsigned long duration_int  = duration_ms / 1000;
        unsigned long duration_frac = (duration_ms % 1000) / 10; // 2 digits

        snprintf(
            info_buf,
            sizeof(info_buf),
            "Liquid Viscosity: %s\nExtrusion Quantity: %s\nDuration: %lu.%02lu s",
            recorded_viscosity,
            recorded_quantity,
            duration_int,
            duration_frac
        );
    }
    else {
        snprintf(
            info_buf,
            sizeof(info_buf),
            "Liquid Viscosity: %s\nExtrusion Quantity: %s\nDuration: N/A",
            recorded_viscosity,
            recorded_quantity
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

        is_running = true;
        run_start_ms = millis();   // ✅ 記錄開始時間
        last_curve_update_ms = 0;
        set_tab3_control_buttons_enabled(false);

        if (realtime_streamer_valid) {
            if (!realtime_streamer.start(true)) {
                is_running = false;
                show_missing_param_dialog("Failed to start SPI streaming");
                return;
            }
        }

        // reset charts using current theoretical range
        RealtimeCurveRenderer::Config curve_cfg;
        curve_cfg.expected_duration_s = recorded_duration_sec_exact;
        curve_cfg.sample_interval_ms  = 100;   // 1 point per second
        curve_cfg.point_count         = 0;      // auto-calc
        curve_cfg.volume_max_uL       = realtime_curve_volume_max_uL * 1.05f; // chart Y axis range
        curve_cfg.current_max_A       = 5; // chart Y axis range

        if (curve_cfg.volume_max_uL < 1.0f) {
            curve_cfg.volume_max_uL = 1.0f;
        }
        if (curve_cfg.current_max_A < 0.1f) {
            curve_cfg.current_max_A = 0.1f;
        }

        RealtimeCurveRenderer::reset(curve_cfg);

        update_system_progress_data();
        show_countdown_dialog(recorded_duration_sec, tab1);

        // ✅ 強制第一點
        if (realtime_motion_planner_valid && RealtimeCurveRenderer::isInitialized()) {
            PlungerMotionPlanner::Sample s0 = realtime_motion_planner.evaluate(0.0f);

            RealtimeCurveRenderer::pushSample(
                0.0f,
                s0.volume,
                s0.currentEst
            );
        }

        lv_label_set_text(label, "STOP");
        lv_obj_set_style_bg_color(btn, lv_color_hex(0xFF0000), 0);

        lv_obj_del(confirm_dialog);
        confirm_dialog = NULL;
    }, LV_EVENT_CLICKED, start_btn);

    // NO → cancel
    lv_obj_add_event_cb(btn_no, [](lv_event_t*){
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

void show_countdown_dialog(int duration_sec, lv_obj_t* parent)
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
    lv_label_set_text_fmt(countdown_label, "%d s", duration_sec);
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
    lv_obj_align(arc_progress, LV_ALIGN_LEFT_MID, 40, 0);

    lbl_percent = lv_label_create(parent);
    lv_label_set_text(lbl_percent, "0%");
    lv_obj_set_style_text_font(lbl_percent, &montserrat_48, 0);
    lv_obj_align_to(lbl_percent, arc_progress, LV_ALIGN_CENTER, 5, 0);

    /* ===== Syringe Icon (暫用符號) ===== */
    // lv_obj_t* syringe = lv_label_create(parent);
    // lv_label_set_text(syringe, LV_SYMBOL_CHARGE); // 之後可換成圖片
    // lv_obj_set_style_text_font(syringe, &montserrat_48, 0);
    // lv_obj_align_to(syringe, arc_progress, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    /* ===== Info Text ===== */
    lv_coord_t x =330;
    lv_coord_t y = 50;

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
    lv_label_set_text(title, "User Manual for DEVELPOER INFO");
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
    lv_img_set_zoom(img, img_zoom);
    lv_obj_center(img);

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

static void set_tab3_control_buttons_enabled(bool enabled)
{
    lv_obj_t* buttons[2] = {
        btn_tab3_parameter_setting,
        btn_tab3_motor_control
    };

    for (int i = 0; i < 2; i++) {
        lv_obj_t* btn = buttons[i];
        if (!btn) continue;

        if (enabled) {
            lv_obj_clear_state(btn, LV_STATE_DISABLED);
            lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);

            lv_obj_set_style_bg_color(btn, lv_color_hex(0X80C000), 0);  // ✅ restore color
            lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
            lv_obj_set_style_border_opa(btn, LV_OPA_COVER, 0);

            lv_obj_t* img = lv_obj_get_child(btn, 0);
            if (img) {
                lv_obj_set_style_img_opa(img, LV_OPA_COVER, 0);
            }
        }
        else {
            lv_obj_add_state(btn, LV_STATE_DISABLED);
            lv_obj_clear_flag(btn, LV_OBJ_FLAG_CLICKABLE);

            lv_obj_set_style_bg_color(btn, lv_color_hex(0xC8C8C8), 0);
            lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
            lv_obj_set_style_border_opa(btn, LV_OPA_COVER, 0);

            lv_obj_t* img = lv_obj_get_child(btn, 0);
            if (img) {
                lv_obj_set_style_img_opa(img, LV_OPA_50, 0);
            }
        }
    }
}

void update_system_progress_data()
{
    if (!is_running) return;

    unsigned long elapsed_ms = millis() - run_start_ms;

    float total_sec = recorded_duration_sec_exact;
    float elapsed_sec_f = elapsed_ms / 1000.0f;

    float remaining_sec_f = total_sec - elapsed_sec_f;
    if (remaining_sec_f < 0.0f)
        remaining_sec_f = 0.0f;

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
    unsigned long total_ms =
        (unsigned long)(recorded_duration_sec_exact * 1000.0f + 0.5f);

    unsigned long dur_int  = total_ms / 1000;
    unsigned long dur_frac = (total_ms % 1000) / 10;

    lv_label_set_text_fmt(
        lbl_duration,
        "Duration (s): %lu.%02lu",
        dur_int,
        dur_frac
    );

    // Remaining time
    // 轉成 xx.xx
    unsigned long remaining_ms = (unsigned long)(remaining_sec_f * 1000.0f + 0.5f);
    unsigned long sec_int = remaining_ms / 1000;
    unsigned long sec_frac = (remaining_ms % 1000) / 10;

    lv_label_set_text_fmt(
        lbl_remaining,
        "Remaining Time (s): \n%lu.%02lu",
        sec_int,
        sec_frac
    );

    // Elapsed time
    unsigned long elapsed_int  = elapsed_ms / 1000;
    unsigned long elapsed_frac = (elapsed_ms % 1000) / 10;

    lv_label_set_text_fmt(
        lbl_elapsed,
        "Elapsed Time (s): %lu.%02lu",
        elapsed_int,
        elapsed_frac
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

    if (!build_realtime_motion_planner_from_current_settings()) {
        return false;
    }

    recorded_duration_sec_exact = realtime_motion_planner.totalDuration();

    recorded_duration_sec = (int)(recorded_duration_sec_exact + 0.5f);
    if (recorded_duration_sec < 1) {
        recorded_duration_sec = 1;
    }

    recorded_duration_valid = true;
    return true;
}

static bool build_realtime_motion_planner_from_current_settings()
{
    realtime_motion_planner_valid = false;
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
    flow_params.S = CustomizedParametersScreen::getSavedFloat(CustomizedParametersScreen::DEV_P_S);
    flow_params.r = CustomizedParametersScreen::getSavedFloat(CustomizedParametersScreen::DEV_P_r);
    flow_params.l = CustomizedParametersScreen::getSavedFloat(CustomizedParametersScreen::DEV_P_l);
    flow_params.lead_pitch = CustomizedParametersScreen::getSavedFloat(CustomizedParametersScreen::DEV_P_LEAD_PITCH);
    flow_params.I_limit = CustomizedParametersScreen::getSavedFloat(CustomizedParametersScreen::DEV_P_I_LIMIT);
    flow_params.stroke_per_ml_mm = CustomizedParametersScreen::getSavedFloat(CustomizedParametersScreen::DEV_P_STROKE_PER_ML_MM);
    flow_params.max_linear_speed_mm_s = CustomizedParametersScreen::getSavedFloat(CustomizedParametersScreen::DEV_P_MAX_LINEAR_SPEED_MM_S);
    flow_params.I_plunger = CustomizedParametersScreen::getSavedFloat(CustomizedParametersScreen::DEV_P_I_PLUNGER);

    // fixed values for now
    flow_params.eta = 0.30f;
    flow_params.Kt = 0.45f;
    flow_params.max_rpm = 300.0f;
    flow_params.steps_per_rev = 3200.0f;
    flow_params.max_step_freq = 50000.0f;
    flow_params.beta = 0.30f;
    flow_params.K_buckling = 0.5f;
    flow_params.I_plunger = 0.0f;

    FlowConstraintModel::Result flow_result = FlowConstraintModel::compute(flow_params);
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

    realtime_curve_volume_max_uL = motion_cmd.volume;
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

    realtime_curve_current_max_A = peak_current_A;

    return true;
}

static bool build_realtime_spi_streamer_from_current_settings()
{
    realtime_streamer_valid = false;

    HardwareConfig hw_cfg;
    KinematicsConfig kin_cfg;
    DriverConfig drv_cfg;
    TrackingConfig track_cfg;
    InaConfig ina_cfg;

    // ===== 硬體設定 =====
    hw_cfg.pin_oe = 4;
    hw_cfg.enable_hardware_pin = 7;
    hw_cfg.chip_select_pin = 10;
    hw_cfg.ina228_addr = 0x40;
    hw_cfg.use_spi1 = true;
    hw_cfg.spi_clock_hz = 5000000;
    hw_cfg.use_wire1_for_ina = true;

    // ===== 運動學 / 校正 =====
    kin_cfg.microsteps_per_mm = 3507.0f;       // ← 這裡填你目前校正值
    kin_cfg.planner_meter_to_real_unit = 1000.0f; // planner輸出m，driver吃mm

    // ===== Driver參數 =====
    drv_cfg.run_current_percent = 80;
    drv_cfg.pwm_offset_percent = 40;
    drv_cfg.pwm_gradient_percent = 15;
    drv_cfg.reverse_direction = false;
    drv_cfg.stealth_chop_threshold = 50;

    const float vmax_mm_s = CustomizedParametersScreen::getSavedFloat(CustomizedParametersScreen::DEV_P_MAX_LINEAR_SPEED_MM_S);

    // ===== Tracking參數 =====
    // 這組不是擠出7-segment本身，而是讓TMC追得上streaming target
    track_cfg.max_velocity = vmax_mm_s;
    track_cfg.max_acceleration = 5.0f;
    track_cfg.start_velocity = 0.5f;
    track_cfg.stop_velocity = 0.5f;
    track_cfg.first_velocity = 1.0f;
    track_cfg.first_acceleration = 2.0f;
    track_cfg.max_deceleration = 1.0f;
    track_cfg.first_deceleration = 2.0f;
    track_cfg.stream_period_ms = 10;
    track_cfg.force_final_target = true;

    // ===== INA228 =====
    ina_cfg.shunt_ohms = 0.02f;
    ina_cfg.max_current_a = 5.0f;
    ina_cfg.averaging = INA228_COUNT_16;
    ina_cfg.voltage_conv = INA228_TIME_150_us;
    ina_cfg.current_conv = INA228_TIME_280_us;

    if (!realtime_streamer.begin(hw_cfg, kin_cfg, drv_cfg, track_cfg, ina_cfg)) {
        return false;
    }

    if (!realtime_streamer.attachPlanner(&realtime_motion_planner)) {
        return false;
    }

    realtime_streamer_valid = true;
    return true;
}

void setup() {
  Display.begin();
  TouchDetector.begin();

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
      45,                    // 圖片縮放
      MotorControlScreen::getScreenHandle(),            // 要跳轉的 screen
      MotorControlScreen::build        // 若尚未建立，先呼叫 builder
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
}

void loop(){
    lv_timer_handler();

    // --- auto stop after duration ---
    if (is_running) {
        bool finished_by_streamer = false;

        if (realtime_streamer_valid) {
            finished_by_streamer = realtime_streamer.isFinished();
        }

        unsigned long elapsed_sec = (millis() - run_start_ms) / 1000;
        bool finished_by_time = (!realtime_streamer_valid && elapsed_sec >= recorded_duration_sec);

        if (finished_by_streamer || finished_by_time) {
            if (realtime_motion_planner_valid && RealtimeCurveRenderer::isInitialized()) {
                float T = recorded_duration_sec_exact;
                PlungerMotionPlanner::Sample sT = realtime_motion_planner.evaluate(T);

                float real_current_A = 0.0f;
                if (realtime_streamer_valid) {
                    real_current_A = realtime_streamer.getStatus().measured_current_mA / 1000.0f;
                }

                RealtimeCurveRenderer::pushSample(
                    T,
                    sT.volume,
                    real_current_A
                );
            }

            is_running = false;
            set_tab3_control_buttons_enabled(true);

            if (realtime_streamer_valid && realtime_streamer.isRunning()) {
                realtime_streamer.stop(true);
            }

            if (RealtimeCurveRenderer::isInitialized()) {
                RealtimeCurveRenderer::releaseTempData();
            }

            realtime_motion_planner_valid = false;
            realtime_streamer_valid = false;

            if (start_btn) {
                lv_obj_t* label = lv_obj_get_child(start_btn, 0);
                lv_label_set_text(label, "START");
                lv_obj_set_style_bg_color(start_btn, lv_color_hex(0x0000FF), 0);
            }

            lv_arc_set_value(arc_progress, 100);
            lv_label_set_text(lbl_percent, "100%");
            lv_obj_align_to(lbl_percent, arc_progress, LV_ALIGN_CENTER, 0, 0);

            clear_countdown_dialog();
            show_finish_dialog();
        }
    }

    // --- countdown update ---
    if (is_running && countdown_dialog && countdown_label) {
        if (millis() - last_countdown_update >= 50) {
            last_countdown_update = millis();

            unsigned long elapsed_sec =
                (millis() - run_start_ms) / 1000;

            int remaining =
                recorded_duration_sec - elapsed_sec;

            if (remaining > 0) {
                unsigned long elapsed_ms = millis() - run_start_ms;

                float total_sec = recorded_duration_sec_exact;
                float elapsed_sec_f = elapsed_ms / 1000.0f;

                float remaining_sec_f = total_sec - elapsed_sec_f;
                if (remaining_sec_f < 0.0f) remaining_sec_f = 0.0f;

                unsigned long remaining_ms = (unsigned long)(remaining_sec_f * 1000.0f + 0.5f);
                unsigned long sec_int = remaining_ms / 1000;
                unsigned long sec_frac = (remaining_ms % 1000) / 10;

                lv_label_set_text_fmt(
                    countdown_label,
                    "%lu.%02lu s",
                    sec_int,
                    sec_frac
                );
            }
        }
    }

    // --- System Progress update ---
    if (is_running) {
        unsigned long elapsed_ms = (millis() - run_start_ms);
        unsigned long total_ms = (unsigned long)(recorded_duration_sec_exact * 1000.0f);

        if (elapsed_ms > total_ms)
         elapsed_ms = total_ms;
        
        int percent = (int)((elapsed_ms * 100) / total_ms);

        lv_arc_set_value(arc_progress, percent);
        lv_label_set_text_fmt(lbl_percent, "%d%%", percent);
    }

    update_system_progress_data();

    if (realtime_streamer_valid) {
        realtime_streamer.update();
    }

    MotorControlScreen::update();

    // --- realtime theoretical curve update ---
    if (is_running && realtime_motion_planner_valid && RealtimeCurveRenderer::isInitialized()) {
        if (millis() - last_curve_update_ms >= 50) {
            last_curve_update_ms = millis();

            float t_s = (millis() - run_start_ms) / 1000.0f;
            if (t_s > recorded_duration_sec_exact) {
                t_s = recorded_duration_sec_exact;
            }

            PlungerMotionPlanner::Sample sample = realtime_motion_planner.evaluate(t_s);

            float current_A = 0.0f;
            if (realtime_streamer_valid) {
                current_A = realtime_streamer.getStatus().measured_current_mA / 1000.0f;
            }

            RealtimeCurveRenderer::pushSample(
                t_s,
                sample.volume,   // 保留理論 7-segment volume
                current_A        // 改為真實 INA228 電流
            );
        }
    }

    delay(5);
}