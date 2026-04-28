#include "CustomizedParametersScreen.h"
#include "fonts.h"

#include <cstring>
#include <cstdlib>
#include <cstdio>

namespace CustomizedParametersScreen {

namespace {

// ===== screen objects =====
static lv_obj_t* main_screen_obj = NULL;
static lv_obj_t* screen_obj = NULL;
static lv_obj_t* scroll_obj = NULL;
static lv_obj_t* kb_obj = NULL;
static lv_obj_t* kb_target_ta = NULL;
static lv_obj_t* default_confirm_dialog = NULL;

// ===== style =====
static lv_style_t st_cell;
static bool st_cell_inited = false;

// ===== row definition =====
typedef struct {
    lv_obj_t* ta;
    const char* key;
    const char* unit;
} ParamRow;

static ParamRow rows[DEV_P_COUNT];

// ===== store =====
static ParamValue param_store[DEV_P_COUNT] = {
    {"viscosity",   "mPa·s", "60000",   ""},

    {"stroke_per_ml_mm",      "mm",    "13.0",    ""},
    {"lead_pitch",  "m/rev", "0.0015",  ""},
    {"max_linear_speed_mm_s", "mm/s",  "8.0",    ""},
    {"I_limit",     "A",     "2.3",     ""},
    {"I_plunger",             "m^4",   "8.0927e-11", ""},

    {"R",           "m",     "0.0049",  ""},
    {"L",           "m",     "0.062",   ""},
    {"shaft",       "m",     "0.0093",  ""},
    {"shaft_walls", "m",     "0.0008",  ""},
    {"E",           "Pa",    "1e9",     ""},
    {"S",           "-",     "3.0",     ""},
    {"r",           "m",     "0.00013", ""},
    {"l",           "m",     "0.012",   ""},
    // {"volume",      "uL",    "1000",    ""},
};

// ===== keypad map =====
static const char * num_kb_map[] = {
    "1", "2", "3", "\n",
    "4", "5", "6", "\n",
    "7", "8", "9", "\n",
    "0", "00", ".", "\n",
    LV_SYMBOL_BACKSPACE, LV_SYMBOL_NEW_LINE, ""
};

// ===== helpers =====
static void initCellStyle()
{
    if (st_cell_inited) return;
    st_cell_inited = true;

    lv_style_init(&st_cell);
    lv_style_set_radius(&st_cell, 8);
    lv_style_set_border_width(&st_cell, 1);
    lv_style_set_border_color(&st_cell, lv_color_hex(0xD0D0D0));
    lv_style_set_bg_opa(&st_cell, LV_OPA_COVER);
    lv_style_set_bg_color(&st_cell, lv_color_hex(0xFFFFFF));
    lv_style_set_pad_all(&st_cell, 10);
}

static void kbEventCb(lv_event_t* e)
{
    lv_obj_t* kb = lv_event_get_target(e);
    const char* txt = lv_btnmatrix_get_btn_text(kb, lv_btnmatrix_get_selected_btn(kb));
    if (!txt || !kb_target_ta) return;

    if (strcmp(txt, LV_SYMBOL_BACKSPACE) == 0) {
        lv_textarea_del_char(kb_target_ta);
    }
    else if (strcmp(txt, LV_SYMBOL_NEW_LINE) == 0) {
        lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
        kb_target_ta = NULL;
    }
    else {
        lv_textarea_add_text(kb_target_ta, txt);
    }
}

static void attachKeyboardTo(lv_obj_t* ta)
{
    if (!kb_obj) return;
    kb_target_ta = ta;
    lv_obj_clear_flag(kb_obj, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(kb_obj);
}

static void taFocusCb(lv_event_t* e)
{
    lv_obj_t* ta = lv_event_get_target(e);
    attachKeyboardTo(ta);
}

static void createKeyboard(lv_obj_t* parent)
{
    if (kb_obj) return;

    kb_obj = lv_btnmatrix_create(parent);
    lv_obj_set_size(kb_obj, 240, 260);
    lv_obj_align(kb_obj, LV_ALIGN_BOTTOM_RIGHT, -20, -20);
    lv_btnmatrix_set_map(kb_obj, num_kb_map);
    lv_obj_add_flag(kb_obj, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(kb_obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(kb_obj, kbEventCb, LV_EVENT_VALUE_CHANGED, NULL);
}

static lv_obj_t* createTableRow(
    lv_obj_t* parent,
    const char* line1,
    const char* line2,
    const char* placeholder,
    lv_obj_t** out_ta
){
    initCellStyle();

    lv_obj_t* row = lv_obj_create(parent);
    lv_obj_remove_style_all(row);
    lv_obj_set_size(row, 720, 68);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(row, 24, 0);

    lv_obj_t* left = lv_obj_create(row);
    lv_obj_remove_style_all(left);
    lv_obj_add_style(left, &st_cell, 0);
    lv_obj_set_size(left, 340, 68);
    lv_obj_clear_flag(left, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* text = lv_label_create(left);

    char left_buf[128];
    if (line1 && line2 && strlen(line2) > 0) {
        snprintf(left_buf, sizeof(left_buf), "%s\n%s", line1, line2);
    } else if (line1) {
        snprintf(left_buf, sizeof(left_buf), "%s", line1);
    } else {
        left_buf[0] = '\0';
    }

    lv_label_set_text(text, left_buf);
    lv_obj_set_style_text_font(text, &montserrat_20, 0);
    lv_label_set_long_mode(text, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(text, 320);
    lv_obj_center(text);

    lv_obj_t* ta = lv_textarea_create(row);
    lv_obj_remove_style_all(ta);
    lv_obj_add_style(ta, &st_cell, 0);
    lv_obj_set_size(ta, 340, 68);
    lv_textarea_set_one_line(ta, true);
    lv_textarea_set_placeholder_text(ta, placeholder ? placeholder : "");
    lv_obj_set_style_text_font(ta, &montserrat_28, 0);
    lv_obj_set_style_text_align(ta, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_set_scrollbar_mode(ta, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(ta, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_add_event_cb(ta, taFocusCb, LV_EVENT_FOCUSED, NULL);
    lv_obj_add_event_cb(ta, taFocusCb, LV_EVENT_CLICKED, NULL);

    if (out_ta) *out_ta = ta;
    return row;
}

static void backBtnEventCb(lv_event_t* e)
{
    LV_UNUSED(e);

    if (main_screen_obj) {
        lv_scr_load(main_screen_obj);
    }
    lv_async_call(destroyAsync, NULL);
}

static void saveBtnEventCb(lv_event_t* e)
{
    LV_UNUSED(e);
    saveParameters();
    showSaveDoneDialog();
}

} // anonymous namespace

void setMainScreen(lv_obj_t* screen)
{
    main_screen_obj = screen;
}

void initStore()
{
    for (int i = 0; i < DEV_P_COUNT; i++) {
        strncpy(param_store[i].saved_str,
                param_store[i].default_str,
                sizeof(param_store[i].saved_str) - 1);
        param_store[i].saved_str[sizeof(param_store[i].saved_str) - 1] = '\0';
    }
}

void build()
{
    if (screen_obj) return;

    screen_obj = lv_obj_create(NULL);
    lv_obj_clear_flag(screen_obj, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* title = lv_label_create(screen_obj);
    lv_label_set_text(title, "Customized Parameters");
    lv_obj_set_style_text_font(title, &montserrat_40, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

    scroll_obj = lv_obj_create(screen_obj);
    lv_obj_set_size(scroll_obj, 760, 320);
    lv_obj_align(scroll_obj, LV_ALIGN_TOP_MID, 0, 70);
    lv_obj_set_style_pad_all(scroll_obj, 10, 0);
    lv_obj_set_scroll_dir(scroll_obj, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(scroll_obj, LV_SCROLLBAR_MODE_ACTIVE);

    lv_obj_set_flex_flow(scroll_obj, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(scroll_obj,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_gap(scroll_obj, 10, 0);

    createKeyboard(screen_obj);

    // ===== build() rows =====
    createTableRow(scroll_obj, "viscosity", "Fluid dynamic viscosity", "e.g. 60000", &rows[DEV_P_VISCOSITY].ta);
    rows[DEV_P_VISCOSITY].key = "viscosity";
    rows[DEV_P_VISCOSITY].unit = "mPa·s";

    createTableRow(scroll_obj, "stroke / 1 mL", "Linear stroke needed for 1 mL", "e.g. 13.0", &rows[DEV_P_STROKE_PER_ML_MM].ta);
    rows[DEV_P_STROKE_PER_ML_MM].key = "stroke_per_ml_mm";
    rows[DEV_P_STROKE_PER_ML_MM].unit = "mm";

    createTableRow(scroll_obj, "lead pitch (m)", "Lead screw lead", "e.g. 0.0015", &rows[DEV_P_LEAD_PITCH].ta);
    rows[DEV_P_LEAD_PITCH].key = "lead_pitch";
    rows[DEV_P_LEAD_PITCH].unit = "m/rev";

    createTableRow(scroll_obj, "max linear speed (mm/s)", "Maximum linear motion speed", "e.g. 8.0", &rows[DEV_P_MAX_LINEAR_SPEED_MM_S].ta);
    rows[DEV_P_MAX_LINEAR_SPEED_MM_S].key = "max_linear_speed_mm_s";
    rows[DEV_P_MAX_LINEAR_SPEED_MM_S].unit = "mm/s";

    createTableRow(scroll_obj, "I_limit (A)", "Stepper phase current limit", "e.g. 2.3", &rows[DEV_P_I_LIMIT].ta);
    rows[DEV_P_I_LIMIT].key = "I_limit";
    rows[DEV_P_I_LIMIT].unit = "A";

    createTableRow(scroll_obj, "I_plunger", "Second moment of area", "e.g. 8.0927e-11", &rows[DEV_P_I_PLUNGER].ta);
    rows[DEV_P_I_PLUNGER].key = "I_plunger";
    rows[DEV_P_I_PLUNGER].unit = "m^4";

    // remaining original order
    createTableRow(scroll_obj, "R", "Plunger tip radius", "e.g. 0.0049", &rows[DEV_P_R].ta);
    rows[DEV_P_R].key = "R";
    rows[DEV_P_R].unit = "m";

    createTableRow(scroll_obj, "L", "Plunger length", "e.g. 0.062", &rows[DEV_P_L].ta);
    rows[DEV_P_L].key = "L";
    rows[DEV_P_L].unit = "m";

    createTableRow(scroll_obj, "shaft", "Plunger shaft outer width", "e.g. 0.0093", &rows[DEV_P_SHAFT].ta);
    rows[DEV_P_SHAFT].key = "shaft";
    rows[DEV_P_SHAFT].unit = "m";

    createTableRow(scroll_obj, "shaft wall", "Shaft wall thickness", "e.g. 0.0008", &rows[DEV_P_SHAFT_WALL].ta);
    rows[DEV_P_SHAFT_WALL].key = "shaft_walls";
    rows[DEV_P_SHAFT_WALL].unit = "m";

    createTableRow(scroll_obj, "E", "Elastic modulus", "e.g. 1e9", &rows[DEV_P_E].ta);
    rows[DEV_P_E].key = "E";
    rows[DEV_P_E].unit = "Pa";

    createTableRow(scroll_obj, "S", "Safety factor", "e.g. 3.0", &rows[DEV_P_S].ta);
    rows[DEV_P_S].key = "S";
    rows[DEV_P_S].unit = "-";

    createTableRow(scroll_obj, "r (cannula)", "Cannula inner radius", "e.g. 0.00013", &rows[DEV_P_r].ta);
    rows[DEV_P_r].key = "r";
    rows[DEV_P_r].unit = "m";

    createTableRow(scroll_obj, "l (cannula)", "Cannula length", "e.g. 0.012", &rows[DEV_P_l].ta);
    rows[DEV_P_l].key = "l";
    rows[DEV_P_l].unit = "m";

    for (int i = 0; i < DEV_P_COUNT; i++) {
        if (rows[i].ta) {
            lv_textarea_set_text(rows[i].ta, param_store[i].saved_str);
        }
    }

    lv_obj_t* btn_back = lv_btn_create(screen_obj);
    lv_obj_set_size(btn_back, 160, 60);
    lv_obj_set_style_bg_color(btn_back, lv_palette_main(LV_PALETTE_GREY), LV_PART_MAIN);
    lv_obj_align(btn_back, LV_ALIGN_BOTTOM_LEFT, 20, -20);
    lv_obj_add_event_cb(btn_back, backBtnEventCb, LV_EVENT_CLICKED, NULL);

    lv_obj_t* lbl_back = lv_label_create(btn_back);
    lv_label_set_text(lbl_back, "BACK");
    lv_obj_set_style_text_font(lbl_back, &montserrat_24, 0);
    lv_obj_center(lbl_back);

    lv_obj_t* btn_save = lv_btn_create(screen_obj);
    lv_obj_set_size(btn_save, 160, 60);
    lv_obj_align(btn_save, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_obj_add_event_cb(btn_save, saveBtnEventCb, LV_EVENT_CLICKED, NULL);

    lv_obj_t* lbl_save = lv_label_create(btn_save);
    lv_label_set_text(lbl_save, "SAVE");
    lv_obj_set_style_text_font(lbl_save, &montserrat_24, 0);
    lv_obj_center(lbl_save);

    lv_obj_t* btn_default = lv_btn_create(screen_obj);
    lv_obj_set_size(btn_default, 160, 60);
    lv_obj_align(btn_default, LV_ALIGN_BOTTOM_RIGHT, -20, -20);
    lv_obj_add_event_cb(btn_default, [](lv_event_t* e){
        LV_UNUSED(e);
        showDefaultConfirmDialog();
    }, LV_EVENT_CLICKED, NULL);

    lv_obj_t* lbl_default = lv_label_create(btn_default);
    lv_label_set_text(lbl_default, "DEFAULT");
    lv_obj_set_style_text_font(lbl_default, &montserrat_24, 0);
    lv_obj_center(lbl_default);
}

lv_obj_t* getScreen()
{
    return screen_obj;
}

lv_obj_t** getScreenHandle()
{
    return &screen_obj;
}

void destroy()
{
    if (screen_obj) {
        lv_obj_del(screen_obj);
        screen_obj = NULL;
    }

    scroll_obj = NULL;
    kb_obj = NULL;
    kb_target_ta = NULL;
    default_confirm_dialog = NULL;

    for (int i = 0; i < DEV_P_COUNT; i++) {
        rows[i].ta = NULL;
        rows[i].key = NULL;
        rows[i].unit = NULL;
    }
}

void destroyAsync(void* user_data)
{
    LV_UNUSED(user_data);
    destroy();
}

void saveParameters()
{
    for (int i = 0; i < DEV_P_COUNT; i++) {
        if (!rows[i].ta) continue;

        const char* txt = lv_textarea_get_text(rows[i].ta);
        if (txt && strlen(txt) > 0) {
            strncpy(param_store[i].saved_str,
                    txt,
                    sizeof(param_store[i].saved_str) - 1);
            param_store[i].saved_str[sizeof(param_store[i].saved_str) - 1] = '\0';
        }
    }
}

void resetParametersToDefault()
{
    for (int i = 0; i < DEV_P_COUNT; i++) {
        strncpy(param_store[i].saved_str,
                param_store[i].default_str,
                sizeof(param_store[i].saved_str) - 1);
        param_store[i].saved_str[sizeof(param_store[i].saved_str) - 1] = '\0';

        if (rows[i].ta) {
            lv_textarea_set_text(rows[i].ta, param_store[i].saved_str);
        }
    }
}

const char* getSavedValue(int idx)
{
    if (idx < 0 || idx >= DEV_P_COUNT) return "";
    return param_store[idx].saved_str;
}

float getSavedFloat(int idx)
{
    if (idx < 0 || idx >= DEV_P_COUNT) return 0.0f;
    return atof(param_store[idx].saved_str);
}

ParamValue* getStore()
{
    return param_store;
}

void showSaveDoneDialog()
{
    lv_obj_t* dialog = lv_obj_create(lv_scr_act());
    lv_obj_set_size(dialog, 440, 220);
    lv_obj_center(dialog);
    lv_obj_set_style_radius(dialog, 12, 0);
    lv_obj_set_style_pad_all(dialog, 20, 0);
    lv_obj_set_style_bg_color(dialog, lv_palette_lighten(LV_PALETTE_GREY, 3), 0);

    lv_obj_t* label = lv_label_create(dialog);
    lv_label_set_text(label, "Parameters saved");
    lv_obj_set_style_text_font(label, &montserrat_34, 0);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 20);

    lv_obj_t* btn_ok = lv_btn_create(dialog);
    lv_obj_set_size(btn_ok, 120, 60);
    lv_obj_align(btn_ok, LV_ALIGN_BOTTOM_MID, 0, 0);

    lv_obj_t* lbl_ok = lv_label_create(btn_ok);
    lv_label_set_text(lbl_ok, "OK");
    lv_obj_set_style_text_font(lbl_ok, &montserrat_26, 0);
    lv_obj_center(lbl_ok);

    lv_obj_add_event_cb(btn_ok, [](lv_event_t* e){
        lv_obj_t* dialog = (lv_obj_t*)lv_event_get_user_data(e);
        lv_obj_del(dialog);
    }, LV_EVENT_CLICKED, dialog);
}

void showDefaultConfirmDialog()
{
    if (default_confirm_dialog) return;

    default_confirm_dialog = lv_obj_create(lv_scr_act());
    lv_obj_set_size(default_confirm_dialog, 500, 240);
    lv_obj_center(default_confirm_dialog);
    lv_obj_set_style_radius(default_confirm_dialog, 12, 0);
    lv_obj_set_style_bg_color(default_confirm_dialog, lv_palette_lighten(LV_PALETTE_GREY, 3), 0);
    lv_obj_set_style_pad_all(default_confirm_dialog, 20, 0);

    lv_obj_t* label = lv_label_create(default_confirm_dialog);
    lv_label_set_text(label, "Restore all parameters\nto default values?");
    lv_obj_set_style_text_font(label, &montserrat_30, 0);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(label, 420);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 20);

    lv_obj_t* btn_yes = lv_btn_create(default_confirm_dialog);
    lv_obj_set_size(btn_yes, 140, 55);
    lv_obj_align(btn_yes, LV_ALIGN_BOTTOM_LEFT, 40, -20);
    lv_obj_set_style_bg_color(btn_yes, lv_palette_main(LV_PALETTE_RED), 0);

    lv_obj_t* lbl_yes = lv_label_create(btn_yes);
    lv_label_set_text(lbl_yes, "YES");
    lv_obj_set_style_text_font(lbl_yes, &montserrat_24, 0);
    lv_obj_center(lbl_yes);

    lv_obj_t* btn_no = lv_btn_create(default_confirm_dialog);
    lv_obj_set_size(btn_no, 140, 55);
    lv_obj_align(btn_no, LV_ALIGN_BOTTOM_RIGHT, -40, -20);

    lv_obj_t* lbl_no = lv_label_create(btn_no);
    lv_label_set_text(lbl_no, "NO");
    lv_obj_set_style_text_font(lbl_no, &montserrat_24, 0);
    lv_obj_center(lbl_no);

    lv_obj_add_event_cb(btn_yes, [](lv_event_t* e){
        LV_UNUSED(e);
        resetParametersToDefault();

        lv_obj_del(default_confirm_dialog);
        default_confirm_dialog = NULL;

        showSaveDoneDialog();
    }, LV_EVENT_CLICKED, NULL);

    lv_obj_add_event_cb(btn_no, [](lv_event_t* e){
        LV_UNUSED(e);
        lv_obj_del(default_confirm_dialog);
        default_confirm_dialog = NULL;
    }, LV_EVENT_CLICKED, NULL);
}

} // namespace CustomizedParametersScreen