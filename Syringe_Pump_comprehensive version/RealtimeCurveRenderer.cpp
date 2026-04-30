#include "RealtimeCurveRenderer.h"
#include "fonts.h"

bool RealtimeCurveRenderer::s_initialized = false;

lv_obj_t* RealtimeCurveRenderer::s_volume_panel = NULL;
lv_obj_t* RealtimeCurveRenderer::s_current_panel = NULL;

lv_obj_t* RealtimeCurveRenderer::s_volume_chart = NULL;
lv_obj_t* RealtimeCurveRenderer::s_current_chart = NULL;

lv_chart_series_t* RealtimeCurveRenderer::s_volume_series = NULL;
lv_chart_series_t* RealtimeCurveRenderer::s_current_series = NULL;

RealtimeCurveRenderer::Config RealtimeCurveRenderer::s_cfg;

float RealtimeCurveRenderer::s_last_t_s = 0.0f;
uint16_t RealtimeCurveRenderer::s_written_points = 0;

uint16_t RealtimeCurveRenderer::s_last_written_index = 0;
bool* RealtimeCurveRenderer::s_index_written = NULL;
uint16_t RealtimeCurveRenderer::s_allocated_point_count = 0;

lv_obj_t* RealtimeCurveRenderer::s_volume_xlabel = NULL;
lv_obj_t* RealtimeCurveRenderer::s_volume_ylabel = NULL;
lv_obj_t* RealtimeCurveRenderer::s_volume_x0_label = NULL;
lv_obj_t* RealtimeCurveRenderer::s_volume_x1_label = NULL;
lv_obj_t* RealtimeCurveRenderer::s_volume_y0_label = NULL;
lv_obj_t* RealtimeCurveRenderer::s_volume_y1_label = NULL;

lv_obj_t* RealtimeCurveRenderer::s_current_xlabel = NULL;
lv_obj_t* RealtimeCurveRenderer::s_current_ylabel = NULL;
lv_obj_t* RealtimeCurveRenderer::s_current_x0_label = NULL;
lv_obj_t* RealtimeCurveRenderer::s_current_x1_label = NULL;
lv_obj_t* RealtimeCurveRenderer::s_current_y0_label = NULL;
lv_obj_t* RealtimeCurveRenderer::s_current_y1_label = NULL;

void RealtimeCurveRenderer::freeBuffers()
{
    if (s_index_written) {
        delete[] s_index_written;
        s_index_written = NULL;
    }
    s_allocated_point_count = 0;
}

bool RealtimeCurveRenderer::allocateBuffers(uint16_t point_count)
{
    freeBuffers();

    if (point_count < 2) return false;

    s_index_written = new bool[point_count];
    if (!s_index_written) {
        s_allocated_point_count = 0;
        return false;
    }

    for (uint16_t i = 0; i < point_count; i++) {
        s_index_written[i] = false;
    }

    s_allocated_point_count = point_count;
    return true;
}

uint16_t RealtimeCurveRenderer::calculatePointCount(const Config& cfg)
{
    const uint16_t MIN_POINTS = 60;
    const uint16_t MAX_POINTS = 300;

    if (cfg.point_count >= 2) {
        return cfg.point_count;
    }

    // ⭐ 動態取得 chart width（更穩）
    uint16_t width_px = 270;
    if (s_volume_chart) {
        width_px = lv_obj_get_width(s_volume_chart);
    }

    // ⭐ 留出 rendering margin（不要貼滿）
    float fill_ratio = 0.85f;

    uint16_t count = (uint16_t)(width_px * fill_ratio);

    // ⭐ 根據 duration 做最低解析度保護
    if (cfg.expected_duration_s > 0.0f) {
        float min_dt = 0.05f; // 50 ms resolution
        uint16_t duration_based = (uint16_t)(cfg.expected_duration_s / min_dt);

        if (duration_based < count) {
            count = duration_based;
        }
    }

    // clamp
    if (count < MIN_POINTS) count = MIN_POINTS;
    if (count > MAX_POINTS) count = MAX_POINTS;

    return count;
}

bool RealtimeCurveRenderer::isInitialized()
{
    return s_initialized;
}

lv_coord_t RealtimeCurveRenderer::clampCoord(int32_t v)
{
    if (v < 0) return 0;
    if (v > 1000) return 1000;
    return (lv_coord_t)v;
}

lv_coord_t RealtimeCurveRenderer::mapToChartY(float value, float y_max)
{
    if (y_max <= 0.0f) return 0;
    if (value < 0.0f) value = 0.0f;
    if (value > y_max) value = y_max;

    float ratio = value / y_max;
    int32_t mapped = (int32_t)(ratio * 1000.0f + 0.5f);
    return clampCoord(mapped);
}

void RealtimeCurveRenderer::createVolumeChart()
{
    if (!s_volume_panel) return;

    // X/Y axis labels
    // s_volume_ylabel = lv_label_create(s_volume_panel);
    // lv_label_set_text(s_volume_ylabel, "Volume");
    // lv_obj_set_style_text_font(s_volume_ylabel, &montserrat_18, 0);
    // lv_obj_align(s_volume_ylabel, LV_ALIGN_TOP_LEFT, 6, 26);

    s_volume_xlabel = lv_label_create(s_volume_panel);
    lv_label_set_text(s_volume_xlabel, "Time (s)");
    lv_obj_set_style_text_font(s_volume_xlabel, &montserrat_18, 0);
    lv_obj_align(s_volume_xlabel, LV_ALIGN_BOTTOM_MID, 0, 5);

    // axis end labels
    s_volume_y1_label = lv_label_create(s_volume_panel);
    lv_label_set_text(s_volume_y1_label, "0");
    lv_obj_set_style_text_font(s_volume_y1_label, &montserrat_18, 0);
    lv_obj_align(s_volume_y1_label, LV_ALIGN_TOP_LEFT, -8, 20);

    s_volume_y0_label = lv_label_create(s_volume_panel);
    lv_label_set_text(s_volume_y0_label, "0");
    lv_obj_set_style_text_font(s_volume_y0_label, &montserrat_18, 0);
    lv_obj_align(s_volume_y0_label, LV_ALIGN_BOTTOM_LEFT, 15, -25);

    s_volume_x0_label = lv_label_create(s_volume_panel);
    lv_label_set_text(s_volume_x0_label, "0");
    lv_obj_set_style_text_font(s_volume_x0_label, &montserrat_18, 0);
    lv_obj_align(s_volume_x0_label, LV_ALIGN_BOTTOM_LEFT, 38, -1);

    s_volume_x1_label = lv_label_create(s_volume_panel);
    lv_label_set_text(s_volume_x1_label, "0");
    lv_obj_set_style_text_font(s_volume_x1_label, &montserrat_18, 0);
    lv_obj_align(s_volume_x1_label, LV_ALIGN_BOTTOM_RIGHT, -5, -1);

    s_volume_chart = lv_chart_create(s_volume_panel);
    lv_obj_set_size(s_volume_chart,  270, 170);
    lv_obj_align(s_volume_chart, LV_ALIGN_BOTTOM_RIGHT, 0, -22);

    lv_chart_set_type(s_volume_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_range(s_volume_chart, LV_CHART_AXIS_PRIMARY_Y, 0, 1000);
    lv_chart_set_point_count(s_volume_chart, s_cfg.point_count);
    lv_chart_set_update_mode(s_volume_chart, LV_CHART_UPDATE_MODE_SHIFT);

    lv_obj_set_style_line_width(s_volume_chart, 2, LV_PART_ITEMS);
    lv_obj_set_style_size(s_volume_chart, 0, LV_PART_INDICATOR);

    s_volume_series = lv_chart_add_series(
        s_volume_chart,
        lv_palette_main(LV_PALETTE_BLUE),
        LV_CHART_AXIS_PRIMARY_Y
    );

    for (uint16_t i = 0; i < s_cfg.point_count; i++) {
        s_volume_series->y_points[i] = LV_CHART_POINT_NONE;
    }

    lv_chart_set_x_start_point(s_volume_chart, s_volume_series, 0);
    lv_chart_refresh(s_volume_chart);
}

void RealtimeCurveRenderer::createCurrentChart()
{
    if (!s_current_panel) return;

    // s_current_ylabel = lv_label_create(s_current_panel);
    // lv_label_set_text(s_current_ylabel, "Current");
    // lv_obj_set_style_text_font(s_current_ylabel, &montserrat_18, 0);
    // lv_obj_align(s_current_ylabel, LV_ALIGN_TOP_LEFT, 6, 26);

    s_current_xlabel = lv_label_create(s_current_panel);
    lv_label_set_text(s_current_xlabel, "Time (s)");
    lv_obj_set_style_text_font(s_current_xlabel, &montserrat_18, 0);
    lv_obj_align(s_current_xlabel, LV_ALIGN_BOTTOM_MID, 0, 5);

    s_current_y1_label = lv_label_create(s_current_panel);
    lv_label_set_text(s_current_y1_label, "0");
    lv_obj_set_style_text_font(s_current_y1_label, &montserrat_18, 0);
    lv_obj_align(s_current_y1_label, LV_ALIGN_TOP_LEFT, -8, 20);

    s_current_y0_label = lv_label_create(s_current_panel);
    lv_label_set_text(s_current_y0_label, "0");
    lv_obj_set_style_text_font(s_current_y0_label, &montserrat_18, 0);
    lv_obj_align(s_current_y0_label, LV_ALIGN_BOTTOM_LEFT, 15, -25);

    s_current_x0_label = lv_label_create(s_current_panel);
    lv_label_set_text(s_current_x0_label, "0");
    lv_obj_set_style_text_font(s_current_x0_label, &montserrat_18, 0);
    lv_obj_align(s_current_x0_label, LV_ALIGN_BOTTOM_LEFT, 38, -1);

    s_current_x1_label = lv_label_create(s_current_panel);
    lv_label_set_text(s_current_x1_label, "0");
    lv_obj_set_style_text_font(s_current_x1_label, &montserrat_18, 0);
    lv_obj_align(s_current_x1_label, LV_ALIGN_BOTTOM_RIGHT, -5, -1);

    s_current_chart = lv_chart_create(s_current_panel);
    lv_obj_set_size(s_current_chart, 270, 170);
    lv_obj_align(s_current_chart, LV_ALIGN_BOTTOM_RIGHT, 0, -22);

    lv_chart_set_type(s_current_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_range(s_current_chart, LV_CHART_AXIS_PRIMARY_Y, 0, 1000);
    lv_chart_set_point_count(s_current_chart, s_cfg.point_count);
    lv_chart_set_update_mode(s_current_chart, LV_CHART_UPDATE_MODE_SHIFT);

    lv_obj_set_style_line_width(s_current_chart, 2, LV_PART_ITEMS);
    lv_obj_set_style_size(s_current_chart, 0, LV_PART_INDICATOR);

    s_current_series = lv_chart_add_series(
        s_current_chart,
        lv_palette_main(LV_PALETTE_RED),
        LV_CHART_AXIS_PRIMARY_Y
    );

    for (uint16_t i = 0; i < s_cfg.point_count; i++) {
        s_current_series->y_points[i] = LV_CHART_POINT_NONE;
    }

    lv_chart_set_x_start_point(s_current_chart, s_current_series, 0);
    lv_chart_refresh(s_current_chart);
}

void RealtimeCurveRenderer::init(
    lv_obj_t* volume_panel,
    lv_obj_t* current_panel,
    const Config& cfg
)
{
    s_volume_panel = volume_panel;
    s_current_panel = current_panel;
    s_cfg = cfg;

    uint16_t point_count = calculatePointCount(s_cfg);
    s_cfg.point_count = point_count;

    s_last_t_s = 0.0f;
    s_last_written_index = 0;
    s_written_points = 0;

    if (!allocateBuffers(point_count)) {
        s_initialized = false;
        return;
    }

    createVolumeChart();
    createCurrentChart();

    s_initialized = (s_volume_chart != NULL && s_current_chart != NULL);
}

void RealtimeCurveRenderer::reset(
    const Config& cfg
)
{
    s_cfg = cfg;

    uint16_t point_count = calculatePointCount(s_cfg);
    s_cfg.point_count = point_count;

    s_last_t_s = 0.0f;
    s_last_written_index = 0;
    s_written_points = 0;

    if (!allocateBuffers(point_count)) {
        s_initialized = false;
        return;
    }

    if (s_volume_chart) {
        lv_chart_set_point_count(s_volume_chart, s_cfg.point_count);
        lv_chart_set_range(s_volume_chart, LV_CHART_AXIS_PRIMARY_Y, 0, 1000);
    }

    if (s_current_chart) {
        lv_chart_set_point_count(s_current_chart, s_cfg.point_count);
        lv_chart_set_range(s_current_chart, LV_CHART_AXIS_PRIMARY_Y, 0, 1000);
    }

    if (s_volume_series) {
        for (uint16_t i = 0; i < s_cfg.point_count; i++) {
            s_volume_series->y_points[i] = LV_CHART_POINT_NONE;
        }
        lv_chart_refresh(s_volume_chart);
    }

    if (s_current_series) {
        for (uint16_t i = 0; i < s_cfg.point_count; i++) {
            s_current_series->y_points[i] = LV_CHART_POINT_NONE;
        }
        lv_chart_refresh(s_current_chart);
    }

    s_initialized = true;

    char buf[32];
    // volume chart labels
    if (s_volume_x0_label) lv_label_set_text(s_volume_x0_label, "0");

    if (s_volume_x1_label) {
        unsigned long t_ms = (unsigned long)(s_cfg.expected_duration_s * 1000.0f + 0.5f);
        unsigned long t_int = t_ms / 1000;
        unsigned long t_frac = (t_ms % 1000) / 10;
        snprintf(buf, sizeof(buf), "%lu.%02lu", t_int, t_frac);
        lv_label_set_text(s_volume_x1_label, buf);
    }

    if (s_volume_y0_label) lv_label_set_text(s_volume_y0_label, "0");

    if (s_volume_y1_label) {
        float volume_ml = s_cfg.volume_max_uL / 1000.0f;
        unsigned long y_ms = (unsigned long)(volume_ml * 100.0f + 0.5f);
        unsigned long y_int  = y_ms / 100;
        unsigned long y_frac = y_ms % 100;
        snprintf(buf, sizeof(buf), "%lu.%02lu", y_int, y_frac);
        lv_label_set_text(s_volume_y1_label, buf);
    }

    // current chart labels
    if (s_current_x0_label) lv_label_set_text(s_current_x0_label, "0");

    if (s_current_x1_label) {
        unsigned long t_ms = (unsigned long)(s_cfg.expected_duration_s * 1000.0f + 0.5f);
        unsigned long t_int = t_ms / 1000;
        unsigned long t_frac = (t_ms % 1000) / 10;
        snprintf(buf, sizeof(buf), "%lu.%02lu", t_int, t_frac);
        lv_label_set_text(s_current_x1_label, buf);
    }

    if (s_current_y0_label) lv_label_set_text(s_current_y0_label, "0");

    if (s_current_y1_label) {
        unsigned long y_ms = (unsigned long)(s_cfg.current_max_A * 100.0f + 0.5f);
        unsigned long y_int = y_ms / 100;
        unsigned long y_frac = y_ms % 100;
        snprintf(buf, sizeof(buf), "%lu.%02lu", y_int, y_frac);
        lv_label_set_text(s_current_y1_label, buf);
    }
}

void RealtimeCurveRenderer::pushSample(
    float t_s,
    float volume_uL,
    float current_A
)
{
    if (!s_initialized) return;
    if (!s_volume_chart || !s_current_chart) return;
    if (!s_volume_series || !s_current_series) return;
    if (s_cfg.point_count < 2) return;
    if (s_cfg.expected_duration_s <= 0.0f) return;

    if (t_s < 0.0f) t_s = 0.0f;
    if (t_s > s_cfg.expected_duration_s) t_s = s_cfg.expected_duration_s;

    float ratio = t_s / s_cfg.expected_duration_s;
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;

    uint16_t index = (uint16_t)(ratio * (float)(s_cfg.point_count - 1) + 0.5f);
    if (index >= s_cfg.point_count) {
        index = s_cfg.point_count - 1;
    }

    lv_coord_t y_volume  = mapToChartY(volume_uL, s_cfg.volume_max_uL);
    lv_coord_t y_current = mapToChartY(current_A,  s_cfg.current_max_A);

    if (s_written_points == 0) {
        // first point
        s_volume_series->y_points[index]  = y_volume;
        s_current_series->y_points[index] = y_current;
        s_index_written[index] = true;

        s_last_written_index = index;
        s_written_points = 1;

        lv_chart_refresh(s_volume_chart);
        lv_chart_refresh(s_current_chart);
        return;
    }

    if (index > s_last_written_index) {
        // fill only NEW indices
        for (uint16_t i = s_last_written_index + 1; i <= index; i++) {
            s_volume_series->y_points[i]  = y_volume;
            s_current_series->y_points[i] = y_current;
            s_index_written[i] = true;
        }
        s_last_written_index = index;
        s_written_points++;
    }
    else if (index == s_last_written_index) {
        // do nothing: avoid overwriting the same point repeatedly
        return;
    }
    else {
        // unexpected backward time: optional ignore or overwrite only this index
        s_volume_series->y_points[index]  = y_volume;
        s_current_series->y_points[index] = y_current;
        s_index_written[index] = true;
    }

    lv_chart_refresh(s_volume_chart);
    lv_chart_refresh(s_current_chart);
}

void RealtimeCurveRenderer::releaseTempData()
{
    // 保留 chart 與 series，讓畫面上的曲線繼續存在
    // 只釋放計算/更新過程中的暫存資料
    freeBuffers();

    s_last_t_s = 0.0f;
    s_written_points = 0;
    s_last_written_index = 0;
}
