#ifndef REALTIME_CURVE_RENDERER_H
#define REALTIME_CURVE_RENDERER_H

#include <Arduino.h>
#include "lvgl.h"

class RealtimeCurveRenderer {
public:
    struct Config {
        uint16_t point_count;        // optional direct override, 0 means auto-calc
        float expected_duration_s;   // expected total duration for axis scaling
        float volume_max_uL;         // y-axis upper hint for volume chart
        float current_max_A;         // y-axis upper hint for current chart
        uint32_t sample_interval_ms; // plot one point every N ms when auto-calculating point_count

        Config()
        : point_count(0),
          expected_duration_s(10.0f),
          volume_max_uL(1000.0f),
          current_max_A(3.0f),
          sample_interval_ms(500)
        {}
    };

    static void init(
        lv_obj_t* volume_panel,
        lv_obj_t* current_panel,
        const Config& cfg
    );

    static void reset(
        const Config& cfg
    );

    static void pushSample(
        float t_s,        // time since start (s)
        float volume_uL,  // theoretical dispensed volume (uL)
        float current_A   // theoretical current (A)
    );

    static bool isInitialized();
    static void releaseTempData();

private:
    static bool s_initialized;

    static lv_obj_t* s_volume_panel;
    static lv_obj_t* s_current_panel;

    static lv_obj_t* s_volume_chart;
    static lv_obj_t* s_current_chart;

    static lv_chart_series_t* s_volume_series;
    static lv_chart_series_t* s_current_series;

    static Config s_cfg;

    static float s_last_t_s;
    static uint16_t s_written_points;

    static void createVolumeChart();
    static void createCurrentChart();

    static lv_coord_t mapToChartY(
        float value,
        float y_max
    );

    static lv_coord_t clampCoord(
        int32_t v
    );

    static uint16_t s_last_written_index;
    static bool* s_index_written;
    static uint16_t s_allocated_point_count; // dynamic allocation
    static void freeBuffers();
    static bool allocateBuffers(uint16_t point_count);
    static uint16_t calculatePointCount(const Config& cfg);
    static lv_obj_t* s_volume_xlabel;
    static lv_obj_t* s_volume_ylabel;
    static lv_obj_t* s_volume_x0_label;
    static lv_obj_t* s_volume_x1_label;
    static lv_obj_t* s_volume_y0_label;
    static lv_obj_t* s_volume_y1_label;

    static lv_obj_t* s_current_xlabel;
    static lv_obj_t* s_current_ylabel;
    static lv_obj_t* s_current_x0_label;
    static lv_obj_t* s_current_x1_label;
    static lv_obj_t* s_current_y0_label;
    static lv_obj_t* s_current_y1_label;
};

#endif