/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

// This demo UI is adapted from LVGL official example: https://docs.lvgl.io/master/examples.html#scatter-chart

#include "lvgl.h"

// static void draw_event_cb(lv_event_t *e)
// {
//     lv_obj_draw_part_dsc_t *dsc = lv_event_get_draw_part_dsc(e);
//     if (dsc->part == LV_PART_ITEMS) {
//         lv_obj_t *obj = lv_event_get_target(e);
//         lv_chart_series_t *ser = lv_chart_get_series_next(obj, NULL);
//         uint32_t cnt = lv_chart_get_point_count(obj);
//         /*Make older value more transparent*/
//         dsc->rect_dsc->bg_opa = (LV_OPA_COVER *  dsc->id) / (cnt - 1);

//         /*Make smaller values blue, higher values red*/
//         lv_coord_t *x_array = lv_chart_get_x_array(obj, ser);
//         lv_coord_t *y_array = lv_chart_get_y_array(obj, ser);
//         /*dsc->id is the tells drawing order, but we need the ID of the point being drawn.*/
//         uint32_t start_point = lv_chart_get_x_start_point(obj, ser);
//         uint32_t p_act = (start_point + dsc->id) % cnt; /*Consider start point to get the index of the array*/
//         lv_opa_t x_opa = (x_array[p_act] * LV_OPA_50) / 200;
//         lv_opa_t y_opa = (y_array[p_act] * LV_OPA_50) / 1000;

//         dsc->rect_dsc->bg_color = lv_color_mix(lv_palette_main(LV_PALETTE_RED),
//                                                lv_palette_main(LV_PALETTE_BLUE),
//                                                x_opa + y_opa);
//     }
// }

// static void add_data(lv_timer_t *timer)
// {
//     lv_obj_t *chart = timer->user_data;
//     lv_chart_set_next_value2(chart, lv_chart_get_series_next(chart, NULL), lv_rand(0, 200), lv_rand(0, 1000));
// }

void quantum_ui(lv_disp_t *disp)
{
      /*Change the active screen's background color*/
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x003a57), LV_PART_MAIN);

    /*Create a white label, set its text and align it to the center*/
    lv_obj_t * label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "Hello world");
    lv_obj_set_style_text_color(lv_scr_act(), lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}
