/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

// This demo UI is adapted from LVGL official example: https://docs.lvgl.io/master/examples.html#scatter-chart

#include "lvgl.h"

// The pixel number in horizontal and vertical
#define LCD_H_RES 800
#define LCD_V_RES 480

void quantum_ui(lv_disp_t *disp)
{

    static lv_coord_t col_dsc[] = {LCD_H_RES / 2 - 15, LCD_H_RES / 2 - 15, LV_GRID_TEMPLATE_LAST};
    static lv_coord_t row_dsc[] = {LCD_V_RES / 2 - 15, LCD_V_RES / 2 - 15, LV_GRID_TEMPLATE_LAST};

    /*Create a container with grid*/
    lv_obj_t *background = lv_obj_create(lv_scr_act());
    lv_obj_set_style_grid_column_dsc_array(background, col_dsc, 0);
    lv_obj_set_style_grid_row_dsc_array(background, row_dsc, 0);
    lv_obj_set_size(background, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_center(background);
    lv_obj_set_layout(background, LV_LAYOUT_GRID);

    // lv_obj_add_flag(background, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(lv_scr_act(), LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *dice_btn = lv_obj_create(background);
    /*Stretch the cell horizontally and vertically too
     *Set span to 1 to make the cell 1 column/row sized*/
    lv_obj_set_grid_cell(dice_btn, LV_GRID_ALIGN_STRETCH, 0, 1,
                         LV_GRID_ALIGN_STRETCH, 0, 2);

    lv_obj_t *dice_btn2 = lv_obj_create(background);

    lv_obj_set_grid_cell(dice_btn2, LV_GRID_ALIGN_STRETCH, 1, 1,
                         LV_GRID_ALIGN_STRETCH, 0, 1);

    lv_obj_t *dice_btn4 = lv_obj_create(background);
    // /*Stretch the cell horizontally and vertically too
    //  *Set span to 1 to make the cell 1 column/row sized*/
    lv_obj_set_grid_cell(dice_btn4, LV_GRID_ALIGN_STRETCH, 1, 1,
                         LV_GRID_ALIGN_STRETCH, 1, 1);

    lv_obj_t *btn_text_label = lv_label_create(dice_btn);
    lv_obj_t *btn2_text_label = lv_label_create(dice_btn2);
    lv_obj_t *btn4_text_label = lv_label_create(dice_btn4);

    lv_label_set_text(btn_text_label, "btn1");
    lv_label_set_text(btn2_text_label, "btn2");
    lv_label_set_text(btn4_text_label, "btn4");

    lv_obj_center(btn_text_label);
    lv_obj_center(btn2_text_label);
    lv_obj_center(btn4_text_label);
}
