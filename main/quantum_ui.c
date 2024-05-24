#include "lvgl.h"
#include <stdio.h>  //for basic printf commands
#include <string.h> //for handling strings
                    // system applications for light weight ip apps

// The pixel number in horizontal and vertical
#define LCD_H_RES 800
#define LCD_V_RES 480


static void
roll_dice_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *btn = lv_event_get_target(e);
    if (code == LV_EVENT_CLICKED)
    {
        static uint8_t cnt = 0;
        cnt++;

        /*Get the first child of the button which is the label and change its text*/
        lv_obj_t *label = lv_obj_get_child(btn, 0);
        lv_label_set_text_fmt(label, "Button: %d", cnt);
    }
}
void setup_grid(lv_obj_t *background)
{

    lv_obj_set_style_bg_color(background, lv_color_hex(0x5b5b5b), LV_PART_MAIN);
    static lv_coord_t col_dsc[] = {LCD_H_RES / 2 - 15, LCD_H_RES / 2 - 15, LV_GRID_TEMPLATE_LAST};
    static lv_coord_t row_dsc[] = {LCD_V_RES / 2 - 15, LCD_V_RES / 2 - 15, LV_GRID_TEMPLATE_LAST};

    lv_obj_set_style_grid_column_dsc_array(background, col_dsc, 0);
    lv_obj_set_style_grid_row_dsc_array(background, row_dsc, 0);
    lv_obj_set_size(background, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_center(background);
    lv_obj_set_layout(background, LV_LAYOUT_GRID);

    lv_obj_clear_flag(lv_scr_act(), LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *player_list = lv_obj_create(background);
    lv_obj_set_grid_cell(player_list, LV_GRID_ALIGN_STRETCH, 1, 1,
                         LV_GRID_ALIGN_STRETCH, 1, 1);

    lv_obj_t *dice_btn = lv_btn_create(background);
    lv_obj_set_grid_cell(dice_btn, LV_GRID_ALIGN_STRETCH, 1, 1,
                         LV_GRID_ALIGN_STRETCH, 0, 1);
    lv_obj_add_event_cb(dice_btn, roll_dice_cb, LV_EVENT_ALL, NULL);

    lv_obj_t *button_output = lv_obj_create(background);
    lv_obj_set_grid_cell(button_output, LV_GRID_ALIGN_STRETCH, 0, 1,
                         LV_GRID_ALIGN_STRETCH, 0, 2);

    lv_obj_t *player_list_text_label = lv_label_create(player_list);
    lv_obj_t *dice_text_label = lv_label_create(dice_btn);
    lv_obj_t *button_output_text_label = lv_label_create(button_output);

    lv_label_set_text(player_list_text_label, "Player 1");
    lv_label_set_text(dice_text_label, "Roll the dice");
    lv_label_set_text(button_output_text_label, "6");

    lv_obj_center(player_list_text_label);
    lv_obj_center(dice_text_label);
    lv_obj_center(button_output_text_label);
    lv_obj_set_style_bg_color(dice_btn, lv_color_hex(0xbcbcbc), LV_PART_MAIN);
    lv_obj_set_style_bg_color(button_output, lv_color_hex(0xbcbcbc), LV_PART_MAIN);
    lv_obj_set_style_bg_color(player_list, lv_color_hex(0xbcbcbc), LV_PART_MAIN);
}

void quantum_ui(lv_disp_t *disp)
{

    /*Create a container with grid*/
    lv_obj_t *background = lv_btn_create(lv_scr_act());
    setup_grid(background);
}
