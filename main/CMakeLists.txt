file(GLOB_RECURSE SOURCES "../components/lvgl__lvgl/demos/benchmark/*.c" 
                             "../components/lvgl__lvgl/demos/benchmark/assets/*.c")
idf_component_register(SRCS "rgb_lcd_example_main.c" "lvgl_demo_ui.c" ${SOURCES}
                        # 480_800_16_bmp.c
                       INCLUDE_DIRS "." "../components/lvgl__lvgl/demos/benchmark/")
