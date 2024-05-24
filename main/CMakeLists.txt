file(GLOB_RECURSE SOURCES "../components/lvgl__lvgl/demos/benchmark/*.c" 
                             "../components/lvgl__lvgl/demos/benchmark/assets/*.c")
idf_component_register(SRCS "quantum_ui.c" "quantum_ui_main.c" "wifi_connector.h" ${SOURCES}
                        # 480_800_16_bmp.c
                       INCLUDE_DIRS "." "../components/lvgl__lvgl/demos/benchmark/")
