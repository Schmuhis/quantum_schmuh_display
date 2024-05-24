cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(rgb_panel)
idf_build_set_property(COMPILE_OPTIONS "-DLV_ATTRIBUTE_TICK_INC=IRAM_ATTR" APPEND)
