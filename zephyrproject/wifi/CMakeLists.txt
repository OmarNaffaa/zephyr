cmake_minimum_required(VERSION 3.22)
set(BOARD esp32_devkitc_wrover_procpu)

find_package(Zephyr)
project(wifi)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})