# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(BOARD stm32_g431cb)

list(APPEND ZEPHYR_EXTRA_MODULES
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers
)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(RobotChassis)

set(UTIL_PATH util)
set(DEVICE_PATH device)
set(MIDDLEWARE_PATH middleware)

add_subdirectory(util)
add_subdirectory(device)
add_subdirectory(middleware)
# add_subdirectory(test)

file(GLOB application_sources app/Src/*.cpp)
target_sources(app PRIVATE ${application_sources})
