# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(BOARD stm32_g431cb)

list(APPEND ZEPHYR_EXTRA_MODULES
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers
)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(RobotActuator)

set(UTIL_PATH util)
set(HAL_PATH hal)
set(DEVICE_PATH device)
set(MIDDLEWARE_PATH middleware)
set(FRAMEWORK_PATH framework)
set(ALGORITHM_PATH algorithm)

add_subdirectory(util)
add_subdirectory(hal)
add_subdirectory(device)
add_subdirectory(algorithm)
add_subdirectory(middleware)
add_subdirectory(framework)
# add_subdirectory(test)

file(GLOB application_sources app/Src/*.cpp)
target_sources(app PRIVATE ${application_sources})
