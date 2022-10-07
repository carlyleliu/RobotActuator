zephyr_include_directories(${UTIL_PATH}/Inc)
zephyr_include_directories(Inc)

aux_source_directory(Src HAL_SRCS)
#set(HAL_SRCS
#    Src/pwm_device.cpp
#    Src/adc_device.cpp
#)

zephyr_library_sources(
    ${HAL_SRCS}
)
