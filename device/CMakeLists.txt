zephyr_include_directories(${UTIL_PATH}/Inc)
zephyr_include_directories(Inc)

# aux_source_directory(Src DEVICE_SRCS)
set(DEVICE_SRCS
    Src/led_device.cpp
    Src/pwm_device.cpp
    Src/adc_device.cpp)

zephyr_library_sources(
    ${DEVICE_SRCS}
)
