zephyr_include_directories(${UTIL_PATH}/Inc)
zephyr_include_directories(Inc)

# aux_source_directory(Src DEVICE_SRCS)
set(DEVICE_SRCS
    Src/led_device.cpp
    Src/absolute_encoder.cpp
    Src/angle_encoder_abstract.cpp
    Src/linear_hall_angle_encoder.cpp)

zephyr_library_sources(
    ${DEVICE_SRCS}
)
