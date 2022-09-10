zephyr_include_directories(${UTIL_PATH}/Inc)

zephyr_include_directories(motor_control/motor/Inc)
zephyr_include_directories(motor_control/sensor/Inc)
zephyr_include_directories(motor_control/chassis/Inc)
zephyr_include_directories(motor_control/algorithm/Inc)

aux_source_directory(motor_control/motor/Src MOTOR_CONTROL_SRCS)
aux_source_directory(motor_control/algorithm/Src MOTOR_CONTROL_ALGORITHM_SRCS)
aux_source_directory(motor_control/sensor/Src MOTOR_CONTROL_SENSOR_SRCS)
aux_source_directory(motor_control/chassis/Src MOTOR_CONTROL_CHASSIS_SRCS)


zephyr_library_sources(
    ${MOTOR_CONTROL_SRCS}
    ${MOTOR_CONTROL_ALGORITHM_SRCS}
    ${MOTOR_CONTROL_SENSOR_SRCS}
    ${MOTOR_CONTROL_CHASSIS_SRCS}
)
