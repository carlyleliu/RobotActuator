zephyr_include_directories(${UTIL_PATH}/Inc)
zephyr_include_directories(Inc)

aux_source_directory(Src DEVICE_SRCS)

zephyr_library_sources(
    ${DEVICE_SRCS}
)
