zephyr_include_directories(${UTIL_PATH}/Inc)
zephyr_include_directories(${ALGORITHM_PATH}/Inc)

zephyr_include_directories(Inc)

aux_source_directory(Src FRAMEWORK_SRCS)

zephyr_library_sources(
    ${FRAMEWORK_SRCS}
)
