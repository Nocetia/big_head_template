cmake_minimum_required(VERSION 3.22)

# 独立生成：cmake -P configs/scripts/generate_config.cmake
get_filename_component(_config_dir "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
file(READ "${_config_dir}/user_config.json" _json)

include("${CMAKE_CURRENT_LIST_DIR}/values.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/communication.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/imu.cmake")
config_vector3(_imu_offset application imu_offset)
include("${CMAKE_CURRENT_LIST_DIR}/boards.cmake")

# 全部校验通过后才写文件；内容不变时保留时间戳。
configure_file("${CMAKE_CURRENT_LIST_DIR}/user_config.hpp.in"
    "${_config_dir}/include/user_config.hpp" @ONLY)

if(NOT CMAKE_SCRIPT_MODE_FILE)
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
        "${_config_dir}/user_config.json"
        "${_config_dir}/../boards/wrapper/h723_mc02/include/config.hpp"
        "${_config_dir}/../boards/wrapper/f407_cboard/include/config.hpp"
    )
endif()
