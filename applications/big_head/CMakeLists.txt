set(_big_head_config "${CMAKE_CURRENT_LIST_DIR}/../big_head_config.hpp")
file(STRINGS "${_big_head_config}" _big_head_enabled_line
    REGEX "inline constexpr bool big_head_enabled")
if(NOT "${_big_head_enabled_line}" MATCHES "=[ ]*(true|false)")
    message(FATAL_ERROR "big_head_config.hpp must define big_head_enabled as true or false")
endif()
set(_big_head_enabled "${CMAKE_MATCH_1}")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_big_head_config}")

if(_big_head_enabled AND BOARD STREQUAL "h723_mc02" AND NOT USER_TEST)
    if(NOT USER_USB_ENABLED)
        message(FATAL_ERROR "big_head requires USB to be enabled in configs/user_config.json")
    endif()
    if(NOT USER_AHRS_DEVICE STREQUAL "bmi088")
        message(FATAL_ERROR "big_head requires the BMI088 AHRS backend")
    endif()
    if(NOT USER_REMOTER_VT03)
        message(FATAL_ERROR "big_head requires VT03 input")
    endif()
    target_sources(${PROJECT_NAME} PRIVATE
        "${_big_head_config}"
        src/app.cpp
        src/control.cpp
    )
    target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_LIST_DIR}/..")
endif()
