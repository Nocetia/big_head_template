cmake_minimum_required(VERSION 3.22)
project(big_head_tests LANGUAGES CXX)
enable_testing()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
get_filename_component(REPO "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)

add_executable(protocol_test protocol_test.cpp)
add_executable(control_test control_test.cpp ../src/control.cpp)
add_executable(remote_test remote_test.cpp
    "${REPO}/devices/remoter/remoter/src/remoter.cpp"
    "${REPO}/devices/remoter/dr16/src/dr16.cpp"
    "${REPO}/devices/remoter/vt03/src/vt03.cpp"
    "${REPO}/devices/motor/dji/src/dji_motor.cpp"
    "${REPO}/devices/motor/dji/src/dji_manager.cpp"
)
foreach(test IN ITEMS protocol_test control_test remote_test)
    target_include_directories(${test} PRIVATE
        ../src
        "${REPO}/libs/crc/include"
        "${REPO}/libs/control/include"
        "${REPO}/libs/math/include"
        "${REPO}/libs/types/include"
        "${REPO}/bsp/can/include"
        "${REPO}/devices/remoter/remoter/include"
        "${REPO}/devices/motor/dji/include"
        "${REPO}/devices/motor/motor/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/stubs"
    )
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${test} PRIVATE -Wall -Wextra -Werror)
        if(MINGW)
            # Device decoders use GNU packed bitfields, as on the ARM target.
            target_compile_options(${test} PRIVATE -mno-ms-bitfields)
        endif()
    endif()
    add_test(NAME ${test} COMMAND ${test})
endforeach()
