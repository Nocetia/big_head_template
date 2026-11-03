cmake_minimum_required(VERSION 3.22)

set(BOARD "h723_mc02" CACHE STRING "Target board")
if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE
        "${CMAKE_CURRENT_SOURCE_DIR}/boards/cubemx/${BOARD}/cmake/gcc-arm-none-eabi.cmake"
        CACHE FILEPATH "Board toolchain"
    )
endif()

project(embedded_framework LANGUAGES C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(${PROJECT_NAME})
add_subdirectory(boards)
add_subdirectory(libs)
add_subdirectory(bsp)
include(configs/scripts/generate_config.cmake)
add_subdirectory(devices)
add_subdirectory(modules)
target_link_libraries(${PROJECT_NAME} modules)

# Start RFID from ThreadX initialization while leaving the BSP startup hook unchanged.
target_link_options(${PROJECT_NAME} PRIVATE "LINKER:--wrap=tx_application_define")
add_subdirectory(applications/big_head)
