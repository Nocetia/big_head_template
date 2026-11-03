option(USER_TEST "Build the H723 M3508 SMC test application" OFF)

if(USER_TEST)
    if(NOT BOARD STREQUAL "h723_mc02")
        message(FATAL_ERROR "The M3508 SMC test currently requires h723_mc02")
    endif()
    target_sources(${PROJECT_NAME} PRIVATE
        src/test_smc.cpp
    )
endif()
