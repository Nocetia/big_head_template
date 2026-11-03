if(USER_BIG_HEAD_APP AND NOT USER_TEST)
    target_sources(${PROJECT_NAME} PRIVATE
        src/app.cpp
        src/control.cpp
    )
endif()
