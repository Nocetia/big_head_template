if(USER_BIG_HEAD_APP)
    target_sources(${PROJECT_NAME} PRIVATE
        src/app.cpp
        src/control.cpp
    )
endif()
