cmake_minimum_required(VERSION 3.16)
project(MazeAndWebApp)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(WIN32)
    # ── Windows: MTO_maze (OpenCV) ──────────────────────────────────────────
    project(MTO_maze)
    set(OpenCV_DIR C:/Users/jenve/Desktop/School/master/mto/opencv/build)
    find_package(OpenCV REQUIRED)
    include_directories(${OpenCV_INCLUDE_DIRS})

    add_executable(MTO_maze
        src/main_jen.cpp
        ./include/sources/utils.cpp
        ./include/sources/cell.cpp
        ./include/sources/maze.cpp
        ./include/headers/cell.h
        ./include/headers/config.h
        ./include/headers/direction.h
        ./include/headers/maze.h
        ./include/headers/position.h
        ./include/headers/utils.h
        ./include/headers/wall.h
    )

    message(STATUS "Building MTO_maze for Windows")
    target_link_libraries(MTO_maze ${OpenCV_LIBS})

elseif(UNIX)
    # ── Linux: webApp ───────────────────────────────────────────────────────
    project(webApp)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build)

    add_executable(webApp
        src/webBasedMain.cpp
        src/pi2c.cpp
    )

    message(STATUS "Building webApp for Linux")
    target_include_directories(webApp PRIVATE include)

else()
    message(FATAL_ERROR "Unsupported OS. Only Windows and Linux/Unix are supported.")
endif()