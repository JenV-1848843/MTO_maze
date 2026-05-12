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
    set(OpenCV_DIR ~/opencv/build)
    find_package(OpenCV REQUIRED)
    include_directories(${OpenCV_INCLUDE_DIRS})

    # Check if gpiod is available (for Raspberry Pi hardware)
    find_package(PkgConfig QUIET)
    if(PKG_CONFIG_FOUND)
        pkg_check_modules(GPIOD libgpiod)
    endif()
    if(NOT GPIOD_FOUND)
        message(STATUS "libgpiod not found - MTO_maze will not include servo control (Pi-specific)")
    endif()

    #add_executable(webApp
    #    #src/webBasedMain.cpp
    #    src/pi2c.cpp
	#include/webcontrol.hpp
	#include/SystemConfig.hpp
    #)

    # MTO_maze sources (with conditional servo control)
    set(MTO_MAZE_SOURCES
        src/main_jen.cpp
        ./include/sources/utils.cpp
        ./include/sources/cell.cpp
        ./include/sources/maze.cpp
        ./include/headers/cell.h
        ./include/headers/config.h
        ./include/headers/direction.h
maze snapshot
latest · /var/www/html/maze_snapshot.jpg Maze snapshot
loaded 13:18:59
target cell
X
Y
grid 0–7 · (0,0) = top-left

        list(APPEND MTO_MAZE_SOURCES src/motorcontrol.cpp)
    endif()

    add_executable(MTO_maze ${MTO_MAZE_SOURCES})

    # target_include_directories(webApp PRIVATE include)
    target_link_libraries(MTO_maze ${OpenCV_LIBS})

    if(GPIOD_FOUND)
        target_include_directories(MTO_maze PRIVATE ${GPIOD_INCLUDE_DIRS})
        target_link_libraries(MTO_maze ${GPIOD_LIBRARIES})
        target_compile_definitions(MTO_maze PRIVATE SERVO_CONTROL_ENABLED=1)
    endif()

else()
    message(FATAL_ERROR "Unsupported OS. Only Windows and Linux/Unix are supported.")
endif()