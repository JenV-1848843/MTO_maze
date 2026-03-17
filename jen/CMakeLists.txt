cmake_minimum_required(VERSION 4.3)
set(CMAKE_CXX_STANDARD 17)

project(MTO_maze)

set(OpenCV_DIR C:/Users/jenve/Desktop/School/master/mto/opencv/build)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(MTO_maze 
    src/main.cpp
    src/sources/utils.cpp
    src/sources/cell.cpp
    src/sources/maze.cpp
    src/headers/cell.h
    src/headers/config.h
    src/headers/direction.h
    src/headers/maze.h
    src/headers/position.h
    src/headers/utils.h
    src/headers/wall.h
)

# set(BUILD_MODE "DESKTOP" CACHE STRING "Build mode: DESKTOP, TERMINAL or RPI")

message(STATUS "Build mode: ${BUILD_MODE}")

# Apply compile definitions based on mode
if(BUILD_MODE STREQUAL "DESKTOP")
    target_compile_definitions(MTO_maze PRIVATE DESKTOP_BUILD)
elseif(BUILD_MODE STREQUAL "TERMINAL")
    target_compile_definitions(MTO_maze PRIVATE TERMINAL_BUILD)
elseif(BUILD_MODE STREQUAL "RPI")
    target_compile_definitions(MTO_maze PRIVATE RPI_BUILD)
else()
    message(FATAL_ERROR "Invalid BUILD_MODE. Use DESKTOP, TERMINAL, or RPI.")
endif()

target_link_libraries(MTO_maze ${OpenCV_LIBS})
