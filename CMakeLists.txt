cmake_minimum_required(VERSION 3.16)
project(webApp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build)

include_directories(include)

add_executable(webApp
    src/webBasedMain.cpp
    src/pi2c.cpp
)

target_include_directories(webApp PRIVATE include)