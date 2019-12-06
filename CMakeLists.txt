cmake_minimum_required (VERSION 3.1)

# define cmake modules path
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMake" ${CMAKE_MODULE_PATH})

# define the project
project("robocar" VERSION 0.1.0 LANGUAGES CXX)

# compiler options
add_compile_options(-std=c++17)

# list of dependencies
find_package(pigpio MODULE REQUIRED)

# add the src directory as include dir
include_directories("${PROJECT_SOURCE_DIR}/src")

# set the output diretory
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)


# LIBRARIES & EXECUTABLES #
set(CONTROL_FILES
  src/control/motor.cc
  src/control/motor.h
)
add_library(control ${CONTROL_FILES})
target_link_libraries(control pigpio)

set(ROBOCAR_FILES
  src/main.cc
)
add_executable(robocar ${ROBOCAR_FILES})
target_link_libraries(robocar control)
