cmake_minimum_required (VERSION 3.1)

# define cmake modules path
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMake" ${CMAKE_MODULE_PATH})

# define the project
project("robocar" VERSION 0.1.0 LANGUAGES CXX)

# compiler options
add_compile_options(-std=c++17)

# add the src directory as include dir
include_directories("${PROJECT_SOURCE_DIR}/src"
                    "${PROJECT_SOURCE_DIR}")

# set the output diretory
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

################
# DEPENDENCIES #
################
find_package(pigpio MODULE REQUIRED)
find_package(OpenCV MODULE REQUIRED)
add_subdirectory(third_party/raspicam)

###########################
# LIBRARIES & EXECUTABLES #
###########################
# runtime
set(RUNTIME_FILES
  src/runtime/mpsc_queue.h
  src/runtime/service_thread.cc
  src/runtime/service_thread.h
  src/runtime/task_executor_thread.cc
  src/runtime/task_executor_thread.h
)
add_library(runtime ${RUNTIME_FILES})
target_link_libraries(runtime pthread)
set_target_properties(runtime PROPERTIES LINKER_LANGUAGE CXX)

# sensors
set(SENSORS_FILES
  src/sensors/camera.cc
  src/sensors/camera.h
)
add_library(sensors ${SENSORS_FILES})
target_link_libraries(sensors ${OpenCV_LIBS} raspicam_cv)

# perception
set(PERCEPTION_FILES
  src/perception/lane_detector.cc
  src/perception/lane_detector.h
)
add_library(perception ${PERCEPTION_FILES})
target_link_libraries(perception runtime ${OpenCV_LIBS})

# control
set(CONTROL_FILES
  src/control/control_service.cc
  src/control/control_service.h
  src/control/motor.cc
  src/control/motor.h
  src/control/util.cc
  src/control/util.h
  src/control/vehicle.cc
  src/control/vehicle.h
)
add_library(control ${CONTROL_FILES})
target_link_libraries(control runtime pigpio atomic)

# robocar
set(ROBOCAR_FILES
  src/car.cc
  src/car.h
  src/main.cc
)
add_executable(robocar ${ROBOCAR_FILES})
target_link_libraries(robocar runtime sensors perception control)


#########
# TESTS #
#########
option(BUILD_TESTS "Compile the tests" OFF)
if (BUILD_TESTS)
  find_package(GMock MODULE REQUIRED)
  include(GoogleTest REQUIRED RESULT_VARIABLE HAVE_CMAKE_GTEST)
  enable_testing()

  macro(add_gtest test_source test_name)
    add_executable(${test_name}
      ${test_source}
      src/tests/test_main.cc)
    target_include_directories(
      ${test_name} PUBLIC ${LIBGTEST_INCLUDE_DIR})
    target_link_libraries(${test_name} control runtime ${LIBGMOCK_LIBRARIES})
    add_test(${test_name} bin/${test_name})
  endmacro(add_gtest)

  # runtime tests
  add_gtest(src/runtime/tests/mpsc_queue_test.cc MPSCQueueTest)
  add_gtest(src/runtime/tests/service_thread_test.cc ServiceThreadTest)
  add_gtest(
    src/runtime/tests/task_executor_thread_test.cc TaskExecutorThreadTest)

  # control tests

endif()
