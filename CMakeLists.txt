cmake_minimum_required (VERSION 3.1)

# define cmake modules path
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMake" ${CMAKE_MODULE_PATH})

# define the project
project("robocar" VERSION 0.1.0 LANGUAGES CXX)

# compiler options
add_compile_options(-std=c++17)
add_compile_options(-Wall)
add_compile_options(-Wno-psabi)
add_compile_options(-O2)

# add the src directory as include dir
include_directories("${PROJECT_SOURCE_DIR}/src"
                    "${PROJECT_SOURCE_DIR}")

# set the output diretory
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)


##############
# SANITIZERS #
##############
option(ENABLE_ASAN "Enable thread sanitizer" OFF)
if (ENABLE_ASAN)
  add_compile_options(-fsanitize=address,undefined)
  add_compile_options(-fno-omit-frame-pointer)
  add_compile_options(-fPIC)
  add_compile_options(-pie)
  add_compile_options(-O0)
  add_compile_options(-g)
  SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address,undefined")
endif()

################
# DEPENDENCIES #
################
find_package(pigpio MODULE REQUIRED)
find_package(OpenCV MODULE REQUIRED)
add_subdirectory(third_party/raspicam)

###########################
# LIBRARIES & EXECUTABLES #
###########################
# math
set(MATH_FILES
  src/math/floating_point.h
  src/math/poly.cc
  src/math/poly.h
)
add_library(math ${MATH_FILES})
target_link_libraries(math ${OpenCV_LIBS})
set_target_properties(math PROPERTIES LINKER_LANGUAGE CXX)

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
  src/perception/lane.h
  src/perception/lane_detector.cc
  src/perception/lane_detector.h
)
add_library(perception ${PERCEPTION_FILES})
target_link_libraries(perception math runtime ${OpenCV_LIBS})

# planning
set(PLANNING_FILES
  src/planning/planner.cc
  src/planning/planner.h
  src/planning/planning_result.h
)
add_library(planning ${PLANNING_FILES})
target_link_libraries(planning runtime ${OpenCV_LIBS})

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
target_link_libraries(robocar runtime sensors perception planning control)


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
    target_link_libraries(${test_name} math runtime ${LIBGMOCK_LIBRARIES})
    add_test(${test_name} bin/${test_name})
  endmacro(add_gtest)

  # math tests
  add_gtest(src/math/tests/math_test.cc MathTest)

  # runtime tests
  add_gtest(src/runtime/tests/mpsc_queue_test.cc MPSCQueueTest)
  add_gtest(src/runtime/tests/service_thread_test.cc ServiceThreadTest)
  add_gtest(
    src/runtime/tests/task_executor_thread_test.cc TaskExecutorThreadTest)

endif()
