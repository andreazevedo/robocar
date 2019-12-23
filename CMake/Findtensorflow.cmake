# Find the tensorflow library
#
# This module defines:
#   tensorflow_FOUND
#   tensorflow_INCLUDE_DIR
#   tensorflow_LIBRARY


set(tensorflow_INCLUDE_DIR
    ${PROJECT_SOURCE_DIR}/third_party/tensorflow/)
set(tensorflow_LIBRARY
    ${tensorflow_INCLUDE_DIR}/tensorflow/lite/tools/make/gen/rpi_armv7l/lib/libtensorflow-lite.a)

set(tensorflow_FOUND
    EXISTS ${tensorflow_LIBRARY})

if (${tensorflow_FOUND})
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(
    tensorflow
    DEFAULT_MSG 
    tensorflow_INCLUDE_DIR
    tensorflow_LIBRARY)

  include_directories(
    "${tensorflow_INCLUDE_DIR}"
    "${tensorflow_INCLUDE_DIR}/tensorflow/lite/tools/make/downloads/flatbuffers/include")
endif()
