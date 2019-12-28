#pragma once

#include <string>

namespace robocar {
namespace inference {

/**
 * Represents the position of an object in the image.
 * The positions are floating-point numbers in the [0,1] range representing
 * the location of the object in the input image.
 */
struct ObjectLocation {
  float top{0.0};
  float left{0.0};
  float bottom{0.0};
  float right{0.0};
};

/**
 * Represents an object detected by the CNN.
 */
struct DetectedObject {
  int classId{0};
  std::string className;
  float score{0.0};
  ObjectLocation location;
};

}  // namespace inference
}  // namespace robocar
