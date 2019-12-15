#pragma once

#include <opencv2/core/base.hpp>
#include "third_party/raspicam/src/raspicam_cv.h"

namespace robocar {
namespace perception {

/**
 * Provide basic access to camera.
 */
class Camera {
 public:
  Camera();
  ~Camera();

  // non-copyable
  Camera(const Camera&) = delete;
  const Camera& operator=(Camera&) = delete;
  // movable
  Camera(Camera&&) = default;
  Camera& operator=(Camera&&) = default;

  /**
   * Captures one image.
   */
  cv::Mat captureFrame();

 private:
  raspicam::RaspiCam_Cv piCam_;
};

}  // namespace perception
}  // namespace robocar
