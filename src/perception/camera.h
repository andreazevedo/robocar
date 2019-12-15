#pragma once

#include <optional>

#include <opencv2/core/base.hpp>
#include "third_party/raspicam/src/raspicam_cv.h"

namespace robocar {
namespace perception {

/**
 * Provide basic access to camera.
 */
class Camera {
 public:

  /**
   * Constructs the camera and gets it ready to be used.
   *
   * @param cameraRotation  The rotation of the camera (e.g. use 180 if it's
   *                        upside down. Allowed values are 90, 180, and 270.
   */
  Camera(size_t cameraRotation = 0);

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
  const std::optional<cv::RotateFlags> rotateFlags_;
};

}  // namespace perception
}  // namespace robocar
