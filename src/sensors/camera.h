#pragma once

#include <optional>

#include <opencv2/core/base.hpp>
#include "third_party/raspicam/src/raspicam_cv.h"

namespace robocar {
namespace sensors {

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
   * @param color           True for colored images. False for gray images.
   */
  Camera(size_t cameraRotation = 0, bool color = true);

  ~Camera();

  // non-copyable
  Camera(const Camera&) = delete;
  const Camera& operator=(Camera&) = delete;
  // movable
  Camera(Camera&&) = default;
  Camera& operator=(Camera&&) = default;

  /**
   * Captures one image.
   *
   * @param color   True if the frame should be taken in color.
   */
  cv::Mat captureFrame();

 private:
  raspicam::RaspiCam_Cv piCam_;
  const std::optional<cv::RotateFlags> rotateFlags_;
};

}  // namespace sensors
}  // namespace robocar
