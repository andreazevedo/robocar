#pragma once

#include <optional>

#include <opencv2/core/base.hpp>

#include "third_party/raspicam/src/raspicam_cv.h"

#include "math/floating_point.h"

namespace robocar {
namespace sensors {

/**
 * Provide basic access to camera.
 */
class Camera {
 public:
  // Resolution of the pictures taken.
  static constexpr size_t kWidth = 640;
  static constexpr size_t kHeight = 480;

  // These values are from specifications + experimentation.
  static constexpr float kFocalLengthMm = 2.4f;
  static constexpr float kSensorSizeDiagonalMm =
      math::sqrt(math::pow2(3.67f) + math::pow2(2.74f));
  static constexpr float kSensorSizeDiagonalPixels =
      math::sqrt(math::pow2(kWidth) + math::pow2(kHeight));

  /**
   * Calculates the distance from the camera to the object, in millimeters.
   *
   * @param realWorldDiagonalSizeMm     The size of the diagonal of the object,
   *                                    in millimeters.
   * @param inCameraDiagonalSizePixels  The size of the diagonal of the object
   *                                    when seen in the camera, in pixels.
   *
   * @return  The approximate distance from the camera to the object,
   *          in millimeters.
   */
  static float distanceToObjectMm(float realWorldDiagonalSizeMm,
                                  float inCameraDiagonalSizePixels);

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
