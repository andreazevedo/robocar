#include "sensors/camera.h"

#include <stdexcept>

#include <opencv2/core/base.hpp>
#include <opencv2/videoio.hpp>

#include "third_party/raspicam/src/raspicam_cv.h"

namespace robocar {
namespace sensors {

namespace {
std::optional<cv::RotateFlags> getRotateFlags(size_t cameraRotation) {
  if (cameraRotation == 0) {
    return {};
  } else if (cameraRotation == 90) {
    return cv::ROTATE_90_CLOCKWISE;
  } else if (cameraRotation == 180) {
    return cv::ROTATE_180;
  } else if (cameraRotation == 270) {
    return cv::ROTATE_90_COUNTERCLOCKWISE;
  }
  throw std::logic_error("Unallowed rotation provided: " +
                         std::to_string(cameraRotation));
}

cv::Mat rotate(const cv::Mat& frame, cv::RotateFlags rotateFlags) {
  cv::Mat rotatedFrame;
  cv::rotate(frame, rotatedFrame, rotateFlags);
  return rotatedFrame;
}

}  // namespace

Camera::Camera(size_t cameraRotation, bool color)
    : rotateFlags_(getRotateFlags(cameraRotation)) {
  // set rotation

  // set camera params
  if (color) {
    piCam_.set(cv::CAP_PROP_FORMAT, CV_8UC3);
  } else {
    piCam_.set(cv::CAP_PROP_FORMAT, CV_8UC1);
  }
  piCam_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  piCam_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
  piCam_.set(cv::CAP_PROP_FPS, 30);
  piCam_.set(cv::CAP_PROP_EXPOSURE, 30);
  // piCam_.set(cv::CAP_PROP_BRIGHTNESS, 55);
  // piCam_.set(cv::CAP_PROP_CONTRAST, 55);
  // piCam_.set(cv::CAP_PROP_SATURATION, 50);
  //if (color) {
  //  piCam_.set(cv::CAP_PROP_WHITE_BALANCE_RED_V, -1);
  //  piCam_.set(cv::CAP_PROP_WHITE_BALANCE_BLUE_U, -1);
  //}
  if (!piCam_.open()) {
    throw std::runtime_error("Error opening the camera");
  }
}
Camera::~Camera() { piCam_.release(); }

cv::Mat Camera::captureFrame() {
  cv::Mat frame;

  piCam_.grab();
  piCam_.retrieve(frame);

  if (rotateFlags_) {
    frame = rotate(frame, *rotateFlags_);
  }

  return frame;
}

}  // namespace sensors
}  // namespace robocar
