#include "perception/camera.h"

#include <stdexcept>

#include <opencv2/core/base.hpp>
#include <opencv2/videoio.hpp>

#include "third_party/raspicam/src/raspicam_cv.h"

namespace robocar {
namespace perception {

Camera::Camera() {
  // set camera params
  piCam_.set(cv::CAP_PROP_FORMAT, CV_8UC1);  
  piCam_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  piCam_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
  piCam_.set(cv::CAP_PROP_BRIGHTNESS, 50);
  if (!piCam_.open()) {
    throw std::runtime_error("Error opening the camera");
  }
}
Camera::~Camera() { piCam_.release(); }

cv::Mat Camera::captureFrame() {
  cv::Mat image;

  piCam_.grab();
  piCam_.retrieve(image);

  return image;
}

}  // namespace perception
}  // namespace robocar
