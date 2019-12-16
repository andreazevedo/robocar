#pragma once

#include <vector>

#include <opencv2/core/matx.hpp>

namespace robocar {
namespace perception {

class LaneDetector {
 public:
  /**
   * Given the left and right lines of the current lane, calculate the average
   * slope of both lines. That is, it returns the slope that the car should
   * follow, as it will average the slope of the left line of the line, and of
   * the right line of the lane.
   *
   * @param lines   The lines of the lane.
   */
  double getAverageSlope(const std::vector<cv::Vec4i>& lines);

  /**
   * Given the "view" of the car, detect the lines of the current lane.
   *
   * @param frame   Photo of the view of the car.
   */
  std::vector<cv::Vec4i> detectLines(const cv::Mat& frame);

  /**
   * Whether or not we should save debug images.
   *
   * @param val   True to save debug images. False otherwise.
   */
  void setSaveDebugImages(bool val) noexcept { saveDebugImages_ = val; }

 private:
  bool saveDebugImages_{false};
};

}  // namespace perception
}  // namespace robocar
