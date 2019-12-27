#pragma once

#include <optional>
#include <vector>

#include <opencv2/core/matx.hpp>

#include "perception/lane.h"

namespace robocar {
namespace perception {

class LaneDetector {
 public:
  /**
   * Calculate the average slope of both lines. That is, it returns the slope
   * that the car should follow, as it will average the slope of the left line
   * of the line, and of the right line of the lane.
   *
   * @param frame   Photo of the view of the car. Must be in gray scale.
   *
   * @return  The average slope, or nullopt if a slope couldn't be calculated.
   */
  [[deprecated("Use getLane() instead")]] std::optional<double> getAverageSlope(
      const cv::Mat& frame);

  /**
   * Get information about the lane.
   *
   * @param frame   Photo of the view of the car. Must be in gray scale.
   */
  Lane getLane(const cv::Mat& frame);

  /**
   * Whether or not we should save debug images.
   *
   * @param val   True to save debug images. False otherwise.
   */
  void setSaveDebugImages(bool val) noexcept { saveDebugImages_ = val; }

 private:
  bool saveDebugImages_{false};

  /**
   * Given the "view" of the car, detect the lines of the current lane.
   *
   * @param frame   Photo of the view of the car. Must be in gray scale.
   */
  std::vector<cv::Vec4i> detectLines(const cv::Mat& frame);
};

}  // namespace perception
}  // namespace robocar
