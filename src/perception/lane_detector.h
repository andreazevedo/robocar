#pragma once

#include <optional>
#include <vector>

#include <opencv2/core/matx.hpp>

namespace robocar {
namespace perception {

/**
 * Represents one of the lines of the lane (either left or right line).
 */
struct LaneLine {
  double slope{0.0};
  double intercept{0};
};

/**
 * A lane - it might be incomplete if e.g. we camera can't see one of the lines
 */
struct Lane {
  std::optional<LaneLine> left;
  std::optional<LaneLine> right;
};

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
  std::optional<double> getAverageSlope(const cv::Mat& frame);

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
