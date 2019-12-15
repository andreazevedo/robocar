#pragma once

#include <vector>

#include <opencv2/core/matx.hpp>

namespace robocar {
namespace perception {

class LaneDetector {
 public:
  double getFinalSlope(const std::vector<cv::Vec4i>& lines);

  std::vector<cv::Vec4i> detectLines(const cv::Mat& frame);

  void setSaveDebugImages(bool val) noexcept { saveDebugImages_ = val; }

 private:
  bool saveDebugImages_{false};
};

}  // namespace perception
}  // namespace robocar
