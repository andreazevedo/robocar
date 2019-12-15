#pragma once

#include <vector>

#include <opencv2/core/matx.hpp>

namespace robocar {
namespace perception {

class LaneDetector {
 public:
  LaneDetector(bool saveDebugImages = false);

  std::vector<cv::Vec4i> detectLines(const cv::Mat& frame);

 private:
  bool saveDebugImages_{false};
};

}  // namespace perception
}  // namespace robocar
