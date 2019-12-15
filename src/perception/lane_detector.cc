#include "perception/lane_detector.h"

#include <vector>

#include <opencv2/opencv.hpp>

namespace robocar {
namespace perception {

LaneDetector::LaneDetector(bool saveDebugImages)
    : saveDebugImages_(saveDebugImages) {}

std::vector<cv::Vec4i> LaneDetector::detect(const cv::Mat& frame) {
  // convert to b&w
  cv::Mat gray;
  cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

  // blurry image
  cv::Mat blurred;
  cv::GaussianBlur(gray, blurred, cv::Size(3, 3), 0);

  // canny - get edges
  cv::Mat edged;
  cv::Canny(blurred, edged, 85, 85);

  if (saveDebugImages_) {
    cv::imwrite("bin/images/gray.jpg", gray);
    cv::imwrite("bin/images/blurred.jpg", blurred);
    cv::imwrite("bin/images/edged.jpg", edged);
  }

  // Hough lines
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(edged, lines, 1, CV_PI / 180, 50, 50, 10);
  return lines;
}

}  // namespace perception
}  // namespace robocar
