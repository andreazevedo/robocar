#include "perception/lane_detector.h"

#include <cmath>
#include <vector>

#include <opencv2/opencv.hpp>

namespace robocar {
namespace perception {

LaneDetector::LaneDetector(bool saveDebugImages)
    : saveDebugImages_(saveDebugImages) {}

double LaneDetector::getSteeringAngle(const cv::Mat& frame) {
  auto lines = detectLines(frame);

  double theta = 0.0;
  for (const auto& line : lines) {
    int x1 = line[0];
    int y1 = line[1];
    int x2 = line[2];
    int y2 = line[3];
    theta += ::atan2((y2 - y1), (x2 - x1));
  }
  return theta / lines.size();
}

std::vector<cv::Vec4i> LaneDetector::detectLines(const cv::Mat& frame) {
  // get only the part of the image relevat for lane detection
  int x = 0;
  int y = frame.rows * 0.25;
  int width = frame.cols;
  int height = (frame.rows - y) * 0.6;
  cv::Mat cropped = frame(cv::Rect(x, y, width, height));

  // convert to b&w
  cv::Mat gray;
  cv::cvtColor(cropped, gray, cv::COLOR_BGR2GRAY);

  // blurry image
  cv::Mat blurred;
  cv::GaussianBlur(gray, blurred, cv::Size(3, 3), 0);

  // canny - get edges
  cv::Mat edged;
  cv::Canny(blurred, edged, 85, 85);

  // Hough lines
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(edged, lines, 1, CV_PI / 180, 50, 50, 10);

  if (saveDebugImages_) {
    cv::imwrite("bin/images/gray.jpg", gray);
    cv::imwrite("bin/images/blurred.jpg", blurred);
    cv::imwrite("bin/images/edged.jpg", edged);

    cv::Mat withLines = gray;
    cv::cvtColor(gray, withLines, cv::COLOR_GRAY2BGR);
    for (size_t i = 0; i < lines.size(); ++i) {
      const auto& l = lines[i];
      int r = (50 * (i + 0)) % 250;
      int g = (25 * (i + 0)) % 250;
      int b = (00 * (i + 0)) % 250;
      cv::line(withLines, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),
               cv::Scalar(r, g, b), 3, cv::LINE_AA);

      int x1 = l[0];
      int y1 = l[1];
      int x2 = l[2];
      int y2 = l[3];
      double theta = ::atan2((y2 - y1), (x2 - x1));
      std::string txt = std::to_string(i) + ": " + std::to_string(theta);
      cv::putText(withLines, txt, cv::Point(10, 10 * i),
                  cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(r, g, b), 1,
                  cv::LINE_AA);
    }
    cv::imwrite("bin/images/with_lines.jpg", withLines);
  }

  return lines;
}

}  // namespace perception
}  // namespace robocar
