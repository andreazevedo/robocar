#include "perception/lane_detector.h"

#include <cmath>
#include <vector>

#include <opencv2/opencv.hpp>

namespace robocar {
namespace perception {

double LaneDetector::getFinalSlope(const std::vector<cv::Vec4i>& lines) {
  double rightTheta = 0.0;
  double leftTheta = 0.0;
  size_t rightLinesCount = 0;
  size_t leftLinesCount = 0;
  for (const auto& line : lines) {
    double x1 = line[0];
    double y1 = line[1];
    double x2 = line[2];
    double y2 = line[3];
    double theta = ::atan2((y2 - y1), (x2 - x1));
    if (isnan(theta) || abs(theta) < 0.01) {
      // pretty much a horizontal line, ignore.
      continue;
    } else if (theta > 0.0) {
      // right lane
      rightTheta += theta;
      rightLinesCount++;
    } else if (theta < 0.0) {
      // right lane
      leftTheta += theta;
      leftLinesCount++;
    }
  }

  if (rightLinesCount == 0 && leftLinesCount == 0) {
    return 0.0;
  } else if (rightLinesCount == 0) {
    return (leftTheta / leftLinesCount);
  } else if (leftLinesCount == 0) {
    return (rightTheta / rightLinesCount);
  }

  return (rightTheta / rightLinesCount) - (leftTheta / leftLinesCount);
}

std::vector<cv::Vec4i> LaneDetector::detectLines(const cv::Mat& frame) {
  // get only the part of the image relevat for lane detection
  int x = 0;
  int y = frame.rows * 0.30;
  int width = frame.cols;
  int height = (frame.rows - y) * 0.7;
  cv::Mat cropped = frame(cv::Rect(x, y, width, height));

  // convert to b&w
  const cv::Mat& gray = cropped;  // it's already b&w
  // cv::cvtColor(cropped, gray, cv::COLOR_BGR2GRAY);

  // blurry image
  cv::Mat blurred;
  cv::GaussianBlur(gray, blurred, cv::Size(3, 3), 0);

  // crop the blurried image again
  cv::Mat croppedBlurred =
      blurred(cv::Rect(2, 0, blurred.cols - 2, blurred.rows));

  // canny - get edges
  cv::Mat edged;
  cv::Canny(croppedBlurred, edged, 85, 85);

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
    double theta = getFinalSlope(lines);
    std::string txt = "Theta: " + std::to_string(theta);
    cv::putText(withLines, txt, cv::Point(10, 10 * (lines.size() + 1)),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1,
                cv::LINE_AA);
    cv::imwrite("bin/images/with_lines.jpg", withLines);
  }

  return lines;
}

}  // namespace perception
}  // namespace robocar
