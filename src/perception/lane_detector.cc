#include "perception/lane_detector.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <optional>
#include <vector>

#include <opencv2/opencv.hpp>

#include "math/poly.h"
#include "math/statistics.h"
#include "perception/lane.h"

namespace robocar {
namespace perception {

namespace {

FrameSize getFrameSize(const cv::Mat& frame) {
  assert(frame.cols > 0);
  assert(frame.rows > 0);
  return FrameSize{static_cast<size_t>(frame.cols),
                   static_cast<size_t>(frame.rows)};
}

LaneLine calcAverageLine(const std::vector<LaneLine>& lines) {
  LaneLine avg;
  for (const auto& line : lines) {
    avg.slope += line.slope;
    avg.intercept += line.intercept;
  }
  avg.slope /= lines.size();
  avg.intercept /= lines.size();
  avg.slopeStdDeviation = math::stdDeviation(
      lines.begin(), lines.end(), [](const auto& line) { return line.slope; });
  avg.numLines = lines.size();
  return avg;
}

cv::Vec4i createPoints(const LaneLine& laneLine, int width, int height) {
  const int y1 = height;
  const int y2 = y1 * 0.05;

  // bound the coordinates within the frame
  const int x1 = std::max(
      -width,
      std::min(2 * width, int((y1 - laneLine.intercept) / laneLine.slope)));
  const int x2 = std::max(
      -width,
      std::min(2 * width, int((y2 - laneLine.intercept) / laneLine.slope)));
  return {x1, y1, x2, y2};
}

double getAverageSlopeImpl(const std::vector<cv::Vec4i>& lines) {
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
    if (isnan(theta) || ::fabs(theta) < 0.01) {
      // pretty much a horizontal line, ignore.
      continue;
    } else if (theta > 0.0) {
      // right lane
      rightTheta += theta;
      rightLinesCount++;
    } else if (theta < 0.0) {
      // left lane
      leftTheta += theta;
      leftLinesCount++;
    }
  }

  double finalTheta;
  if (rightLinesCount == 0 && leftLinesCount == 0) {
    finalTheta = 0.0;
  } else if (rightLinesCount == 0) {
    finalTheta = (leftTheta / leftLinesCount);
  } else if (leftLinesCount == 0) {
    finalTheta = (rightTheta / rightLinesCount);
  } else {
    finalTheta = (rightTheta / rightLinesCount) - (leftTheta / leftLinesCount);
  }

  // Adjust angle - we want horizontal lines to be infinite, not vertical.
  if (finalTheta < 0) {
    finalTheta = (1.0 + finalTheta) * -1;
  } else {
    finalTheta = 1.0 - finalTheta;
  }
  return finalTheta;
}

Lane getLaneImpl(const std::vector<cv::Vec4i>& lines, FrameSize frameSize) {
  constexpr double lineBoundary = 1.0 / 3.0;
  const size_t leftLineEnd = frameSize.width - (frameSize.width * lineBoundary);
  const size_t rightLineStart = frameSize.width * lineBoundary;

  std::vector<LaneLine> leftFit;
  std::vector<LaneLine> rightFit;

  for (const auto& line : lines) {
    size_t x1 = line[0];
    size_t y1 = line[1];
    size_t x2 = line[2];
    size_t y2 = line[3];
    double theta = ::atan2((y2 - y1), (x2 - x1));
    if (isnan(theta) || ::fabs(theta) < 0.001) {
      // pretty much a horizontal line, ignore.
      continue;
    }
    auto fit = math::polyfit({cv::Point(x1, y1), cv::Point(x2, y2)}, 1);
    float intercept = fit[0];
    float slope = fit[1];
    if (slope < 0) {
      if (x1 < leftLineEnd && x2 < leftLineEnd) {
        leftFit.emplace_back(LaneLine{slope, intercept});
      }
    } else {
      if (x1 > rightLineStart && x2 > rightLineStart) {
        rightFit.emplace_back(LaneLine{slope, intercept});
      }
    }
  }

  Lane lane;
  lane.frameSize = frameSize;
  if (leftFit.size() > 0) {
    lane.left.emplace(calcAverageLine(leftFit));
  }
  if (rightFit.size() > 0) {
    lane.right.emplace(calcAverageLine(rightFit));
  }
  return lane;
}

void drawLine(cv::Mat& dstFrame, const cv::Vec4i& line, cv::Scalar color) {
  cv::line(dstFrame, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]),
           color, 3, cv::LINE_AA);
}

cv::Mat cropLaneSection(const cv::Mat& frame) {
  int x = 0;
  int y = frame.rows * 0.4;
  int width = frame.cols;
  int height = (frame.rows - y) * 1.0;
  cv::Mat cropped = frame(cv::Rect(x, y, width, height));
  return cropped;
}

cv::Mat getEdges(const cv::Mat& gray) {
  // blurry image
  cv::Mat blurred;
  cv::GaussianBlur(gray, blurred, cv::Size(3, 3), 0);

  // crop the blurried image again (cause we might have a line on the left)
  cv::Mat croppedBlurred =
      blurred(cv::Rect(2, 0, blurred.cols - 2, blurred.rows));

  // canny - get edges
  cv::Mat edges;
  cv::Canny(croppedBlurred, edges, 85, 85);

  return edges;
}

std::vector<cv::Vec4i> getLines(const cv::Mat& edges) {
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 50, 10);
  return lines;
}

void saveDebugImages(const cv::Mat& original, const cv::Mat& gray,
                     const cv::Mat& edges,
                     const std::vector<cv::Vec4i>& lines) {
  cv::imwrite("bin/images/original.jpg", original);
  cv::imwrite("bin/images/gray.jpg", gray);
  cv::imwrite("bin/images/edges.jpg", edges);

  cv::Mat withLines;
  cv::cvtColor(gray, withLines, cv::COLOR_GRAY2BGR);
  for (size_t i = 0; i < lines.size(); ++i) {
    const auto& l = lines[i];
    int b = (50 * (i + 0)) % 250;
    int g = (25 * (i + 0)) % 250;
    int r = (00 * (i + 0)) % 250;
    drawLine(withLines, l, cv::Scalar(b, g, r));
    int x1 = l[0];
    int y1 = l[1];
    int x2 = l[2];
    int y2 = l[3];
    double theta = ::atan2((y2 - y1), (x2 - x1));
    std::string txt = std::to_string(i) + ": " + std::to_string(theta);
    cv::putText(withLines, txt, cv::Point(10, 10 * i), cv::FONT_HERSHEY_SIMPLEX,
                0.3, cv::Scalar(b, g, r), 1, cv::LINE_AA);
  }
  double theta = getAverageSlopeImpl(lines);
  std::string txt = "Theta: " + std::to_string(theta);
  cv::putText(withLines, txt, cv::Point(10, 10 * (lines.size() + 1)),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1,
              cv::LINE_AA);
  cv::imwrite("bin/images/with_lines.jpg", withLines);

  Lane lane = getLaneImpl(lines, getFrameSize(edges));
  cv::Mat withLinesTwo;
  cv::cvtColor(gray, withLinesTwo, cv::COLOR_GRAY2BGR);
  if (lane.left) {
    auto toDraw =
        createPoints(*lane.left, withLinesTwo.cols, withLinesTwo.rows);
    drawLine(withLinesTwo, toDraw, cv::Scalar(255, 0, 0));
  }
  if (lane.right) {
    auto toDraw =
        createPoints(*lane.right, withLinesTwo.cols, withLinesTwo.rows);
    drawLine(withLinesTwo, toDraw, cv::Scalar(0, 255, 0));
  }
  cv::imwrite("bin/images/with_lines_two.jpg", withLinesTwo);
}

bool isGrayScale(const cv::Mat& frame) {
  cv::Mat diffMat;
  cv::Mat bgr[3];
  cv::split(frame, bgr);
  cv::absdiff(bgr[0], bgr[1], diffMat);

  if (cv::countNonZero(diffMat)) {
    return false;
  }

  cv::absdiff(bgr[0], bgr[2], diffMat);
  return !cv::countNonZero(diffMat);
}

cv::Mat getGrayImage(const cv::Mat& frame) {
  if (isGrayScale(frame)) {
    return frame;
  } else {
    // cv::Mat gray;
    // cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    // return gray;

    // cv::Mat hsv;
    // cv::cvtColor(frame, hsv, cv::COLOR_RGB2HSV);
    // std::vector<cv::Mat> hsvPlanes;
    // cv::split(hsv, hsvPlanes);
    // return hsvPlanes[0];  // hue channel

    cv::Mat hls;
    cv::cvtColor(frame, hls, cv::COLOR_RGB2HLS);
    std::vector<cv::Mat> hlsPlanes;
    cv::split(hls, hlsPlanes);
    return hlsPlanes[1];  // light channel
  }
}

}  // namespace

std::optional<double> LaneDetector::getAverageSlope(const cv::Mat& frame) {
  auto lines = detectLines(frame);
  if (lines.empty()) {
    return std::nullopt;
  }
  return getAverageSlopeImpl(lines);
}

Lane LaneDetector::getLane(const cv::Mat& frame) {
  auto lines = detectLines(frame);
  return getLaneImpl(lines, getFrameSize(frame));
}

std::vector<cv::Vec4i> LaneDetector::detectLines(const cv::Mat& frame) {
  // get only the part of the image relevat for lane detection
  const cv::Mat cropped = cropLaneSection(frame);

  // convert to b&w
  const cv::Mat gray = getGrayImage(cropped);

  // get just the edges in the image
  const cv::Mat edges = getEdges(gray);

  // Hough lines
  std::vector<cv::Vec4i> lines = getLines(edges);

  // save debug images
  if (saveDebugImages_) {
    saveDebugImages(frame, gray, edges, lines);
  }

  return lines;
}

}  // namespace perception
}  // namespace robocar
