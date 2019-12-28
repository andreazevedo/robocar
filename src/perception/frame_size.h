#pragma once

#include <cassert>

#include <opencv2/core/mat.hpp>

namespace robocar {
namespace perception {

/**
 * The size of the frame that contains the lanes.
 */
struct FrameSize {
  size_t width{0};
  size_t height{0};
};

/**
 * Returns the size of the frame, given a frame.
 *
 * @param frame   The frame.
 *
 * @return        The size of the provided frame.
 */
inline FrameSize getFrameSize(const cv::Mat& frame) {
  assert(frame.cols > 0);
  assert(frame.rows > 0);
  return FrameSize{static_cast<size_t>(frame.cols),
                   static_cast<size_t>(frame.rows)};
}

}  // namespace perception
}  // namespace robocar
