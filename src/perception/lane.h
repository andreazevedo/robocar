#pragma once

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
 * Represents one of the lines of the lane (either left or right line).
 */
struct LaneLine {
  double slope{0.0};
  double intercept{0.0};

  // The std deviation of the slope.
  // Can be used to judge confidence level (lower stddev == higher confidence)
  double stdDeviation{0.0};
};

/**
 * A lane - it might be incomplete if e.g. we camera can't see one of the lines
 */
struct Lane {
  FrameSize frameSize;
  std::optional<LaneLine> left;
  std::optional<LaneLine> right;
};

}  // namespace perception
}  // namespace robocar
