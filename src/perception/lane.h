#pragma once

#include "perception/frame_size.h"

namespace robocar {
namespace perception {

/**
 * Represents one of the lines of the lane (either left or right line).
 */
struct LaneLine {
  // Negative slope means the top of the line is more to the right than bottom
  // of the line.
  // Usually left line has negative slope.
  double slope{0.0};
  double intercept{0.0};

  // The std deviation of the slope.
  // Can be used to judge confidence level (lower stddev == higher confidence)
  double slopeStdDeviation{0.0};

  // Number of lines used to form this line
  size_t numLines{0};
};

/**
 * A lane - it might be incomplete if e.g. we camera can't see one of the lines
 */
struct Lane {
  FrameSize frameSize;
  std::optional<LaneLine> left;
  std::optional<LaneLine> right;

  bool isEmpty() const { return !left && !right; }
};

}  // namespace perception
}  // namespace robocar
