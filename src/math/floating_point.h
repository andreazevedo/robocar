#pragma once

#include <cmath>

namespace robocar {
namespace math {

constexpr double kEpsilon = 1e-5;

inline double equals(double lhs, double rhs) {
  return ::fabs(lhs - rhs) < kEpsilon;
}

inline double isZero(double value) { return equals(value, 0.0); }

}  // namespace math
}  // namespace robocar
