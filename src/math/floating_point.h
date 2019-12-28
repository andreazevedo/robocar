#pragma once

#include <cmath>
#include <limits>

namespace robocar {
namespace math {

namespace detail {

float constexpr sqrtImpl(float x, float curr, float prev) {
  return curr == prev ? curr : sqrtImpl(x, 0.5 * (curr + x / curr), curr);
}

}  // namespace detail

constexpr double kEpsilon = 1e-5;

inline double equals(double lhs, double rhs) {
  return ::fabs(lhs - rhs) < kEpsilon;
}

inline double isZero(double value) { return equals(value, 0.0); }

/*
 * Constexpr version of the square root.
 * For a finite and non-negative value of "x", returns an approximation for
 * the square root of "x". Otherwise, returns NaN
 */
float constexpr sqrt(float x) {
  return x >= 0 && x < std::numeric_limits<float>::infinity()
             ? detail::sqrtImpl(x, x, 0)
             : std::numeric_limits<float>::quiet_NaN();
}

float constexpr pow2(float x) { return x * x; }

}  // namespace math
}  // namespace robocar
