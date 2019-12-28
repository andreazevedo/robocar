#pragma once

#include <chrono>

namespace robocar {
namespace time {

using Clock = std::chrono::steady_clock;
using Time = std::chrono::time_point<Clock>;
using DurationMs = std::chrono::milliseconds;

inline DurationMs constexpr calculateDuration(Time start, Time end) noexcept {
  return std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
}

}  // namespace time
}  // namespace robocar
