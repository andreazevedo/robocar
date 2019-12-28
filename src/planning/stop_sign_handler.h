#pragma once

#include <optional>

#include "perception/obstacles.h"
#include "time/time.h"

namespace robocar {
namespace planning {

/**
 * Handles stop signs.
 */
class StopSignHandler {
 public:
  bool shouldStop(const perception::Obstacles& obstacles);

 private:
  // The time the vehicle should wait at the stop sign before proceding.
  static constexpr time::DurationMs kTimeToWait{2000};

  // The currently tracked stop sign.
  std::optional<perception::Agent> stopSign_;
  // The last time we've seen the stop sign. Used to calculate speed.
  time::Time signLastSeenAt_;
  // How fast the stop sign is approaching, in cm / ms.
  float speedCmPerMs_{0.0f};

  // Whether we are currently stopped at a stop sign.
  bool stopped_{false};
  // When we stopped at the stop sign.
  time::Time stoppedAt_;

  float getProjectedDistance(time::Time now) const;
  void updateSpeed(float newSpeed);
};

}  // namespace planning
}  // namespace robocar
