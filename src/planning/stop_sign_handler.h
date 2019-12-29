#pragma once

#include "perception/obstacles.h"
#include "planning/agent_tracker.h"
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
  static constexpr time::DurationMs kTimeToWait{1500};

  // The currently tracked stop sign.
  AgentTracker stopSign_;

  // Whether we are currently stopped at a stop sign.
  bool stopped_{false};

  // When we stopped at the stop sign.
  time::Time stoppedAt_;
};

}  // namespace planning
}  // namespace robocar
