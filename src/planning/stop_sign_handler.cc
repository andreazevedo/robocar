#include "planning/stop_sign_handler.h"

#include <cassert>
#include <iostream>

#include "perception/obstacles.h"
#include "planning/agent_tracker.h"
#include "time/time.h"

namespace robocar {
namespace planning {

bool StopSignHandler::shouldStop(const perception::Obstacles& obstacles) {
  auto now = time::Clock::now();

  // handle the case we are already stopped.
  if (stopped_) {
    if (time::calculateDuration(stoppedAt_, now) >= kTimeToWait) {
      stopped_ = false;
      return false;
    }
    return true;
  }

  auto stopSign = obstacles.getClosestAgent(perception::AgentType::StopSign);
  stopSign_.update(stopSign);

  if (stopSign_.isTracking()) {
    // We are tracking a stop sign. Stop when it's close enough.
    constexpr float kDistanceToStop = 12.0f;
    float projectedDistance = stopSign_.getProjectedDistance();
    std::cout << "Projected distance: " << projectedDistance << std::endl;
    if (projectedDistance < kDistanceToStop) {
      stopped_ = true;
      stoppedAt_ = now;
      stopSign_.reset();
      return true;
    }
  }
  return false;
}

}  // namespace planning
}  // namespace robocar
