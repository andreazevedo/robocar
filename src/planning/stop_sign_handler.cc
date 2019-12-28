#include "planning/stop_sign_handler.h"

#include <cassert>
#include <iostream>

#include "math/floating_point.h"
#include "perception/obstacles.h"
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
  if (stopSign) {
    if (stopSign_) {
      // calculate approaching speed.
      float distDiffCm = stopSign_->distanceCm() - stopSign->distanceCm();
      float timeDiffMs = time::calculateDuration(signLastSeenAt_, now).count();
      float newSpeed = distDiffCm / timeDiffMs;
      updateSpeed(newSpeed);
      std::cout << "Approaching speed: " << speedCmPerMs_ << std::endl;
    }
    // We just saw a new stop sign, start tracking it.
    signLastSeenAt_ = now;
    stopSign_.emplace(*stopSign);
  }

  if (stopSign_ && !stopSign) {
    // We were seeing a stop sign before, and now it's out of sight.
    // We might need to stop.
    constexpr float kDistanceToStop = 10.0f;
    float projectedDistance = getProjectedDistance(now);
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

float StopSignHandler::getProjectedDistance(time::Time now) const {
  assert(stopSign_);
  float timeElapsedMs = time::calculateDuration(signLastSeenAt_, now).count();
  return stopSign_->distanceCm() - (speedCmPerMs_ * timeElapsedMs);
}

void StopSignHandler::updateSpeed(float newSpeed) {
  speedCmPerMs_ = std::max(speedCmPerMs_, newSpeed);
}

}  // namespace planning
}  // namespace robocar
