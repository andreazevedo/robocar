#include "planning/agent_tracker.h"

#include <cassert>
#include <optional>
#include <stdexcept>
#include <iostream>

#include "perception/agent.h"
#include "time/time.h"

namespace robocar {
namespace planning {

void AgentTracker::update(std::optional<perception::Agent> agent) noexcept {
  auto now = time::Clock::now();

  if (agent) {
    if (agent_) {
      // calculate approaching speed.
      float distDiffCm = agent_->distanceCm() - agent->distanceCm();
      float timeDiffMs = time::calculateDuration(agentLastSeenAt_, now).count();
      float newSpeed = distDiffCm / timeDiffMs;
      updateSpeed(newSpeed);

      std::cout << "Approaching speed: " << speedCmPerMs_ << std::endl;
    }
    // We just saw a new stop sign, start tracking it.
    agentLastSeenAt_ = now;
    agent_.emplace(*agent);
  }
}

void AgentTracker::updateSpeed(float newSpeed) noexcept {
  speedCmPerMs_ = std::max(speedCmPerMs_, newSpeed);
}

float AgentTracker::getProjectedDistance() const {
  if (!agent_) {
    throw std::runtime_error("Agent cannot be empty!");
  }
  auto now = time::Clock::now();
  float timeElapsedMs = time::calculateDuration(agentLastSeenAt_, now).count();
  return agent_->distanceCm() - (speedCmPerMs_ * timeElapsedMs);
}

}  // namespace planning
}  // namespace robocar
