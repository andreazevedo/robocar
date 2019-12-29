#include "planning/traffic_light_handler.h"

#include <iostream>

#include "perception/obstacles.h"
#include "planning/agent_tracker.h"

namespace robocar {
namespace planning {

bool TrafficLightHandler::shouldStop(const perception::Obstacles& obstacles) {
  // first, look for green lights - if we see a green light we can proceed.
  auto greenLight =
      obstacles.getClosestAgent(perception::AgentType::TrafficLightGreen);
  if (greenLight.has_value()) {
    stopped_ = false;
    redLight_.reset();
    return false;
  }

  if (stopped_) {
    // we are stopped at a red light and we haven't seen a green light.
    // just keep waiting.
    return true;
  }

  // look for red lights.
  auto redLight =
      obstacles.getClosestAgent(perception::AgentType::TrafficLightRed);
  redLight_.update(redLight);
  if (redLight_.isTracking()) {
    // We are tracking a red light. Stop when it's close enough.
    constexpr float kDistanceToStop = 25.0f;
    float projectedDistance = redLight_.getProjectedDistance();
    std::cout << "Projected distance: " << projectedDistance << std::endl;
    if (projectedDistance < kDistanceToStop) {
      stopped_ = true;
      redLight_.reset();
      return true;
    }
  }

  return false;
}

}  // namespace planning
}  // namespace robocar
