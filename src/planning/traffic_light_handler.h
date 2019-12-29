#pragma once

#include "perception/obstacles.h"
#include "planning/agent_tracker.h"

namespace robocar {
namespace planning {

/**
 * Handles stop signs.
 */
class TrafficLightHandler {
 public:
  bool shouldStop(const perception::Obstacles& obstacles);

 private:
  // The currently tracked traffic light.
  // Note that we only track red lights. We only care about green lights when
  // we are already stopped at a red light.
  AgentTracker redLight_;

  // Whether we are currently stopped at a red light.
  bool stopped_{false};
};

}  // namespace planning
}  // namespace robocar
