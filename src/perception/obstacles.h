#pragma once

#include <optional>
#include <vector>

#include "perception/agent.h"
#include "perception/lane.h"

namespace robocar {
namespace perception {

/**
 * Output of the perception service.
 */
struct Obstacles {
  Lane lane;
  std::vector<Agent> agents;

  /**
   * Returns the closest agent of a certain type, or null if none is found.
   *
   * @param type  The type of the agent.
   *
   * @return  The closest agent of the given type, or nullopt_t if not found.
   */
  inline std::optional<Agent> getClosestAgent(AgentType type) const noexcept {
    std::optional<Agent> closestAgent;
    for (const auto& agent : agents) {
      if (agent.type() == type) {
        if (!closestAgent || agent.distanceCm() < closestAgent->distanceCm()) {
          closestAgent.emplace(agent);
        }
      }
    }
    return closestAgent;
  }
};

}  // namespace perception
}  // namespace robocar
