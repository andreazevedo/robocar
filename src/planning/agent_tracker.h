#pragma once

#include <optional>

#include "perception/agent.h"
#include "time/time.h"

namespace robocar {
namespace planning {

/**
 * Helper class to track an agent.
 */
class AgentTracker {
 public:
  /**
   * Update this agent.
   * Use this function when the agent is detected.
   *
   * @param agent   The agent.
   */
  void update(std::optional<perception::Agent> agent) noexcept;

  /**
   * Resets the agent.
   * Use this function when, e.g., the vehicle already acted on this agent and
   * moved past it.
   */
  void reset() noexcept { agent_.reset(); }

  /**
   * Calculates the projected distance to the agent.
   */
  float getProjectedDistance() const;

  /**
   * Whether an agent is currently being tracked.
   */
  bool isTracking() const { return agent_.has_value(); }

 private:
  // The currently tracked agent
  std::optional<perception::Agent> agent_;

  // The last time we've seen the agent. Used to calculate speed.
  time::Time agentLastSeenAt_;

  // How fast we are approaching the agent, in centimeters per millisecond.
  float speedCmPerMs_{0.0f};

  // Updates the speed at which the agent is approaching.
  void updateSpeed(float newSpeed) noexcept;
};

}  // namespace planning
}  // namespace robocar
