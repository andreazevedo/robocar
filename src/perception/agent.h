#pragma once

#include "inference/object_detector.h"

namespace robocar {
namespace perception {

/**
 * The type of the agent.
 */
enum class AgentType { StopSign };

/**
 * An agent that the autonomous vehicle needs to track.
 */
class Agent {
 public:
  Agent(AgentType type, inference::ObjectLocation location,
        float distanceCm) noexcept
      : type_(type), location_(location), distanceCm_(distanceCm) {}

  /**
   * The type of the agent.
   * @see AgentType
   */
  AgentType type() const noexcept { return type_; }

  /**
   * The location of the agent, in relation to the front view of the vehicle.
   * @see inference::ObjectLocation
   */
  inference::ObjectLocation location() const noexcept { return location_; }

  /**
   * The distance of the agent from the front of the vehicle, in centimeters.
   */
  float distanceCm() const noexcept { return distanceCm_; }

 private:
  const AgentType type_;
  const inference::ObjectLocation location_;
  const float distanceCm_;
};

}  // namespace perception
}  // namespace robocar
