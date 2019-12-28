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
        float distance) noexcept
      : type_(type), location_(location), distance_(distance) {}

  AgentType type() const noexcept { return type_; }
  inference::ObjectLocation location() const noexcept { return location_; }
  float distance() const noexcept { return distance_; }

 private:
  const AgentType type_;
  const inference::ObjectLocation location_;
  const float distance_;
};

}  // namespace perception
}  // namespace robocar
