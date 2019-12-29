#pragma once

#include "inference/object.h"
#include "perception/frame_size.h"

namespace robocar {
namespace perception {

/**
 * The type of the agent.
 */
enum class AgentType { StopSign, TrafficLightRed, TrafficLightGreen };

/**
 * An agent that the autonomous vehicle needs to track.
 */
class Agent {
 public:
  /**
   * Creates a new agent.
   *
   * @param type        The type of the agent.
   * @param frameSize   The size of the frame where the agent was detected.
   * @param location    The location of the agent with regards to the frame.
   */
  Agent(AgentType type, const FrameSize& frameSize,
        const inference::ObjectLocation& location) noexcept;

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

/**
 * Calculates the distance from the front of the vehicle to the agent,
 * in centimeters.
 *
 * @param type          The type of the agent.
 * @param frameSize     The size of the frame that included the agent.
 *                      @see FrameSize
 * @param location      The position of the agent in the frame.
 *                      @see inference::ObjectLocation.
 *
 * @return            The distance (in centimeters) from the vehicle to
 *                    the agent.
 */
float getDistanceToAgentCm(AgentType type, const FrameSize& frameSize,
                           const inference::ObjectLocation& location);

}  // namespace perception
}  // namespace robocar
