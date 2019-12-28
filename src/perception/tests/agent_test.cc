#include <gtest/gtest.h>

#include "perception/agent.h"

using robocar::inference::ObjectLocation;
using robocar::perception::AgentType;
using robocar::perception::FrameSize;

namespace {

// 5% error allowed
constexpr float kErrorMargin = 0.05f;

}  // namespace

TEST(Agent, getDistanceToAgentCm_superShort) {
  FrameSize frameSize{300, 300};
  ObjectLocation location{0.315495, 0.551075, 0.922918, 0.748338};

  constexpr float kExpectedDistance = 15.0f;
  float actualDistance =
      getDistanceToAgentCm(AgentType::StopSign, frameSize, location);
  EXPECT_NEAR(kExpectedDistance, actualDistance,
              kExpectedDistance * kErrorMargin);
}

TEST(Agent, getDistanceToAgentCm_short) {
  FrameSize frameSize{300, 300};
  ObjectLocation location{0.321324, 0.85275, 0.62492, 0.98332};

  constexpr float kExpectedDistance = 30.0f;
  float actualDistance =
      getDistanceToAgentCm(AgentType::StopSign, frameSize, location);
  EXPECT_NEAR(kExpectedDistance, actualDistance,
              kExpectedDistance * kErrorMargin);
}

TEST(Agent, getDistanceToAgentCm_large) {
  FrameSize frameSize{300, 300};
  ObjectLocation location{0.342324, 0.708037, 0.482214, 0.780861};

  constexpr float kExpectedDistance = 60.0f;
  float actualDistance =
      getDistanceToAgentCm(AgentType::StopSign, frameSize, location);
  EXPECT_NEAR(kExpectedDistance, actualDistance,
              kExpectedDistance * kErrorMargin);
}
