#include "perception/agent.h"

#include <cmath>
#include <iostream>

#include "inference/object.h"
#include "math/floating_point.h"
#include "perception/frame_size.h"
#include "sensors/camera.h"

namespace robocar {
namespace perception {

namespace {

float getAgentRealWorldDiagonalSizeCm(AgentType type) {
  switch (type) {
    case AgentType::StopSign:
      constexpr float kStopSignWidthCm = 2.8f;
      constexpr float kStopSignHeightCm = 6.4f;
      constexpr float kStopSignDiagonalSizeCm =
          math::sqrt((kStopSignWidthCm * kStopSignWidthCm) +
                     (kStopSignHeightCm * kStopSignHeightCm));
      return kStopSignDiagonalSizeCm;
  }
  std::cerr << "Unpexpected agent type!" << std::endl;
  std::terminate();
}

float getAgentDiagonalSizeInPixels(const FrameSize& frameSize,
                                   const inference::ObjectLocation& location) {
  const float widthPercent = location.right - location.left;
  const float heightPercent = location.bottom - location.top;
  const float width = widthPercent * frameSize.width;
  const float height = heightPercent * frameSize.height;
  const float diagonalSize = ::sqrt((width * width) + (height * height));
  return diagonalSize;
}

}  // namespace

float getDistanceToAgentCm(AgentType type, const FrameSize& frameSize,
                           const inference::ObjectLocation& location) {
  const float agentRealWorldDiagonalSizeCm =
      getAgentRealWorldDiagonalSizeCm(type);
  const float agentDiagonalSizePx =
      getAgentDiagonalSizeInPixels(frameSize, location);
  return sensors::Camera::distanceToObjectMm(
             agentRealWorldDiagonalSizeCm * 10.0f, agentDiagonalSizePx) /
         10.0f;
}

}  // namespace perception
}  // namespace robocar
