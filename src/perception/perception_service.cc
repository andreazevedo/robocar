#include "perception/perception_service.h"

#include <vector>

#include "perception/agent_detector.h"
#include "perception/lane_detector.h"
#include "perception/obstacles.h"

namespace robocar {
namespace perception {

Obstacles PerceptionService::detect(const cv::Mat& frame) {
  Obstacles result;

  // first detect the lane.
  result.lane = laneDetector_.getLane(frame);

  // detect other agents in car's vision
  result.agents = agentDetector_.detectAgents(frame);

  return result;
}

}  // namespace perception
}  // namespace robocar

