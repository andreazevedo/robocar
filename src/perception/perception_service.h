#pragma once

#include <vector>

#include <opencv2/core/mat.hpp>

#include "perception/agent_detector.h"
#include "perception/lane_detector.h"
#include "perception/obstacles.h"

namespace robocar {
namespace perception {

class PerceptionService {
 public:
  /**
   * Runs the perception algorithm to detect the environment around the vehicle.
   *
   * @param frame             The frame with the current vision of the car.
   * @param debugInfoEnabled  Whether to enable debug information.
   *
   * @return  All the obstacles detected by perception.
   */
  Obstacles detect(const cv::Mat& frame);

  /**
   * Returns a reference to the lane detector.
   */
  const perception::LaneDetector& laneDetector() const { return laneDetector_; }
  perception::LaneDetector& laneDetector() { return laneDetector_; }

 private:
  LaneDetector laneDetector_;
  AgentDetector agentDetector_;
};

}  // namespace perception
}  // namespace robocar
