#pragma once

#include <vector>

#include <opencv2/core/matx.hpp>

#include "inference/object_detector.h"
#include "perception/agent.h"

namespace robocar {
namespace perception {

class AgentDetector {
 public:
   std::vector<Agent> detectAgents(const cv::Mat& frame);

 private:
   inference::ObjectDetector detector_;
};

}  // namespace perception
}  // namespace robocar
