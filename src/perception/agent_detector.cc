#include "perception/agent_detector.h"

#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "inference/object_detector.h"
#include "inference/util.h"
#include "perception/agent.h"
#include "perception/frame_size.h"

namespace robocar {
namespace perception {

namespace {

void validateLabels(const std::vector<std::string>& labels) {
  if (labels.size() < 1) {
    std::cerr << "Invalid size of labels vector" << std::endl;
    std::terminate();
  }
  if (labels[0] != "stop sign") {
    std::cerr << "Unexpected labels[0]. Actual: " << labels[0] << std::endl;
    std::terminate();
  }
}

inference::ObjectDetector createDetector() {
  auto labels = robocar::inference::loadLabels(
      "models/ssd_mobilenet_robocar/labelmap.txt");
  validateLabels(labels);

  return robocar::inference::ObjectDetector(
      "models/ssd_mobilenet_robocar/model.tflite", true, std::move(labels));
}

AgentType getAgentType(int classId) {
  if (classId == 0) {
    return AgentType::StopSign;
  }
  std::cerr << "Unexpected class id: " << classId << std::endl;
  std::terminate();
}

}  // namespace

AgentDetector::AgentDetector() : detector_(createDetector()) {}

std::vector<Agent> AgentDetector::detectAgents(const cv::Mat& frame) {
  auto objects = detector_.runDetection(frame, nullptr);

  constexpr float kMinScore = 0.75f;
  std::vector<Agent> agents;
  for (const auto& obj : objects) {
    if (obj.score >= kMinScore) {
      auto type = getAgentType(obj.classId);
      agents.emplace_back(type, getFrameSize(frame), obj.location);

      std::cout << "Agent found: " << obj.classId
                << ". Distance: " << agents.back().distanceCm() << std::endl;
    }
  }
  return agents;
}

}  // namespace perception
}  // namespace robocar
