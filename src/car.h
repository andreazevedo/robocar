#pragma once

#include "control/control_service.h"
#include "perception/lane_detector.h"
#include "sensors/camera.h"

namespace robocar {

/**
 * Main API for the self-driving car.
 */
class Car {
 public:
  Car();

  const sensors::Camera& camera() const { return camera_; }
  sensors::Camera& camera() { return camera_; }

  const perception::LaneDetector& laneDetector() const { return laneDetector_; }
  perception::LaneDetector& laneDetector() { return laneDetector_; }

  const control::ControlService& controlService() const {
    return controlService_;
  }
  control::ControlService& controlService() { return controlService_; }

 private:
  sensors::Camera camera_;
  perception::LaneDetector laneDetector_;
  control::ControlService controlService_;
};

}  // namespace robocar

