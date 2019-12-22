#pragma once

#include "control/control_service.h"
#include "perception/lane_detector.h"
#include "planning/planner.h"
#include "runtime/service_thread.h"
#include "sensors/camera.h"

namespace robocar {

/**
 * Main API for the self-driving car.
 */
class Car {
 public:
  Car(bool debugInfoEnabled = false);

  void startAutonomyLoop();
  void stopAutonomyLoop();

  const sensors::Camera& camera() const { return camera_; }
  sensors::Camera& camera() { return camera_; }

  const perception::LaneDetector& laneDetector() const { return laneDetector_; }
  perception::LaneDetector& laneDetector() { return laneDetector_; }

  const control::ControlService& controlService() const {
    return controlService_;
  }
  control::ControlService& controlService() { return controlService_; }

  void loopOnce();

 private:
  static constexpr size_t kExecutionRateHz = 6;

  sensors::Camera camera_;
  perception::LaneDetector laneDetector_;
  planning::Planner planner_;
  control::ControlService controlService_;
  runtime::ServiceThread thread_;
  std::atomic<bool> autonomyEnabled_{false};
  const bool debugInfoEnabled_{false};

  void loop();
};

}  // namespace robocar
