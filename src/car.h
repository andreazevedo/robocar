#pragma once

#include "control/control_service.h"
#include "perception/perception_service.h"
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

  /**
   * Engages autonomy mode.
   */
  void startAutonomyLoop();

  /**
   * Disengages autonomy mode.
   */
  void stopAutonomyLoop();

  /**
   * Returns the camera module.
   */
  const sensors::Camera& camera() const { return camera_; }
  sensors::Camera& camera() { return camera_; }

  /**
   * Returns the perception service.
   */
  const perception::PerceptionService& perceptionService() const {
    return perceptionService_;
  }
  perception::PerceptionService& perceptionService() {
    return perceptionService_;
  }

  /**
   * Returns the control service.
   */
  const control::ControlService& controlService() const {
    return controlService_;
  }
  control::ControlService& controlService() { return controlService_; }

  /**
   * Runs the autonomy loop once.
   */
  void loopOnce();

 private:
  /**
   * How frequently to run the autonomy loop, when autonomy in engaged.
   * In hertz.
   */
  static constexpr size_t kExecutionRateHz = 10;

  sensors::Camera camera_;
  perception::PerceptionService perceptionService_;
  planning::Planner planner_;
  control::ControlService controlService_;
  runtime::ServiceThread thread_;
  std::atomic<bool> autonomyEnabled_{false};
  const bool debugInfoEnabled_{false};

  void loop();
};

}  // namespace robocar
