#include "car.h"

#include <chrono>
#include <iostream>
#include <thread>

#include "control/motor.h"
#include "runtime/service_thread.h"

namespace robocar {

Car::Car(bool debugInfoEnabled)
    : camera_(180),
      thread_([this]() { loop(); }, kExecutionRateHz),
      debugInfoEnabled_(debugInfoEnabled) {
  controlService_.setMotorState(control::Motor::State::Forward);
}

void Car::enableAutonomy() {
  autonomyEnabled_.store(true, std::memory_order_relaxed);
}
void Car::disableAutonomy() {
  autonomyEnabled_.store(false, std::memory_order_relaxed);
  controlService_.setThrottle(0.0);
}

void Car::loop() {
  if (!autonomyEnabled_) {
    return;
  }

  auto frame = camera_.captureFrame();
  auto lines = laneDetector_.detectLines(frame);
  if (lines.empty()) {
    // no lines detected - probably the image is to blurry. Stop the car
    controlService_.setSteeringAngle(0.0);
    controlService_.setThrottle(0.0);
    std::cout << "Lines not found!" << std::endl;
    return;
  }
  double slope = laneDetector_.getAverageSlope(lines);
  planning::PlanningResult plan = planner_.calculateRoute(slope);

  controlService_.setSteeringAngle(plan.steeringAngle);
  controlService_.setThrottle(plan.throttle);

  if (debugInfoEnabled_) {
    std::cout << "Angle: " << plan.steeringAngle << ". Lines: " << lines.size()
              << std::endl;
  }
}

}  // namespace robocar

