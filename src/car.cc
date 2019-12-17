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
  std::optional<double> slope = laneDetector_.getAverageSlope(frame);
  if (!slope) {
    // no lines detected - probably the image is to blurry. Stop the car
    controlService_.setSteeringAngle(0.0);
    controlService_.setThrottle(0.0);
    std::cout << "Couldn't compute slope!" << std::endl;
    return;
  }
  planning::PlanningResult plan = planner_.calculateRoute(*slope);

  controlService_.setSteeringAngle(plan.steeringAngle);
  controlService_.setThrottle(plan.throttle);

  if (debugInfoEnabled_) {
    std::cout << "Angle: " << plan.steeringAngle << "." << std::endl;
  }
}

}  // namespace robocar

