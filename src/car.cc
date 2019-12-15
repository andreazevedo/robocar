#include "car.h"

#include <chrono>
#include <iostream>
#include <thread>

#include "control/motor.h"
#include "runtime/service_thread.h"

namespace robocar {

Car::Car() : camera_(180), thread_([this]() { loop(); }, kExecutionRateHz) {
  controlService_.setMotorState(control::Motor::State::Forward);
}

void Car::loop() {
  if (!autonomyEnabled_) {
    controlService_.setThrottle(0.0);
    return;
  }

  auto frame = camera_.captureFrame();
  auto lines = laneDetector_.detectLines(frame);
  if (lines.empty()) {
    // no lines detected - probably the image is to blurry. Stop the car
    controlService_.setSteeringAngle(0.0);
    controlService_.setThrottle(0.0);
    return;
  }

  double slope = laneDetector_.getFinalSlope(lines);
  double angle = slope * 150;
  angle *= -1;
  angle = std::min(90.0, angle);
  angle = std::max(-90.0, angle);
  std::cout << "Angle: " << angle << ". Lines: " << lines.size() << std::endl;

  double throttle = 0.60;
  if (::abs(angle) > 38) {
    throttle = 0.75;
  }
  controlService_.setSteeringAngle(angle);
  controlService_.setThrottle(throttle);
}

}  // namespace robocar

