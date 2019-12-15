#include "car.h"

#include <chrono>
#include <iostream>
#include <thread>

#include "control/motor.h"
#include "runtime/service_thread.h"

namespace robocar {

Car::Car()
    : camera_(180),
      laneDetector_(false /* saveDebugImages */),
      thread_([this]() { loop(); }, kExecutionRateHz) {
  controlService_.setMotorState(control::Motor::State::Forward);
}

void Car::loop() {
  if (!autonomyEnabled_) {
    controlService_.setThrottle(0.0);
    return;
  }

  auto frame = camera_.captureFrame();
  double angle = laneDetector_.getSteeringAngle(frame);
  angle *= 90;
  angle = std::min(90.0, angle);
  angle = std::max(-90.0, angle);
  std::cout << "Angle: " << angle << std::endl;

  controlService_.setSteeringAngle(angle);
  double throttle = 0.5;
  if (::abs(angle) > 40) {
    throttle = 0.6;
  }
  controlService_.setThrottle(0.55);
}

}  // namespace robocar

