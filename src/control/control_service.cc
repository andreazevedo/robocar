#include "control/control_service.h"

#include <cmath>
#include <stdexcept>

namespace robocar {
namespace control {

ControlService::ControlService() noexcept
    : frontLeft_{1 /* motorId */},
      frontRight_{2 /* motorId */},
      rearLeft_{4 /* motorId */},
      rearRight_{3 /* motorId */} {}

void ControlService::setState(Motor::State newState) noexcept {
  state_ = newState;
  frontLeft_.setState(state_, false /* applyState */);
  frontRight_.setState(state_, false /* applyState */);
  rearLeft_.setState(state_, false /* applyState */);
  rearRight_.setState(state_, true /* applyState */);
}

void ControlService::setThrottle(double throttle) {
  if (throttle < 0.0 || throttle > 1.0) {
    throw std::out_of_range("Allowed range of throtle is [0.0, 1.0].");
  }
  throttle_ = throttle;
  applyThrottleAndSteeringAngle();
}

void ControlService::setSteeringAngle(double angle) {
  if (angle < kMinSteeringAngle || angle > kMaxSteeringAngle) {
    throw std::out_of_range("Steering angle outside of valid range.");
  }
  steeringAngle_ = angle;
  applyThrottleAndSteeringAngle();
}
double ControlService::adjustSteeringAngle(double adj) noexcept {
  double newSteeringAngle = steeringAngle_ + adj;
  newSteeringAngle = std::max(kMinSteeringAngle, newSteeringAngle);
  newSteeringAngle = std::min(kMaxSteeringAngle, newSteeringAngle);

  steeringAngle_ = newSteeringAngle;
  applyThrottleAndSteeringAngle();
  return steeringAngle_;
}
double ControlService::adjustThrottle(double adj) noexcept {
  double newThrottle = throttle_ + adj;
  newThrottle = std::max(0.0, newThrottle);
  newThrottle = std::min(1.0, newThrottle);

  throttle_ = newThrottle;
  applyThrottleAndSteeringAngle();
  return throttle_;
}

void ControlService::applyThrottleAndSteeringAngle() noexcept {
  uint8_t leftSpeed = Motor::kMaxSpeed * throttle_;
  uint8_t rightSpeed = Motor::kMaxSpeed * throttle_;

  if (steeringAngle_ > 0.0) {
    rightSpeed = rightSpeed * (1.0 - (steeringAngle_ / 90.0));
  } else if (steeringAngle_ < 0.0) {
    leftSpeed = leftSpeed * (1.0 - (abs(steeringAngle_) / 90.0));
  }

  rearRight_.setSpeed(rightSpeed);
  rearLeft_.setSpeed(leftSpeed);
  frontLeft_.setSpeed(leftSpeed);
  frontRight_.setSpeed(rightSpeed);
}

}  // namespace control
}  // namespace robocar
