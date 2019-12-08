#include "control/control_service.h"

#include <stdexcept>

namespace robocar {
namespace control {

ControlService::ControlService() noexcept
    : frontLeft_{1 /* motorId */},
      frontRight_{2 /* motorId */},
      rearLeft_{3 /* motorId */},
      rearRight_{4 /* motorId */} {}

void ControlService::setState(Motor::State newState) noexcept {
  state_ = newState;
  frontLeft_.setState(state_, false /* applyState */);
  frontRight_.setState(state_, false /* applyState */);
  rearLeft_.setState(state_, false /* applyState */);
  rearRight_.setState(state_, true /* applyState */);
}

void ControlService::setSpeed(uint8_t speed) noexcept {
  speed_ = speed;
  applySteeringAndSpeed();
}

void ControlService::applySteeringAndSpeed() noexcept {
  int leftSpeed = speed_;
  uint8_t rightSpeed = speed_;

  rearRight_.setSpeed(rightSpeed);
  rearLeft_.setSpeed(leftSpeed);
  frontLeft_.setSpeed(leftSpeed);
  frontRight_.setSpeed(rightSpeed);
}

}  // namespace control
}  // namespace robocar
