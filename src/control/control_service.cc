#include "control/control_service.h"

namespace robocar {
namespace control {

ControlService::ControlService() noexcept
    : frontLeft_{1 /* motorId */},
      frontRight_{2 /* motorId */},
      rearLeft_{3 /* motorId */},
      rearRight_{4 /* motorId */} {}

void ControlService::setState(Motor::State newState) noexcept {
  frontLeft_.setState(newState, false /* applyState */);
  frontRight_.setState(newState, false /* applyState */);
  rearLeft_.setState(newState, false /* applyState */);
  rearRight_.setState(newState, true);
}

void ControlService::setSpeed(uint8_t speed) noexcept {
  frontLeft_.setSpeed(speed);
  frontRight_.setSpeed(speed);
  rearLeft_.setSpeed(speed);
  rearRight_.setSpeed(speed);
}

}  // namespace control
}  // namespace robocar
