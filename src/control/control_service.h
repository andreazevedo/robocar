#pragma once

#include <control/motor.h>

namespace robocar {
namespace control {

/**
 * Service responsible for controlling the car.
 */
class ControlService {
 public:
  ControlService() noexcept;

  void setState(Motor::State newState) noexcept;
  void setSpeed(uint8_t speed) noexcept;

 private:
  Motor frontLeft_;
  Motor frontRight_;
  Motor rearLeft_;
  Motor rearRight_;
};

}  // namespace control
}  // namespace robocar
