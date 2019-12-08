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

  // Sets the state of the motors.
  void setState(Motor::State newState) noexcept;

  // Sets the speed of the motors.
  void setSpeed(uint8_t speed) noexcept;

 private:
  Motor frontLeft_;
  Motor frontRight_;
  Motor rearLeft_;
  Motor rearRight_;

  uint8_t speed_{0};
  Motor::State state_{Motor::State::Release};
  double steering_{0.0};

  void applySteeringAndSpeed() noexcept;
};

}  // namespace control
}  // namespace robocar
