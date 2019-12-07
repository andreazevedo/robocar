#pragma once

#include <control/motor.h>

namespace robocar {
namespace control {

/**
 * Service responsible for controlling the car.
 */
class ControlService {
  ControlService() noexcept;

 private:
  Motor frontLeft_;
  Motor frontRight_;
  Motor rearLeft_;
  Motor rearRight_;
};

}  // namespace control
}  // namespace robocar
