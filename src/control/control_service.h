#pragma once

#include <control/motor.h>

namespace robocar {
namespace control {

/**
 * Service responsible for controlling the car.
 */
class ControlService {
 public:
  constexpr static double kMinSteeringAngle = -90.0;
  constexpr static double kMaxSteeringAngle = +90.0;

  ControlService() noexcept;

  // Sets the state of the motors.
  void setState(Motor::State newState) noexcept;

  // Sets the throtlle percentage.
  //
  // @param throttle  Must be in the [0.0, 1.0] range.
  //
  // @throws  std::out_of_range
  void setThrottle(double throttle);

  // Sets the steering angle.
  //
  // @param angle  Must be between -90.0 and +90.0.
  //
  // @throws std::out_of_range
  void setSteeringAngle(double angle);

  // Adjusts steering angle by "adj".
  //
  // @return  New angle value
  double adjustSteeringAngle(double adj) noexcept;

  // Adjusts throttle percentage.
  //
  // @return  New throttle percentage value.
  double adjustThrottle(double adj) noexcept;

  // Read API
  Motor::State state() const noexcept {
    return state_;
  }
  double throttle() const noexcept {
    return throttle_;
  }
  double steeringAngle() const noexcept {
    return steeringAngle_;
  }

 private:
  Motor frontLeft_;
  Motor frontRight_;
  Motor rearLeft_;
  Motor rearRight_;

  double throttle_{0.0};
  double steeringAngle_{0.0};
  Motor::State state_{Motor::State::Release};

  void applyThrottleAndSteeringAngle() noexcept;
};

}  // namespace control
}  // namespace robocar
