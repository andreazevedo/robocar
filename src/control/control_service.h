#pragma once

#include <atomic>

#include "control/motor.h"
#include "control/vehicle.h"
#include "runtime/mpsc_queue.h"
#include "runtime/service_thread.h"

namespace robocar {
namespace control {

/**
 * Service responsible for controlling the car.
 */
class ControlService {
 public:
  ControlService();

  // Adjusts vehicle state with delta values.
  void adjustVehicleState(double throttleDelta, double steeringAngleDelta);
  // Sets vehicle state with absolute values.
  void setThrottle(double throttle);
  void setSteeringAngle(double steeringAngle);
  // Sets the motor state
  void setMotorState(Motor::State state);

  // API to read the current state
  double throttle() const noexcept {
    return throttle_.load(std::memory_order_relaxed);
  }
  double steeringAngle() const noexcept {
    return steeringAngle_.load(std::memory_order_relaxed);
  }
  Motor::State state() const noexcept {
    return state_.load(std::memory_order_relaxed);
  }

 private:
  static constexpr size_t kTaskQueueSize = 1000;
  static constexpr size_t kExecutionRateHz = 100;

  Vehicle vehicle_;
  runtime::MPSCQueue<std::function<void()>> queue_;

  // This class keeps a thread-safe copy of the vehicle state.
  std::atomic<double> throttle_{0.0};
  std::atomic<double> steeringAngle_{0.0};
  std::atomic<Motor::State> state_{Motor::State::Release};

  // The thread must be the first to be destroyed
  runtime::ServiceThread thread_;

  void run();
};

}  // namespace control
}  // namespace robocar
