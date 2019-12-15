#include "control/control_service.h"

#include <atomic>
#include <cassert>
#include <mutex>

#include "control/motor.h"
#include "control/vehicle.h"
#include "runtime/mpsc_queue.h"
#include "runtime/service_thread.h"

namespace robocar {
namespace control {

ControlService::ControlService()
    : queue_(kTaskQueueSize),
      throttle_(vehicle_.throttle()),
      steeringAngle_(vehicle_.steeringAngle()),
      state_(vehicle_.state()),
      thread_([this]() { loop(); }, kExecutionRateHz) {}

void ControlService::adjustVehicleState(double throttleDelta,
                                        double steeringAngleDelta) {
  bool scheduled = queue_.push([this, throttleDelta, steeringAngleDelta]() {
    double newThrottle = vehicle_.adjustThrottle(throttleDelta);
    double newSteeringAngle = vehicle_.adjustSteeringAngle(steeringAngleDelta);
    throttle_.store(newThrottle, std::memory_order_relaxed);
    steeringAngle_.store(newSteeringAngle, std::memory_order_relaxed);
  });
  assert(scheduled);
}

void ControlService::setThrottle(double throttle) {
  bool scheduled = queue_.push([this, throttle]() {
    vehicle_.setThrottle(throttle);
    throttle_.store(throttle, std::memory_order_relaxed);
  });
  assert(scheduled);
}

void ControlService::setSteeringAngle(double steeringAngle) {
  bool scheduled = queue_.push([this, steeringAngle]() {
    vehicle_.setSteeringAngle(steeringAngle);
    steeringAngle_.store(steeringAngle, std::memory_order_relaxed);
  });
  assert(scheduled);
}

void ControlService::setMotorState(Motor::State state) {
  bool scheduled = queue_.push([this, state]() {
    vehicle_.setState(state);
    state_.store(state, std::memory_order_relaxed);
  });
  assert(scheduled);
}

void ControlService::loop() {
  std::function<void()> task;
  while (queue_.pop(task)) {
    task();
  }
  Motor::applyStateNoDelay();
}

}  // namespace control
}  // namespace robocar
