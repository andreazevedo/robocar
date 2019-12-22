#include "car.h"

#include <chrono>
#include <iostream>
#include <thread>

#include "control/motor.h"
#include "planning/plan.h"
#include "runtime/service_thread.h"

namespace robocar {

Car::Car(bool debugInfoEnabled)
    : camera_(180, true /* color */),
      thread_([this]() { loop(); }, kExecutionRateHz),
      debugInfoEnabled_(debugInfoEnabled) {
  controlService_.setMotorState(control::Motor::State::Forward);
}

void Car::enableAutonomy() {
  autonomyEnabled_.store(true, std::memory_order_relaxed);
}
void Car::disableAutonomy() {
  autonomyEnabled_.store(false, std::memory_order_relaxed);
  controlService_.setThrottle(0.0);
}

void Car::loop() {
  if (!autonomyEnabled_) {
    return;
  }

  auto frame = camera_.captureFrame();        // sense
  auto lane = laneDetector_.getLane(frame);   // detect
  auto plan = planner_.calculateRoute(lane);  // plan

  // act
  controlService_.setSteeringAngle(plan.steeringAngle());
  controlService_.setThrottle(plan.throttle());

  if (debugInfoEnabled_) {
    std::cout << "LL: " << (lane.left ? 'Y' : 'N')
              << ". RL: " << (lane.right ? 'Y' : 'N')
              << ". Throttle: " << plan.throttle()
              << ". Angle: " << plan.steeringAngle() << std::endl;

    static size_t loopCount = 0;
    laneDetector_.setSaveDebugImages(++loopCount % 20 == 0);
  }
}

}  // namespace robocar

