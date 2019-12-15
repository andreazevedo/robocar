#include "car.h"

#include "control/motor.h"

namespace robocar {

Car::Car() : camera_{180}, laneDetector_{true /* saveDebugImages */} {
  controlService_.setMotorState(control::Motor::State::Forward);
}

}  // namespace robocar

