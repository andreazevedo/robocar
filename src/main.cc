#include <cstdio>
#include <cstdlib>
#include <iostream>

#include "car.h"
#include "control/motor.h"
#include "control/util.h"

using robocar::control::Motor;

int main(int argc, char *argv[]) {
  robocar::control::globalPigpioInitialize();

  robocar::Car car;

  // Set terminal to raw
  system("/bin/stty raw");
  std::atexit([]() { system("/bin/stty cooked"); });

  char c = '\0';
  while ((c = getchar()) != 'q') {
    switch (c) {
      case 'r':
        car.controlService().setMotorState(Motor::State::Backward);
        break;
      case 'f':
        car.controlService().setMotorState(Motor::State::Forward);
        break;
      case 'b':
        car.controlService().setMotorState(Motor::State::Release);
        break;
      case 'w':
        car.controlService().adjustVehicleState(+0.1 /* throttle */,
                                                0.0 /* steering */);
        break;
      case 's':
        car.controlService().adjustVehicleState(-0.1 /* throttle */,
                                                0.0 /* steering */);
        break;
      case 'a':
        car.controlService().adjustVehicleState(0.0 /* throttle */,
                                                -10.0 /* steering */);
        break;
      case 'd':
        car.controlService().adjustVehicleState(0.0 /* throttle */,
                                                +10.0 /* steering */);
        break;
      case 'p':
        system("/bin/stty cooked");
        std::cout << '\n';
        std::cout << "Throttle: " << car.controlService().throttle()
                  << " | Steering angle: "
                  << car.controlService().steeringAngle() << " | State: "
                  << Motor::getStateString(car.controlService().state())
                  << std::endl;
        system("/bin/stty raw");
        break;
      case 'c':
        auto frame = car.camera().captureFrame();
        car.laneDetector().detect(frame);
        cv::imwrite("bin/images/photo.jpg", frame);
    }
  }
}
