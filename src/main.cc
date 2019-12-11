#include <unistd.h>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <thread>

#include "control/control_service.h"
#include "control/motor.h"
#include "control/util.h"

using namespace robocar::control;

int main(int argc, char *argv[]) {
  globalPigpioInitialize();

  ControlService controlService;
  controlService.setMotorState(Motor::State::Forward);

  // Set terminal to raw
  system("/bin/stty raw");
  std::atexit([]() { system("/bin/stty cooked"); });

  char c = '\0';
  while ((c = getchar()) != 'q') {
    switch (c) {
      case 'r':
        controlService.setMotorState(Motor::State::Backward);
        break;
      case 'f':
        controlService.setMotorState(Motor::State::Forward);
        break;
      case 'b':
        controlService.setMotorState(Motor::State::Release);
        break;
      case 'w':
        controlService.adjustVehicleState(+0.1 /* throttle */,
                                          0.0 /* steering */);
        break;
      case 's':
        controlService.adjustVehicleState(-0.1 /* throttle */,
                                          0.0 /* steering */);
        break;
      case 'a':
        controlService.adjustVehicleState(0.0 /* throttle */,
                                          -10.0 /* steering */);
        break;
      case 'd':
        controlService.adjustVehicleState(0.0 /* throttle */,
                                          +10.0 /* steering */);
        break;
      case 'p':
        system("/bin/stty cooked");
        std::cout << std::endl;
        std::cout << "Throttle: " << controlService.throttle()
                  << " | Steering angle: " << controlService.steeringAngle()
                  << " | State: "
                  << Motor::getStateString(controlService.state()) << std::endl;
        system("/bin/stty raw");
        break;
    }
  }
}
