#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <iostream>

#include "control/control_service.h"
#include "control/motor.h"
#include "control/util.h"

using namespace robocar::control;

int main(int argc, char *argv[]) {
  globalPigpioInitialize();

  ControlService control;
  control.setState(Motor::State::Forward);

  // Set terminal to raw
  system("/bin/stty raw");
  std::atexit([]() { system("/bin/stty cooked"); });

  char c;
  while ((c = getchar()) != 'q') {
    switch (c) {
      case 'r':
        control.setState(Motor::State::Backward);
        break;
      case 'f':
        control.setState(Motor::State::Forward);
        break;
      case 'b':
        control.setState(Motor::State::Release);
        break;
      case 'w':
        control.adjustThrottle(+0.1);
        break;
      case 's':
        control.adjustThrottle(-0.1);
        break;
      case 'a':
        control.adjustSteeringAngle(-10.0);
        break;
      case 'd':
        control.adjustSteeringAngle(+10.0);
        break;
      case 'p':
        system("/bin/stty cooked");
        std::cout << std::endl;
        std::cout << "Throttle: " << control.throttle()
                  << " | Steering angle: " << control.steeringAngle()
                  << " | State: " << Motor::getStateString(control.state())
                  << std::endl;
        system("/bin/stty raw");
        break;
    }
  }
}
