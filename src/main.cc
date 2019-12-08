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
      case 'w':
        control.setThrottle(0.8);
        break;
      case 's':
        control.setThrottle(0.0);
        break;
      case 'a':
        control.setSteeringAngle(-80.0);
        break;
      case 'd':
        control.setSteeringAngle(+80.0);
        break;
    }
  }
}
