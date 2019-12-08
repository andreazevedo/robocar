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

  char c;
  bool running = true;
  while (running) {
    c = getchar();
    switch (c) {
      case 'q':
        running = false;
        std::cout << "Exiting..." << std::endl;
        break;
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
