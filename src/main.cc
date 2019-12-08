#include <unistd.h>
#include <cstdio>
#include <cstdlib>

#include "control/control_service.h"
#include "control/motor.h"
#include "control/util.h"

using namespace robocar::control;

int main(int argc, char *argv[]) {
  globalPigpioInitialize();

  {
    ControlService control;
    control.setState(Motor::State::Forward);
    control.setSpeed(150);
    sleep(5);
    control.setSpeed(125);
    sleep(3);
    control.setSpeed(100);
    sleep(2);
  }
  sleep(1);
}
