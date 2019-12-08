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
    control.setThrottle(0.8);
    sleep(1);
    control.setSteeringAngle(-90.0);  // turn left
    sleep(3);
    control.setSteeringAngle(+90.0);  // turn right
    sleep(3);
  }
  sleep(1);
}
