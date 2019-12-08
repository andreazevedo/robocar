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
    control.setThrottle(0.6);
    control.setSteeringAngle(0.0);
    sleep(3);
    control.setSteeringAngle(-25.0);
    sleep(3);
    control.setSteeringAngle(+50.0);
    sleep(3);
  }
  sleep(1);
}
