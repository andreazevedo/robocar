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
    //Motor motor(2 /* motor ID */);
    //motor.setState(Motor::State::Forward);
    //motor.setSpeed(255);
    //sleep(3);
    //motor.setSpeed(200);
    //sleep(2);
    //motor.setSpeed(150);
    //sleep(1);

    ControlService control;
    control.setState(Motor::State::Forward);
    control.setSpeed(255);
    sleep(5);
    control.setSpeed(200);
    sleep(3);
    control.setSpeed(150);
    sleep(2);
  }
  sleep(1);
}
