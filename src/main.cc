#include <unistd.h>
#include <cstdio>
#include <cstdlib>

#include <pigpio.h>

#include "control/motor.h"

using namespace robocar::control;

int main(int argc, char *argv[]) {
  if (gpioInitialise() < 0) return 1;

  {
    Motor motor(2 /* motor ID */);
    motor.setState(Motor::State::Forward);
    motor.setSpeed(255);
    sleep(3);
    motor.setSpeed(200);
    sleep(2);
    motor.setSpeed(150);
    sleep(1);
  }
  sleep(1);

  gpioTerminate();
}
