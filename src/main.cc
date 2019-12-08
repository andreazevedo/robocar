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

  std::atomic<char> input{'0'};
  std::thread controlThread([&input]() {
    ControlService control;
    control.setState(Motor::State::Forward);
    char c = input;
    while (c != 'q') {
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
      input = '0';
      Motor::applyStateNoDelay();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      c = input;
    }
  });

  // Set terminal to raw
  system("/bin/stty raw");
  std::atexit([]() { system("/bin/stty cooked"); });
  while ((input = getchar()) != 'q') {
  }

  controlThread.join();
}
