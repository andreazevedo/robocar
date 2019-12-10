#include <unistd.h>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <thread>

#include "control/vehicle.h"
#include "control/motor.h"
#include "control/util.h"

using namespace robocar::control;

int main(int argc, char *argv[]) {
  globalPigpioInitialize();

  std::atomic<char> input{'0'};
  std::thread controlThread([&input]() {
    Vehicle vehicle;
    vehicle.setState(Motor::State::Forward);
    char c = input;
    while (c != 'q') {
      switch (c) {
        case 'r':
          vehicle.setState(Motor::State::Backward);
          break;
        case 'f':
          vehicle.setState(Motor::State::Forward);
          break;
        case 'b':
          vehicle.setState(Motor::State::Release);
          break;
        case 'w':
          vehicle.adjustThrottle(+0.1);
          break;
        case 's':
          vehicle.adjustThrottle(-0.1);
          break;
        case 'a':
          vehicle.adjustSteeringAngle(-10.0);
          break;
        case 'd':
          vehicle.adjustSteeringAngle(+10.0);
          break;
        case 'p':
          system("/bin/stty cooked");
          std::cout << std::endl;
          std::cout << "Throttle: " << vehicle.throttle()
                    << " | Steering angle: " << vehicle.steeringAngle()
                    << " | State: " << Motor::getStateString(vehicle.state())
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
