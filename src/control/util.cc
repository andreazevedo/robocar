#include "control/util.h"

#include <cstdlib>
#include <exception>

#include <pigpio.h>

namespace robocar {
namespace control {

void globalPigpioInitialize() {
  int res = gpioInitialise();
  if (res == PI_INIT_FAILED) {
    std::terminate();  // Failed to initialize pigpio API.
  }

  std::atexit([]() { gpioTerminate(); });
}

}  // namespace control
}  // namespace robocar

