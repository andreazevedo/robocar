#pragma once

namespace robocar {
namespace control {

/**
 * Initializes the pigpio API.
 * Also installs an atexit hook to shutdown the API.
 */
void globalPigpioInitialize();

}  // namespace control
}  // namespace robocar
