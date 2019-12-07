#include "control/motor.h"

#include <exception>
#include <utility>

#include <pigpio.h>

namespace robocar {
namespace control {

/**
 * This code may be used to drive the Adafruit (or clones) Motor Shield.
 *
 * The code as written only supports DC motors.
 *
 * http://shieldlist.org/adafruit/motor
 *
 * The shield pinouts are
 *
 * D12 MOTORLATCH
 * D11 PMW motor 1
 * D10 Servo 1
 * D9  Servo 2
 * D8  MOTORDATA
 *
 * D7  MOTORENABLE
 * D6  PWM motor 4
 * D5  PWM motor 3
 * D4  MOTORCLK
 * D3  PWM motor 2
 *
 * The motor states (forward, backward, brake, release) are encoded using the
 * MOTOR latch pins.  This saves four gpios.
 */

namespace {

// Assign GPIOs to drive the shield pins

// clang-format off
//                Shield          Pi
constexpr uint8_t kMotorLatch   = 14;
constexpr uint8_t kMotorClk     = 24;
constexpr uint8_t kMotorEnable  = 25;
constexpr uint8_t kMotorData    = 15;

constexpr uint8_t kMotor1Pwd    = 13;
constexpr uint8_t kMotor2Pwd    = 22;
constexpr uint8_t kMotor3Pwd    = 27;
constexpr uint8_t kMotor4Pwd    = 17;
// clang-format on

constexpr uint8_t kMotor1A = 2;
constexpr uint8_t kMotor1B = 3;
constexpr uint8_t kMotor2A = 1;
constexpr uint8_t kMotor2B = 4;
constexpr uint8_t kMotor3A = 5;
constexpr uint8_t kMotor3B = 7;
constexpr uint8_t kMotor4A = 0;
constexpr uint8_t kMotor4B = 6;

constexpr size_t getBit(uint8_t b) { return 1 << b; }

uint8_t getPwdPort(uint8_t motorId) {
  switch (motorId) {
    case 1:
      return kMotor1Pwd;
    case 2:
      return kMotor2Pwd;
    case 3:
      return kMotor3Pwd;
    case 4:
      return kMotor4Pwd;
    default:
      // Invalid motor ID!
      std::terminate();
  }
}

std::pair<uint8_t, uint8_t> getStateBits(uint8_t motorId) {
  switch (motorId) {
    case 1:
      return {kMotor1A, kMotor1B};
    case 2:
      return {kMotor2A, kMotor2B};
    case 3:
      return {kMotor3A, kMotor3B};
    case 4:
      return {kMotor4A, kMotor4B};
    default:
      // Invalid motor ID!
      std::terminate();
  }
}
uint8_t getStateBitA(uint8_t motorId) { return getStateBits(motorId).first; }
uint8_t getStateBitB(uint8_t motorId) { return getStateBits(motorId).second; }

void checkResult(int res) {
  if (res != 0) {
    // Call to PGPIO failed, terminating!
    std::terminate();
  }
}

}  // namespace

Motor::Motor(uint8_t id) noexcept
    : id_{id},
      pwdPort_{getPwdPort(id)},
      stateBitA_{getStateBitA(id)},
      stateBitB_{getStateBitB(id)} {
  init();
}
Motor::~Motor() noexcept {
  setState(State::Release);
  setSpeed(0);
}

void Motor::init() noexcept {
  checkResult(gpioSetMode(kMotorLatch, PI_OUTPUT));
  checkResult(gpioSetMode(kMotorEnable, PI_OUTPUT));
  checkResult(gpioSetMode(kMotorData, PI_OUTPUT));
  checkResult(gpioSetMode(kMotorClk, PI_OUTPUT));
  checkResult(gpioSetMode(pwdPort_, PI_OUTPUT));
  checkResult(gpioPWM(pwdPort_, 0));

  gLatchState &= ~getBit(stateBitA_) & ~getBit(stateBitB_);

  applyState();
}

void Motor::setState(State newState, bool apply) noexcept {
  state_ = newState;
  switch (newState) {
    case State::Forward:
      gLatchState |= getBit(stateBitA_);
      gLatchState &= ~getBit(stateBitB_);
      break;
    case State::Backward:
      gLatchState &= ~getBit(stateBitA_);
      gLatchState |= getBit(stateBitB_);
      break;
    case State::Release:
      gLatchState &= ~getBit(stateBitA_);
      gLatchState &= ~getBit(stateBitB_);
      break;
    case State::Break:
      return;
  }

  if (apply) {
    Motor::applyState();
  }
}

void Motor::setSpeed(uint8_t speed) noexcept {
  speed_ = speed;
  checkResult(gpioPWM(pwdPort_, speed_));
}

/* static */
uint8_t Motor::gLatchState = 0;
/* static */
void Motor::applyState() noexcept {
  checkResult(gpioWrite(kMotorLatch, PI_LOW));
  checkResult(gpioWrite(kMotorData, PI_LOW));

  for (uint8_t i = 0; i < 8; i++) {
    gpioDelay(10);  // 10 micros delay
    checkResult(gpioWrite(kMotorClk, PI_LOW));

    if (gLatchState & getBit(7 - i)) {
      checkResult(gpioWrite(kMotorData, PI_HIGH));
    } else {
      checkResult(gpioWrite(kMotorData, PI_LOW));
    }

    gpioDelay(10);  // 10 micros delay
    checkResult(gpioWrite(kMotorClk, PI_HIGH));
  }

  checkResult(gpioWrite(kMotorLatch, PI_HIGH));
}

}  // namespace control
}  // namespace robocar
