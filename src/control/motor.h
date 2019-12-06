#include <cstdint>

namespace robocar {
namespace control {

/**
 * Low level API used to control DC motors
 */
class Motor {
 public:
  enum class State : uint8_t {
    Forward = 1,
    Backward = 2,
    Break = 3,
    Release = 4
  };

  /**
   * Constructs a new motor
   *
   * @param id  The id of the motor. Must be an interger from 1 to 4.
   */
  explicit Motor(uint8_t id) noexcept;
  ~Motor() noexcept;

  // Non-copyable but movable
  Motor(const Motor&) = delete;
  const Motor& operator=(const Motor&) = delete;
  Motor(Motor&&) = default;
  Motor& operator=(Motor&&) = default;

  // Sets the state of the motor.
  void setState(State newState) noexcept;

  // Sets the speed of the motor.
  // @param speed   Must be between 0 and 255.
  void setSpeed(uint8_t speed) noexcept;

  // API to query motor data
  uint8_t id() const noexcept { return id_; }
  uint8_t speed() const noexcept { return speed_; }
  State state() const noexcept { return state_; }

 private:
  // The latch state used to control all 4 motors.
  static uint8_t gLatchState;

  // The id of this motor.
  const uint8_t id_{0};

  // Port used to control speed.
  const uint8_t pwdPort_{0};

  // Bits used to control the state of the motor.
  const uint8_t stateBitA_{0};
  const uint8_t stateBitB_{0};

  // The current state of the motor.
  State state_{State::Release};

  // The current speed of the motor.
  uint8_t speed_{0};

  // Inits this motor
  void init() noexcept;

  // Applies latch state to all motors!
  static void latchTx() noexcept;
};

}  // namespace control
}  // namespace robocar
