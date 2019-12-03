#include <unistd.h>
#include <cstdio>
#include <cstdlib>

#include <pigpio.h>

/*
   This code may be used to drive the Adafruit (or clones) Motor Shield.

   The code as written only supports DC motors.

   http://shieldlist.org/adafruit/motor

   The shield pinouts are

   D12 MOTORLATCH
   D11 PMW motor 1
   D10 Servo 1
   D9  Servo 2
   D8  MOTORDATA

   D7  MOTORENABLE
   D6  PWM motor 4
   D5  PWM motor 3
   D4  MOTORCLK
   D3  PWM motor 2

   The motor states (forward, backward, brake, release) are encoded using the
   MOTOR_ latch pins.  This saves four gpios.
*/

typedef unsigned char uint8_t;

#define BIT(bit) (1 << (bit))

/* assign gpios to drive the shield pins */

/*      Shield      Pi */

#define MOTORLATCH 14
#define MOTORCLK 24
#define MOTORENABLE 25
#define MOTORDATA 15

#define MOTOR_1_PWM 13
//#define MOTOR_3_PWM 13
//#define MOTOR_4_PWM  8

/*
   The only other connection needed between the Pi and the shield
   is ground to ground. I used Pi P1-6 to shield gnd (next to D13).
*/

/* assignment of motor states to latch */

#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR4_A 0
#define MOTOR4_B 6
#define MOTOR3_A 5
#define MOTOR3_B 7

#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define RELEASE 4

static uint8_t latch_state;

void checkResult(int res) {
  if (res != 0) {
    printf("Call failed! Aborting!");
    abort();
  }
}

void latch_tx(void) {
  unsigned char i;

  checkResult(gpioWrite(MOTORLATCH, PI_LOW));

  checkResult(gpioWrite(MOTORDATA, PI_LOW));

  for (i = 0; i < 8; i++) {
    gpioDelay(10);  // 10 micros delay

    checkResult(gpioWrite(MOTORCLK, PI_LOW));

    if (latch_state & BIT(7 - i))
      checkResult(gpioWrite(MOTORDATA, PI_HIGH));
    else
      checkResult(gpioWrite(MOTORDATA, PI_LOW));

    gpioDelay(10);  // 10 micros delay

    checkResult(gpioWrite(MOTORCLK, PI_HIGH));
  }

  checkResult(gpioWrite(MOTORLATCH, PI_HIGH));
}

void init(void) {
  latch_state = 0;

  latch_tx();

  gpioWrite(MOTORENABLE, PI_LOW);
}

void DCMotorInit(uint8_t num) {
  switch (num) {
    case 1:
      latch_state &= ~BIT(MOTOR1_A) & ~BIT(MOTOR1_B);
      break;
    case 2:
      latch_state &= ~BIT(MOTOR2_A) & ~BIT(MOTOR2_B);
      break;
    case 3:
      latch_state &= ~BIT(MOTOR3_A) & ~BIT(MOTOR3_B);
      break;
    case 4:
      latch_state &= ~BIT(MOTOR4_A) & ~BIT(MOTOR4_B);
      break;
    default:
      return;
  }

  latch_tx();

  printf("Latch=%08X\n", latch_state);
}

void DCMotorRun(uint8_t motornum, uint8_t cmd) {
  uint8_t a, b;

  switch (motornum) {
    case 1:
      a = MOTOR1_A;
      b = MOTOR1_B;
      break;
    case 2:
      a = MOTOR2_A;
      b = MOTOR2_B;
      break;
    case 3:
      a = MOTOR3_A;
      b = MOTOR3_B;
      break;
    case 4:
      a = MOTOR4_A;
      b = MOTOR4_B;
      break;
    default:
      return;
  }

  switch (cmd) {
    case FORWARD:
      latch_state |= BIT(a);
      latch_state &= ~BIT(b);
      break;
    case BACKWARD:
      latch_state &= ~BIT(a);
      latch_state |= BIT(b);
      break;
    case RELEASE:
      latch_state &= ~BIT(a);
      latch_state &= ~BIT(b);
      break;
    default:
      return;
  }

  latch_tx();

  printf("Latch=%08X\n", latch_state);
}

int main(int argc, char *argv[]) {
  int i;

  if (gpioInitialise() < 0) return 1;

  checkResult(gpioSetMode(MOTORLATCH, PI_OUTPUT));
  checkResult(gpioSetMode(MOTORENABLE, PI_OUTPUT));
  checkResult(gpioSetMode(MOTORDATA, PI_OUTPUT));
  checkResult(gpioSetMode(MOTORCLK, PI_OUTPUT));

  checkResult(gpioSetMode(MOTOR_1_PWM, PI_OUTPUT));

  checkResult(gpioPWM(MOTOR_1_PWM, 0));

  init();

  // Start motor
  checkResult(gpioPWM(MOTOR_1_PWM, 255));
  DCMotorRun(1, FORWARD);

  sleep(2);

  checkResult(gpioPWM(MOTOR_1_PWM, 128));
  sleep(1);

  // Stop and release motor
  DCMotorRun(1, RELEASE);
  checkResult(gpioPWM(MOTOR_1_PWM, 0));

  usleep(100 * 1000);

  gpioTerminate();
}
