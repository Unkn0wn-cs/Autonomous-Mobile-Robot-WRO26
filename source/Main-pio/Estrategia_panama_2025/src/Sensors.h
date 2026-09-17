// Sensors.h - everything the robot senses with: the four wheel encoders, the
// BNO08x heading sensor, the Pixy2 camera and the microswitches, plus the
// boot-time I2C bus scan.
//
// Team Outer Heaven - WRO 2026.
//
// The actuators (motors, servo, rotor), the robot selector and its tuned
// values are in Hardware.h.
//
// SENSOR PINS (the actuator pins and the timer map are in Hardware.h):
//   14            start switch
//   18, 19        back / side microswitches (polled, NOT interrupts)
//   20, 21        I2C SDA / SCL - BNO08x heading sensor
//   50-53         SPI - Pixy2 camera (SS = 53 on the Mega)
//   A8-A15        the four quadrature encoders - PORTK is full
//
// This file and Sensors.cpp depend on nothing else in src/, so a bench test
// can compile them on their own (heading_test does).

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <QuadratureEncoder.h>
#include <Pixy2.h>

// ---------------------------------------------------------------------------
// Wheel encoders - one quadrature encoder per motor (motor layout in
// Hardware.h).
//
// The Encoders constructor assigns its interrupt slot from a STATIC COUNTER,
// so the declaration order in Sensors.cpp decides which slot each one gets.
// All four live in one translation unit precisely so that order is guaranteed.
// Do not reorder them, and do not split them across files.
// ---------------------------------------------------------------------------

extern Encoders encoderLeft;       // A15, A14  motor3, front left  - measures distance
extern Encoders encoderRight;      // A13, A12  motor4, front right - measures distance
extern Encoders encoderRearRight;  // A11, A10  motor1, rear right  - speed only
extern Encoders encoderRearLeft;   // A9,  A8   motor2, rear left   - speed only

// ---------------------------------------------------------------------------
// Heading - BNO08x.
//
// Mounted horizontally, chip side up, on I2C (SDA pin 20, SCL pin 21). Yaw about
// the vertical axis is the only value used.
//
// Uses the 6-axis GAME rotation vector (accelerometer + gyroscope, no
// magnetometer): the motors and chassis sit centimetres from the sensor and
// would corrupt a magnetometer. Heading is therefore RELATIVE to power-on,
// which is all "drive straight" and "turn 80 degrees" need.
//
// The sensor is used to hold a heading while the robot translates. Turning
// is done by encoder count (move.rotate()); the sensor never drives a turn.
//
// SIGN CONVENTION:
//   the reading INCREASES when the wheels run the B F F B rotation pattern
//   (rotate(x, false)) and DECREASES for F B B F (rotate(x, true)). The
//   heading-hold loop depends on this: its positive differential drives
//   F B B F and must lower a positive error. square_test prints a verdict after
//   every turn; if it says FLIP, change HEADING_SIGN in Sensors.cpp.
//
// Three reference points are kept:
//   TARGET  captured at the start of every straight or strafe by move.h, so the
//           regulator can hold the heading that move began on.
//   ZERO    set when the back microswitch confirms the robot is square against
//           the back wall, and at power-on; headingSinceZero() is the heading
//           relative to that wall - the mat's north. The general strategy's
//           heading recovery (routine 8) measures against it.
//   BOOT    the heading at power-on. Only the telemetry line reads it, to show
//           how far the robot has turned since it was switched on.
//
// FAIL-SAFES:
//   * no report for 100 ms  -> headingError() returns 0 (heading hold idles)
//   * no report for 1000 ms -> headingAvailable() returns false; the
//                              since-zero / since-boot readings return 0
//   * sensor reset          -> its reference frame is new, so all three
//                              references are re-captured from the first
//                              report after it
//   * I2C bus stuck         -> Wire times out and resets the bus instead of
//                              hanging the firmware
// ---------------------------------------------------------------------------

// Starts the sensor (tries 0x4A then 0x4B), sets the I2C clock and timeout,
// subscribes to the rotation report and captures both references. Returns
// false if the sensor does not answer; every function below then returns
// 0 / false and movement runs without heading hold.
bool headingBegin();

// Polls the sensor. Rate-limits itself, so it can be called from anywhere -
// loop() calls it, and so does the regulator's heading hook.
void headingUpdate();

bool     headingAvailable();   // sensor present, has reported, and not stale
float    headingNow();         // degrees, 0-360, sensor's own sign
uint8_t  headingAddress();     // 0x4A, 0x4B, or 0
uint8_t  headingAccuracy();    // sensor's own confidence, 0-3
unsigned long headingAgeMs();  // ms since the last report
unsigned long headingReportCount(); // reports received since headingBegin()
unsigned long headingResetCount();  // sensor resets seen since headingBegin()

// Movement-layer reference: hold the heading a move started on.
void  headingCaptureTarget();
float headingError();          // degrees from target, -180..+180, 0 when stale

// Routine-layer reference: the heading when the robot last squared on a wall.
void  headingZero();
float headingSinceZero();      // degrees from zero, -180..+180

// Telemetry reference: the heading at power-on.
float headingSinceBoot();      // degrees from boot, -180..+180, 0 when stale

// ---------------------------------------------------------------------------
// Pixy2 camera (SPI) and its colour signatures. cameraBegin() opens the link;
// generalStrategy.cpp reads the blocks.
// ---------------------------------------------------------------------------

extern Pixy2 pixy;

const int purpleSignature = 2;
const int orangeSignature = 1;

// pixy.init(), keeping its result and the firmware version for the status
// block. Returns PIXY_RESULT_OK (0) or a negative Pixy2 error; blocks up to
// 5 s when the camera does not answer. Call once from setup().
int8_t cameraBegin();

// ---------------------------------------------------------------------------
// Microswitches. Active LOW with the internal pull-up; polled, never on an
// interrupt. handleMicroSwitches() in generalStrategy.cpp reads the two
// bumpers and zeroes the heading on the back one; setup() waits on the start
// switch.
// ---------------------------------------------------------------------------

const byte backSwitchPin = 18;
const byte sideSwitchPin = 19;
const byte switchPin     = 14;   // start switch

// Sets the three switch pins as inputs with pull-ups. The encoders configure
// their own pins when constructed; the BNO08x and the Pixy2 have headingBegin()
// and pixy.init(). Call once from setup().
void initSensors();

// ---------------------------------------------------------------------------
// I2C bus scan
// ---------------------------------------------------------------------------

// Scans the whole I2C bus and prints what it finds. Boot-time diagnostic only;
// slow (126 transactions), never call it from loop().
int testI2C();

// ---------------------------------------------------------------------------
// Telemetry, on Serial (USB) and Serial2 (the Bluetooth module on TX2 pin 16 /
// RX2 pin 17, at BLUETOOTH_BAUD in Sensors.cpp).
//
// First a STATUS BLOCK - which robot, every sensor's state, the opening
// routine and a verdict - at the end of setup() and again 5 s later, so a
// phone that connects late still sees it. Then a TABLE of the drive state,
// one row every 250 ms, header repeated every 10 rows. Every value that has
// a direction carries its sign, so a column never shifts between + and -:
//
//     r   s |    hdg     err corr | pwm  m1  m2  m3  m4 | mm/s    m1    m2    m3    m4
//     4   2 |   -1.2   -0.35   +3 |     232 240 228 235 |       +310  -305  +312  -300
//
//   r s   routine and state (a bench test shows its own cycle and step)
//   hdg   heading since power-on, degrees      err   error the regulator sees
//   corr  heading differential, whole PWM      pwm   motor1..motor4, 0 = released
//   mm/s  motor1..motor4, signed by encoder direction
//
// Never blocks: every call sends at most one line, and only when both ports
// have room for all of it. Whatever is not a sensor is handed in as an
// argument, so this file stays free of the drive layer and the strategy.
// ---------------------------------------------------------------------------

// Opens Serial2 and queues the status block. `robot` names the build in the
// banner, `strategy` the compiled strategy's name, `straightMM` is the main
// straight, `mmPerCount` converts encoder counts to mm/s, `openingRoutine` is
// what selectOpeningRoutine() chose (0-3, or 4 for no ball). Call once at the
// end of setup().
void telemetryBegin(const char* robot, const char* strategy, int straightMM,
                    float mmPerCount, int openingRoutine);

// One call per loop() pass. pwm[] is motor1..motor4 (0 = released) and
// headingCorr the PWM the heading loop is adding.
void telemetryUpdate(int routine, int state, const int pwm[4], float headingCorr);
