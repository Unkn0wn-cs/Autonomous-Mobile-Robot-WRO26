// Heading.h - heading from the BNO08x.
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
//   every turn; if it says FLIP, change HEADING_SIGN in Heading.cpp.
//
// Three reference points are kept:
//   TARGET  captured at the start of every straight or strafe by move.h, so the
//           regulator can hold the heading that move began on.
//   ZERO    set when the back microswitch confirms the robot is square against
//           a wall; headingSinceZero() is the heading relative to that wall.
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

#pragma once

#include <Arduino.h>

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
