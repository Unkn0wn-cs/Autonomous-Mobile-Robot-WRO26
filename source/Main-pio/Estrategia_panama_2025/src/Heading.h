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
// SIGN CONVENTION (shared by the regulator and the routines):
//   the reading INCREASES when the wheels run the rotateCW() pattern
//   (m1 B, m2 F, m3 F, m4 B) and DECREASES for rotateCCW() (F, B, B, F).
//   Routines 7/8 and the heading-hold loop both depend on this. square_test
//   prints a verdict after every turn; if it says FLIP, change HEADING_SIGN in
//   Heading.cpp.
//
// Two reference points are kept, because two parts of the firmware need
// different ones:
//   TARGET  captured at the start of every straight or strafe by move.h, so the
//           regulator can hold the heading that move began on.
//   ZERO    set when the back microswitch confirms the robot is square against
//           a wall, so routines 7 and 8 can turn a known angle away from it.
//
// FAIL-SAFES:
//   * no report for 100 ms  -> headingError() returns 0 (heading hold idles)
//   * no report for 1000 ms -> headingAvailable() returns false (routines 7/8
//                              fall back to encoder-counted turns)
//   * sensor reset          -> its reference frame is new, so both references
//                              are re-captured from the first report after it
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
