// strafe_test.cpp - strafe bench test. NOT the competition firmware.
//
// Strafes STRAFE_MM to the RIGHT, pauses, STRAFE_MM to the LEFT, pauses, and
// repeats, so the robot ends every cycle where it started. Nothing else moves:
// the rotor is held OFF and the gate parked closed.
//
// THE ROBOT MOVES AS SOON AS IT IS POWERED - and right after an upload, since
// the upload resets the board. There is no start switch and no countdown, the
// same as the competition firmware after setup(). Put it on the floor with a
// metre free to each side before uploading.
//
// The strafes are the routines' own move.right() / move.left() (lib/move/
// move.h), the pause the routines' move.stopForMillis(), so whatever this
// test shows is what a strafe inside a routine does.
//
// BUILD AND RUN
//   pio run -e strafe_test -t upload
//   pio device monitor -e strafe_test        (115200 baud, USB)
//   or the Bluetooth Serial Monitor app on the tablet, 9600 baud
//
// WHAT TO WATCH - the telemetry table (Sensors.h), on USB and Bluetooth, one
// row every 250 ms:
//
//     r   s |    hdg     err corr | pwm  m1  m2  m3  m4 | mm/s    m1    m2    m3    m4
//     1   0 |   -1.2   -0.35   +3 |     232 240 228 235 |       +310  -305  +312  -300
//
//   r      cycle number
//   s      0 strafing RIGHT, 1 pause, 2 strafing LEFT, 3 pause, 4 finished
//   hdg    heading since power-on
//   err    degrees the robot has turned since this strafe began - the number a
//          strafe is judged by. The heading PID should push it back to zero.
//   corr   the PWM differential the PID is applying, out of +-40
//   pwm    motor1..motor4. All four drive in a strafe; 0 while braked.
//   mm/s   each motor's speed, signed by its encoder. Going right, m1 and m3
//          run one way and m2 and m4 the other (F B F B); going left the signs
//          swap. A wheel far from the other three is the one to look at.
//
// On USB only, after every strafe: the counts each wheel travelled and the
// heading it ended with, then the overshoot and the encoder error counts once
// it has settled.

#include <Arduino.h>
#include <Wire.h>

#include "Hardware.h"
#include "Sensors.h"

// ---------------------------------------------------------------------------
// Test parameters
// ---------------------------------------------------------------------------

static const int STRAFE_MM = 500;   // per strafe; one cycle = right, pause, left, pause
static const int CYCLES    = 4;     // 0 = run until switched off
static const int PAUSE_MS  = 250;   // same as `mili` in Strategy.h (the test compiles no strategy)

// ---------------------------------------------------------------------------
// State, in the same style as generalStrategy.cpp
// ---------------------------------------------------------------------------

static int state = 0;   // 0 strafe right, 1 pause, 2 strafe left, 3 pause, 4 finished
static int cycle = 0;

// ---------------------------------------------------------------------------

static void announce(const __FlashStringHelper* which) {
  Serial.print(F("\ncycle "));  Serial.print(cycle + 1);
  Serial.print(F("  "));         Serial.print(which);
  Serial.print(F("  "));         Serial.print(STRAFE_MM);
  Serial.println(F(" mm"));
}

// Printed when a strafe reaches its count: what each wheel travelled and how
// far the heading drifted during the strafe.
static void report() {
  Serial.print(F("    wheels "));
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(F("m")); Serial.print(i + 1); Serial.print(F("="));
    Serial.print(move.regulator.progress(i)); Serial.print(F(" "));
  }
  Serial.print(F("  heading off by "));
  Serial.print(headingError(), 2);
  Serial.println(F(" deg"));
}

// Printed once the robot has settled after a strafe: how far past the target
// the front wheels ended up, and whether any encoder skipped transitions
// (a rising error count means missed edges, i.e. under-counting). The
// regulator still holds the target of the strafe that just ended.
static void settled() {
  long over = move.frontTravelCounts() - move.regulator.target();
  Serial.print(F("    overshoot "));
  Serial.print(over / move.regulator.countsPerMM, 1);
  Serial.print(F(" mm   encoder errors "));
  Serial.print(encoderRearRight.getEncoderErrorCount()); Serial.print(' ');
  Serial.print(encoderRearLeft.getEncoderErrorCount());  Serial.print(' ');
  Serial.print(encoderLeft.getEncoderErrorCount());      Serial.print(' ');
  Serial.println(encoderRight.getEncoderErrorCount());
}

void setup() {
  Serial.begin(115200);

  Serial.println(F("\n=== STRAFE TEST - moves immediately ==="));
  Serial.print(F("robot: "));
  Serial.println(robotSide == LEFT ? F("LEFT / wall") : F("RIGHT / ramp"));

  initHardware();                 // also hands the regulator its PWM band
  initSensors();
  disableDrivers();               // rotor OFF for the whole test
  myservo.write(closedGate);

  Wire.begin();
  if (headingBegin()) {
    Serial.println(F("BNO08x ready - heading hold ACTIVE"));
  } else {
    Serial.println(F("BNO08x NOT found - running WITHOUT heading hold"));
  }

  Serial.print(F("strafe trims pwms "));
  for (uint8_t i = 0; i < 4; i++) { Serial.print(pwms[i]); Serial.print(' '); }
  Serial.println(F(" (only their differences count)"));
  Serial.print(F("counts/mm "));   Serial.println(move.regulator.countsPerMM, 3);
  Serial.print(F("ramp start "));  Serial.print(move.regulator.rampStartPWM);
  Serial.print(F("   cruise "));   Serial.print(move.regulator.cruisePWM);
  Serial.print(F("   min "));      Serial.print(move.regulator.minPWM);
  Serial.print(F("   max "));      Serial.println(move.regulator.maxPWM);
  Serial.print(F("heading gains P ")); Serial.print(move.regulator.kHeadingP, 1);
  Serial.print(F(" I "));              Serial.print(move.regulator.kHeadingI, 1);
  Serial.print(F(" D "));              Serial.print(move.regulator.kHeadingD, 2);
  Serial.print(F("   max differential +-")); Serial.println(move.regulator.maxHeadingCorrection);

  // Opens the Bluetooth port and queues the status block; loop() sends it.
  telemetryBegin(robotSide == LEFT ? "LEFT (wall)" : "RIGHT (ramp)", "strafe_test",
                 STRAFE_MM, 3.14159265f * diameter / pulses, 4);

  announce(F("RIGHT"));
}

void loop() {

  headingUpdate();

  // At most one line per pass, and only when it fits the serial buffers.
  telemetryUpdate(cycle + 1, state, move.wheelPWM, move.regulator.headingCorr());

  switch (state) {
    case 0:
      if (move.right(STRAFE_MM)) { report(); state++; }
      break;
    case 1:
      if (move.stopForMillis(PAUSE_MS)) {
        settled();
        announce(F("LEFT"));
        state++;
      }
      break;
    case 2:
      if (move.left(STRAFE_MM)) { report(); state++; }
      break;
    case 3:
      if (move.stopForMillis(PAUSE_MS)) {
        settled();
        cycle++;

        if (CYCLES != 0 && cycle >= CYCLES) {
          move.stop();
          Serial.println(F("\n=== test finished ==="));
          state = 4;      // park
          break;
        }
        announce(F("RIGHT"));
        state = 0;
      }
      break;
    case 4:
      // finished - hold still
      break;
  }
}
