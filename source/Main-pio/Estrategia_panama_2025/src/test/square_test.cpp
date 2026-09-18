// square_test.cpp - movement bench test. NOT the competition firmware.
//
// Drives a real 500 mm square: drive a side, pause, turn 90 degrees, pause,
// four times round. setup() does not move the robot at all.
//
//        <------+          After four sides the robot is back where it
//        |      ^          started, facing the way it started. Anything else
//        v      |          is error you can measure on the floor.
//        +------>
//
// Written in the same shape as the competition routines in
// src/generalStrategy.cpp - a switch(state) whose cases advance on
// `if (move.X(...))`, with move.stopForMillis(SETTLE_MS) for the settle time.
// It calls the same Move, the same WheelRegulator and the same BNO08x, so
// whatever this square does is what routine 4 will do.
//
// BUILD AND RUN
//   pio run -e square_test -t upload
//   pio device monitor -e square_test        (115200 baud)
//
// WHAT TO WATCH
//   heading  degrees off the heading this move started on. THE important number.
//            It should be pushed back toward zero within a fraction of a second
//            and stay there.
//   corr     the differential PWM the heading PID is applying, out of +-40.
//            Busy early in a move, quiet once it is tracking.
//   phase / profile / pwm / v
//            accel, cruise or decel; the speed profile (100 % = cruise); the
//            common PWM the regulator is applying; the measured speed and, in
//            decel, the commanded speed it is tracking. The PWM should fall
//            well below 200 during decel and v should follow the command.
//   overshoot
//            printed after each move has settled: how far past the target the
//            front wheels ended up. The number this whole test is about.
//   encoder errors
//            skipped transitions per encoder (m1 m2 m3 m4). Climbing counts
//            mean missed edges - the robot travels further than it counts.
//   SIGN     printed after every turn. The heading reading must move the way
//            Sensors.h says it does for the wheel pattern used. "OK" means the
//            heading loop corrects toward straight, not away from it; "FLIP"
//            means change HEADING_SIGN in Sensors.cpp. Check this on the first
//            turn - with the wrong sign the heading loop pushes the robot AWAY
//            from straight on every side.
//
// SAFETY: the rotor is held OFF throughout and the gate parked closed. The
// camera object is compiled in (it shares Sensors.cpp with the encoders and
// the heading sensor) but never initialised or read.

#include <Arduino.h>
#include <Wire.h>

#include "Hardware.h"
#include "Sensors.h"

// ---------------------------------------------------------------------------
// Test parameters
// ---------------------------------------------------------------------------

static const int SIDE_MM   = 500;   // half a metre per side
static const int LAPS      = 4;     // 0 = run forever
static const int SETTLE_MS = 250;   // pause after each move, same as `mili` in the routines

// Wheel travel for a 90 degree turn: the value the routines use for a quarter
// turn (rotate(166); rotate(146) for their 80 degree turns). Adjust if the
// robot over- or under-turns.
static const int QUARTER_TURN_MM = 166;

// Which rotate() pattern the turns use: false = B F F B (the rotateCW()
// pattern, heading reading should INCREASE), true = F B B F (rotateCCW(),
// reading should DECREASE). Flip to run the square the other way round.
static const bool TURN_PATTERN_FBBF = false;

static const unsigned long TRACE_EVERY_MS = 250;

// ---------------------------------------------------------------------------
// State, in the same style as generalStrategy.cpp
// ---------------------------------------------------------------------------

static int state = 0;       // step within the current side
static int side  = 0;       // 0-3, which side of the square
static int lap   = 0;

// ---------------------------------------------------------------------------

// Sampled mid-move.
static void trace() {
  static const char* const PHASE[3] = {"accel ", "cruise", "decel "};
  Serial.print(F("      "));
  Serial.print(PHASE[move.regulator.phase()]);
  Serial.print(F(" profile "));
  Serial.print((int)(move.regulator.profile() * 100.0f));
  Serial.print(F("%  pwm "));
  Serial.print((int)move.regulator.commonPWM());
  Serial.print(F("  v "));
  Serial.print((int)move.regulator.speedMMs());
  if (move.regulator.phase() == WheelRegulator::Decel) {
    Serial.print(F("/"));
    Serial.print((int)move.regulator.commandMMs());
  }
  Serial.print(F(" mm/s  heading "));
  Serial.print(headingError(), 2);
  Serial.print(F(" deg  corr "));
  Serial.print(move.regulator.headingCorr(), 1);
  Serial.print(F("  age "));
  Serial.print(headingAgeMs());
  Serial.println(F(" ms"));
}

// Printed once the robot has settled after a move: how far past the target
// the trusted wheels ended up (the travel the finish was judged on), and
// whether any encoder skipped transitions (a rising error count means missed
// edges, i.e. under-counting). The regulator still holds the target of the
// move that just ended.
static void settled() {
  long over = move.travelCounts() - move.regulator.target();
  Serial.print(F("    overshoot "));
  Serial.print(over / move.regulator.countsPerMM, 1);
  Serial.print(F(" mm   encoder errors "));
  Serial.print(encoderRearRight.getEncoderErrorCount()); Serial.print(' ');
  Serial.print(encoderRearLeft.getEncoderErrorCount());  Serial.print(' ');
  Serial.print(encoderLeft.getEncoderErrorCount());      Serial.print(' ');
  Serial.println(encoderRight.getEncoderErrorCount());
}

static void announce(const __FlashStringHelper* what, int millimetres) {
  Serial.print(F("\nlap "));   Serial.print(lap + 1);
  Serial.print(F("  side "));  Serial.print(side + 1); Serial.print(F("/4  "));
  Serial.print(what);
  Serial.print(F("  "));       Serial.print(millimetres);
  Serial.println(F(" mm"));
}

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

// How far the heading reading moved during the turn, against the direction
// Sensors.h promises for the pattern used.
static void signVerdict() {
  if (!headingAvailable()) {
    Serial.println(F("    SIGN: no sensor, cannot check"));
    return;
  }
  float turned = headingSinceZero();            // zeroed just before the turn
  float expected = TURN_PATTERN_FBBF ? -90.0f : 90.0f;
  Serial.print(F("    turned "));
  Serial.print(turned, 1);
  Serial.print(F(" deg by the sensor (expected about "));
  Serial.print(expected, 0);
  Serial.print(F(")   SIGN: "));
  if (turned * expected > 0) Serial.println(F("OK"));
  else                       Serial.println(F("FLIP HEADING_SIGN in Sensors.cpp"));
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) { }

  Serial.println(F("\n=== 500 mm SQUARE ==="));
  Serial.print(F("robot: "));
  Serial.println(robotSide == LEFT ? F("LEFT / wall") : F("RIGHT / ramp"));

  initHardware();                 // also hands the regulator its PWM band
  initSensors();                  // start switch pin
  disableDrivers();               // rotor OFF for the whole test
  myservo.write(closedGate);

  Wire.begin();
  if (headingBegin()) {
    Serial.println(F("BNO08x ready - heading hold ACTIVE"));
  } else {
    Serial.println(F("BNO08x NOT found - running WITHOUT heading hold"));
  }

  Serial.print(F("counts/mm "));   Serial.println(move.regulator.countsPerMM, 3);
  Serial.print(F("ramp start "));  Serial.print(move.regulator.rampStartPWM);
  Serial.print(F("   cruise "));   Serial.print(move.regulator.cruisePWM);
  Serial.print(F("   max "));      Serial.println(move.regulator.maxPWM);
  Serial.print(F("heading gains P ")); Serial.print(move.regulator.kHeadingP, 1);
  Serial.print(F(" I "));              Serial.print(move.regulator.kHeadingI, 1);
  Serial.print(F(" D "));              Serial.print(move.regulator.kHeadingD, 2);
  Serial.print(F("   max differential +-")); Serial.println(move.regulator.maxHeadingCorrection);

  if (robotSide == LEFT) {
    Serial.println(F("press the start switch (pin 14) to begin..."));
    while (digitalRead(switchPin) != HIGH) { }
  } else {
    Serial.println(F("starting in 3 s - clear the area"));
    delay(3000);
  }

  announce(F("DRIVE"), SIDE_MM);
}

void loop() {

  headingUpdate();

  // Telemetry, kept outside the state machine so the cases below read exactly
  // like the ones in generalStrategy.cpp.
  static unsigned long lastTrace = 0;
  if (millis() - lastTrace >= TRACE_EVERY_MS) {
    lastTrace = millis();
    if (state == 0 || state == 2) trace();
  }

  switch (state) {
    case 0:
      if (move.forward(SIDE_MM)) { report(); state++; }
      break;
    case 1:
      if (move.stopForMillis(SETTLE_MS)) {
        settled();
        announce(F("TURN 90"), QUARTER_TURN_MM);
        headingZero();              // measure the turn from here
        state++;
      }
      break;
    case 2:
      if (move.rotate(QUARTER_TURN_MM, TURN_PATTERN_FBBF)) { report(); signVerdict(); state++; }
      break;
    case 3:
      if (move.stopForMillis(SETTLE_MS)) {
        settled();
        state = 0;
        side++;

        if (side >= 4) {
          side = 0;
          lap++;
          Serial.println(F("\n--- lap complete: back at the start, same heading ---"));

          if (LAPS != 0 && lap >= LAPS) {
            move.stop();
            disableDrivers();
            Serial.println(F("\n=== test finished ==="));
            state = 4;      // park
            break;
          }
        }
        announce(F("DRIVE"), SIDE_MM);
      }
      break;
    case 4:
      // finished - hold still
      break;
  }
}
