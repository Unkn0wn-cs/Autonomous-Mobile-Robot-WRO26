// Hardware.cpp - THE ROBOT SELECTOR, then construction and control of
// everything physical.
//
// ###########################################################################
// #                                                                         #
// #   TO SWITCH ROBOTS: uncomment ONE block below and comment out the other. #
// #   This is the only place either robot is selected.                       #
// #                                                                         #
// ###########################################################################
//
// Leaving both blocks active will not compile (duplicate definitions).
// Leaving both commented out will not link (undefined references).

#include "Hardware.h"
#include "Sensors.h"

// ===========================================================================
// LEFT - WALL
// ===========================================================================
  // int pwmf[4] = {245, 243, 243, 245};
  // int pwms[4] = {220, 225, 220, 225};
  // extern const long pulses = 900;  // encoder counts per wheel revolution
  // side robotSide = LEFT;
  // int slowRotorSpeed = 90;
  // int closedGate = 180;
  // int openGate = 55;
  // int lenght = 1100;
  // // Purple ball zones {xA, yA, xB, yB}, Pixy pixels. Format in Hardware.h.
  // extern const int ballZones[NUM_BALL_ZONES][4] = {
  //   { 135,  20,  160,   0 },   // routine 0 - upper left
  //   { 235,  25,  260,  10 },   // routine 1 - upper right
  //   { 135,  40,  160,  20 },   // routine 2 - lower left
  //   { 260,  55,  290,  25 }    // routine 3 - lower right
  // };

// ===========================================================================
// RIGHT - RAMP
// ===========================================================================
  int pwmf[4] = {220, 243, 243, 220};
  int pwms[4] = {200, 200, 200, 200};
  extern const long pulses = 1350;  // encoder counts per wheel revolution
  side robotSide = RIGHT;
  int slowRotorSpeed = 140;
  int fastRotorSpeed = 200;
  int closedGate =116;
  int openGate = 0;
  int lenght = 640;
  // Purple ball zones {xA, yA, xB, yB}, Pixy pixels. Format in Hardware.h.
  extern const int ballZones[NUM_BALL_ZONES][4] = {
    { 142,  32,  160,  15 },   // routine 0 - upper left
    { 245,  33,  260,  20 },   // routine 1 - upper right
    { 143,  51,  170,  25 },   // routine 2 - lower left
    { 271,  60,  290,  35 }    // routine 3 - lower right
  };

// ===========================================================================
// Everything below is the same on both robots.
// ===========================================================================

// ---------------------------------------------------------------------------
// Motors, on the Adafruit Motor Shield v1.
// ---------------------------------------------------------------------------

AF_DCMotor motor1(1); // rear right
AF_DCMotor motor2(2); // rear left
AF_DCMotor motor3(3); // front left
AF_DCMotor motor4(4); // front right

// The Move constructor only stores references to the motors and encoders, so
// it does not matter that the encoders are constructed in Sensors.cpp.
Move move(
  motor1, motor2, motor3, motor4,
  encoderRearRight, encoderRearLeft, encoderLeft, encoderRight, // motor1..motor4
  pwmf[0], pwmf[1], pwmf[2], pwmf[3],      // forward/backward trims
  pwms[0], pwms[1], pwms[2], pwms[3]       // strafe/diagonal trims
);

Servo myservo;

// ---------------------------------------------------------------------------
// Rotor speed control.
//
// The rotor stores or shoots depending on the gate servo; these three only set
// how fast it spins. Direction is fixed in setup() and never changes.
// ---------------------------------------------------------------------------

void enableSlowDrivers() {
  pinMode(enable34, OUTPUT);
  analogWrite(enable34, slowRotorSpeed);
}

void enableDrivers() {
  pinMode(enable34, OUTPUT);
  analogWrite(enable34, fastRotorSpeed);
}

void disableDrivers() {
  pinMode(enable34, OUTPUT);
  analogWrite(enable34, 0);
}

// The regulator's heading input. Polls the sensor first (rate-limited inside
// headingUpdate), so a move keeps getting fresh readings even from code that
// does not return to loop() between passes.
static float regulatorHeadingError() {
  headingUpdate();
  return headingError();
}

void initHardware() {
  // Encoder counts per millimetre for this robot. Every distance the Move
  // library is given in mm - the moves themselves, the regulator's speeds
  // and the profile lengths below - goes through it, so it is set first.
  const float countsPerMM = (float)pulses / (3.14159265f * diameter);
  move.regulator.countsPerMM = countsPerMM;

  // Which way move.inner() strafes on this robot; move.outer() is the other.
  move.innerIsLeft = (robotSide == RIGHT);

  // The movement layer reads heading through these hooks so lib/move stays
  // independent of the sensor. HEADING_SIGN is applied inside Sensors.cpp.
  move.setHeadingHooks(&regulatorHeadingError, &headingCaptureTarget);

  // PWM levels for these motors: wheels break free at ~200, so the accel ramp
  // starts just above that; 255 is the ceiling. Cruise sits below the ceiling
  // so the heading differential has room before the regulator has to shift
  // the whole set down.
  move.regulator.maxPWM       = 248;
  move.regulator.rampStartPWM = 205;
  move.regulator.cruisePWM    = 232;

  // Heading hold: PWM of differential per degree, and the most it may apply.
  // Proportional only for the first run; see WheelRegulator.h for I and D.
  move.regulator.kHeadingP            = 12.0f;
  move.regulator.kHeadingI            = 0.0f;
  move.regulator.kHeadingD            = 0.0f;
  move.regulator.maxHeadingCorrection = 40;

  // Distances the regulator works in, given in mm because the two robots count
  // very differently per millimetre.
  //   burst  moves shorter than this run straight at cruise with no ramp,
  //          no deceleration and no correction - wall nudges
  //   ramp   the accel ramp length (22 % of the move, clamped to this range)
  //   decel  the closed-loop deceleration length (30 % of the move, clamped)
  move.regulator.burstThresholdCounts = (long)(30.0f  * countsPerMM);
  move.regulator.minRampCounts        = (long)(25.0f  * countsPerMM);
  move.regulator.maxRampCounts        = (long)(220.0f * countsPerMM);
  move.regulator.minDecelCounts       = (long)(40.0f  * countsPerMM);
  move.regulator.maxDecelCounts       = (long)(200.0f * countsPerMM);

  // Wall approach: backward moves longer than 200 mm meet the back wall
  // before their commanded distance (routine 6 reverses lenght + 250). They
  // stay at cruise until 200 mm before the target, brake hard over 100 mm
  // down to 200 mm/s and hold that speed over the last 100 mm, so a wall
  // inside that stretch is met at 200 mm/s and one that comes earlier is met
  // while still braking. The hold takes 0.5 s of the 4 s moveTimeoutMs.
  move.longBackwardMM          = 200;
  move.backwardEnd.decelCounts = (long)(100.0f * countsPerMM);
  move.backwardEnd.creepCounts = (long)(100.0f * countsPerMM);
  move.backwardEnd.endSpeedMMs = 200.0f;

  // Gate servo.
  myservo.attach(10);

  // Rotor L293D.
  pinMode(enable34, OUTPUT);
  pinMode(input3, OUTPUT);
  pinMode(input4, OUTPUT);
}
