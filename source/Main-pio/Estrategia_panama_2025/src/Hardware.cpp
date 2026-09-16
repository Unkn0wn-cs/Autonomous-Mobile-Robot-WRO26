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
  // int pwmf[4] = {243, 243, 243, 243};
  // int pwms[4] = {255, 255, 255, 255};
  // extern const long pulses = 800;  // encoder counts per wheel revolution
  // side robotSide = LEFT;
  // int slowRotorSpeed = 90;
  // int fastRotorSpeed = 200;
  // const int maxPWM           = 255;  // ceiling for every wheel
  // const int minPWM           = 235;  // floor for every driven wheel and the decel loop
  // const int rampStartPWM     = 235;  // where the accel ramp starts
  // const int normalCruisePWM  = 250;  // every routine
  // const int captureCruisePWM = 240;
  // const float wallHugDeg     = 3.0f; // angle the wall moves hold toward the wall
  // const int headingLostDeg   = 70;   // off north by this -> routine 8
  // const int headingSquareDeg = 8;    // routine 8 stops turning inside this
  // const int headingTurnPWM   = 230;  // routine 8's open-loop turn
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
  int slowRotorSpeed = 120;
  int fastRotorSpeed = 200;
  const int maxPWM           = 255;  // ceiling for every wheel
  const int minPWM           = 235;  // floor for every driven wheel and the decel loop
  const int rampStartPWM     = 235;  // where the accel ramp starts
  const int normalCruisePWM  = 230;  // every routine
  const int captureCruisePWM = 190;
  const float wallHugDeg     = 2.0f; // angle the wall moves hold toward the wall
  const int headingLostDeg   = 70;   // off north by this -> routine 8
  const int headingSquareDeg = 8;    // routine 8 stops turning inside this
  const int headingTurnPWM   = 230;  // routine 8's open-loop turn
  int closedGate =116;
  int openGate = 0;
  int lenght = 680;
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

  // =========================================================================
  // MOVEMENT CALIBRATION. Every number that decides how the robot drives is
  // set here (the per-robot trims pwmf/pwms and the cruise levels are in the
  // blocks above and in Hardware.h). Each wheel runs at
  //     common + trim + heading differential
  // where common follows the speed profile below.
  // =========================================================================

  // ---- PWM levels (per robot, from the block above) ------------------------
  // minPWM holds every driven wheel at or above the level the wheels need to
  // keep turning: the deceleration loop cannot go below it and a heading
  // correction that would push one pair under it is clipped there. A
  // correction that would push a wheel over maxPWM shifts the whole set down
  // instead. The accel ramp runs from rampStartPWM up to the cruise.
  move.regulator.maxPWM       = maxPWM;
  move.regulator.minPWM       = minPWM;
  move.regulator.rampStartPWM = rampStartPWM;
  move.regulator.cruisePWM    = normalCruisePWM;

  // ---- Wall hug -----------------------------------------------------------
  // forwardp / backwardp / forwardq hold this angle toward the wall with the
  // heading PID: enough lean to keep the leading corner on the wall, held so
  // it cannot grow into a turn. Per robot (weaker motors need more angle).
  move.wallHugDeg = wallHugDeg;

  // ---- Speed profile: shape -----------------------------------------------
  // Lengths in mm because the two robots count very differently per
  // millimetre.
  //   burst  moves shorter than this run straight at cruise with no ramp,
  //          no deceleration and no correction - wall nudges. The diagonals
  //          run this way whatever their length.
  //   ramp   the open-loop accel ramp, rampFraction of the move clamped to
  //          min..max
  //   decel  the closed-loop deceleration, decelFraction of the move clamped
  //          to min..max
  move.regulator.burstThresholdCounts = (long)(30.0f  * countsPerMM);
  move.regulator.rampFraction         = 0.22f;
  move.regulator.minRampCounts        = (long)(25.0f  * countsPerMM);
  move.regulator.maxRampCounts        = (long)(220.0f * countsPerMM);
  move.regulator.decelFraction        = 0.30f;
  move.regulator.minDecelCounts       = (long)(40.0f  * countsPerMM);
  move.regulator.maxDecelCounts       = (long)(200.0f * countsPerMM);

  // ---- Speed profile: the deceleration loop -------------------------------
  // A PI on the mean encoder speed brings the robot from the speed it had
  // when the decel began down to a creep at the target: endSpeedFraction of
  // that speed, never below minEndSpeedMMs - as far as minPWM lets it, since
  // the PWM never goes under the floor. stallEscapePWM is added per tick
  // while the robot is under half the creep speed, so a wheel stuck on a low
  // PWM is freed.
  move.regulator.kSpeedP          = 0.15f;   // PWM per mm/s of error
  move.regulator.kSpeedI          = 2.0f;    // PWM per mm/s per second
  move.regulator.endSpeedFraction = 0.15f;
  move.regulator.minEndSpeedMMs   = 40.0f;
  move.regulator.stallEscapePWM   = 2;

  // ---- Wall approach ------------------------------------------------------
  // Backward moves longer than longBackwardMM meet the back wall before their
  // commanded distance (routine 6 reverses lenght + 250). They stay at cruise
  // until 200 mm before the target, brake hard over 100 mm down to 200 mm/s
  // and hold that speed over the last 100 mm, so a wall inside that stretch
  // is met at 200 mm/s and one that comes earlier is met while still
  // braking. minPWM bounds this too: the wall is met at 200 mm/s or at the
  // speed the floor gives, whichever is higher. The hold takes 0.5 s of the
  // 4 s moveTimeoutMs.
  move.longBackwardMM          = 200;
  move.backwardEnd.decelCounts = (long)(100.0f * countsPerMM);
  move.backwardEnd.creepCounts = (long)(100.0f * countsPerMM);
  move.backwardEnd.endSpeedMMs = 200.0f;

  // ---- Heading hold -------------------------------------------------------
  // PID on the BNO08x error, output a PWM differential between the wheel
  // pairs. Proportional only for the first run; I and D starting points are
  // 3.0 and 0.6. Errors inside the deadband do not drive the P term.
  move.regulator.kHeadingP            = 12.0f;   // PWM per degree
  move.regulator.kHeadingI            = 0.0f;
  move.regulator.kHeadingD            = 0.4f;
  move.regulator.headingDeadbandDeg   = 0.12f;
  move.regulator.maxHeadingCorrection = 40;      // PWM, per wheel pair
  move.regulator.headingIntegralLimit = 12.0f;   // PWM

  // ---- Timing -------------------------------------------------------------
  move.regulator.updateIntervalMs = 4;      // regulator tick, ms
  move.moveTimeoutMs              = 4000;   // hard cap on any single move

  // Gate servo.
  myservo.attach(10);

  // Rotor L293D.
  pinMode(enable34, OUTPUT);
  pinMode(input3, OUTPUT);
  pinMode(input4, OUTPUT);
}
