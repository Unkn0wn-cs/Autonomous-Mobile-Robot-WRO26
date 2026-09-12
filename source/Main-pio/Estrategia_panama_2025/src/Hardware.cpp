// Hardware.cpp - construction and control of everything physical.

#include "Hardware.h"
#include "RobotConfig.h"
#include "Heading.h"

int LED = 34;

// ---------------------------------------------------------------------------
// Motors, on the Adafruit Motor Shield v1.
// ---------------------------------------------------------------------------

AF_DCMotor motor1(1); // rear right
AF_DCMotor motor2(2); // rear left
AF_DCMotor motor3(3); // front left
AF_DCMotor motor4(4); // front right

// ---------------------------------------------------------------------------
// Encoders. ORDER IS SIGNIFICANT - see the warning in Hardware.h.
//
// The front pair measures travelled distance and decides when a move is over;
// all four feed the mean speed the deceleration loop tracks.
// ---------------------------------------------------------------------------

Encoders encoderLeft(A15, A14);      // motor3, front left
Encoders encoderRight(A13, A12);     // motor4, front right
Encoders encoderRearRight(A11, A10); // motor1, rear right
Encoders encoderRearLeft(A9, A8);    // motor2, rear left

// Constructed after the motors and encoders above, in the same translation
// unit, so initialisation order is guaranteed.
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
// how fast it spins. Direction is fixed in initHardware() and never changes.
// ---------------------------------------------------------------------------

void enableSlowDrivers() {
  pinMode(enable34, OUTPUT);
  analogWrite(enable34, slowRotorSpeed);
}

void enableDrivers() {
  pinMode(enable34, OUTPUT);
  analogWrite(enable34, 254);
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
  // The movement layer reads heading through these hooks so lib/move stays
  // independent of the sensor. HEADING_SIGN is applied inside Heading.cpp.
  move.setHeadingHooks(&regulatorHeadingError, &headingCaptureTarget);

  // PWM levels for these motors: wheels break free at ~200, so the accel ramp
  // starts just above that; 255 is the ceiling. Cruise sits below the ceiling
  // so the heading differential has room before the regulator has to shift
  // the whole set down.
  move.regulator.maxPWM       = 255;
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
  const float countsPerMM = (float)pulses / (3.14159265f * diameter);
  move.regulator.countsPerMM          = countsPerMM;
  move.regulator.burstThresholdCounts = (long)(30.0f  * countsPerMM);
  move.regulator.minRampCounts        = (long)(25.0f  * countsPerMM);
  move.regulator.maxRampCounts        = (long)(220.0f * countsPerMM);
  move.regulator.minDecelCounts       = (long)(40.0f  * countsPerMM);
  move.regulator.maxDecelCounts       = (long)(200.0f * countsPerMM);

  // Wall approach: backward moves longer than 200 mm meet the back wall
  // before their commanded distance (routine 6 reverses lenght + 250, the
  // wall comes up to 150 mm early). They brake over 250 mm down to 200 mm/s
  // and hold that speed over the last 150 mm, so the wall is met at 200 mm/s
  // wherever it comes. The hold alone takes 0.75 s of the 4 s moveTimeoutMs;
  // a slower approach or a longer hold costs more. The threshold is rounded
  // exactly as mm() rounds, so mm(200) itself is not "longer than 200 mm" on
  // either robot.
  move.longBackwardCounts      = move.mmToPulses(200.0f, diameter, pulses);
  move.backwardEnd.decelCounts = (long)(250.0f * countsPerMM);
  move.backwardEnd.creepCounts = (long)(150.0f * countsPerMM);
  move.backwardEnd.endSpeedMMs = 200.0f;

  //servo--------------------------------------------
  myservo.attach(10);

  //rotor
  pinMode(enable34, OUTPUT);
  pinMode(input3, OUTPUT);
  pinMode(input4, OUTPUT);

  // LED DEBUGER SUPERIOR GRAN RESERVA PRO MAX ROJO TRUMP MAGA UNIMET #FORMAFALICA
  pinMode(LED, OUTPUT);

  //microSwitch
  pinMode(backSwitchPin, INPUT_PULLUP);
  pinMode(sideSwitchPin, INPUT_PULLUP);
  pinMode(switchPin, INPUT_PULLUP);
}
