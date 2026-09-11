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
// all four take part in wheel synchronisation.
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

  // Usable PWM band for these motors: wheels break free at ~200, 255 is the
  // ceiling. Cruise sits inside the band so the regulator can push a wheel up
  // as well as slow it down.
  move.regulator.minMovePWM   = 200;
  move.regulator.maxPWM       = 255;
  move.regulator.rampStartPWM = 205;
  move.regulator.cruisePWM    = 232;

  // Moves shorter than the burst threshold skip the ramp and regulation and run
  // straight at cruise - the wall does the aligning on those. All three are in
  // mm because the two robots count very differently per millimetre.
  const float countsPerMM = (float)pulses / (3.14159265f * diameter);
  move.regulator.burstThresholdCounts = (long)(120.0f * countsPerMM);
  move.regulator.minRampCounts        = (long)(25.0f  * countsPerMM);
  move.regulator.maxRampCounts        = (long)(220.0f * countsPerMM);

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
