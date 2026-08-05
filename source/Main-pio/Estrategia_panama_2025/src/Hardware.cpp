// Hardware.cpp - construction and control of everything physical.

#include "Hardware.h"
#include "RobotConfig.h"

int LED = 34;

// ---------------------------------------------------------------------------
// Motors, on the Adafruit Motor Shield v1.
// ---------------------------------------------------------------------------

AF_DCMotor motor1(1); // Motor 1 on the Adafruit Motor Shield
AF_DCMotor motor2(2); // Motor 2 on the Adafruit Motor Shield
AF_DCMotor motor3(3); // Motor 3 on the Adafruit Motor Shield
AF_DCMotor motor4(4); // Motor 4 on the Adafruit Motor Shield

// ---------------------------------------------------------------------------
// Encoders.
//
// ORDER IS SIGNIFICANT - see the warning in Hardware.h. These four must stay
// together, in this order, in this file.
//
// The front pair measures travelled distance and decides when a move is over,
// exactly as it did before the rear pair existed. That is why every distance
// already tuned into the routines is still valid.
// ---------------------------------------------------------------------------

Encoders encoderLeft(A15, A14);  // motor3, front left
Encoders encoderRight(A13, A12); // motor4, front right

// Rear encoders, added 2026 so all four wheels can be regulated against each
// other. Pins confirmed by the author.
Encoders encoderRearRight(A11, A10); // motor1, rear right
Encoders encoderRearLeft(A9, A8);    // motor2, rear left

// Constructed after the motors and encoders above, in the same translation
// unit, so initialisation order is guaranteed.
Move move(
  motor1, motor2, motor3, motor4,
  encoderRearRight, encoderRearLeft, encoderLeft, encoderRight, // motor1..motor4
  pwmf[0], pwmf[1], pwmf[2], pwmf[3],      // Forward/backward PWM values
  pwms[0], pwms[1], pwms[2], pwms[3]       // Left/right/diagonal PWM values
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

void initHardware() {
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

// UNUSED. Testing only LED
void blink() {
  digitalWrite(LED, HIGH);
  delay(500);
  digitalWrite(LED, LOW);
  delay(500);
  return;
}
