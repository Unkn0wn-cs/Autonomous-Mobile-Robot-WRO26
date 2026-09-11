// Hardware.h - pins, motors, encoders, servo and rotor.
//
// Everything that physically exists on the robot is declared here and owned by
// Hardware.cpp. Nothing else in the firmware may create motors or encoders.
//
// PIN MAP (verified against the libraries, do not reuse without checking):
//   3, 5, 6, 11   motor PWM (AFMotor: M2=3, M4=5, M3=6, M1=11)
//   4, 7, 8, 12   motor shield shift register (CLK, ENABLE, DATA, LATCH)
//   9             rotor L293D enable (analogWrite, Timer2)
//   10            gate servo (Servo library, Timer5)
//   14            start switch
//   16, 17        Serial2 TX2 / RX2 - Bluetooth telemetry module (main.cpp)
//   18, 19        back / side microswitches (polled, NOT interrupts)
//   20, 21        I2C (BNO08x heading sensor)
//   34            debug LED
//   46, 48        rotor L293D input4 / input3
//   50-53         SPI (Pixy2, SS = 53 on the Mega)
//   A8-A15        the four quadrature encoders - PORTK is full
//
// TIMER OWNERSHIP - stealing one of these breaks motors with no compile error:
//   Timer1  motor1 PWM        Timer3  motor2 + motor4 PWM
//   Timer4  motor3 PWM        Timer5  Servo library
//   Timer2  analogWrite(9), the rotor
// The Servo library allocates Timer5 first, then Timer1/3/4 at 12 servos each.
// Attaching a 13th servo would take Timer1 and silently kill motor1.

#pragma once

#include <Arduino.h>
#include <AFMotor.h>
#include <QuadratureEncoder.h>
#include <Servo.h>
#include "move.h"

// ---------------------------------------------------------------------------
// Pin numbers
// ---------------------------------------------------------------------------

extern int LED;               // debug LED, pin 34

const int enable34 = 9;       // rotor L293D pin 9  (PWM speed)
const int input4   = 46;      // rotor L293D pin 10 (direction)
const int input3   = 48;      // rotor L293D pin 15 (direction)

const byte backSwitchPin = 18;
const byte sideSwitchPin = 19;
const byte switchPin     = 14;

// ---------------------------------------------------------------------------
// Drivetrain
//
// Layout seen from above, front of the robot pointing up:
//
//         FRONT
//    motor3   motor4     <- front pair, these two measure distance
//    motor2   motor1     <- rear pair, synchronisation only
//         BACK
// ---------------------------------------------------------------------------

extern AF_DCMotor motor1;
extern AF_DCMotor motor2;
extern AF_DCMotor motor3;
extern AF_DCMotor motor4;

// The Encoders constructor assigns its interrupt slot from a STATIC COUNTER,
// so the declaration order in Hardware.cpp decides which slot each one gets.
// All four live in one translation unit precisely so that order is guaranteed.
// Do not reorder them, and do not move them to another file.
extern Encoders encoderLeft;       // motor3, front left  - measures distance
extern Encoders encoderRight;      // motor4, front right - measures distance
extern Encoders encoderRearRight;  // motor1, rear right  - sync only
extern Encoders encoderRearLeft;   // motor2, rear left   - sync only

extern Move move;

// ---------------------------------------------------------------------------
// Gate servo. closedGate stores balls, openGate shoots them.
// ---------------------------------------------------------------------------

extern Servo myservo;

// ---------------------------------------------------------------------------
// Rotor. Direction is fixed once in initHardware(); only the speed varies.
// ---------------------------------------------------------------------------

void enableSlowDrivers();  // rotor at slowRotorSpeed (90 LEFT / 180 RIGHT)
void enableDrivers();      // rotor at full speed (254)
void disableDrivers();     // rotor stopped

// Sets pin modes, attaches the servo, fixes the rotor direction and hands the
// regulator its PWM band and heading hooks. Call once from setup(), before
// anything drives.
void initHardware();
