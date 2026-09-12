// Hardware.h - what is physically on the robot and moves it: which of the two
// robots this build is for and its tuned values, the wheel geometry, the
// motors and the Move instance that drives them, the gate servo and the rotor.
//
// Team Outer Heaven - WRO 2026.
//
// The two robots share one firmware. Exactly one configuration block in
// Hardware.cpp must be uncommented. Everything that differs between the wall
// robot and the ramp robot lives there and nowhere else. See Hardware.cpp to
// switch robots.
//
// The sensors (encoders, BNO08x, Pixy2, microswitches) and their pins are in
// Sensors.h. Nothing else in the firmware may create motors or encoders.
//
// ACTUATOR PINS (verified against the libraries, do not reuse without checking):
//   3, 5, 6, 11   motor PWM (AFMotor: M2=3, M4=5, M3=6, M1=11)
//   4, 7, 8, 12   motor shield shift register (CLK, ENABLE, DATA, LATCH)
//   9             rotor L293D enable (analogWrite, Timer2)
//   10            gate servo (Servo library, Timer5)
//   46, 48        rotor L293D input4 / input3
// Also in use: 16, 17 Serial2 TX2 / RX2, the Bluetooth telemetry module
// (Sensors.h), and the sensor pins listed in Sensors.h.
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
#include <Servo.h>
#include "move.h"

// ---------------------------------------------------------------------------
// Which robot
// ---------------------------------------------------------------------------

// Which of the two robots this build is for. This is the master switch: most
// left/right decisions in the routines are mirrored on it, so ANY new
// directional logic must branch on robotSide.
enum side {
  RIGHT,
  LEFT
};

extern side robotSide;       // which robot this build drives

// ---------------------------------------------------------------------------
// Per robot values. Defined in Hardware.cpp, inside the active block.
// ---------------------------------------------------------------------------

// Per-motor balance for forward/backward (pwmf) and for strafe/diagonal moves
// (pwms). The regulator drives every wheel from its own cruisePWM (set in
// initHardware()) and uses only the DIFFERENCES between these four numbers as
// per-wheel trims: {220,243,243,220} means "wheels 2 and 3 need ~23 more than
// wheels 1 and 4 to run straight". Raising all four by the same amount changes
// nothing.
extern int pwmf[4];
extern int pwms[4];

extern const long pulses;    // encoder counts per wheel revolution
extern int slowRotorSpeed;   // rotor PWM used by enableSlowDrivers()
extern int closedGate;       // servo angle: gate closed (storing)
extern int openGate;         // servo angle: gate open (shooting)
extern int lenght;           // length of the main straight in mm

// ---------------------------------------------------------------------------
// Purple ball position zones, one rectangle per opening routine.
//
// The Pixy2 image is 316 x 208 and its ORIGIN IS THE TOP-LEFT corner, so x
// grows to the right and y grows DOWNWARD: a point that looks "lower" on
// screen has the LARGER y.
//
// Every ball position is one rectangle written as two OPPOSITE CORNERS:
//                              { xA, yA, xB, yB }
// The order of the two corners does not matter - the scorer normalises to
// min/max - so {lower-left, upper-right} and {upper-left, bottom-right} are
// both accepted. Write down whichever pair is easier to read off PixyMon.
//
// HOW TO CALIBRATE
//   1. Put a purple ball at one position and leave the robot exactly where it
//      starts.
//   2. Read the block Pixy reports (PixyMon, or the "blob edges" line
//      selectOpeningRoutine() prints on Serial, which already gives the four
//      edges: left, top, right, bottom).
//   3. Enter a rectangle that CONTAINS that blob. No need to pad it by hand:
//      the detector allows BALL_ZONE_TOLERANCE px of slack on every side (see
//      generalStrategy.cpp), so write down what you actually measured.
//   4. Zones may overlap or touch; the scorer picks the best match, not the
//      first one, and grades near-misses by distance so the closer rectangle
//      always wins. What must NOT happen is a zone stretching over a
//      neighbouring ball position.
//   5. Anything outside every zone (plus its tolerance) is treated as noise
//      and ignored, so keep the rectangles away from purple-ish reflections
//      on the mat or the walls.
//
// The row index IS the routine that will be executed for that ball:
//   row 0 -> routine 0 : ball high & left  in the image ("upper left")
//   row 1 -> routine 1 : ball high & right in the image ("upper right")
//   row 2 -> routine 2 : ball low  & left  in the image ("lower left")
//   row 3 -> routine 3 : ball low  & right in the image ("lower right")
// No match at all leaves routine 4, the normal no-ball lane loop.
//
// A row left as { 0, 0, 0, 0 } is an empty rectangle: it has no area, so it
// can never match anything. An un-filled row therefore just means "this ball
// position is never chosen" and the robot falls through to routine 4 - it
// will never guess a position it has no numbers for.
// ---------------------------------------------------------------------------

const int NUM_BALL_ZONES = 4;
extern const int ballZones[NUM_BALL_ZONES][4];

// ---------------------------------------------------------------------------
// Wheel geometry, identical on both robots.
// ---------------------------------------------------------------------------

const int diameter = 60;  // wheel diameter in mm; with `pulses` gives counts/mm

// ---------------------------------------------------------------------------
// Drivetrain: four 45-degree omni wheels on the Adafruit Motor Shield v1.
//
// Layout seen from above, front of the robot pointing up:
//
//         FRONT
//    motor3   motor4     <- front pair, their encoders measure distance
//    motor2   motor1     <- rear pair, speed measurement only
//         BACK
//
// The encoder on each motor is in Sensors.h.
// ---------------------------------------------------------------------------

extern AF_DCMotor motor1;
extern AF_DCMotor motor2;
extern AF_DCMotor motor3;
extern AF_DCMotor motor4;

extern Move move;

// ---------------------------------------------------------------------------
// Gate servo, pin 10. closedGate stores balls, openGate shoots them.
// ---------------------------------------------------------------------------

extern Servo myservo;

// ---------------------------------------------------------------------------
// Rotor, on its own L293D. Direction is fixed once in setup() (input3 HIGH,
// input4 LOW); only the speed on enable34 varies.
// ---------------------------------------------------------------------------

const int enable34 = 9;       // rotor L293D pin 9  (PWM speed)
const int input4   = 46;      // rotor L293D pin 10 (direction)
const int input3   = 48;      // rotor L293D pin 15 (direction)

void enableSlowDrivers();  // rotor at slowRotorSpeed (90 LEFT / 180 RIGHT)
void enableDrivers();      // rotor at full speed (254)
void disableDrivers();     // rotor stopped

// Hands the Move library everything robot specific (counts per mm, which way
// is inner, heading hooks, PWM band and profile distances), attaches the
// servo and sets the rotor pins as outputs. Call once from setup(), before
// anything drives.
void initHardware();
