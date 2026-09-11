// RobotConfig.h - which robot is this firmware for, and its tuned values.
//
// Team Outer Heaven - WRO 2026.
//
// The two robots share one firmware. Exactly one configuration block in
// RobotConfig.cpp must be uncommented. Everything that differs between the wall
// robot and the ramp robot lives there and nowhere else.
//
// See RobotConfig.cpp to switch robots.

#pragma once

#include <Arduino.h>

// Which lane of the field the robot is currently running.
enum rlane {
  OUTER,
  MIDDLE,
  INNER
};

// Which of the two robots this build is for. This is the master switch: most
// left/right decisions in the routines are mirrored on it, so ANY new
// directional logic must branch on robotSide.
enum side {
  RIGHT,
  LEFT
};

// ---------------------------------------------------------------------------
// Per robot values. Defined in RobotConfig.cpp, inside the active block.
// ---------------------------------------------------------------------------

// Per-motor balance for forward/backward (pwmf) and for strafe/diagonal moves
// (pwms). The regulator drives every wheel from its own cruisePWM (set in
// Hardware.cpp) and uses only the DIFFERENCES between these four numbers as
// per-wheel trims: {220,243,243,220} means "wheels 2 and 3 need ~23 more than
// wheels 1 and 4 to run straight". Raising all four by the same amount changes
// nothing.
extern int pwmf[4];
extern int pwms[4];

extern const long pulses;    // encoder counts per wheel revolution
extern side robotSide;       // which robot this build drives
extern int slowRotorSpeed;   // rotor PWM used by enableSlowDrivers()
extern int closedGate;       // servo angle: gate closed (storing)
extern int openGate;         // servo angle: gate open (shooting)

// Length of the main straight in mm. Assigned in setup() from robotSide
// (640 for RIGHT, 1100 for LEFT). Declared outside both robot blocks.
extern int lenght;

// ---------------------------------------------------------------------------
// Shared constants, identical on both robots.
// These are `const` at namespace scope, so each translation unit gets its own
// copy and there is no multiple definition problem.
// ---------------------------------------------------------------------------

const int mili = 250;     // standard pause between moves, in milliseconds
const int diameter = 60;  // wheel diameter in mm, used by mm()
