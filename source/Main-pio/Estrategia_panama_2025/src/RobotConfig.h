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

extern int pwmf[4];          // forward/backward PWM, one per motor
extern int pwms[4];          // strafe/diagonal PWM, one per motor
extern const long pulses;    // encoder counts per wheel revolution
extern side robotSide;       // which robot this build drives
extern int slowRotorSpeed;   // rotor PWM used by enableSlowDrivers()
extern int closedGate;       // servo angle: gate closed (storing)
extern int openGate;         // servo angle: gate open (shooting)

// Length of the main straight in mm. Assigned in setup() from robotSide
// (640 for RIGHT, 1100 for LEFT), so its initial value is irrelevant.
//
// NOTE: this used to be declared inside the LEFT block only, which meant the
// RIGHT configuration did not compile at all. It now lives outside both blocks.
// This is the one change made here that the LEFT robot cannot notice.
extern int lenght;

// UNUSED. Never read anywhere in the firmware. Kept because the comment records
// the intended purple ball bounding box format.
// {x upper left 1st ball, y upper left 1st ball, x bottom right 1st ball, ...}
extern int pixyBalls[16];

// ---------------------------------------------------------------------------
// Shared constants, identical on both robots.
// These are `const` at namespace scope, so each translation unit gets its own
// copy and there is no multiple definition problem.
// ---------------------------------------------------------------------------

const int mili = 250;     // standard pause between moves, in milliseconds
const int diameter = 60;  // wheel diameter in mm, used by mm()
