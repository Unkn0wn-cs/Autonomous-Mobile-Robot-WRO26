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
//      Routines.cpp), so write down what you actually measured.
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
// Shared constants, identical on both robots.
// These are `const` at namespace scope, so each translation unit gets its own
// copy and there is no multiple definition problem.
// ---------------------------------------------------------------------------

const int mili = 250;     // standard pause between moves, in milliseconds
const int diameter = 60;  // wheel diameter in mm, used by mm()
