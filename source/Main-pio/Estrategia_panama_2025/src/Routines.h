// Routines.h - the strategy state machine.
//
// TWO LEVELS OF STATE:
//   `routine` picks which strategy is running
//   `state`   steps through that strategy
// Both are plain globals and transitions are plain assignments.
//
//   routine 0-3   opening purple ball handling, one per camera quadrant
//   routine 4     main lane loop
//   routine 5     diagonal lane
//   routine 6     return, then camera weighting picks the next lane
//   routine 7     corner checking reset, rotates on the gyro
//   routine 8     reorient to 0 degrees, then falls through into routine 9
//   routine 9     parking / Pixy ball tracking
//   routine 10    debugging
//
// EVERY movement call is non-blocking: it is called again and again from loop()
// and returns true only once, when it has finished. Never wrap one in a loop
// that waits for it.

#pragma once

#include <Arduino.h>
#include "RobotConfig.h"

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------

extern int routine;  // which strategy is running; default 4
extern int state;    // step within that strategy. Routine 4 counts DOWN for the
                     // OUTER lane (-1 to -6) and UP for MIDDLE/INNER (0 to 5).
extern bool first;   // true until the first full lap has been set up
extern rlane lane;   // which lane the robot is currently committed to

extern int beta;   // heading tolerance in degrees
extern int alpha;  // heading target in degrees

// Limits the camera lane decision to twice per corner reset. Reset to 0 on
// every pass through routine 7.
extern int connections;

// Millis at the end of setup().
//
// KNOWN BUG, PRESERVED DELIBERATELY: this is an `int`, which is 16 bits on AVR,
// so it truncates millis() and wraps every 32.767 seconds. It is currently only
// read by the disabled endgame timing block, so it does no harm today. If that
// block is ever re-enabled, this must become unsigned long first.
extern int startTime;

// ---------------------------------------------------------------------------
// Camera lane weighting
// ---------------------------------------------------------------------------

const int NUM_FRANJAS = 3;
extern int pesos[NUM_FRANJAS];  // accumulated orange blob area per franja
extern int camera;              // UNUSED. Written in routine 6, never read.

// ---------------------------------------------------------------------------
// Endgame timing flags.
//
// DISABLED. Every line that sets these to true is commented out in
// updateEndgameTiming(), so all three are permanently false. Consequences:
//
//   * routine 9 is UNREACHABLE - its only entry point is routine 4 state 5,
//     gated on `lastRoutine || midRoutine`
//   * routine 10 is UNREACHABLE - it is only entered from routine 9
//
// Both routines are kept intact so the timing block can simply be uncommented
// to bring the endgame behaviour back.
// ---------------------------------------------------------------------------

extern bool lastRoutine;
extern bool midRoutine;
extern bool midRoutineDone;

// ---------------------------------------------------------------------------
// Operations
// ---------------------------------------------------------------------------

// Maps a camera blob position to one of three franjas using two diagonal
// boundary lines. The `right` argument selects which robot's boundary constants
// to use.
int classifyLane(float x, float y, bool right);

// Scans up to 120 Pixy frames for the purple ball and picks the opening routine
// (0-3) from which quadrant it is in. Leaves routine at its default of 4 if no
// ball is found. Called once from setup().
void selectOpeningRoutine();

// Edge detects the two microswitches and advances `state`. A back switch press
// also zeroes the heading.
void handleMicroSwitches();

// The disabled endgame timing block. See the note on lastRoutine above.
void updateEndgameTiming();

// Runs one pass of the routine state machine.
//
// MUST BE THE LAST THING loop() DOES: routine 4 state 5 contains a bare
// `return` for the LEFT robot on the INNER lane, which is expected to skip
// everything that would have followed it.
void runRoutines();
