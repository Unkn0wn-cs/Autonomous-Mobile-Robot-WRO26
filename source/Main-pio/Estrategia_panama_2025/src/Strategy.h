// Strategy.h - what a strategy is, which one is built, and its options.
//
// A STRATEGY is one .cpp file that defines everything declared below: the two
// state globals, `startTime`, `strategyName`, and the four functions main.cpp
// calls. Exactly one strategy is compiled into a firmware, chosen by the
// PlatformIO environment (platformio.ini):
//
//   general   src/generalStrategy.cpp   the lane loop with the camera opening
//   control   src/controlStrategy.cpp   the wall robot's control routines
//
// TO SWITCH STRATEGIES: upload the other environment (PlatformIO sidebar ->
// Project Tasks -> <env> -> Upload, or `pio run -e control -t upload`). The
// toolbar buttons use `default_envs`. The strategy prints its name on the USB
// boot line and in the first line of the Bluetooth status block - read it
// before every match.
//
// TO ADD A STRATEGY: copy controlStrategy.cpp, give it its own `strategyName`,
// and add an environment whose build_src_filter excludes the other strategy
// files.
//
// TWO LEVELS OF STATE:
//   `routine` picks which routine is running
//   `state`   steps through that routine
// Both are plain globals and transitions are plain assignments. In the general
// strategy:
//
//   routine 0-3   opening purple ball handling, one per calibrated ball zone
//   routine 4     main lane loop
//   routine 5     diagonal lane
//   routine 6     return, then camera weighting picks the next lane
//   routine 7     corner checking reset, turns on the heading sensor
//   routine 8     heading recovery: stop, turn back to north, then routine 6
//   routine 9     parking / Pixy ball tracking
//   routine 10    debugging
//
// EVERY movement call is non-blocking: it is called again and again from loop()
// and returns true only once, when it has finished. Never wrap one in a loop
// that waits for it.

#pragma once

#include <Arduino.h>

// Name printed at boot and in the telemetry status block; a string literal
// baked into the firmware by the strategy that was compiled.
extern const char* strategyName;

// ---------------------------------------------------------------------------
// General strategy options
// ---------------------------------------------------------------------------

// Mid and late game kicks. With the option on, updateEndgameTiming() runs the
// clock below against `startTime`:
//   MID_KICK   lane forced (OUTER, or MIDDLE if already in routine 4 on OUTER)
//              and the straight shortened by 30 mm; the next time routine 4
//              finishes a straight it goes to routine 9 (LEFT: camera ball
//              tracking; RIGHT: routine 10)
//   MID_DONE   until MID_END: straight restored, normal laps resume
//   LATE_KICK  lane OUTER, straight -30 mm, routine 9 for the rest of the match
// With the option off nothing is ever set and routines 9 and 10 are unreachable.
static const bool          GENERAL_ENDGAME_KICKS = false;
static const unsigned long GENERAL_MID_KICK_MS   = 45000;
static const unsigned long GENERAL_MID_DONE_MS   = 61000;
static const unsigned long GENERAL_MID_END_MS    = 100000;
static const unsigned long GENERAL_LATE_KICK_MS  = 105000;

// Which lane of the field the robot is currently running. OUTER is furthest
// from the centre wall, INNER closest; move.inner()/move.outer() translate
// that into a left or right strafe for this robot.
enum rlane {
  OUTER,
  MIDDLE,
  INNER
};

const int mili = 400;     // standard pause between moves, in milliseconds

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------

extern int routine;  // which strategy is running; default 4
extern int state;    // step within that strategy. Routine 4 counts DOWN for the
                     // OUTER lane (-1 to -6) and UP for MIDDLE/INNER (0 to 5).
extern bool first;   // true until the first full lap has been set up
extern rlane lane;   // which lane the robot is currently committed to

// Limits the camera lane decision to twice per corner reset. Reset to 0 on
// every pass through routine 7.
extern int connections;

// Millis when the match clock started (set in setup(), before the camera
// scan). The endgame kicks are timed from it.
extern unsigned long startTime;

// ---------------------------------------------------------------------------
// Camera lane weighting
// ---------------------------------------------------------------------------

const int NUM_FRANJAS = 3;
extern int pesos[NUM_FRANJAS];  // accumulated orange blob area per franja

// ---------------------------------------------------------------------------
// Endgame timing flags, set by updateEndgameTiming() when GENERAL_ENDGAME_KICKS
// is on. Routine 4 state 5 goes to routine 9 on `lastRoutine || midRoutine`;
// routine 6 state 7 forces the OUTER sequence on them. With the option off
// all three stay false and routines 9 and 10 are unreachable.
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

// Scans the Pixy for up to 900 ms for the purple ball and picks the opening
// routine (0-3) from which calibrated zone (ballZones, Hardware.cpp) its
// bounding box matches, confirmed over several frames by a vote. Gives up
// after 350 ms if nothing ball-like has been seen. Leaves routine at its
// default of 4 if no ball is confirmed. Called once from setup().
void selectOpeningRoutine();

// Edge detects the two microswitches and advances `state`. A back switch press
// also zeroes the heading. Neither switch advances routine 8, so a bump during
// the recovery turn cannot skip one of its steps.
void handleMicroSwitches();

// The match clock for the endgame kicks (GENERAL_ENDGAME_KICKS above). Call
// once per loop().
void updateEndgameTiming();

// Runs one pass of the routine state machine.
//
// MUST BE THE LAST THING loop() DOES: routine 4 state 5 contains a bare
// `return` for the LEFT robot on the INNER lane, which is expected to skip
// everything that would have followed it.
void runRoutines();
