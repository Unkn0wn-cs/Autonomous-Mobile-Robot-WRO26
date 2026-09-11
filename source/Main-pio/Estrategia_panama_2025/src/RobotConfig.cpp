// RobotConfig.cpp - THE ROBOT SELECTOR.
//
// ###########################################################################
// #                                                                         #
// #   TO SWITCH ROBOTS: uncomment ONE block below and comment out the other. #
// #   This is the only place either robot is selected.                       #
// #                                                                         #
// ###########################################################################
//
// Leaving both blocks active will not compile (duplicate definitions).
// Leaving both commented out will not link (undefined references).

#include "RobotConfig.h"

// ===========================================================================
// LEFT - WALL
// ===========================================================================
  // int pwmf[4] = {245, 243, 243, 245};
  // int pwms[4] = {220, 225, 220, 225};
  // extern const long pulses = 900;  // encoder counts per wheel revolution
  // side robotSide = LEFT;
  // int slowRotorSpeed = 90;
  // int closedGate = 170;
  // int openGate = 55;

// ===========================================================================
// RIGHT - RAMP
// ===========================================================================
  int pwmf[4] = {220, 243, 243, 220};
  int pwms[4] = {200, 200, 200, 200};
  extern const long pulses = 1650;  // encoder counts per wheel revolution
  side robotSide = RIGHT;
  int slowRotorSpeed = 180;
  int closedGate = 96;
  int openGate = 0;

// ===========================================================================
// Shared between both robots. Do not move this into a block above (it would
// leave the other robot's build with an undefined reference).
// setup() overwrites it based on robotSide.
// ===========================================================================
  int lenght = 0;
