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
// LEFT - WALL                                                       [ACTIVE]
// ===========================================================================
  int pwmf[4] = {245, 243, 243, 245};
  int pwms[4] = {220, 225, 220, 225};
  extern const long pulses = 900; // Number of pulses for each movement step
  side robotSide = LEFT;
  int slowRotorSpeed = 90;
  int closedGate = 170;
  int openGate = 55;
  int pixyBalls[16] = {0}; //Position of the puple Balls 🟣🟣🟣 writen as {x upper left corner 1st ball, y upper left corner 1st ball, x bottom right corner 1st ball, y bottom right corner 1st ball, ...}

// ===========================================================================
// RIGHT - RAMP                                                    [INACTIVE]
// ===========================================================================
  // int pwmf[4] = {230, 243, 243, 230};
  // int pwms[4] = {200, 200, 200, 200};
  // extern const long pulses = 1650; // Number of pulses for each movement step
  // side robotSide = RIGHT;
  // int slowRotorSpeed = 180;
  // int closedGate = 96;
  // int openGate = 0;
  // int pixyBalls[16] = {0}; //Position of the puple Balls 🟣🟣🟣

// ===========================================================================
// Shared between both robots. Do not move this into a block above: it was
// previously inside the LEFT block only, which is why the RIGHT build failed.
// setup() overwrites it based on robotSide.
// ===========================================================================
  int lenght = 0;
