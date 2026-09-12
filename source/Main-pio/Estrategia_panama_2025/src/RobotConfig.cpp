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
  int pwmf[4] = {245, 243, 243, 245};
  int pwms[4] = {220, 225, 220, 225};
  extern const long pulses = 900;  // encoder counts per wheel revolution
  side robotSide = LEFT;
  int slowRotorSpeed = 90;
  int closedGate = 180;
  int openGate = 55;
  int lenght = 1100;
  // Purple ball zones {xA, yA, xB, yB}, Pixy pixels. Format in RobotConfig.h.
  extern const int ballZones[NUM_BALL_ZONES][4] = {
    { 135,  20,  160,   0 },   // routine 0 - upper left
    { 235,  25,  260,  10 },   // routine 1 - upper right
    { 135,  40,  160,  20 },   // routine 2 - lower left
    { 260,  55,  290,  25 }    // routine 3 - lower right
  };

// ===========================================================================
// RIGHT - RAMP
// ===========================================================================
  // int pwmf[4] = {220, 243, 243, 220};
  // int pwms[4] = {200, 200, 200, 200};
  // extern const long pulses = 1350;  // encoder counts per wheel revolution
  // side robotSide = RIGHT;
  // int slowRotorSpeed = 180;
  // int closedGate =116;
  // int openGate = 0;
  // int lenght = 640;
  // // Purple ball zones {xA, yA, xB, yB}, Pixy pixels. Format in RobotConfig.h.
  // extern const int ballZones[NUM_BALL_ZONES][4] = {
  //   { 142,  32,  160,  15 },   // routine 0 - upper left
  //   { 245,  33,  260,  20 },   // routine 1 - upper right
  //   { 143,  51,  170,  25 },   // routine 2 - lower left
  //   { 271,  60,  290,  35 }    // routine 3 - lower right
  // };
