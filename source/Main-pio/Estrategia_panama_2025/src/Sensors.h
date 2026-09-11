// Sensors.h - camera, I2C bus scan and microswitch inputs.
//
// Heading lives in Heading.h.

#pragma once

#include <Arduino.h>
#include "Wire.h"
#include <Pixy2.h>

// ---------------------------------------------------------------------------
// Pixy2 camera (SPI) and its colour signatures
// ---------------------------------------------------------------------------

extern Pixy2 pixy;

const int purpleSignature = 2;
const int orangeSignature = 1;

// ---------------------------------------------------------------------------
// Microswitch state
// ---------------------------------------------------------------------------

extern volatile bool backSwitchPressed;  // set on every back switch press
extern bool lastBackSwitchState;         // for edge detection
extern bool lastSideSwitchState;         // for edge detection

// ---------------------------------------------------------------------------
// Operations
// ---------------------------------------------------------------------------

// Scans the whole I2C bus and prints what it finds. Boot-time diagnostic only;
// slow (126 transactions), never call it from loop().
int testI2C();

// Called on a back microswitch press: the robot is square against the wall, so
// this becomes the heading reference routines 7 and 8 turn away from.
void onSwitchPress();
