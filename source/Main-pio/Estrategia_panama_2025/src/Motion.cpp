// Motion.cpp - strategy level movement helpers.

#include "Motion.h"
#include "Hardware.h"
#include "RobotConfig.h"

int mm(int mm) {
  return (move.mmToPulses(mm, diameter, pulses));
}

bool inner(int mili) {
  if (robotSide == RIGHT) {
    if (move.left(mm(mili))) return true;
  } else if (robotSide == LEFT) {
    if (move.right(mm(mili))) return true;
  }
  return false;
}

bool outer(int mili) {
  if (robotSide == RIGHT) {
    if (move.right(mm(mili))) return true;
  } else if (robotSide == LEFT) {
    if (move.left(mm(mili))) return true;
  }
  return false;
}
