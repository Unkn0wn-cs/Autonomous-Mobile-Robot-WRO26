// Motion.h - strategy level movement helpers.
//
// These sit between the routines and the Move library. They exist so the
// routines can say "go towards the outside of the field" without working out,
// every single time, whether that means left or right for this particular robot.

#pragma once

#include <Arduino.h>

// Converts millimetres into encoder counts for the current robot.
//
// Uses `pulses` as counts per wheel revolution with a 60 mm wheel, so roughly
// 4.77 counts/mm on LEFT and 8.75 counts/mm on RIGHT.
//
// CAREFUL: not every call site in the routines uses this. Several pass RAW
// ENCODER COUNTS directly - move.forward(80), move.backward(600), outer(750),
// inner(180). Always check which unit a number is in before changing it.
int mm(int mm);

// Strafe towards the inside of the field. Mirrored on robotSide.
//
// CAREFUL: this converts with mm() INTERNALLY. Passing an already converted
// value applies the conversion twice. See the note on outer().
bool inner(int mili);

// Strafe towards the outside of the field. Mirrored on robotSide.
//
// CAREFUL: converts with mm() internally, and two call sites pass an already
// converted value - `outer(mm(20))` in routine 4 state -2 and routine 7 state 6.
// That is mm(mm(20)), about 453 counts rather than 95. The robot is tuned around
// this behaviour, so it is preserved deliberately. Do not "fix" it without
// re-running the whole course.
bool outer(int mili);
