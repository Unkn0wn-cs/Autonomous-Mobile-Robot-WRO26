// Per-wheel speed synchroniser for the 4-wheel omnidirectional base.
// Team Outer Heaven - WRO 2026.
//
// WHY THIS EXISTS
// ---------------
// move.h drives every wheel at a fixed PWM taken from pwmf[] / pwms[] and stops
// once the front encoders have counted enough pulses. Nothing keeps the four
// wheels turning at the SAME rate while that happens, so any difference in motor
// strength, friction, wheel load or battery sag makes the robot drift sideways,
// judder, and finish the move crooked.
//
// THE ONE RULE
// ------------
// Every motion this robot performs - forward, backward, strafe, rotate, and the
// two-wheel diagonals - is supposed to turn each PARTICIPATING wheel through the
// same number of encoder counts. Driving smoothly and straight therefore reduces
// to a single rule:
//
//     make every driven wheel travel the same distance.
//
// Because only the magnitude matters, this class compares |counts travelled|.
// That means it does not care which way round any encoder's A/B pair was wired,
// and it needs no polarity calibration.
//
// CORRECT DOWNWARD ONLY
// ---------------------
// The reference is the SLOWEST driven wheel. A wheel that has run ahead of it has
// its PWM trimmed down, proportionally to how far ahead it is. No wheel is ever
// pushed above its nominal PWM. This matters on this robot specifically: nominal
// is already 243-245 out of 255, so there is no headroom to speed a lagging wheel
// up. Trimming downward can never saturate, and a lagging wheel is never whipped
// forward - the others simply wait for it.
//
// SMOOTH STARTS AND STOPS
// -----------------------
// On top of the trim, the nominal PWM is shaped by a distance-based trapezoid, so
// the robot eases into and out of every move instead of slamming from 0 to 245
// and back to 0. The ramp is measured in encoder counts rather than milliseconds
// so it behaves identically regardless of how fast loop() happens to be running.

#pragma once

#include <Arduino.h>

class WheelRegulator {
  public:
    static const uint8_t WHEEL_COUNT = 4;

    // ---- Synchronisation tuning -------------------------------------------
    // PWM counts removed per encoder count that a wheel is ahead of the slowest.
    // Raise it if the robot still drifts; lower it if the wheels hunt or stutter.
    int syncGain = 4;

    // Hard ceiling on that correction. This is the safety net: if an encoder ever
    // fails and reads zero, every other wheel looks infinitely far ahead, and
    // without this cap they would all be trimmed down to a crawl.
    int maxTrim = 70;

    // A driven wheel is never taken below this. Trim it too far and the motor
    // stalls instead of slowing, which makes the drift worse, not better.
    int minPWM = 110;

    // ---- Acceleration profile tuning --------------------------------------
    // PWM used at the very start and the very end of a move, as a fraction of the
    // nominal PWM, in 1/256ths. 145/256 is about 57%.
    uint8_t startFactor = 215;
    uint8_t endFactor = 200;

    // How much of the move is spent ramping, in 1/256ths of the total distance.
    uint8_t accelFraction = 64; // 25% of the move
    uint8_t decelFraction = 77; // 30% of the move

    // Ceiling on the ramp length in encoder counts, so that a long straight does
    // not spend hundreds of millimetres accelerating.
    long maxRampCounts = 300;

    // The trims are recomputed on this fixed period. Recomputing them every single
    // loop() would make syncGain depend on how fast the loop happens to run.
    uint8_t updateIntervalMs = 10;

    // Arms the regulator for a new move of `targetCounts` encoder counts.
    void begin(long targetCounts) {
      _target = targetCounts > 0 ? targetCounts : 0;

      _accelCounts = scaleFraction(_target, accelFraction);
      _decelCounts = scaleFraction(_target, decelFraction);
      if (_accelCounts > maxRampCounts) _accelCounts = maxRampCounts;
      if (_decelCounts > maxRampCounts) _decelCounts = maxRampCounts;

      for (uint8_t i = 0; i < WHEEL_COUNT; i++) _trim[i] = 0;
      _factor = startFactor;
      _firstUpdate = true;
    }

    // Feeds the regulator the current state of the move.
    //   driven[i]   - false for wheels this motion leaves released
    //   progress[i] - |encoder counts travelled| by wheel i since the move began
    // Safe to call as often as you like; it rate-limits itself internally.
    void update(const bool driven[WHEEL_COUNT], const long progress[WHEEL_COUNT]) {
      unsigned long now = millis();
      if (!_firstUpdate && (now - _lastUpdate) < updateIntervalMs) return;
      _firstUpdate = false;
      _lastUpdate = now;

      // The slowest driven wheel is the reference everyone is held back to.
      // The furthest-along one drives the accel/decel profile, so that a dead
      // encoder reading zero can never stop the robot from reaching its ramp-down.
      long slowest = -1;
      long furthest = 0;
      for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
        if (!driven[i]) continue;
        if (slowest < 0 || progress[i] < slowest) slowest = progress[i];
        if (progress[i] > furthest) furthest = progress[i];
      }
      if (slowest < 0) slowest = 0; // nothing driven; nothing to synchronise

      _factor = profileFactor(furthest);

      for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
        if (!driven[i]) {
          _trim[i] = 0;
          continue;
        }
        long lead = progress[i] - slowest;
        long trim = lead * (long)syncGain;
        if (trim > maxTrim) trim = maxTrim;
        if (trim < 0) trim = 0;
        _trim[i] = (int)trim;
      }
    }

    // The PWM wheel `index` should actually be driven at, given its nominal PWM.
    int pwmFor(uint8_t index, int nominalPWM) const {
      if (index >= WHEEL_COUNT) return nominalPWM;

      int pwm = (int)(((long)nominalPWM * (long)_factor) >> 8);
      pwm -= _trim[index];

      if (pwm < minPWM) pwm = minPWM;
      if (pwm > nominalPWM) pwm = nominalPWM; // never above nominal, by design
      return pwm;
    }

    // How far each wheel is currently being held back. Useful for tuning.
    int trimFor(uint8_t index) const {
      return index < WHEEL_COUNT ? _trim[index] : 0;
    }

    // Current position on the acceleration profile, in 1/256ths of nominal.
    uint8_t profileFactorNow() const { return _factor; }

  private:
    long _target = 0;
    long _accelCounts = 0;
    long _decelCounts = 0;

    int _trim[WHEEL_COUNT] = {0, 0, 0, 0};
    uint8_t _factor = 256 - 1;

    unsigned long _lastUpdate = 0;
    bool _firstUpdate = true;

    static long scaleFraction(long value, uint8_t fraction) {
      return (value * (long)fraction) >> 8;
    }

    // Trapezoid in the distance domain: ramp up from startFactor, hold at full
    // nominal, then ramp down to endFactor. Taking the smaller of the two ramps
    // means a move too short to reach full speed automatically becomes a
    // triangle instead, with no special case needed.
    uint8_t profileFactor(long travelled) const {
      long up = 256;
      if (_accelCounts > 0 && travelled < _accelCounts) {
        up = startFactor + ((256 - (long)startFactor) * travelled) / _accelCounts;
      }

      long down = 256;
      long remaining = _target - travelled;
      if (remaining < 0) remaining = 0;
      if (_decelCounts > 0 && remaining < _decelCounts) {
        down = endFactor + ((256 - (long)endFactor) * remaining) / _decelCounts;
      }

      long factor = up < down ? up : down;
      if (factor > 255) factor = 255;
      if (factor < 1) factor = 1;
      return (uint8_t)factor;
    }
};
