// WheelRegulator.h - motion control for the 4-wheel omnidirectional base.
//
// Team Outer Heaven - WRO 2026.
//
// Every regulated move does three independent things, each measured by a
// different sensor so they do not fight each other:
//
//   1. SPEED SHAPING   ramp the PWM in at the start and out at the end.
//                      Driven by encoder distance.
//   2. WHEEL SYNC      hold each wheel to the mean travel of the others.
//                      Symmetric about the mean, so it never changes the
//                      robot's overall speed. Encoders.
//   3. HEADING HOLD    PID on the BNO08x heading error, applied as a rotation
//                      superimposed on the translation. Only this loop is
//                      allowed to rotate the robot.
//
// The PWM band (minMovePWM..maxPWM), the ramp start and the cruise value are
// set in initHardware(). Cruise sits inside the band so a wheel can be pushed
// up as well as slowed down.
//
// This class does not decide when a move is finished - move.h does that from
// the front encoders, so every distance tuned into the routines keeps its
// meaning.

#pragma once

#include <Arduino.h>

class WheelRegulator {
  public:
    static const uint8_t WHEEL_COUNT = 4;

    // How much control a move asks for.
    enum Mode {
      // Short nudges, usually finishing against a wall. Straight to cruise PWM,
      // no ramp and no correction: the wall does the aligning.
      Burst,
      // Ramp only. For moves that deliberately drive the wheels at different
      // speeds (forwardp/backwardp press the robot into a wall), where sync
      // and heading hold would cancel the very thing that makes them work.
      Ramp,
      // Ramp, wheel sync and heading hold. For real travel.
      Full
    };

    // ---- The hardware band (overridden in initHardware()) ------------------
    int minMovePWM   = 200;   // below this the wheels do not turn
    int maxPWM       = 255;
    int rampStartPWM = 205;   // where the ramp starts, just above the threshold
    int cruisePWM    = 232;   // regulated cruise, mid-band

    // Any move shorter than this is treated as a Burst whatever it asked for:
    // there is no room to ramp over a few dozen counts. Set from mm in
    // initHardware().
    long burstThresholdCounts = 200;

    // ---- Speed shaping -----------------------------------------------------
    // Fraction of the move spent easing in, and easing out. Clamped between
    // minRampCounts and maxRampCounts (set from mm in initHardware()).
    float rampFraction = 0.22f;
    long  minRampCounts = 40;
    long  maxRampCounts = 1500;

    // ---- Wheel synchronisation (encoders) ----------------------------------
    // PWM per encoder count that a wheel differs from the mean of the others.
    float kSync = 0.30f;
    int   maxSyncCorrection = 12;

    // ---- Heading hold (BNO08x) ---------------------------------------------
    // PWM of differential correction per degree of heading error.
    // I and D are 0: the loop is proportional-only while the P response is
    // being observed on the floor. Starting points when enabling them:
    // kHeadingI 3.0, kHeadingD 0.6.
    float kHeadingP = 9.0f;
    float kHeadingI = 0.0f;
    float kHeadingD = 0.0f;

    // Errors below this do not drive the P term, so the robot does not dither
    // at sensor noise once it is pointing true. I and D see the raw error - a
    // deadband is a step, and a derivative of a step spikes.
    float headingDeadbandDeg = 0.12f;

    int   maxHeadingCorrection = 20;
    float headingIntegralLimit = 8.0f;   // in PWM

    // Fed in by move.h every tick from the BNO08x; 0 when there is no sensor,
    // in which case heading hold simply does nothing.
    float headingErrorDeg = 0.0f;
    bool  headingHoldEnabled = false;

    // Control period. dt is measured, so the gains hold if a tick runs late.
    uint8_t updateIntervalMs = 4;

    // ------------------------------------------------------------------------
    void begin(long targetCounts, Mode mode = Full) {
      _target = targetCounts > 0 ? targetCounts : 0;

      _mode = mode;
      if (_target <= 0 || _target < burstThresholdCounts) _mode = Burst;

      long ramp = (long)(_target * rampFraction);
      if (ramp < minRampCounts) ramp = minRampCounts;
      if (ramp > maxRampCounts) ramp = maxRampCounts;
      long half = (long)(_target * 0.45f);
      if (ramp > half) ramp = half;
      _rampCounts = ramp;

      _profile = 0.0f;
      _headingI = 0.0f;
      _lastHeadingErr = 0.0f;
      _headingD = 0.0f;
      _headingCorr = 0.0f;

      for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
        _sync[i] = 0.0f;
        _dirSign[i] = 1;
        _prog[i] = 0;
      }

      _first = true;
      _lastUpdate = millis();
    }

    // One observation.
    //   driven[i]   - false for wheels this motion releases
    //   progress[i] - |encoder counts| travelled by wheel i since the move began
    //   dirSign[i]  - +1 commanded forward, -1 backward, 0 released
    void update(const bool driven[WHEEL_COUNT],
                const long progress[WHEEL_COUNT],
                const int8_t dirSign[WHEEL_COUNT]) {

      for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
        _prog[i] = progress[i];
        _dirSign[i] = dirSign[i];
      }

      unsigned long now = millis();
      if (_first) {
        _first = false;
        _lastUpdate = now;
        _lastHeadingErr = headingErrorDeg;   // no derivative kick on tick one
        return;
      }
      unsigned long elapsed = now - _lastUpdate;
      if (elapsed < updateIntervalMs) return;
      float dt = elapsed * 0.001f;
      _lastUpdate = now;

      if (_mode == Burst) {
        _profile = 1.0f;
        _headingCorr = 0.0f;
        for (uint8_t i = 0; i < WHEEL_COUNT; i++) _sync[i] = 0.0f;
        return;
      }

      // ---- 1. speed shaping ------------------------------------------------
      long furthest = 0;
      long sum = 0; uint8_t n = 0;
      for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
        if (!driven[i]) continue;
        if (progress[i] > furthest) furthest = progress[i];
        sum += progress[i]; n++;
      }
      float mean = n ? (float)sum / (float)n : 0.0f;

      _profile = rampProfile(furthest);

      // ---- 2. wheel synchronisation ---------------------------------------
      // Deviations are measured against the mean, so they sum to zero and a
      // correction only shares speed out differently. Ramp mode skips this.
      if (_mode == Full) {
        for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
          if (!driven[i]) { _sync[i] = 0.0f; continue; }
          float dev = mean - (float)progress[i];      // + when this wheel is behind
          float c = kSync * dev;
          if (c >  (float)maxSyncCorrection) c =  (float)maxSyncCorrection;
          if (c < -(float)maxSyncCorrection) c = -(float)maxSyncCorrection;
          _sync[i] = c;
        }

        // A heading correction makes two wheels travel further than the other
        // two on purpose. Left alone, the sync loop would see that as an error
        // and pull them back - two loops, opposite commands, same motors. So
        // the rotation component is projected out of the sync correction and
        // rotation is left entirely to the heading loop, which measures it.
        if (headingHoldEnabled) {
          static const int8_t ROT[WHEEL_COUNT] = {1, -1, -1, 1};
          float dot = 0.0f; uint8_t m = 0;
          for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
            if (!driven[i]) continue;
            dot += _sync[i] * (float)ROT[i] * (float)dirSign[i];
            m++;
          }
          if (m > 0) {
            dot /= (float)m;
            for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
              if (!driven[i]) continue;
              _sync[i] -= dot * (float)ROT[i] * (float)dirSign[i];
            }
          }
        }
      } else {
        for (uint8_t i = 0; i < WHEEL_COUNT; i++) _sync[i] = 0.0f;
      }

      // ---- 3. heading hold -------------------------------------------------
      if (_mode == Full && headingHoldEnabled) {
        float raw = headingErrorDeg;

        // A jump this large between two ticks is not the robot turning: the
        // sensor was re-referenced (reset, or a stale read returning). Start
        // the derivative and integral again from here instead of reacting.
        float jump = raw - _lastHeadingErr;
        if (jump > 30.0f || jump < -30.0f) {
          _lastHeadingErr = raw;
          _headingD = 0.0f;
          _headingI = 0.0f;
        }

        // Only P sees the deadband; I and D see the raw error.
        float eP = (raw < headingDeadbandDeg && raw > -headingDeadbandDeg)
                 ? 0.0f : raw;

        float rate = (raw - _lastHeadingErr) / dt;
        _headingD = 0.7f * _headingD + 0.3f * rate;
        _lastHeadingErr = raw;

        float pd = kHeadingP * eP + kHeadingD * _headingD;
        bool saturated = (pd > (float)maxHeadingCorrection) ||
                         (pd < -(float)maxHeadingCorrection);

        // Integral frozen while the output is at its limit (anti-windup).
        if (!saturated) {
          _headingI += kHeadingI * raw * dt;
          if (_headingI >  headingIntegralLimit) _headingI =  headingIntegralLimit;
          if (_headingI < -headingIntegralLimit) _headingI = -headingIntegralLimit;
        }

        float c = pd + _headingI;
        if (c >  (float)maxHeadingCorrection) c =  (float)maxHeadingCorrection;
        if (c < -(float)maxHeadingCorrection) c = -(float)maxHeadingCorrection;
        _headingCorr = c;
      } else {
        _headingCorr = 0.0f;
        _headingI = 0.0f;
      }
    }

    // The PWM wheel `index` should run at, given the per-wheel trim this motion
    // asked for (the relative differences in pwmf[] / pwms[]).
    int pwmFor(uint8_t index, int wheelTrim) {
      if (index >= WHEEL_COUNT) return cruisePWM;

      int base = cruisePWM + wheelTrim;
      if (base > maxPWM) base = maxPWM;
      if (base < minMovePWM) base = minMovePWM;

      // Ramp runs from rampStartPWM up to this wheel's cruise value.
      float target = (_mode == Burst)
                   ? (float)base
                   : (float)rampStartPWM + ((float)base - (float)rampStartPWM) * _profile;

      // Heading is a rotation added on top of the translation: wheel i goes up
      // if it is commanded forward and down if commanded backward, so the same
      // correction turns the robot regardless of which way it is translating.
      static const int8_t ROT[WHEEL_COUNT] = {1, -1, -1, 1};
      float heading = _headingCorr * (float)ROT[index] * (float)_dirSign[index];

      float u = target + _sync[index] + heading;

      int pwm = (int)(u + 0.5f);
      if (pwm > maxPWM) pwm = maxPWM;
      if (pwm < minMovePWM) pwm = minMovePWM;
      return pwm;
    }

    // ---- Telemetry ---------------------------------------------------------
    Mode  mode()          const { return _mode; }
    long  target()        const { return _target; }
    float profile()       const { return _profile; }
    float headingCorr()   const { return _headingCorr; }
    float syncCorr(uint8_t i) const { return i < WHEEL_COUNT ? _sync[i] : 0.0f; }
    long  progress(uint8_t i) const { return i < WHEEL_COUNT ? _prog[i] : 0; }

    long spread(const bool driven[WHEEL_COUNT]) const {
      long lo = -1, hi = 0;
      for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
        if (!driven[i]) continue;
        if (lo < 0 || _prog[i] < lo) lo = _prog[i];
        if (_prog[i] > hi) hi = _prog[i];
      }
      return lo < 0 ? 0 : hi - lo;
    }

  private:
    Mode _mode = Full;
    long _target = 0;
    long _rampCounts = 0;

    float _profile = 0.0f;          // 0 = just creeping, 1 = full cruise
    float _sync[WHEEL_COUNT]  = {0, 0, 0, 0};
    long  _prog[WHEEL_COUNT]  = {0, 0, 0, 0};
    int8_t _dirSign[WHEEL_COUNT] = {1, 1, 1, 1};

    float _headingCorr = 0.0f;
    float _headingI = 0.0f;
    float _headingD = 0.0f;
    float _lastHeadingErr = 0.0f;

    bool _first = true;
    unsigned long _lastUpdate = 0;

    // Zero slope at both ends, so there is no jolt where ramp meets cruise.
    static float smoothstep(float t) {
      if (t <= 0.0f) return 0.0f;
      if (t >= 1.0f) return 1.0f;
      return t * t * (3.0f - 2.0f * t);
    }

    // Speed as a function of position, not time: deceleration always starts a
    // known distance from the target.
    float rampProfile(long travelled) const {
      if (_rampCounts <= 0) return 1.0f;
      float up = smoothstep((float)travelled / (float)_rampCounts);
      float remaining = (float)(_target - travelled);
      if (remaining < 0.0f) remaining = 0.0f;
      float down = smoothstep(remaining / (float)_rampCounts);
      return up < down ? up : down;
    }
};
