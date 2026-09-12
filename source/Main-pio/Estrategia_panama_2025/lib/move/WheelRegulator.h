// WheelRegulator.h - motion control for the 4-wheel omnidirectional base.
//
// Team Outer Heaven - WRO 2026.
//
// Two sensors, two jobs:
//
//   ENCODERS  distance travelled -> which phase of the move we are in
//             (accel / cruise / decel) and, in decel, the mean speed the
//             speed loop tracks. Nothing else. The encoders never steer.
//
//   BNO08x    heading error -> a PID whose output is a PWM DIFFERENTIAL
//             between the two wheel pairs. This is the only thing that keeps
//             the robot straight.
//
// Every tick the four wheel PWMs are
//
//   pwm_i = common + trim_i + differential * ROT_i * dirSign_i
//
//   common        the speed profile: an open-loop ramp from rampStartPWM to
//                 cruisePWM while accelerating (the wheels need ~200 to break
//                 free), cruisePWM, then a closed-loop deceleration to a creep
//                 at the target (or, for a move that meets a wall before its
//                 target, to an approach speed held over the last part)
//   trim_i        this wheel's static offset from the mean of pwmf[]/pwms[]
//   differential  the heading PID output, +-maxHeadingCorrection
//   ROT           {+1, -1, -1, +1}: wheels 1 and 4 against 2 and 3, i.e. the
//                 F B B F rotation pattern
//   dirSign_i     +1 forward, -1 backward, 0 released, so the same
//                 differential turns the robot the same way whatever the move
//
// The differential is never clipped: if a wheel would exceed maxPWM the whole
// set is shifted down instead, so a correction always arrives in full.
//
// This class does not decide when a move is finished - move.h does that from
// the front encoders and brakes at the target.

#pragma once

#include <Arduino.h>

class WheelRegulator {
  public:
    static const uint8_t WHEEL_COUNT = 4;

    // How much control a move asks for.
    enum Mode {
      // Very short nudges, usually finishing against a wall. Straight to cruise
      // PWM, no profile and no correction: the wall does the aligning.
      Burst,
      // Speed profile only, no heading hold. Wall-hugging straights (the wall
      // aligns them), diagonals, and rotations (a turn is meant to change the
      // heading).
      Profile,
      // Speed profile and heading hold. Every free translation.
      Hold
    };

    enum Phase { Accel, Cruise, Decel };

    // ---- PWM levels (overridden in initHardware()) ---------------------------
    int maxPWM       = 248;
    int rampStartPWM = 205;   // where the accel ramp starts: just above breakaway
    int cruisePWM    = 232;   // open-loop cruise

    // Encoder counts per millimetre: every distance Move is given is converted
    // with it, and the speeds and gains below are in mm/s whichever robot this
    // is. Placeholder until initHardware() sets it from `pulses` and `diameter`.
    float countsPerMM = 0;
    // Any move shorter than this is treated as a Burst whatever it asked for.
    long burstThresholdCounts = 200;

    // ---- Accel -------------------------------------------------------------
    // Fraction of the move spent ramping up, clamped to min/maxRampCounts and
    // to 45 % of the move.
    float rampFraction = 0.22f;
    long  minRampCounts = 40;
    long  maxRampCounts = 1500;

    // ---- Decel (closed loop on mean encoder speed) -------------------------
    // Fraction of the move over which the robot slows down, clamped to
    // min/maxDecelCounts and to 50 % of the move.
    float decelFraction = 0.30f;
    long  minDecelCounts = 300;
    long  maxDecelCounts = 1700;

    // Creep speed at the target: a fraction of the speed the robot had when
    // deceleration began, but never below minEndSpeedMMs so it always gets
    // there.
    float endSpeedFraction = 0.15f;
    float minEndSpeedMMs   = 40.0f;

    // How one move finishes, passed to begin(). Every field at 0 gives the
    // rules above. A move that must be slow BEFORE its target (a backward move
    // meeting the back wall short of the commanded distance, see move.h) sets
    // them: a curve of decelCounts down to endSpeedMMs, then that speed held
    // over the last creepCounts. creepCounts is clamped to 60 % of the move
    // and decelCounts to what is left after the accel ramp and the creep.
    struct EndSpec {
      long  decelCounts;   // length of the deceleration curve; 0 = decelFraction rule
      long  creepCounts;   // end speed held over the last this many counts
      float endSpeedMMs;   // speed at the end of the curve; 0 = endSpeedFraction rule
      EndSpec() : decelCounts(0), creepCounts(0), endSpeedMMs(0.0f) {}
    };

    // Speed loop: common PWM = kSpeedP * error + integral, error in mm/s.
    float kSpeedP = 0.15f;   // PWM per mm/s
    float kSpeedI = 2.0f;    // PWM per mm/s per second

    // Extra PWM per tick added while the robot is below half the creep speed
    // but asked to move, so a wheel that stalls on a low PWM is freed quickly.
    int stallEscapePWM = 2;

    // ---- Heading hold (BNO08x) ---------------------------------------------

    float kHeadingP = 12.0f;
    float kHeadingI = 0.0f;
    float kHeadingD = 0.0f;

    // Errors below this do not drive the P term, so the robot does not dither
    // at sensor noise once it is pointing true. I and D see the raw error - a
    // deadband is a step, and a derivative of a step spikes.
    float headingDeadbandDeg = 0.12f;

    int   maxHeadingCorrection = 40;     // PWM, per wheel pair
    float headingIntegralLimit = 12.0f;  // PWM

    // Fed in by move.h every tick from the BNO08x; 0 when there is no sensor,
    // in which case heading hold simply does nothing.
    float headingErrorDeg = 0.0f;

    // Control period. dt is measured, so the gains hold if a tick runs late.
    uint8_t updateIntervalMs = 4;

    // ------------------------------------------------------------------------
    void begin(long targetCounts, Mode mode = Hold, EndSpec end = EndSpec()) {
      _target = targetCounts > 0 ? targetCounts : 0;

      _mode = mode;
      if (_target <= 0 || _target < burstThresholdCounts) _mode = Burst;

      long ramp = (long)(_target * rampFraction);
      if (ramp < minRampCounts) ramp = minRampCounts;
      if (ramp > maxRampCounts) ramp = maxRampCounts;
      long half = (long)(_target * 0.45f);
      if (ramp > half) ramp = half;
      _rampCounts = ramp;

      long creep = end.creepCounts > 0 ? end.creepCounts : 0;
      long creepMax = (long)(_target * 0.6f);
      if (creep > creepMax) creep = creepMax;
      _creepCounts = creep;

      long decel;
      if (end.decelCounts > 0) {
        decel = end.decelCounts;
      } else {
        decel = (long)(_target * decelFraction);
        if (decel < minDecelCounts) decel = minDecelCounts;
        if (decel > maxDecelCounts) decel = maxDecelCounts;
        if (decel > _target / 2) decel = _target / 2;
      }
      long room = _target - _rampCounts - _creepCounts;
      if (decel > room) decel = room;
      if (decel < 0) decel = 0;
      _decelCounts = decel;

      _endSpeedMMs = end.endSpeedMMs > 0.0f ? end.endSpeedMMs : 0.0f;

      _phase   = Accel;
      _profile = 0.0f;
      _common  = (_mode == Burst) ? (float)cruisePWM : (float)rampStartPWM;
      _speed = 0.0f; _lastMeanProg = 0.0f;
      _vPeak = 0.0f; _vEnd = 0.0f; _vCmd = 0.0f;
      _speedI = _common;

      _headingI = 0.0f;
      _lastHeadingErr = 0.0f;
      _headingD = 0.0f;
      _headingCorr = 0.0f;

      for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
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
        _common = (float)cruisePWM;
        _headingCorr = 0.0f;
        return;
      }

      // ---- Speed profile (encoders) --------------------------------------
      long sum = 0; uint8_t n = 0;
      for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
        if (!driven[i]) continue;
        sum += progress[i]; n++;
      }
      float mean = n ? (float)sum / (float)n : 0.0f;
      long travelled = (long)mean;

      // Mean speed of the driven wheels, counts/s, lightly filtered.
      float vRaw = (mean - _lastMeanProg) / dt;
      _lastMeanProg = mean;
      _speed = 0.75f * _speed + 0.25f * vRaw;

      long remaining = _target - travelled;
      if (remaining < 0) remaining = 0;

      if (_phase != Decel && remaining <= _decelCounts + _creepCounts) {
        _phase = Decel;
        _vPeak = _speed;
        if (_endSpeedMMs > 0.0f) {
          _vEnd = _endSpeedMMs * countsPerMM;
        } else {
          _vEnd = endSpeedFraction * _vPeak;
          float creep = minEndSpeedMMs * countsPerMM;
          if (_vEnd < creep) _vEnd = creep;
        }
        _speedI = _common;               // bumpless: continue from the PWM in force
      }

      if (_phase == Accel) {
        _profile = smoothstep((float)travelled / (float)_rampCounts);
        _common  = (float)rampStartPWM + ((float)cruisePWM - (float)rampStartPWM) * _profile;
        if (_profile >= 1.0f) _phase = Cruise;
      } else if (_phase == Cruise) {
        _profile = 1.0f;
        _common  = (float)cruisePWM;
      } else {
        // Constant deceleration: speed falls with the square root of the
        // distance still to go, from vPeak at the start of the phase to vEnd
        // at the start of the creep zone (at the target when there is none),
        // then vEnd is held.
        long toCreep = remaining - _creepCounts;
        if (toCreep < 0) toCreep = 0;
        float frac = (_decelCounts > 0) ? sqrt((float)toCreep / (float)_decelCounts) : 0.0f;
        _vCmd = _vEnd + (_vPeak - _vEnd) * frac;
        if (_vCmd < _vEnd) _vCmd = _vEnd;

        float e = (_vCmd - _speed) / countsPerMM;        // mm/s
        float u = kSpeedP * e + _speedI;
        bool satHi = u >= (float)maxPWM;
        bool satLo = u <= 0.0f;
        if (!(satHi && e > 0.0f) && !(satLo && e < 0.0f)) {
          _speedI += kSpeedI * e * dt;
        }
        if (_speed < 0.5f * _vEnd) _speedI += (float)stallEscapePWM;
        if (_speedI > (float)maxPWM) _speedI = (float)maxPWM;
        if (_speedI < 0.0f)          _speedI = 0.0f;

        u = kSpeedP * e + _speedI;
        if (u > (float)maxPWM) u = (float)maxPWM;
        if (u < 0.0f)          u = 0.0f;
        _common  = u;
        _profile = (_vPeak > 0.0f) ? (_vCmd / _vPeak) : 0.0f;
      }

      // ---- Heading hold (BNO08x) -----------------------------------------
      if (_mode == Hold) {
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

    // The four wheel PWMs for this instant, given each wheel's trim (its
    // offset from the mean of pwmf[]/pwms[]). The heading differential is
    // applied in full: if the highest wheel would exceed maxPWM, all four are
    // shifted down by the excess instead of clipping it.
    void computePWM(const int trim[WHEEL_COUNT], int out[WHEEL_COUNT]) const {
      static const int8_t ROT[WHEEL_COUNT] = {1, -1, -1, 1};
      float u[WHEEL_COUNT];
      float highest = 0.0f;
      for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
        float heading = _headingCorr * (float)ROT[i] * (float)_dirSign[i];
        u[i] = _common + (float)trim[i] + heading;
        if (u[i] > highest) highest = u[i];
      }
      float shift = highest - (float)maxPWM;
      if (shift < 0.0f) shift = 0.0f;
      for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
        int pwm = (int)(u[i] - shift + 0.5f);
        if (pwm > maxPWM) pwm = maxPWM;
        if (pwm < 0)      pwm = 0;
        out[i] = pwm;
      }
    }

    // ---- Telemetry ---------------------------------------------------------
    Mode  mode()          const { return _mode; }
    Phase phase()         const { return _phase; }
    long  target()        const { return _target; }
    float profile()       const { return _profile; }      // 0..1 speed profile
    float commonPWM()     const { return _common; }
    float speedMMs()      const { return _speed / countsPerMM; }
    float commandMMs()    const { return (_phase == Decel) ? _vCmd / countsPerMM : 0.0f; }
    float headingCorr()   const { return _headingCorr; }
    long  progress(uint8_t i) const { return i < WHEEL_COUNT ? _prog[i] : 0; }

  private:
    Mode  _mode  = Hold;
    Phase _phase = Accel;
    long _target = 0;
    long _rampCounts = 0;
    long _decelCounts = 0;
    long _creepCounts = 0;          // end speed held over the last part of the move
    float _endSpeedMMs = 0.0f;      // 0 = endSpeedFraction / minEndSpeedMMs rule

    float _profile = 0.0f;          // 0 = just creeping, 1 = full cruise
    float _common  = 0.0f;          // PWM level shared by all wheels
    float _speed = 0.0f;            // counts/s, filtered
    float _lastMeanProg = 0.0f;
    float _vPeak = 0.0f, _vEnd = 0.0f, _vCmd = 0.0f;   // counts/s
    float _speedI = 0.0f;

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
};
