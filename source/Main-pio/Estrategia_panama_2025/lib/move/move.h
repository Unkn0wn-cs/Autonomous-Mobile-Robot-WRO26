// move.h - motion primitives for the 4-wheel omnidirectional base.
//
// Team Outer Heaven - WRO 2026. Adafruit Motor Shield v1, quadrature encoder on
// every motor, four 45-degree omni wheels.
//
// Layout seen from above, front of the robot pointing up:
//
//        FRONT
//   motor3   motor4
//   motor2   motor1
//
// Every distance-counted primitive below takes its distance in MILLIMETRES
// (wheel travel, so for rotate() the arc each wheel rolls) and is
// NON-BLOCKING: call it every pass of loop() and it returns true once, when
// the move has finished. Straights, diagonals and encoder rotations drive
// the motors through WheelRegulator: the encoders shape the speed over the
// move, the BNO08x heading PID keeps it straight (see WheelRegulator.h). The
// strafes do not: left()/right() run each wheel at its calibrated PWM with a
// small heading trim on top (see "Strafes" below).
//
// pwmFwd* (pwmf[] in Hardware.h) are per-wheel TRIMS: the regulator drives
// every wheel from a common PWM plus this wheel's difference from the mean of
// the four. pwmStrafe* (pwms[]) are the PWM each wheel actually runs at in a
// strafe; the diagonals and rotations use only their differences as trims.
//
// Distance and completion are measured on the TRUSTED encoders (trusted[],
// set in initHardware(); a dead encoder is marked false there): a move ends
// when the second trusted driven wheel reaches the target count - so one
// noisy encoder cannot end it early and one dead encoder cannot hold it - or
// after moveTimeoutMs, whichever comes first. With a single trusted driven
// wheel that wheel decides. The motors are braked the moment the count is
// reached.
//
// This library knows nothing about which robot it is on. Everything robot
// specific - the PWM band, the heading source, the encoder counts per
// millimetre, which encoders to trust and which way inner()/outer() strafe -
// is handed to it by initHardware() (src/Hardware.cpp).

#pragma once
#include <AFMotor.h>
#include <QuadratureEncoder.h>
#include "WheelRegulator.h"


class Move {
  public:
    // Hard cap on any single move. A move that has not reached its count by
    // then (usually because it is pressed against a wall) is stopped and
    // reported as done.
    unsigned long moveTimeoutMs = 4000;

    // Which way inner() strafes: left when true, right when false; outer()
    // goes the opposite way. Set in initHardware() from robotSide (the RIGHT
    // robot strafes left for inner(), the LEFT robot right).
    bool innerIsLeft = true;

    // Wall approach. Backward moves (backward, backwardp, backwardLeft,
    // backwardRight) end on the back wall, and the routines command more
    // distance than there is so that the back microswitch, not the count,
    // ends the move. The wall therefore comes BEFORE the target, where the
    // normal profile is still fast. A backward move longer than
    // longBackwardMM finishes with backwardEnd instead: a curve of
    // decelCounts down to endSpeedMMs, held over the last creepCounts, so the
    // wall is met at that speed. Both are set in initHardware().
    int longBackwardMM = 0;
    WheelRegulator::EndSpec backwardEnd;

    // Wall hug: the angle, in degrees, that forwardp / backwardp / forwardq
    // hold toward the wall they run along. The heading PID keeps the robot
    // at that angle, so the leading corner presses on the wall and the angle
    // can never grow into a turn. Set in initHardware().
    float wallHugDeg = 2.0f;

    // Which encoders to believe, motor1..motor4. Set in initHardware() from
    // the robot's block; a wheel marked false neither ends a move nor feeds
    // the speed profile. Mark a dead encoder false, put it back after repair.
    bool trusted[WheelRegulator::WHEEL_COUNT] = {true, true, true, true};

    // PWM values for forward/backward (trims)
    int pwmFwd1, pwmFwd2, pwmFwd3, pwmFwd4;
    // PWM values for left/right (absolute) and the diagonals/rotations (trims)
    int pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4;

    // ---- Strafes: the heading trim ---------------------------------------
    // left()/right() run each wheel at its pwmStrafe value - the calibrated
    // numbers are what runs - plus a trim on the BNO08x error that keeps the
    // robot aiming forward: strafeHeadingP PWM per degree, never more than
    // strafeHeadingMax, and moving no faster than strafeHeadingSlew PWM per
    // second, so a jump in the error becomes a ramp on the wheels and the
    // trim can never yank one. Starts from 0 on every strafe. Set in
    // initHardware(); keep pwms[] under 255 so the trim has room both ways.
    float strafeHeadingP    = 0.0f;
    int   strafeHeadingMax  = 0;
    float strafeHeadingSlew = 0.0f;
    float strafeCorr        = 0.0f;   // the trim in force, PWM (telemetry)

    // Speed profile and heading hold. Its parameters are set in
    // initHardware() (e.g. move.regulator.cruisePWM = 232).
    WheelRegulator regulator;

    // PWM currently driving motor1..motor4; 0 for a wheel that is released.
    // Updated on every regulated tick and by rotateCW()/rotateCCW(), cleared by
    // stop() and stopForMillis(). Read only by the telemetry line in main.cpp;
    // nothing in here reads it back.
    int wheelPWM[WheelRegulator::WHEEL_COUNT] = {0, 0, 0, 0};

    // enc1..enc4 belong to motor1..motor4. Distance, completion and the mean
    // speed come from the trusted ones.
    Move(AF_DCMotor& m1, AF_DCMotor& m2, AF_DCMotor& m3, AF_DCMotor& m4,
         Encoders& enc1, Encoders& enc2, Encoders& enc3, Encoders& enc4,
         int fwdPWM1 = 255, int fwdPWM2 = 255, int fwdPWM3 = 255, int fwdPWM4 = 255,
         int strafePWM1 = 255, int strafePWM2 = 255, int strafePWM3 = 255, int strafePWM4 = 255)
      : pwmFwd1(fwdPWM1), pwmFwd2(fwdPWM2), pwmFwd3(fwdPWM3), pwmFwd4(fwdPWM4),
        pwmStrafe1(strafePWM1), pwmStrafe2(strafePWM2), pwmStrafe3(strafePWM3), pwmStrafe4(strafePWM4),
        motor1(m1), motor2(m2), motor3(m3), motor4(m4) {
      encoders[0] = &enc1;
      encoders[1] = &enc2;
      encoders[2] = &enc3;
      encoders[3] = &enc4;
    }

    // Records the four requested PWM values as trims and applies them.
    void begin(int pwm = 255, int pwm2 = 255, int pwm3 = 255, int pwm4 = 255) {
      setNominalSpeeds(pwm, pwm2, pwm3, pwm4);
      motor1.setSpeed(pwm);
      motor2.setSpeed(pwm2);
      motor3.setSpeed(pwm3);
      motor4.setSpeed(pwm4);
    }

    // Applies the strafe PWM values directly. Used by routine 9 before its
    // camera-driven strafes.
    void moveBeginStrafe(){
      setNominalSpeeds(pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4);
      motor1.setSpeed(pwmStrafe1);
      motor2.setSpeed(pwmStrafe2);
      motor3.setSpeed(pwmStrafe3);
      motor4.setSpeed(pwmStrafe4);
    }

    // Where the heading comes from. Set once in initHardware(). Function
    // pointers keep this library independent of the sensor; with none set,
    // every movement runs without heading hold and turnTo() turns by count.
    float (*headingSource)()  = 0;   // current error from the held heading, degrees
    void  (*headingCapture)() = 0;   // "hold whatever heading we are on now"
    float (*headingNorth)()   = 0;   // degrees from north, NAN when unavailable

    void setHeadingHooks(float (*source)(), void (*capture)(), float (*north)() = 0) {
      headingSource  = source;
      headingCapture = capture;
      headingNorth   = north;
    }

    // turnTo(): the PWM of its open-loop spin and how close to the target it
    // stops. Set in initHardware().
    int   turnPWM     = 200;
    float turnDoneDeg = 5.0f;

    // Zeroes the start counts of all four wheels and arms the regulator.
    // `targetPulses` sizes the profile; `mode` selects Burst / Profile / Hold;
    // `end` shapes the finish (default: the regulator's normal rules).
    void startMove(long targetPulses = 0,
                   WheelRegulator::Mode mode = WheelRegulator::Hold,
                   WheelRegulator::EndSpec end = WheelRegulator::EndSpec()) {
      for (uint8_t i = 0; i < WheelRegulator::WHEEL_COUNT; i++) {
        wheelStart[i] = encoders[i] ? encoders[i]->getEncoderCount() : 0;
      }
      regulator.begin(targetPulses, mode, end);
      strafeCorr = 0.0f;
      strafing = false;
      moving = true;
    }

    // ---- Wall-hugging straights ------------------------------------------
    // forwardp/backwardp/forwardq run in Hold mode with the heading target
    // offset by wallHugDeg toward the wall: the PID settles the robot at that
    // angle and holds it there, so the leading corner stays pressed on the
    // wall and wall friction cannot turn the robot further.
    //
    //   position false : the wall is on the robot's LEFT  (the LEFT robot)
    //   position true  : the wall is on the robot's RIGHT (the RIGHT robot)
    //
    // The heading reading increases when the robot turns right, so leaning
    // right means feeding the PID `error - wallHugDeg`, leaning left
    // `error + wallHugDeg`. Reversing, the tail leads, so the nose points the
    // other way. forwardq hugs the inner (centre) wall, the mirror of forwardp.
    //
    // forwardp returns 1 when the full distance is reached (and stops), 2 once
    // 14/22 of it is reached (without stopping), 0 otherwise.
    int forwardp(int millimetres, bool position) {
      long pulses = toCounts(millimetres);
      if (position == false){
        armMotion(MOTION_FORWARDP_NEAR, WheelRegulator::Hold, pulses, pwmFwd1, pwmFwd2, pwmFwd3, pwmFwd4);
      }else{
        armMotion(MOTION_FORWARDP_FAR, WheelRegulator::Hold, pulses, pwmFwd1, pwmFwd2, pwmFwd3, pwmFwd4);
      }

      runRegulated(FORWARD, FORWARD, FORWARD, FORWARD, position ? -wallHugDeg : wallHugDeg);
      if (checkDoneWithTimeout(pulses)){
        return 1;
      } else if (checkDoneWithTimeout(((pulses / 22)*14), true)){
        return 2;
      } else {return 0;};
    }

    // Same 1 / 2 / 0 contract as forwardp, with full regulation.
    int forwardRegulated(int millimetres) {
      long pulses = toCounts(millimetres);
      armMotion(MOTION_FORWARD_REGULATED, WheelRegulator::Hold, pulses, pwmFwd1, pwmFwd2, pwmFwd3, pwmFwd4);

      runRegulated(FORWARD, FORWARD, FORWARD, FORWARD);
      if (checkDoneWithTimeout(pulses)){
        return 1;
      } else if (checkDoneWithTimeout(((pulses / 22)*14), true)){
        return 2;
      } else {return 0;};
    }

    bool backwardp(int millimetres, bool position) {
      long pulses = toCounts(millimetres);
      if (position == false){
        armMotion(MOTION_BACKWARDP_NEAR, WheelRegulator::Hold, pulses, pwmFwd1, pwmFwd2, pwmFwd3, pwmFwd4);
      }else{
        armMotion(MOTION_BACKWARDP_FAR, WheelRegulator::Hold, pulses, pwmFwd1, pwmFwd2, pwmFwd3, pwmFwd4);
      }

      runRegulated(BACKWARD, BACKWARD, BACKWARD, BACKWARD, position ? wallHugDeg : -wallHugDeg);
      return checkDoneWithTimeout(pulses);
    }

    bool forwardq(int millimetres, bool position) {
      long pulses = toCounts(millimetres);
      if (position == false){
        armMotion(MOTION_FORWARDQ_NEAR, WheelRegulator::Hold, pulses, pwmFwd1, pwmFwd2, pwmFwd3, pwmFwd4);
      }else{
        armMotion(MOTION_FORWARDQ_FAR, WheelRegulator::Hold, pulses, pwmFwd1, pwmFwd2, pwmFwd3, pwmFwd4);
      }

      runRegulated(FORWARD, FORWARD, FORWARD, FORWARD, position ? wallHugDeg : -wallHugDeg);
      return checkDoneWithTimeout(pulses);
    }

    // ---- Free translations -----------------------------------------------
    // forward/backward: speed profile + heading hold through the regulator.

    bool forward(int millimetres) {
      long pulses = toCounts(millimetres);
      armMotion(MOTION_FORWARD, WheelRegulator::Hold, pulses, pwmFwd1, pwmFwd2, pwmFwd3, pwmFwd4);
      runRegulated(FORWARD, FORWARD, FORWARD, FORWARD);
      return checkDoneWithTimeout(pulses);
    }

    bool backward(int millimetres) {
      long pulses = toCounts(millimetres);
      armMotion(MOTION_BACKWARD, WheelRegulator::Hold, pulses, pwmFwd1, pwmFwd2, pwmFwd3, pwmFwd4);
      runRegulated(BACKWARD, BACKWARD, BACKWARD, BACKWARD);
      return checkDoneWithTimeout(pulses);
    }

    // ---- Strafes ---------------------------------------------------------
    // Each wheel at its calibrated pwmStrafe value for the whole move, plus
    // the heading trim described at the top of the class; no speed profile,
    // braked at the count. The regulator is armed (start counts, timeout,
    // heading capture) but never consulted.

    bool left(int millimetres) {
      long pulses = toCounts(millimetres);
      armMotion(MOTION_LEFT, WheelRegulator::Burst, pulses, pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4);
      runStrafe(BACKWARD, FORWARD, BACKWARD, FORWARD);
      return checkDoneWithTimeout(pulses);
    }

    bool right(int millimetres) {
      long pulses = toCounts(millimetres);
      armMotion(MOTION_RIGHT, WheelRegulator::Burst, pulses, pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4);
      runStrafe(FORWARD, BACKWARD, FORWARD, BACKWARD);
      return checkDoneWithTimeout(pulses);
    }

    // ---- Field-relative strafes ------------------------------------------
    // left()/right() mirrored on innerIsLeft, so the routines can say
    // "towards the inside of the field" without working out, every single
    // time, whether that means left or right for this robot.

    // Strafe towards the inside of the field.
    bool inner(int millimetres) {
      return innerIsLeft ? left(millimetres) : right(millimetres);
    }

    // Strafe towards the outside of the field.
    bool outer(int millimetres) {
      return innerIsLeft ? right(millimetres) : left(millimetres);
    }

    // ---- Diagonals -------------------------------------------------------
    // Two wheels drive, two are released. BurstHold mode: full power to the
    // target, no profile, braked at the count, and the heading PID keeps the
    // robot pointing the way the move started.

    bool forwardLeft(int millimetres) {
      long pulses = toCounts(millimetres);
      armMotion(MOTION_FORWARD_LEFT, WheelRegulator::BurstHold, pulses, pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4);
      runRegulated(RELEASE, FORWARD, RELEASE, FORWARD);
      return checkDoneWithTimeout(pulses);
    }

    bool forwardRight(int millimetres) {
      long pulses = toCounts(millimetres);
      armMotion(MOTION_FORWARD_RIGHT, WheelRegulator::BurstHold, pulses, pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4);
      runRegulated(FORWARD, RELEASE, FORWARD, RELEASE);
      return checkDoneWithTimeout(pulses);
    }

    bool backwardLeft(int millimetres) {
      long pulses = toCounts(millimetres);
      armMotion(MOTION_BACKWARD_LEFT, WheelRegulator::BurstHold, pulses, pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4);
      runRegulated(BACKWARD, RELEASE, BACKWARD, RELEASE);
      return checkDoneWithTimeout(pulses);
    }

    bool backwardRight(int millimetres) {
      long pulses = toCounts(millimetres);
      armMotion(MOTION_BACKWARD_RIGHT, WheelRegulator::BurstHold, pulses, pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4);
      runRegulated(RELEASE, BACKWARD, RELEASE, BACKWARD);
      return checkDoneWithTimeout(pulses);
    }

    // ---- Rotation --------------------------------------------------------
    // Encoder-counted turn by `millimetres` of wheel travel: speed profile
    // only, heading hold OFF (a turn is supposed to change the heading).
    //   side == true   pattern F B B F  (the rotateCCW() pattern)
    //   side == false  pattern B F F B  (the rotateCW()  pattern)
    bool rotate(int millimetres, bool side) {
      long pulses = toCounts(millimetres);
      if (side == true){
        armMotion(MOTION_ROTATE_FBBF, WheelRegulator::Profile, pulses, pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4);
        runRegulated(FORWARD, BACKWARD, BACKWARD, FORWARD);
      } else {
        armMotion(MOTION_ROTATE_BFFB, WheelRegulator::Profile, pulses, pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4);
        runRegulated(BACKWARD, FORWARD, FORWARD, BACKWARD);
      }

      return checkDoneWithTimeout(pulses);
    }

    // Turn on the heading sensor to `targetDeg`, an angle FROM NORTH (the
    // headingNorth hook: degrees from the last back-wall squaring), not a
    // relative turn. Open-loop spin at turnPWM, watched on the sensor, braked
    // once inside turnDoneDeg of the target, capped at moveTimeoutMs. The
    // sign follows the sensor: a target reached the B F F B way (rotateCW())
    // is positive. Without a heading it turns by encoder count instead:
    // `fallbackMM` of wheel travel on `fallbackSide`, as rotate() would.
    // Returns true once, when the turn is over.
    bool turnTo(float targetDeg, int fallbackMM, bool fallbackSide) {
      float north = headingNorth ? headingNorth() : NAN;
      if (isnan(north)) {
        turnTarget = NAN;
        return rotate(fallbackMM, fallbackSide);
      }

      // A new target starts a new turn, which also covers a turn a routine
      // abandoned half way: its timer is not carried into the next one.
      if (turnTarget != targetDeg) {
        turnTarget = targetDeg;
        moveStartTime = millis();
        moving = false;
      }

      float off = north - targetDeg;
      if (off >  180.0f) off -= 360.0f;
      if (off < -180.0f) off += 360.0f;

      bool inside  = off <= turnDoneDeg && off >= -turnDoneDeg;
      bool timeout = millis() - moveStartTime > moveTimeoutMs;
      if (inside || timeout) {
        stop();
        turnTarget = NAN;
        return true;
      }

      // A positive reading means the robot turned the B F F B way; CCW lowers it.
      if (off > 0) rotateCCW(turnPWM, turnPWM, turnPWM, turnPWM);
      else         rotateCW (turnPWM, turnPWM, turnPWM, turnPWM);
      return false;
    }

    // Open-loop rotation at fixed PWM. Not distance-counted: the caller decides
    // when to stop. The heading reading INCREASES under rotateCW() and
    // DECREASES under rotateCCW() - see the heading section of Sensors.h.
    void rotateCW(int pwm, int pwm2, int pwm3, int pwm4) {
      setMotors(BACKWARD, FORWARD, FORWARD, BACKWARD);
      setSpeeds(pwm, pwm2, pwm3, pwm4);
      moving = false;
    }
    void rotateCCW(int pwm, int pwm2, int pwm3, int pwm4) {
      setMotors(FORWARD, BACKWARD, BACKWARD, FORWARD);
      setSpeeds(pwm, pwm2, pwm3, pwm4);
      moving = false;
    }

    // ---- Stopping --------------------------------------------------------
    // On the L293D, both inputs low (RELEASE) with the enable held high is the
    // datasheet's "fast motor stop": the motor is shorted through the driver
    // and brakes. The enable is the PWM, so the brake is applied at full duty.
    // With the enable low the outputs float and the motor coasts.

    // Brakes all four motors and holds them.
    void stop() {
      setMotors(RELEASE, RELEASE, RELEASE, RELEASE);
      brakeHold();
      moving = false;
    }

    // Brakes the motors and returns true once durationMs has passed.
    // One shared timer: only one stopForMillis() can be in progress at a time.
    bool stopForMillis(unsigned long durationMs) {
      static unsigned long startTime = 0;
      static bool stopping = false;

      if (!stopping) {
        setMotors(RELEASE, RELEASE, RELEASE, RELEASE);
        brakeHold();
        moving = false;
        startTime = millis();
        stopping = true;
      }

      if (millis() - startTime < durationMs) {
        return false; // Still stopping
      } else {
        stopping = false;
        return true; // Done stopping
      }
    }

    // Travel of one wheel since the current or last move began, in counts.
    long wheelTravelCounts(uint8_t i) {
      if (i >= WheelRegulator::WHEEL_COUNT || !encoders[i]) return 0;
      long delta = encoders[i]->getEncoderCount() - wheelStart[i];
      return delta < 0 ? -delta : delta;
    }

    // The travel the finish is judged on, in counts: the second-highest among
    // the trusted wheels this move drives (the highest when only one). After
    // a move has ended this is target + overshoot.
    long travelCounts() {
      long best = 0, second = 0; uint8_t n = 0;
      for (uint8_t i = 0; i < WheelRegulator::WHEEL_COUNT; i++) {
        if (!trusted[i] || !activeDriven[i]) continue;
        long t = wheelTravelCounts(i);
        n++;
        if (t > best) { second = best; best = t; }
        else if (t > second) { second = t; }
      }
      return (n >= 2) ? second : best;
    }

    // ---- Completion ------------------------------------------------------
    // The move ends when two trusted driven wheels have reached `pulses` -
    // the one such wheel, if there is only one - or at moveTimeoutMs. With
    // far == false the motors are braked; with far == true they keep running
    // (used for the "14/22 of the way" signal).
    bool checkDoneWithTimeout(long pulses, bool far = false) {
      if (travelCounts() >= pulses) {
        if (!far){
          stop();
        }
        return true;
      }
      if (millis() - moveStartTime > moveTimeoutMs) {
        stop();
        return true;
      }
      return false;
    }

    // The heading correction in force, PWM: the strafe trim while a strafe
    // is the active motion, the regulator's differential otherwise. For the
    // telemetry line.
    float headingCorr() const {
      return (activeMotion == MOTION_LEFT || activeMotion == MOTION_RIGHT)
             ? strafeCorr : regulator.headingCorr();
    }

  private:
    AF_DCMotor& motor1;
    AF_DCMotor& motor2;
    AF_DCMotor& motor3;
    AF_DCMotor& motor4;

    bool moving = false;
    unsigned long moveStartTime = 0; // For movement timeout
    bool strafing = false;           // runStrafe() has run at least once this move
    unsigned long strafeLastMs = 0;  // last runStrafe() pass, for the trim's slew
    float turnTarget = NAN;          // turnTo() target in progress, NAN when none

    // One entry per motor, indexed motor1..motor4 as 0..3.
    Encoders* encoders[WheelRegulator::WHEEL_COUNT];
    long wheelStart[WheelRegulator::WHEEL_COUNT] = {0, 0, 0, 0};
    int trim[WheelRegulator::WHEEL_COUNT] = {0, 0, 0, 0};  // per-wheel PWM offset from the mean
    bool activeDriven[WheelRegulator::WHEEL_COUNT] = {true, true, true, true};  // wheels this motion drives

    // Identifies the movement in progress, so a move abandoned half way
    // through (a microswitch advanced the state machine) cannot leak its start
    // counts into the next one.
    enum MotionId {
      MOTION_NONE = 0,
      MOTION_FORWARD, MOTION_BACKWARD,
      MOTION_LEFT, MOTION_RIGHT,
      MOTION_FORWARD_REGULATED,
      MOTION_FORWARDP_NEAR, MOTION_FORWARDP_FAR,
      MOTION_BACKWARDP_NEAR, MOTION_BACKWARDP_FAR,
      MOTION_FORWARDQ_NEAR, MOTION_FORWARDQ_FAR,
      MOTION_FORWARD_LEFT, MOTION_FORWARD_RIGHT,
      MOTION_BACKWARD_LEFT, MOTION_BACKWARD_RIGHT,
      MOTION_ROTATE_FBBF, MOTION_ROTATE_BFFB
    };
    uint8_t activeMotion = MOTION_NONE;
    long activeTarget = 0;

    // Millimetres to encoder counts for this robot, rounded to the nearest
    // count, from regulator.countsPerMM (set in initHardware(): roughly 4.77
    // counts/mm on LEFT and 7.16 on RIGHT).
    long toCounts(int millimetres) const {
      return static_cast<long>(millimetres * regulator.countsPerMM + 0.5f);
    }

    void setMotors(uint8_t m1, uint8_t m2, uint8_t m3, uint8_t m4) {
      motor1.run(m1);
      motor2.run(m2);
      motor3.run(m3);
      motor4.run(m4);
    }

    // Writes one PWM per motor and records it in wheelPWM.
    void setSpeeds(int p1, int p2, int p3, int p4) {
      wheelPWM[0] = p1; wheelPWM[1] = p2; wheelPWM[2] = p3; wheelPWM[3] = p4;
      motor1.setSpeed(p1);
      motor2.setSpeed(p2);
      motor3.setSpeed(p3);
      motor4.setSpeed(p4);
    }

    // Full-duty enable with both inputs low: the L293D brake. Nothing is
    // driving the wheels, so wheelPWM reads 0.
    void brakeHold() {
      motor1.setSpeed(255);
      motor2.setSpeed(255);
      motor3.setSpeed(255);
      motor4.setSpeed(255);
      for (uint8_t i = 0; i < WheelRegulator::WHEEL_COUNT; i++) wheelPWM[i] = 0;
    }

    // Turns the four requested PWM values into per-wheel TRIMS: how much each
    // wheel differs from the mean of the four. The regulator applies the trims
    // around its own cruisePWM, so the measured per-motor differences in
    // pwmf[]/pwms[] and the +-d wall-hugging offsets are kept exactly while
    // the absolute level comes from the regulator.
    void setNominalSpeeds(int n1, int n2, int n3, int n4) {
      int average = (n1 + n2 + n3 + n4) / 4;
      trim[0] = n1 - average;
      trim[1] = n2 - average;
      trim[2] = n3 - average;
      trim[3] = n4 - average;
    }

    // Starts a movement, or carries on with the one already running.
    // Re-arms if the caller has switched to a different movement or target.
    void armMotion(uint8_t motionId, WheelRegulator::Mode mode, long targetPulses,
                   int n1, int n2, int n3, int n4) {
      if (moving && (motionId != activeMotion || targetPulses != activeTarget)) {
        moving = false;
      }
      if (!moving) {
        activeMotion = motionId;
        activeTarget = targetPulses;
        begin(n1, n2, n3, n4);          // records this motion's per-wheel trims

        // A long backward move finishes with the wall approach.
        WheelRegulator::EndSpec end;
        if (isBackward(motionId) && targetPulses > toCounts(longBackwardMM)) end = backwardEnd;
        startMove(targetPulses, mode, end);  // zeroes all four wheels, arms the profile

        // Translations capture the heading they start on and hold it. A
        // rotation is supposed to change the heading, so it captures nothing;
        // the straight after it captures the new heading.
        bool rotating = (motionId == MOTION_ROTATE_FBBF || motionId == MOTION_ROTATE_BFFB);
        if (!rotating && headingCapture) headingCapture();

        moveStartTime = millis();
      }
    }

    // The motions that drive the robot backwards (rotations are not).
    static bool isBackward(uint8_t motionId) {
      return motionId == MOTION_BACKWARD ||
             motionId == MOTION_BACKWARDP_NEAR || motionId == MOTION_BACKWARDP_FAR ||
             motionId == MOTION_BACKWARD_LEFT  || motionId == MOTION_BACKWARD_RIGHT;
    }

    // Drives the four motors in the requested directions, then re-asserts the
    // regulated PWM for this instant. Called once per loop() while a move runs.
    // `headingOffsetDeg` shifts the heading the PID holds (the wall moves lean
    // by wallHugDeg); 0 holds the heading the move started on.
    void runRegulated(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4,
                      float headingOffsetDeg = 0.0f) {
      setMotors(d1, d2, d3, d4);

      const uint8_t dirs[WheelRegulator::WHEEL_COUNT] = {d1, d2, d3, d4};
      bool driven[WheelRegulator::WHEEL_COUNT];
      long progress[WheelRegulator::WHEEL_COUNT];
      int8_t dirSign[WheelRegulator::WHEEL_COUNT];

      for (uint8_t i = 0; i < WheelRegulator::WHEEL_COUNT; i++) {
        // A released wheel is coasting: it is left out of the mean speed.
        driven[i] = (dirs[i] != RELEASE) && (encoders[i] != 0);
        activeDriven[i] = driven[i];

        progress[i] = wheelTravelCounts(i);

        // Lets the regulator turn the heading differential into a rotation: a
        // wheel commanded backwards has its PWM reduced to push the robot the
        // same way round.
        dirSign[i] = (dirs[i] == FORWARD) ? 1 : ((dirs[i] == BACKWARD) ? -1 : 0);
      }

      // Fresh heading error every tick, shifted by the offset this move holds.
      regulator.headingErrorDeg = headingSource ? headingSource() + headingOffsetDeg : 0.0f;

      regulator.update(driven, trusted, progress, dirSign);

      int pwm[WheelRegulator::WHEEL_COUNT];
      regulator.computePWM(trim, pwm);
      setSpeeds(pwm[0], pwm[1], pwm[2], pwm[3]);

      // A released wheel still has a PWM in its register but is coasting.
      for (uint8_t i = 0; i < WheelRegulator::WHEEL_COUNT; i++) {
        if (dirs[i] == RELEASE) wheelPWM[i] = 0;
      }
    }

    // A strafe pass: the four motors in the requested directions at their
    // calibrated pwmStrafe values, plus the heading trim. The trim follows
    // strafeHeadingP * error, capped at strafeHeadingMax, but may only move
    // strafeHeadingSlew PWM per second, so it ramps rather than jumps. It is
    // applied in the F B B F sense the regulator uses (ROT * dirSign), which
    // lowers a positive error. Nothing else touches the calibrated numbers.
    void runStrafe(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4) {
      setMotors(d1, d2, d3, d4);

      const uint8_t dirs[WheelRegulator::WHEEL_COUNT] = {d1, d2, d3, d4};
      const int     base[WheelRegulator::WHEEL_COUNT] = {pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4};
      static const int8_t ROT[WheelRegulator::WHEEL_COUNT] = {1, -1, -1, 1};
      for (uint8_t i = 0; i < WheelRegulator::WHEEL_COUNT; i++) {
        activeDriven[i] = (dirs[i] != RELEASE) && (encoders[i] != 0);
      }

      // Where the trim wants to be, from the heading error.
      float error  = headingSource ? headingSource() : 0.0f;
      float wanted = strafeHeadingP * error;
      if (wanted >  (float)strafeHeadingMax) wanted =  (float)strafeHeadingMax;
      if (wanted < -(float)strafeHeadingMax) wanted = -(float)strafeHeadingMax;

      // Move toward it no faster than the slew allows. The first pass of a
      // strafe (strafeCorr just zeroed by startMove) only records the time.
      unsigned long now = millis();
      if (!strafing) {
        strafing = true;
      } else {
        float step = strafeHeadingSlew * (float)(now - strafeLastMs) * 0.001f;
        if (wanted > strafeCorr + step)      strafeCorr += step;
        else if (wanted < strafeCorr - step) strafeCorr -= step;
        else                                 strafeCorr = wanted;
      }
      strafeLastMs = now;

      int pwm[WheelRegulator::WHEEL_COUNT];
      for (uint8_t i = 0; i < WheelRegulator::WHEEL_COUNT; i++) {
        int8_t dirSign = (dirs[i] == FORWARD) ? 1 : ((dirs[i] == BACKWARD) ? -1 : 0);
        float u = (float)base[i] + strafeCorr * (float)ROT[i] * (float)dirSign;
        int p = (int)(u + (u >= 0.0f ? 0.5f : -0.5f));
        if (p > 255) p = 255;
        if (p < 0)   p = 0;
        pwm[i] = p;
      }
      setSpeeds(pwm[0], pwm[1], pwm[2], pwm[3]);
    }
};
