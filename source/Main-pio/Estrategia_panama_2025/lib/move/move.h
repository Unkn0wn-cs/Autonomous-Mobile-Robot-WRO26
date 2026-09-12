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
// the move has finished. It drives the motors through WheelRegulator: the
// encoders shape the speed over the move, the BNO08x heading PID keeps it
// straight (see WheelRegulator.h).
//
// pwmFwd* / pwmStrafe* (pwmf[] / pwms[] in Hardware.h) are used as per-wheel
// TRIMS: the regulator drives every wheel from a common PWM plus this wheel's
// difference from the mean of the four.
//
// Distance and completion are measured on the front encoders (motors 3 and 4):
// a move ends when either front wheel reaches the target count, or after
// moveTimeoutMs, whichever comes first. The regulator has already slowed the
// robot to a creep over the last part of the move, and the motors are braked
// the moment the count is reached.
//
// This library knows nothing about which robot it is on. Everything robot
// specific - the PWM band, the heading source, the encoder counts per
// millimetre and which way inner()/outer() strafe - is handed to it by
// initHardware() (src/Hardware.cpp).

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

    // PWM values for forward/backward
    int pwmFwd1, pwmFwd2, pwmFwd3, pwmFwd4;
    // PWM values for left/right/diagonals
    int pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4;

    // Speed profile and heading hold. Its parameters are set in
    // initHardware() (e.g. move.regulator.cruisePWM = 232).
    WheelRegulator regulator;

    // PWM currently driving motor1..motor4; 0 for a wheel that is released.
    // Updated on every regulated tick and by rotateCW()/rotateCCW(), cleared by
    // stop() and stopForMillis(). Read only by the telemetry line in main.cpp;
    // nothing in here reads it back.
    int wheelPWM[WheelRegulator::WHEEL_COUNT] = {0, 0, 0, 0};

    // enc1..enc4 belong to motor1..motor4. Distance and completion come from
    // the front pair (motors 3 and 4); all four feed the mean speed.
    Move(AF_DCMotor& m1, AF_DCMotor& m2, AF_DCMotor& m3, AF_DCMotor& m4,
         Encoders& enc1, Encoders& enc2, Encoders& enc3, Encoders& enc4,
         int fwdPWM1 = 255, int fwdPWM2 = 255, int fwdPWM3 = 255, int fwdPWM4 = 255,
         int strafePWM1 = 255, int strafePWM2 = 255, int strafePWM3 = 255, int strafePWM4 = 255)
      : pwmFwd1(fwdPWM1), pwmFwd2(fwdPWM2), pwmFwd3(fwdPWM3), pwmFwd4(fwdPWM4),
        pwmStrafe1(strafePWM1), pwmStrafe2(strafePWM2), pwmStrafe3(strafePWM3), pwmStrafe4(strafePWM4),
        motor1(m1), motor2(m2), motor3(m3), motor4(m4),
        encoderLeft(enc3), encoderRight(enc4) {
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
    // pointers keep this library independent of the sensor; with neither set,
    // every movement runs without heading hold.
    float (*headingSource)()  = 0;   // current error from the held heading, degrees
    void  (*headingCapture)() = 0;   // "hold whatever heading we are on now"

    void setHeadingHooks(float (*source)(), void (*capture)()) {
      headingSource  = source;
      headingCapture = capture;
    }

    // Zeroes the start counts of all four wheels and arms the regulator.
    // `targetPulses` sizes the profile; `mode` selects Burst / Profile / Hold;
    // `end` shapes the finish (default: the regulator's normal rules).
    void startMove(long targetPulses = 0,
                   WheelRegulator::Mode mode = WheelRegulator::Hold,
                   WheelRegulator::EndSpec end = WheelRegulator::EndSpec()) {
      startLeft = encoderLeft.getEncoderCount();
      startRight = encoderRight.getEncoderCount();
      for (uint8_t i = 0; i < WheelRegulator::WHEEL_COUNT; i++) {
        wheelStart[i] = encoders[i] ? encoders[i]->getEncoderCount() : 0;
      }
      regulator.begin(targetPulses, mode, end);
      moving = true;
    }

    // ---- Wall-hugging straights ------------------------------------------
    // forwardp/backwardp/forwardq trim one diagonal pair by +-d so the robot
    // presses against the wall it is running along. They run in Profile mode:
    // speed profile only, no heading hold, because the wall does the aligning.
    //
    // forwardp returns 1 when the full distance is reached (and stops), 2 once
    // 14/22 of it is reached (without stopping), 0 otherwise.
    int forwardp(int millimetres, bool position) {
      long pulses = toCounts(millimetres);
      const int d = 9;
      if (position == false){
        armMotion(MOTION_FORWARDP_NEAR, WheelRegulator::Profile, pulses, pwmFwd1 + d, pwmFwd2, pwmFwd3, pwmFwd4 + d);
      }else{
        armMotion(MOTION_FORWARDP_FAR, WheelRegulator::Profile, pulses, pwmFwd1 - d, pwmFwd2, pwmFwd3, pwmFwd4 - d);
      }

      runRegulated(FORWARD, FORWARD, FORWARD, FORWARD);
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
      const int d = 6;
      if (position == false){
        armMotion(MOTION_BACKWARDP_NEAR, WheelRegulator::Profile, pulses, pwmFwd1 + d, pwmFwd2, pwmFwd3, pwmFwd4 + d);
      }else{
        armMotion(MOTION_BACKWARDP_FAR, WheelRegulator::Profile, pulses, pwmFwd1 - d, pwmFwd2, pwmFwd3, pwmFwd4 - d);
      }

      runRegulated(BACKWARD, BACKWARD, BACKWARD, BACKWARD);
      return checkDoneWithTimeout(pulses);
    }

    bool forwardq(int millimetres, bool position) {
      long pulses = toCounts(millimetres);
      const int d = 9;
      if (position == false){
        armMotion(MOTION_FORWARDQ_NEAR, WheelRegulator::Profile, pulses, pwmFwd1, pwmFwd2 + d, pwmFwd3 + d, pwmFwd4);
      }else{
        armMotion(MOTION_FORWARDQ_FAR, WheelRegulator::Profile, pulses, pwmFwd1, pwmFwd2 - d, pwmFwd3 - d, pwmFwd4);
      }

      runRegulated(FORWARD, FORWARD, FORWARD, FORWARD);
      return checkDoneWithTimeout(pulses);
    }

    // ---- Free translations -----------------------------------------------
    // Speed profile + heading hold.

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

    bool left(int millimetres) {
      long pulses = toCounts(millimetres);
      armMotion(MOTION_LEFT, WheelRegulator::Hold, pulses, pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4);
      runRegulated(BACKWARD, FORWARD, BACKWARD, FORWARD);
      return checkDoneWithTimeout(pulses);
    }

    bool right(int millimetres) {
      long pulses = toCounts(millimetres);
      armMotion(MOTION_RIGHT, WheelRegulator::Hold, pulses, pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4);
      runRegulated(FORWARD, BACKWARD, FORWARD, BACKWARD);
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
    // Two wheels drive, two are released. Profile mode, no heading hold.

    bool forwardLeft(int millimetres) {
      long pulses = toCounts(millimetres);
      armMotion(MOTION_FORWARD_LEFT, WheelRegulator::Profile, pulses, pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4);
      runRegulated(RELEASE, FORWARD, RELEASE, FORWARD);
      return checkDoneWithTimeout(pulses);
    }

    bool forwardRight(int millimetres) {
      long pulses = toCounts(millimetres);
      armMotion(MOTION_FORWARD_RIGHT, WheelRegulator::Profile, pulses, pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4);
      runRegulated(FORWARD, RELEASE, FORWARD, RELEASE);
      return checkDoneWithTimeout(pulses);
    }

    bool backwardLeft(int millimetres) {
      long pulses = toCounts(millimetres);
      armMotion(MOTION_BACKWARD_LEFT, WheelRegulator::Profile, pulses, pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4);
      runRegulated(BACKWARD, RELEASE, BACKWARD, RELEASE);
      return checkDoneWithTimeout(pulses);
    }

    bool backwardRight(int millimetres) {
      long pulses = toCounts(millimetres);
      armMotion(MOTION_BACKWARD_RIGHT, WheelRegulator::Profile, pulses, pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4);
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

    // Front-encoder travel since the current or last move began, in counts:
    // the larger of the two front wheels, the same measure completion uses.
    // After a move has ended this is target + overshoot.
    long frontTravelCounts() {
      long deltaLeft = abs(encoderLeft.getEncoderCount() - startLeft);
      long deltaRight = abs(encoderRight.getEncoderCount() - startRight);
      return deltaLeft > deltaRight ? deltaLeft : deltaRight;
    }

    // ---- Completion ------------------------------------------------------
    // Either front encoder reaching `pulses` ends the move. With far == false
    // the motors are braked; with far == true they keep running (used for
    // the "14/22 of the way" signal). moveTimeoutMs ends any move regardless.
    bool checkDoneWithTimeout(long pulses, bool far = false) {
      long deltaLeft = abs(encoderLeft.getEncoderCount() - startLeft);
      long deltaRight = abs(encoderRight.getEncoderCount() - startRight);
      if (deltaLeft >= pulses || deltaRight >= pulses) {
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

  private:
    AF_DCMotor& motor1;
    AF_DCMotor& motor2;
    AF_DCMotor& motor3;
    AF_DCMotor& motor4;
    Encoders& encoderLeft;  // Motor 3
    Encoders& encoderRight; // Motor 4

    long startLeft = 0;
    long startRight = 0;
    bool moving = false;
    unsigned long moveStartTime = 0; // For movement timeout

    // One entry per motor, indexed motor1..motor4 as 0..3.
    Encoders* encoders[WheelRegulator::WHEEL_COUNT];
    long wheelStart[WheelRegulator::WHEEL_COUNT] = {0, 0, 0, 0};
    int trim[WheelRegulator::WHEEL_COUNT] = {0, 0, 0, 0};  // per-wheel PWM offset from the mean

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
    void runRegulated(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4) {
      setMotors(d1, d2, d3, d4);

      const uint8_t dirs[WheelRegulator::WHEEL_COUNT] = {d1, d2, d3, d4};
      bool driven[WheelRegulator::WHEEL_COUNT];
      long progress[WheelRegulator::WHEEL_COUNT];
      int8_t dirSign[WheelRegulator::WHEEL_COUNT];

      for (uint8_t i = 0; i < WheelRegulator::WHEEL_COUNT; i++) {
        // A released wheel is coasting: it is left out of the mean speed.
        driven[i] = (dirs[i] != RELEASE) && (encoders[i] != 0);

        long delta = encoders[i] ? (encoders[i]->getEncoderCount() - wheelStart[i]) : 0;
        progress[i] = delta < 0 ? -delta : delta;

        // Lets the regulator turn the heading differential into a rotation: a
        // wheel commanded backwards has its PWM reduced to push the robot the
        // same way round.
        dirSign[i] = (dirs[i] == FORWARD) ? 1 : ((dirs[i] == BACKWARD) ? -1 : 0);
      }

      // Fresh heading error every tick.
      regulator.headingErrorDeg = headingSource ? headingSource() : 0.0f;

      regulator.update(driven, progress, dirSign);

      int pwm[WheelRegulator::WHEEL_COUNT];
      regulator.computePWM(trim, pwm);
      setSpeeds(pwm[0], pwm[1], pwm[2], pwm[3]);

      // A released wheel still has a PWM in its register but is coasting.
      for (uint8_t i = 0; i < WheelRegulator::WHEEL_COUNT; i++) {
        if (dirs[i] == RELEASE) wheelPWM[i] = 0;
      }
    }
};
