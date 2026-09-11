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
// Every distance-counted primitive below is NON-BLOCKING: call it every pass of
// loop() and it returns true once, when the move has finished. It drives the
// motors through WheelRegulator, which ramps the PWM, keeps the four wheels to
// the same travel and holds the heading from the BNO08x (see WheelRegulator.h).
//
// pwmFwd* / pwmStrafe* (pwmf[] / pwms[] in RobotConfig) are used as per-wheel
// TRIMS: the regulator drives every wheel from its own cruisePWM plus this
// wheel's difference from the mean of the four.
//
// Distance and completion are measured on the front encoders (motors 3 and 4):
// a move ends when either front wheel reaches the target count, or after
// moveTimeoutMs, whichever comes first.

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

    // PWM values for forward/backward
    int pwmFwd1, pwmFwd2, pwmFwd3, pwmFwd4;
    // PWM values for left/right/diagonals
    int pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4;

    // Ramp, wheel sync and heading hold. Its parameters are set in
    // initHardware() (e.g. move.regulator.cruisePWM = 232).
    WheelRegulator regulator;

    // enc1..enc4 belong to motor1..motor4. Distance and completion come from
    // the front pair (motors 3 and 4); the rear pair takes part in wheel sync.
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
    // `targetPulses` sizes the ramp; `mode` selects burst / ramp-only / full.
    void startMove(long targetPulses = 0,
                   WheelRegulator::Mode mode = WheelRegulator::Full) {
      startLeft = encoderLeft.getEncoderCount();
      startRight = encoderRight.getEncoderCount();
      for (uint8_t i = 0; i < WheelRegulator::WHEEL_COUNT; i++) {
        wheelStart[i] = encoders[i] ? encoders[i]->getEncoderCount() : 0;
      }
      regulator.begin(targetPulses, mode);
      moving = true;
    }

    // ---- Wall-hugging straights ------------------------------------------
    // forwardp/backwardp/forwardq trim one diagonal pair by +-d so the robot
    // presses against the wall it is running along. They run in Ramp mode:
    // no wheel sync and no heading hold, because the wall does the aligning.
    //
    // forwardp returns 1 when the full distance is reached (and stops), 2 once
    // 14/22 of it is reached (without stopping), 0 otherwise.
    int forwardp(long pulses, bool position) {
      const int d = 9;
      if (position == false){
        armMotion(MOTION_FORWARDP_NEAR, WheelRegulator::Ramp, pulses, pwmFwd1 + d, pwmFwd2, pwmFwd3, pwmFwd4 + d);
      }else{
        armMotion(MOTION_FORWARDP_FAR, WheelRegulator::Ramp, pulses, pwmFwd1 - d, pwmFwd2, pwmFwd3, pwmFwd4 - d);
      }

      runSynchronised(FORWARD, FORWARD, FORWARD, FORWARD);
      if (checkDoneWithTimeout(pulses)){
        return 1;
      } else if (checkDoneWithTimeout(((pulses / 22)*14), true)){
        return 2;
      } else {return 0;};
    }

    // Same 1 / 2 / 0 contract as forwardp, with full regulation.
    int forwardRegulated(long pulses) {
      armMotion(MOTION_FORWARD_REGULATED, WheelRegulator::Full, pulses, pwmFwd1, pwmFwd2, pwmFwd3, pwmFwd4);

      runSynchronised(FORWARD, FORWARD, FORWARD, FORWARD);
      if (checkDoneWithTimeout(pulses)){
        return 1;
      } else if (checkDoneWithTimeout(((pulses / 22)*14), true)){
        return 2;
      } else {return 0;};
    }

    bool backwardp(long pulses, bool position) {
      const int d = 6;
      if (position == false){
        armMotion(MOTION_BACKWARDP_NEAR, WheelRegulator::Ramp, pulses, pwmFwd1 + d, pwmFwd2, pwmFwd3, pwmFwd4 + d);
      }else{
        armMotion(MOTION_BACKWARDP_FAR, WheelRegulator::Ramp, pulses, pwmFwd1 - d, pwmFwd2, pwmFwd3, pwmFwd4 - d);
      }

      runSynchronised(BACKWARD, BACKWARD, BACKWARD, BACKWARD);
      return checkDoneWithTimeout(pulses);
    }

    bool forwardq(long pulses, bool position) {
      const int d = 9;
      if (position == false){
        armMotion(MOTION_FORWARDQ_NEAR, WheelRegulator::Ramp, pulses, pwmFwd1, pwmFwd2 + d, pwmFwd3 + d, pwmFwd4);
      }else{
        armMotion(MOTION_FORWARDQ_FAR, WheelRegulator::Ramp, pulses, pwmFwd1, pwmFwd2 - d, pwmFwd3 - d, pwmFwd4);
      }

      runSynchronised(FORWARD, FORWARD, FORWARD, FORWARD);
      return checkDoneWithTimeout(pulses);
    }

    // ---- Fully regulated translations ------------------------------------
    // Ramp + wheel sync + heading hold.

    bool forward(long pulses) {
      armMotion(MOTION_FORWARD, WheelRegulator::Full, pulses, pwmFwd1, pwmFwd2, pwmFwd3, pwmFwd4);
      runSynchronised(FORWARD, FORWARD, FORWARD, FORWARD);
      return checkDoneWithTimeout(pulses);
    }

    bool backward(long pulses) {
      armMotion(MOTION_BACKWARD, WheelRegulator::Full, pulses, pwmFwd1, pwmFwd2, pwmFwd3, pwmFwd4);
      runSynchronised(BACKWARD, BACKWARD, BACKWARD, BACKWARD);
      return checkDoneWithTimeout(pulses);
    }

    bool left(long pulses) {
      armMotion(MOTION_LEFT, WheelRegulator::Full, pulses, pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4);
      runSynchronised(BACKWARD, FORWARD, BACKWARD, FORWARD);
      return checkDoneWithTimeout(pulses);
    }

    bool right(long pulses) {
      armMotion(MOTION_RIGHT, WheelRegulator::Full, pulses, pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4);
      runSynchronised(FORWARD, BACKWARD, FORWARD, BACKWARD);
      return checkDoneWithTimeout(pulses);
    }

    // ---- Diagonals -------------------------------------------------------
    // Two wheels drive, two are released. Ramp mode: a released wheel has no
    // travel to synchronise against.

    bool forwardLeft(long pulses) {
      armMotion(MOTION_FORWARD_LEFT, WheelRegulator::Ramp, pulses, pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4);
      runSynchronised(RELEASE, FORWARD, RELEASE, FORWARD);
      return checkDoneWithTimeout(pulses);
    }

    bool forwardRight(long pulses) {
      armMotion(MOTION_FORWARD_RIGHT, WheelRegulator::Ramp, pulses, pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4);
      runSynchronised(FORWARD, RELEASE, FORWARD, RELEASE);
      return checkDoneWithTimeout(pulses);
    }

    bool backwardLeft(long pulses) {
      armMotion(MOTION_BACKWARD_LEFT, WheelRegulator::Ramp, pulses, pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4);
      runSynchronised(BACKWARD, RELEASE, BACKWARD, RELEASE);
      return checkDoneWithTimeout(pulses);
    }

    bool backwardRight(long pulses) {
      armMotion(MOTION_BACKWARD_RIGHT, WheelRegulator::Ramp, pulses, pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4);
      runSynchronised(RELEASE, BACKWARD, RELEASE, BACKWARD);
      return checkDoneWithTimeout(pulses);
    }

    // ---- Rotation --------------------------------------------------------
    // Encoder-counted turn: ramp + wheel sync, heading hold OFF (a turn is
    // supposed to change the heading).
    //   side == true   pattern F B B F  (the rotateCCW() pattern)
    //   side == false  pattern B F F B  (the rotateCW()  pattern)
    bool rotate(long pulses, bool side) {
      if (side == true){
        armMotion(MOTION_ROTATE_FBBF, WheelRegulator::Full, pulses, pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4);
        runSynchronised(FORWARD, BACKWARD, BACKWARD, FORWARD);
      } else {
        armMotion(MOTION_ROTATE_BFFB, WheelRegulator::Full, pulses, pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4);
        runSynchronised(BACKWARD, FORWARD, FORWARD, BACKWARD);
      }

      return checkDoneWithTimeout(pulses);
    }

    // Open-loop rotation at fixed PWM, used by routines 7/8 while they turn
    // on the heading sensor. The caller decides when to stop.
    // The heading reading INCREASES under rotateCW() and DECREASES under
    // rotateCCW() - see Heading.h.
    void rotateCW(int pwm, int pwm2, int pwm3, int pwm4) {
      setMotors(BACKWARD, FORWARD, FORWARD, BACKWARD);
      motor1.setSpeed(pwm);
      motor2.setSpeed(pwm2);
      motor3.setSpeed(pwm3);
      motor4.setSpeed(pwm4);
      moving = false;
    }
    void rotateCCW(int pwm, int pwm2, int pwm3, int pwm4) {
      setMotors(FORWARD, BACKWARD, BACKWARD, FORWARD);
      motor1.setSpeed(pwm);
      motor2.setSpeed(pwm2);
      motor3.setSpeed(pwm3);
      motor4.setSpeed(pwm4);
      moving = false;
    }

    // ---- Stopping --------------------------------------------------------

    // Releases all four motors (coast).
    void stop() {
      setMotors(RELEASE, RELEASE, RELEASE, RELEASE);
      moving = false;
    }

    // Releases the motors and returns true once durationMs has passed.
    // One shared timer: only one stopForMillis() can be in progress at a time.
    bool stopForMillis(unsigned long durationMs) {
      static unsigned long startTime = 0;
      static bool stopping = false;

      if (!stopping) {
        setMotors(RELEASE, RELEASE, RELEASE, RELEASE);
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

    // ---- Units -----------------------------------------------------------

    long mmToPulses(float mm, float wheelDiameterMM, int pulsesPerRevolution) {
      float circumference = 3.14159265f * wheelDiameterMM;
      float pulsesPerMM = pulsesPerRevolution / circumference;
      return static_cast<long>(mm * pulsesPerMM + 0.5f); // rounded
    }

    // ---- Completion ------------------------------------------------------
    // Either front encoder reaching `pulses` ends the move. With far == false
    // the motors are released; with far == true they keep running (used for
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

    void setMotors(uint8_t m1, uint8_t m2, uint8_t m3, uint8_t m4) {
      motor1.run(m1);
      motor2.run(m2);
      motor3.run(m3);
      motor4.run(m4);
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
        startMove(targetPulses, mode);  // zeroes all four wheels, arms the ramp

        // Translations capture the heading they start on and hold it. A
        // rotation is supposed to change the heading, so it holds nothing;
        // the straight after it captures the new heading.
        bool rotating = (motionId == MOTION_ROTATE_FBBF || motionId == MOTION_ROTATE_BFFB);
        if (!rotating && headingCapture) headingCapture();
        regulator.headingHoldEnabled = !rotating;

        moveStartTime = millis();
      }
    }

    // Drives the four motors in the requested directions, then re-asserts the
    // regulated PWM for this instant. Called once per loop() while a move runs.
    void runSynchronised(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4) {
      setMotors(d1, d2, d3, d4);

      const uint8_t dirs[WheelRegulator::WHEEL_COUNT] = {d1, d2, d3, d4};
      bool participating[WheelRegulator::WHEEL_COUNT];
      long progress[WheelRegulator::WHEEL_COUNT];
      int8_t dirSign[WheelRegulator::WHEEL_COUNT];

      for (uint8_t i = 0; i < WheelRegulator::WHEEL_COUNT; i++) {
        // A released wheel is coasting and takes no part in the synchronisation.
        participating[i] = (dirs[i] != RELEASE) && (encoders[i] != 0);

        long delta = encoders[i] ? (encoders[i]->getEncoderCount() - wheelStart[i]) : 0;
        progress[i] = delta < 0 ? -delta : delta;

        // Lets the regulator turn a heading correction into a rotation: a
        // wheel commanded backwards has its PWM reduced to push the robot the
        // same way round.
        dirSign[i] = (dirs[i] == FORWARD) ? 1 : ((dirs[i] == BACKWARD) ? -1 : 0);
      }

      // Fresh heading error every tick.
      regulator.headingErrorDeg = headingSource ? headingSource() : 0.0f;

      regulator.update(participating, progress, dirSign);
      applyRegulatedSpeeds();
    }

    void applyRegulatedSpeeds() {
      motor1.setSpeed(regulator.pwmFor(0, trim[0]));
      motor2.setSpeed(regulator.pwmFor(1, trim[1]));
      motor3.setSpeed(regulator.pwmFor(2, trim[2]));
      motor4.setSpeed(regulator.pwmFor(3, trim[3]));
    }
};
