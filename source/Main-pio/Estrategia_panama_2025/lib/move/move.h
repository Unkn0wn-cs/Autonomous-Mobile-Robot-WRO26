//this code was designed as part of the 2025 world robotics competition by team Outer heaven competing in robo sports
//this code is meant to be used with the adafruit motor shield v1 and quadrature encoders of 4 omnidirectional 45d degree wheels
// connected in sequence as top view

//Front of the robot ⬆️
// motor3  motor4
// motor2  motor1

// SYNCHRONISED MOVEMENT (2026)
// ----------------------------
// Every movement below now runs through WheelRegulator, which does two things:
//   1. shapes the PWM with an acceleration and deceleration ramp, so the robot
//      eases into and out of a move instead of slamming from 0 to full and back;
//   2. holds the four wheels to the same travelled distance by trimming whichever
//      wheels have run ahead of the slowest one.
//
// The regulator only ever trims PWM DOWNWARD from the nominal values in pwmFwd*
// and pwmStrafe*. Those nominals already sit at 243-245 out of 255, so there is
// no headroom to speed a lagging wheel up; the others are slowed to meet it.
//
// Distance and completion are still measured on the front encoders exactly as
// they always were, so every distance tuned into the routines stays valid.

#pragma once
#include <AFMotor.h>
#include <QuadratureEncoder.h>
#include "WheelRegulator.h"


class Move {
  public:
    unsigned long moveTimeoutMs = 4000;

    // PWM values for forward/backward
    int pwmFwd1, pwmFwd2, pwmFwd3, pwmFwd4;
    // PWM values for left/right/diagonals
    int pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4;

    // Keeps the four wheels turning together and shapes the accel/decel ramps.
    // Exposed so the sketch can tune it live: move.regulator.syncGain = 5;
    WheelRegulator regulator;

    // Two-encoder build (front wheels only, motors 3 and 4).
    // Movements still get their acceleration and deceleration ramps, and the two
    // front wheels are still held together, but motors 1 and 2 have no feedback
    // and therefore cannot be synchronised.
    Move(AF_DCMotor& m1, AF_DCMotor& m2, AF_DCMotor& m3, AF_DCMotor& m4,
         Encoders& encLeft, Encoders& encRight,
         int fwdPWM1 = 255, int fwdPWM2 = 255, int fwdPWM3 = 255, int fwdPWM4 = 255,
         int strafePWM1 = 255, int strafePWM2 = 255, int strafePWM3 = 255, int strafePWM4 = 255)
      : motor1(m1), motor2(m2), motor3(m3), motor4(m4),
        encoderLeft(encLeft), encoderRight(encRight),
        pwmFwd1(fwdPWM1), pwmFwd2(fwdPWM2), pwmFwd3(fwdPWM3), pwmFwd4(fwdPWM4),
        pwmStrafe1(strafePWM1), pwmStrafe2(strafePWM2), pwmStrafe3(strafePWM3), pwmStrafe4(strafePWM4) {
      encoders[0] = 0;          // motor1 has no encoder in this build
      encoders[1] = 0;          // motor2 has no encoder in this build
      encoders[2] = &encLeft;   // motor3
      encoders[3] = &encRight;  // motor4
      synchronised = false;
    }

    // Four-encoder build. enc1..enc4 belong to motor1..motor4 respectively.
    // Distance and completion are still measured on the front pair (motors 3 and
    // 4) exactly as before, so every distance already tuned into the routines
    // stays valid. The extra two encoders are used purely to keep the wheels
    // turning at the same rate.
    Move(AF_DCMotor& m1, AF_DCMotor& m2, AF_DCMotor& m3, AF_DCMotor& m4,
         Encoders& enc1, Encoders& enc2, Encoders& enc3, Encoders& enc4,
         int fwdPWM1 = 255, int fwdPWM2 = 255, int fwdPWM3 = 255, int fwdPWM4 = 255,
         int strafePWM1 = 255, int strafePWM2 = 255, int strafePWM3 = 255, int strafePWM4 = 255)
      : motor1(m1), motor2(m2), motor3(m3), motor4(m4),
        encoderLeft(enc3), encoderRight(enc4),
        pwmFwd1(fwdPWM1), pwmFwd2(fwdPWM2), pwmFwd3(fwdPWM3), pwmFwd4(fwdPWM4),
        pwmStrafe1(strafePWM1), pwmStrafe2(strafePWM2), pwmStrafe3(strafePWM3), pwmStrafe4(strafePWM4) {
      encoders[0] = &enc1;
      encoders[1] = &enc2;
      encoders[2] = &enc3;
      encoders[3] = &enc4;
      synchronised = true;
    }

    void begin(int pwm = 255, int pwm2 = 255, int pwm3 = 255, int pwm4 = 255) {
      // Set initial speeds if needed
      setNominalSpeeds(pwm, pwm2, pwm3, pwm4);
      motor1.setSpeed(pwm);
      motor2.setSpeed(pwm2);
      motor3.setSpeed(pwm3);
      motor4.setSpeed(pwm4);
    }

    void moveBeginForward(){
      setNominalSpeeds(pwmFwd1, pwmFwd2, pwmFwd3, pwmFwd4);
      motor1.setSpeed(pwmFwd1);
      motor2.setSpeed(pwmFwd2);
      motor3.setSpeed(pwmFwd3);
      motor4.setSpeed(pwmFwd4);
    }

    void moveBeginStrafe(){
      setNominalSpeeds(pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4);
      motor1.setSpeed(pwmStrafe1);
      motor2.setSpeed(pwmStrafe2);
      motor3.setSpeed(pwmStrafe3);
      motor4.setSpeed(pwmStrafe4);
    }

    // Call before starting a new movement.
    // `targetPulses` arms the acceleration profile; pass 0 for an open-ended move.
    void startMove(long targetPulses = 0) {
      startLeft = encoderLeft.getEncoderCount();
      startRight = encoderRight.getEncoderCount();
      for (uint8_t i = 0; i < WheelRegulator::WHEEL_COUNT; i++) {
        wheelStart[i] = encoders[i] ? encoders[i]->getEncoderCount() : 0;
      }
      regulator.begin(targetPulses);
      moving = true;
    }

    //Movement without encoder regulation
    void simpleForward() {
      moveBeginForward();
      motor1.run(FORWARD);
      motor2.run(FORWARD);
      motor3.run(FORWARD);
      motor4.run(FORWARD);
    }

    void simpleBackward() {
      moveBeginForward();
      motor1.run(BACKWARD);
      motor2.run(BACKWARD);
      motor3.run(BACKWARD);
      motor4.run(BACKWARD);
    }

    void simpleLeft() {
      moveBeginStrafe();
      motor1.run(BACKWARD);
      motor2.run(FORWARD);
      motor3.run(BACKWARD);
      motor4.run(FORWARD);
    }

    void simpleRight() {
      moveBeginStrafe();
      motor1.run(FORWARD);
      motor2.run(BACKWARD);
      motor3.run(FORWARD);
      motor4.run(BACKWARD);
    }

    int forwardp(long pulses, bool position) {
      const int d = 9;
      if (position == false){
        armMotion(MOTION_FORWARDP_NEAR, pulses, pwmFwd1 + d, pwmFwd2, pwmFwd3, pwmFwd4 + d);
      }else{
        armMotion(MOTION_FORWARDP_FAR, pulses, pwmFwd1 - d, pwmFwd2, pwmFwd3, pwmFwd4 - d);
      }

      runSynchronised(FORWARD, FORWARD, FORWARD, FORWARD);
      if (checkDoneWithTimeout(pulses)){
        return 1;
      } else if (checkDoneWithTimeout(((pulses / 22)*14), true)){
        return 2;
      } else {return 0;};
    }

    int forwardRegulated(long pulses) {
      armMotion(MOTION_FORWARD_REGULATED, pulses, pwmFwd1, pwmFwd2, pwmFwd3, pwmFwd4);

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
        armMotion(MOTION_BACKWARDP_NEAR, pulses, pwmFwd1 + d, pwmFwd2, pwmFwd3, pwmFwd4 + d);
      }else{
        armMotion(MOTION_BACKWARDP_FAR, pulses, pwmFwd1 - d, pwmFwd2, pwmFwd3, pwmFwd4 - d);
      }

      runSynchronised(BACKWARD, BACKWARD, BACKWARD, BACKWARD);
      return checkDoneWithTimeout(pulses);
    }

    bool forwardq(long pulses, bool position) {
      const int d = 9;
      if (position == false){
        armMotion(MOTION_FORWARDQ_NEAR, pulses, pwmFwd1, pwmFwd2 + d, pwmFwd3 + d, pwmFwd4);
      }else{
        armMotion(MOTION_FORWARDQ_FAR, pulses, pwmFwd1, pwmFwd2 - d, pwmFwd3 - d, pwmFwd4);
      }

      runSynchronised(FORWARD, FORWARD, FORWARD, FORWARD);
      return checkDoneWithTimeout(pulses);
    }

    // Forward
    bool forward(long pulses) {
      armMotion(MOTION_FORWARD, pulses, pwmFwd1, pwmFwd2, pwmFwd3, pwmFwd4);
      runSynchronised(FORWARD, FORWARD, FORWARD, FORWARD);
      return checkDoneWithTimeout(pulses);
    }

    //forward accelerated
    //Superseded by the regulator's built in ramp; kept for backwards compatibility.
    bool forwarda(long pulses) {
      static int d = 80;
      static int a = 80;
      if (!moving) {
        begin(pwmFwd1 - d, pwmFwd2 - d, pwmFwd3 - d, pwmFwd4 - d);
        startMove();
        moveStartTime = millis();
      }
      if (d > 0){
        begin(pwmFwd1 - d, pwmFwd2 - d, pwmFwd3 - d, pwmFwd4 - d);
        d = d - 5;
      }

      static bool decel = false;
      if (checkDoneWithTimeout(pulses - (pulses)/10, true)){
        decel = true;
      }

      if (decel || d < a){
        begin(pwmFwd1 - d, pwmFwd2 - d, pwmFwd3 - d, pwmFwd4 - d);
        d = d + 10;
      }


      setMotors(FORWARD, FORWARD, FORWARD, FORWARD);
      return checkDoneWithTimeout(pulses);
    }

    // Backward
    bool backward(long pulses) {
      armMotion(MOTION_BACKWARD, pulses, pwmFwd1, pwmFwd2, pwmFwd3, pwmFwd4);
      runSynchronised(BACKWARD, BACKWARD, BACKWARD, BACKWARD);
      return checkDoneWithTimeout(pulses);
    }

    // Left (strafe)
    bool left(long pulses) {
      armMotion(MOTION_LEFT, pulses, pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4);
      runSynchronised(BACKWARD, FORWARD, BACKWARD, FORWARD);
      return checkDoneWithTimeout(pulses);
    }

    // Right (strafe)
    bool right(long pulses) {
      armMotion(MOTION_RIGHT, pulses, pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4);
      runSynchronised(FORWARD, BACKWARD, FORWARD, BACKWARD);
      return checkDoneWithTimeout(pulses);
    }

    // Diagonal: Forward-Left
    bool forwardLeft(long pulses) {
      armMotion(MOTION_FORWARD_LEFT, pulses, pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4);
      runSynchronised(RELEASE, FORWARD, RELEASE, FORWARD);
      return checkDoneWithTimeout(pulses);
    }

    // Diagonal: Forward-Right
    bool forwardRight(long pulses) {
      armMotion(MOTION_FORWARD_RIGHT, pulses, pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4);
      runSynchronised(FORWARD, RELEASE, FORWARD, RELEASE);
      return checkDoneWithTimeout(pulses);
    }

    // Diagonal: Backward-Left
    bool backwardLeft(long pulses) {
      armMotion(MOTION_BACKWARD_LEFT, pulses, pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4);
      runSynchronised(BACKWARD, RELEASE, BACKWARD, RELEASE);
      return checkDone(pulses);
    }

    // Diagonal: Backward-Right
    bool backwardRight(long pulses) {
      armMotion(MOTION_BACKWARD_RIGHT, pulses, pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4);
      runSynchronised(RELEASE, BACKWARD, RELEASE, BACKWARD);
      return checkDone(pulses);
    }


    bool rotate(long pulses, bool side) {
      if (side == true){
        armMotion(MOTION_ROTATE_CW, pulses, pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4);
        runSynchronised(FORWARD, BACKWARD, BACKWARD, FORWARD);
      } else {
        armMotion(MOTION_ROTATE_CCW, pulses, pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4);
        runSynchronised(BACKWARD, FORWARD, FORWARD, BACKWARD);
      }

      return checkDone(pulses);
    }



    // Rotation (not encoder regulated)
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

    // Stop all motors
    void stop() {
      setMotors(RELEASE, RELEASE, RELEASE, RELEASE);
      moving = false;
    }

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

    long mmToPulses(float mm, float wheelDiameterMM, int pulsesPerRevolution) {
      // Calculate wheel circumference in mm
      float circumference = 3.14159265f * wheelDiameterMM;
      // Pulses per mm
      float pulsesPerMM = pulsesPerRevolution / circumference;
      // Total pulses for the given mm
      return static_cast<long>(mm * pulsesPerMM + 0.5f); // rounded
    }

    // Calculates the distance in mm for a given number of pulses
    float pulsesToMM(long pulses, float wheelDiameterMM, int pulsesPerRevolution) {
        // Calculate wheel circumference in mm
        float circumference = 3.14159265f * wheelDiameterMM;
        // mm per pulse
        float mmPerPulse = circumference / pulsesPerRevolution;
        // Total mm for the given pulses
        return pulses * mmPerPulse;
    }

    // Accelerate all motors from initialPWM to targetPWM in 'steps' increments
    // Call this repeatedly in your main loop

bool accelerateToPWM(int initialPWM, int targetPWM, int steps = 20, unsigned long stepDelay = 20, bool reverse = false) {
  static int currentStep = 0;
  static unsigned long lastStepTime = 0;
  static int pwm = initialPWM;
  if (currentStep == 0) {
    pwm = initialPWM;
    lastStepTime = millis();
    setAllSpeeds(pwm);
  }
  if ((!reverse && pwm >= targetPWM) || (reverse && pwm <= targetPWM)) {
    setAllSpeeds(targetPWM);
    currentStep = 0;
    return true;
  }
  if (millis() - lastStepTime >= stepDelay) {
    if (!reverse)
      pwm = initialPWM + ((targetPWM - initialPWM) * currentStep) / steps;
    else
      pwm = initialPWM - ((initialPWM - targetPWM) * currentStep) / steps;
    setAllSpeeds(pwm);
    currentStep++;
    lastStepTime = millis();
  }
  if (currentStep > steps) {
    setAllSpeeds(targetPWM);
    currentStep = 0;
    return true;
  }
  return false;
}


  // New: checkDone with timeout
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

    bool checkDone(long pulses) {
    long deltaLeft = abs(encoderLeft.getEncoderCount() - startLeft);
    long deltaRight = abs(encoderRight.getEncoderCount() - startRight);
    if (deltaLeft >= pulses || deltaRight >= pulses) {
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
    int nominal[WheelRegulator::WHEEL_COUNT] = {255, 255, 255, 255};
    bool synchronised = false; // true when all four wheels report back

    // Identifies the movement currently in progress, so that a move which was
    // abandoned half way through cannot leak its start counts into the next one.
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
      MOTION_ROTATE_CW, MOTION_ROTATE_CCW
    };
    uint8_t activeMotion = MOTION_NONE;
    long activeTarget = 0;

    void setMotors(uint8_t m1, uint8_t m2, uint8_t m3, uint8_t m4) {
      motor1.run(m1);
      motor2.run(m2);
      motor3.run(m3);
      motor4.run(m4);
    }

    // Remembers the PWM each wheel is nominally asking for. The regulator only
    // ever trims downward from these numbers, never above them.
    void setNominalSpeeds(int n1, int n2, int n3, int n4) {
      nominal[0] = n1;
      nominal[1] = n2;
      nominal[2] = n3;
      nominal[3] = n4;
    }

    // Starts a movement, or carries on with the one already running.
    // Re-arms if the caller has switched to a different movement, which is what
    // happens when a microswitch pushes the state machine on before the previous
    // move finished; without this the next move would measure from stale counts.
    void armMotion(uint8_t motionId, long targetPulses,
                   int n1, int n2, int n3, int n4) {
      if (moving && (motionId != activeMotion || targetPulses != activeTarget)) {
        moving = false;
      }
      if (!moving) {
        activeMotion = motionId;
        activeTarget = targetPulses;
        begin(n1, n2, n3, n4);      // records this motion's nominal PWM
        startMove(targetPulses);    // zeroes all four wheels, arms the profile
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

      for (uint8_t i = 0; i < WheelRegulator::WHEEL_COUNT; i++) {
        // A released wheel is coasting, and a wheel with no encoder cannot be
        // compared against anything. Neither takes part in the synchronisation.
        participating[i] = (dirs[i] != RELEASE) && (encoders[i] != 0);

        long delta = encoders[i] ? (encoders[i]->getEncoderCount() - wheelStart[i]) : 0;
        progress[i] = delta < 0 ? -delta : delta;
      }

      regulator.update(participating, progress);
      applyRegulatedSpeeds();
    }

    void applyRegulatedSpeeds() {
      motor1.setSpeed(regulator.pwmFor(0, nominal[0]));
      motor2.setSpeed(regulator.pwmFor(1, nominal[1]));
      motor3.setSpeed(regulator.pwmFor(2, nominal[2]));
      motor4.setSpeed(regulator.pwmFor(3, nominal[3]));
    }

    // Helper to set all motor speeds
    void setAllSpeeds(int pwm) {
      setNominalSpeeds(pwm, pwm, pwm, pwm);
      motor1.setSpeed(pwm);
      motor2.setSpeed(pwm);
      motor3.setSpeed(pwm);
      motor4.setSpeed(pwm);
    }

    void setIndividualSpeeds(int pwm1, int pwm2, int pwm3, int pwm4) {
      motor1.setSpeed(pwm1);
      motor2.setSpeed(pwm2);
      motor3.setSpeed(pwm3);
      motor4.setSpeed(pwm4);
    }
};
