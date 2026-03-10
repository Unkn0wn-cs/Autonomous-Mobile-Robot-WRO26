//this code was designed as part of the 2025 world robotics competition by team Outer heaven competing in robo sports
//this code is meant to be used with the adafruit motor shield v1 and quadrature encoders of 4 omnidirectional 45d degree wheels
// connected in sequence as top view

//Front of the robot ⬆️
// motor3  motor4
// motor2  motor1 

#pragma once
#include <AFMotor.h>
#include <QuadratureEncoder.h>


class Move {
  public:
    unsigned long moveTimeoutMs = 3000; 

    // PWM values for forward/backward
    int pwmFwd1, pwmFwd2, pwmFwd3, pwmFwd4;
    // PWM values for left/right/diagonals
    int pwmStrafe1, pwmStrafe2, pwmStrafe3, pwmStrafe4;

    Move(AF_DCMotor& m1, AF_DCMotor& m2, AF_DCMotor& m3, AF_DCMotor& m4,
         Encoders& encLeft, Encoders& encRight,
         int fwdPWM1 = 255, int fwdPWM2 = 255, int fwdPWM3 = 255, int fwdPWM4 = 255,
         int strafePWM1 = 255, int strafePWM2 = 255, int strafePWM3 = 255, int strafePWM4 = 255)
      : motor1(m1), motor2(m2), motor3(m3), motor4(m4),
        encoderLeft(encLeft), encoderRight(encRight),
        pwmFwd1(fwdPWM1), pwmFwd2(fwdPWM2), pwmFwd3(fwdPWM3), pwmFwd4(fwdPWM4),
        pwmStrafe1(strafePWM1), pwmStrafe2(strafePWM2), pwmStrafe3(strafePWM3), pwmStrafe4(strafePWM4) {}

    void begin(int pwm = 255, int pwm2 = 255, int pwm3 = 255, int pwm4 = 255) {
      // Set initial speeds if needed
      motor1.setSpeed(pwm);
      motor2.setSpeed(pwm2);
      motor3.setSpeed(pwm3);   
      motor4.setSpeed(pwm4);
    } 

    void moveBeginForward(){
      motor1.setSpeed(pwmFwd1);
      motor2.setSpeed(pwmFwd2);
      motor3.setSpeed(pwmFwd3);   
      motor4.setSpeed(pwmFwd4);
    }

    void moveBeginStrafe(){
      motor1.setSpeed(pwmStrafe1);
      motor2.setSpeed(pwmStrafe2);
      motor3.setSpeed(pwmStrafe3);   
      motor4.setSpeed(pwmStrafe4);
    }

    // Call before starting a new movement
    void startMove() {
      startLeft = encoderLeft.getEncoderCount();
      startRight = encoderRight.getEncoderCount();
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
      int d = 8;
      if (position == false){
        begin(pwmFwd1 + d, pwmFwd2, pwmFwd3, pwmFwd4 + d);  
      }else{
        begin(pwmFwd1 - d, pwmFwd2, pwmFwd3 ,pwmFwd4 - d);  
      }
      
      if (!moving) {
        startMove();
        moveStartTime = millis();
      }
      setMotors(FORWARD, FORWARD, FORWARD, FORWARD);
      if (checkDoneWithTimeout(pulses)){
        return 1;
      } else if (checkDoneWithTimeout(pulses / 2, true)){
        return 2;
      } else {return 0;};
    }

    bool backwardp(long pulses, bool position) {
    int d = 15;
    if (position == false){
      begin(pwmFwd1 + d, pwmFwd2, pwmFwd3, pwmFwd4 + d);  
    }else{
      begin(pwmFwd1 - d, pwmFwd2, pwmFwd3 ,pwmFwd4 - d);  
    }
    
    if (!moving) {
      startMove();
      moveStartTime = millis();
    }
    setMotors(BACKWARD , BACKWARD, BACKWARD, BACKWARD);
    return checkDoneWithTimeout(pulses);
  }

    bool forwardq(long pulses, bool position) {
    int d = 15;
    if (position == false){
      begin(pwmFwd1, pwmFwd2 + d, pwmFwd3 + d, pwmFwd4);  
    }else{
      begin(pwmFwd1, pwmFwd2 - d, pwmFwd3 - d, pwmFwd4);  
    }
    
    if (!moving) {
      startMove();
      moveStartTime = millis();
    }
    setMotors(FORWARD, FORWARD, FORWARD, FORWARD);
    return checkDoneWithTimeout(pulses);
  }

        // Forward
    bool forward(long pulses) {
      if (!moving) {
        moveBeginForward();
        startMove();
        moveStartTime = millis();
      }


      setMotors(FORWARD, FORWARD, FORWARD, FORWARD);
      return checkDoneWithTimeout(pulses);
    }

    //forward accelerated
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
      if (!moving) {
        moveBeginForward();
        startMove();
        moveStartTime = millis();
      }
      setMotors(BACKWARD, BACKWARD, BACKWARD, BACKWARD);
      return checkDoneWithTimeout(pulses);
    }

    // Left (strafe)
    bool left(long pulses) {
      if (!moving) {
        moveBeginStrafe();
        startMove();
        moveStartTime = millis();
      }
      setMotors(BACKWARD, FORWARD, BACKWARD, FORWARD);
      return checkDoneWithTimeout(pulses);
    }

    // Right (strafe)
    bool right(long pulses) {
      if (!moving) {
        moveBeginStrafe();
        startMove();
        moveStartTime = millis();
      }
      setMotors(FORWARD, BACKWARD, FORWARD, BACKWARD);
      return checkDoneWithTimeout(pulses);
    }

    // Diagonal: Forward-Left
    bool forwardLeft(long pulses) {
      if (!moving) {
        moveBeginStrafe();
        startMove();
      }
      setMotors(RELEASE, FORWARD, RELEASE, FORWARD);
      return checkDoneWithTimeout(pulses);
    }

    // Diagonal: Forward-Right
    bool forwardRight(long pulses) {
      if (!moving) {
        moveBeginStrafe();
        startMove();
      }
      setMotors(FORWARD, RELEASE, FORWARD, RELEASE);
      return checkDoneWithTimeout(pulses);
    }

    // Diagonal: Backward-Left
    bool backwardLeft(long pulses) {
      if (!moving) {
        moveBeginStrafe();
        startMove();
      }
      setMotors(BACKWARD, RELEASE, BACKWARD, RELEASE);
      return checkDone(pulses);
    }

    // Diagonal: Backward-Right
    bool backwardRight(long pulses) {
      if (!moving) {
        moveBeginStrafe();
        startMove();
      }
      setMotors(RELEASE, BACKWARD, RELEASE, BACKWARD);
      return checkDone(pulses);
    }


    bool rotate(long pulses, bool side) {
      if (!moving) {
        moveBeginStrafe();
        startMove();
      }

      if (side == true){
        setMotors(FORWARD, BACKWARD, BACKWARD, FORWARD);
      } else {
        setMotors(BACKWARD, FORWARD, FORWARD, BACKWARD);
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
    
    // ...existing code...

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


    void setMotors(uint8_t m1, uint8_t m2, uint8_t m3, uint8_t m4) {
      motor1.run(m1);
      motor2.run(m2);
      motor3.run(m3);
      motor4.run(m4);
    }


    // Helper to set all motor speeds
    void setAllSpeeds(int pwm) {
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
