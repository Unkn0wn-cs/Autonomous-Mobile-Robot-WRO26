#include "C:\Users\samue\OneDrive\Documents\GitHub\singapore25\Codes\Movement\Encoder\Ecoder measured movement\lib\move\move.h"
#include <AFMotor.h>
#include <QuadratureEncoder.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include <Servo.h>
#include <Pixy2.h>

//objects
Servo myservo;
MPU6050 sensor;
Pixy2 pixy; 

//Hey Dani calibrate these pwm values to stabilize the robot's movement
int pwmf[4] = {235, 240, 240, 234}; // values for back and forth
int pwms[4] = {225, 210, 210, 210}; // values for strafing 

//motor definitions
AF_DCMotor motor1(1); 
AF_DCMotor motor2(2);
AF_DCMotor motor3(3); 
AF_DCMotor motor4(4); 

Encoders encoderLeft(A15, A14); 
Encoders encoderRight(A12 , A13); 

Move move(
  motor1, motor2, motor3, motor4, encoderLeft, encoderRight,
  pwmf[0], pwmf[1], pwmf[2], pwmf[3],      
  pwms[0], pwms[1], pwms[2], pwms[3]       
);

const int enable34 = 9; 

//SPATIAL AWARENESS CODE 
int pos[2] = {0,0};

//Pulse to mm variables
int diameter = 60; //in mm
int pulses = 1020; // 1020 in Left robot, 1650 in Right robot

void setup() {
  Serial.begin(115200); 

  pinMode(enable34, OUTPUT);
  digitalWrite(enable34, HIGH); 

  pixy.init();
  pixy.setLamp(255, 0); 

  myservo.attach(10);
}

void loop() {

  static int state = 1;

  long currentEncoderLeft = encoderLeft.getEncoderCount();
  long currentEncoderRight = encoderRight.getEncoderCount();

  // Note: If turning causes encoders to spin in opposite directions, this average might act weird.
  double currentMmForward = move.pulsesToMM((currentEncoderLeft + currentEncoderRight) / 2, diameter, pulses);
  double currentMmStrafe = move.pulsesToMM((currentEncoderLeft - currentEncoderRight) /2, diameter, pulses);

  // 1. Highest Priority: Have we reached the distance cap?
  if (currentMmForward >= 650 || abs(currentMmStrafe) > 200) {//|| abs(currentMmStrafe) > 200
    // move.stop();
    encoderLeft.setEncoderCount(0);
    encoderRight.setEncoderCount(0);
    state = 2;
    Serial.println("Target distance reached. Stopping.");
    return; // Exits the loop early so it ignores the camera until reset
  }

  // 2. Normal Tracking Logic
  pixy.ccc.getBlocks();
  int i = 0;


switch(state){
  case 1:
    if (pixy.ccc.numBlocks) {
    int x = pixy.ccc.blocks[i].m_x;
    
    if (x < 100 && !(abs(currentMmStrafe) > 200)) { 
      move.simpleLeft();
      Serial.print("MOVE RIGHT\n"); 
      
    } else if (x > 280) { //+-40
      move.simpleRight();
      Serial.print("MOVE LEFT\n");
      
    } else{
      // Ball is centered (between 140 and 240)
      move.simpleForward();
      Serial.print("MOVE FORWARD\n");
    }
  } else {
    // Stop if no ball is seen to prevent running away
    move.stop();
  }
  break;
  case 2:
    if(move.backward(move.mmToPulses(700, diameter, pulses))) state = 1;
  break;
}

}