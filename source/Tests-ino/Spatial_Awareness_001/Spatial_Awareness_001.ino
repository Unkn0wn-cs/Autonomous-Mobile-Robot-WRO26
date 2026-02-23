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
int pwmf[4] = {235, 240, 240, 234};// values for back and forth and maybe rotation but I don't remember
int pwms[4] = {225, 210, 210, 210};// values for straffing 
//motor definitions
AF_DCMotor motor1(1); 
AF_DCMotor motor2(2);
AF_DCMotor motor3(3); 
AF_DCMotor motor4(4); 

Encoders encoderLeft(A15, A14);	// Create an Encoder object name leftEncoder, using digitalpin 2 & 3
Encoders encoderRight(A13 , A12); // Encoder object name rightEncoder using analog pin A0 and A1 

Move move(//def of the move obj
  motor1, motor2, motor3, motor4, encoderLeft, encoderRight,
  pwmf[0], pwmf[1], pwmf[2], pwmf[3],      // Forward/backward PWM values
  pwms[0], pwms[1], pwms[2], pwms[3]       // Left/right/diagonal PWM values
);

const int enable34 = 9; // L293D pin 9 Tthe one on the 1st servo 

//SPATIAL AWARENESS CODE 
pos[2] = {0,0}

//Pulse to mm variables
int diameter = 60 //in mm
int pulses = 1020 // 1020 in  Left robot, 1650 in Right robot

void setup() {
  pinMode(enable34, OUTPUT);//enable rotor Output
  digitalWrite(enable34, HIGH); // Enable motor driver

  pixy.init();
  pixy.setLamp(255, 0); //Power of the Pixy's built in LED 

  myservo.attach(10)

}

void loop() {

long currentEncoderLeft = encoderLeft.getEncoderCount();
long currentEncoderRight = encoderRight.getEncoderCount();

double currentMmForward = move.pulsesToMM((currentEncoderLeft + currentEncoderRight) / 2, diameter, pulses)


pixy.ccc.getBlocks();
i = 1;
if (pixy.ccc.numBlocks){
  int x = pixy.ccc.blocks[i].m_x;
  if (x < 140)
  { 
    move.simpleLeft();
    Serial.print("MOVE RIGHT\n");
    pixy.ccc.blocks[i].print();

  } else if (x > 240){
    move.simpleRight();
    Serial.print("MOVE LEFT\n");
    pixy.ccc.blocks[i].print();

  } else if (x < 240 && x > 140 && currentMmForward < 200){
    pixy.ccc.blocks[i].print();
    move.simpleForward();
  } else if (currentMmForward > 200){
    move.stop();
  }
}