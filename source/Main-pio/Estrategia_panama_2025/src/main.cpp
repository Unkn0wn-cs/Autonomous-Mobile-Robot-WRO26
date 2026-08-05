// main.cpp - entry point.
//
// Autonomous ping pong robot, Team Outer Heaven, WRO 2026.
//
// This file deliberately contains almost nothing. It wires the modules together
// and does nothing else, so that the startup order stays obvious at a glance:
//
//   RobotConfig.h/.cpp  WHICH ROBOT this build is for  <- switch robots here
//   Hardware.h/.cpp     pins, motors, encoders, servo, rotor
//   Sensors.h/.cpp      gyro, camera, I2C, microswitches
//   Motion.h/.cpp       mm(), inner(), outer()
//   Routines.h/.cpp     the strategy state machine
//   lib/move/           Move + WheelRegulator, the closed loop drive layer
//
// Read lib/move/Movement.md before changing anything. Several oddities in the
// routines are load bearing and are marked KNOWN where they appear.

#include <Arduino.h>

#include "RobotConfig.h"
#include "Hardware.h"
#include "Sensors.h"
#include "Motion.h"
#include "Routines.h"

void setup() {

  Serial.begin(9600);

  pixy.init();

  // Length of the main straight, in mm, for this robot.
  if(robotSide == RIGHT){
    lenght = 640;
  }else{
    lenght = 1100;
  }

  // Servo, rotor pins, LED and microswitch pins.
  //
  // NOTE: the three microswitch pinMode() calls used to sit further down, after
  // the gyro was initialised. They are inert either way - pins 14/18/19 have no
  // interaction with Serial, I2C or the MPU6050 - so they were folded in here.
  // This is the only ordering change made during the restructure.
  initHardware();

  // MPU6050
  Wire.begin();           // start I2C

  // The bus is scanned once, here, instead of on every pass of loop(). Probing
  // all 126 addresses takes long enough that it made the loop period both long
  // and irregular, and the wheel regulator cannot be tuned against a loop whose
  // period keeps changing.
  //
  // This line is REQUIRED: updateGyro() refuses to read the gyro when
  // i2cDeviceCount is 0, so without it the heading never updates and mpu is
  // forced false, which silently drops routines 7 and 8 into their
  // encoder-counted rotation fallback.
  i2cDeviceCount = testI2C();

  sensor.initialize();    // start the sensor

  if (sensor.testConnection()) {Serial.println("Sensor started correctly"); mpu = true;}
  else{Serial.println("Error starting the sensor"); mpu = false;}

  tiempo_prev = millis();
  resetGyroAngles();

  // The wall robot waits on its start switch; the ramp robot starts immediately.
  if (robotSide == LEFT){
    bool pinpressed = false;
    while(!pinpressed){
      if ((digitalRead(switchPin)) == HIGH){
        pinpressed = true;
      }
    }
  }

  startTime = millis();

  myservo.write(closedGate);
  digitalWrite(input3, HIGH);
  digitalWrite(input4, LOW);
  enableDrivers();

  // Looks for the purple ball and picks the opening routine (0-3).
  // Leaves routine at its default of 4 if it does not find one.
  selectOpeningRoutine();
}

void loop() {

  // Inputs first: a switch press can advance the state machine before it runs.
  handleMicroSwitches();

  // Disabled endgame timing. See the note on lastRoutine in Routines.h.
  updateEndgameTiming();

  // Heading integration.
  updateGyro();

  // MUST BE LAST: routine 4 state 5 contains a bare `return` that is expected
  // to skip everything after it.
  runRoutines();
}
