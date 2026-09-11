// main.cpp - entry point.
//
// Autonomous ping pong robot, Team Outer Heaven, WRO 2026.
//
// This file only wires the modules together, so the startup order stays obvious:
//
//   RobotConfig.h/.cpp  WHICH ROBOT this build is for  <- switch robots here
//   Hardware.h/.cpp     pins, motors, encoders, servo, rotor
//   Heading.h/.cpp      BNO08x heading
//   Sensors.h/.cpp      camera, I2C scan, microswitches
//   Motion.h/.cpp       mm(), inner(), outer()
//   Routines.h/.cpp     the strategy state machine
//   lib/move/           Move + WheelRegulator, the drive layer
//
// Several oddities in the routines are load bearing and are marked KNOWN where
// they appear.
//
// SERIAL: 115200 baud (platformio.ini monitor_speed for megaatmega2560).

#include <Arduino.h>

#include "RobotConfig.h"
#include "Hardware.h"
#include "Sensors.h"
#include "Motion.h"
#include "Routines.h"
#include "Heading.h"

// One line every TELEMETRY_EVERY_MS:
//   r/s   routine and state      err   heading error the regulator sees, deg
//   corr  heading correction, PWM     age   ms since the last sensor report
//   rst   sensor resets so far   hz    loop() passes per second
// Short enough to fit the 64-byte serial buffer, so it never blocks loop().
// Set TELEMETRY to false to silence it.
static const bool          TELEMETRY          = true;
static const unsigned long TELEMETRY_EVERY_MS = 250;

static void printTelemetry() {
  static unsigned long lastPrint = 0;
  static unsigned long loops = 0;
  loops++;

  unsigned long now = millis();
  if (now - lastPrint < TELEMETRY_EVERY_MS) return;
  unsigned long hz = loops * 1000UL / (now - lastPrint);
  lastPrint = now;
  loops = 0;

  float err  = headingError();
  float corr = move.regulator.headingCorr();

  Serial.print(F("r=")); Serial.print(routine);
  Serial.print(F(" s=")); Serial.print(state);
  Serial.print(F(" err=")); Serial.print(err, 2);
  Serial.print(F(" corr=")); Serial.print(corr, 1);
  Serial.print(F(" age=")); Serial.print(headingAgeMs());
  Serial.print(F(" rst=")); Serial.print(headingResetCount());
  Serial.print(F(" hz=")); Serial.println(hz);
}

void setup() {

  Serial.begin(115200);

  pixy.init();

  // Length of the main straight, in mm, for this robot.
  if(robotSide == RIGHT){
    lenght = 640;
  }else{
    lenght = 1100;
  }

  initHardware();

  Wire.begin();
  testI2C();   // boot-time diagnostic: prints every device on the bus

  if (headingBegin()) Serial.println("BNO08x heading sensor ready");
  else                Serial.println("BNO08x NOT found - driving without heading hold");

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

  // Fresh BNO08x reading. Rate-limits itself internally.
  headingUpdate();

  if (TELEMETRY) printTelemetry();

  // MUST BE LAST: routine 4 state 5 contains a bare `return` that is expected
  // to skip everything after it.
  runRoutines();
}
