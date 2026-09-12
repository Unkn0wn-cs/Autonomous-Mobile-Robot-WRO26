// main.cpp - entry point.
//
// Autonomous ping pong robot, Team Outer Heaven, WRO 2026.
//
// This file only wires the modules together, so the startup order stays obvious:
//
//   Hardware.h/.cpp         WHICH ROBOT this build is for  <- switch robots here
//                           and its pins, motors, servo, rotor, Move instance
//   Sensors.h/.cpp          encoders, BNO08x heading, camera, microswitches
//   generalStrategy.h/.cpp  the strategy state machine
//   lib/move/               Move + WheelRegulator, the drive layer (moves in
//                           millimetres, converted to encoder counts inside)
//
// Several oddities in the routines are load bearing and are marked KNOWN where
// they appear.
//
// SERIAL: Serial is the USB cable, 115200 baud (platformio.ini monitor_speed
// for megaatmega2560). Serial2 (TX2 pin 16, RX2 pin 17) is the Bluetooth
// module; both carry the telemetry (Sensors.h), Serial also the boot messages.

#include <Arduino.h>

#include "Hardware.h"
#include "Sensors.h"
#include "generalStrategy.h"

void setup() {

  Serial.begin(115200);

  cameraBegin();

  initHardware();
  initSensors();

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

  // Opens the Bluetooth port and queues the status block; loop() sends it.
  telemetryBegin(robotSide == LEFT ? "LEFT (wall)" : "RIGHT (ramp)", lenght,
                 3.14159265f * diameter / pulses, routine);
}

void loop() {

  // Inputs first: a switch press can advance the state machine before it runs.
  handleMicroSwitches();

  // Disabled endgame timing. See the note on lastRoutine in generalStrategy.h.
  updateEndgameTiming();

  // Fresh BNO08x reading. Rate-limits itself internally.
  headingUpdate();

  // At most one line per pass, and only when it fits the serial buffers.
  telemetryUpdate(routine, state, move.wheelPWM, move.regulator.headingCorr());

  // MUST BE LAST: routine 4 state 5 contains a bare `return` that is expected
  // to skip everything after it.
  runRoutines();
}
