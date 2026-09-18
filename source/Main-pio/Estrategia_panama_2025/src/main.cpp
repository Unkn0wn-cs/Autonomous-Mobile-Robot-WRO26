// main.cpp - entry point.
//
// Autonomous ping pong robot, Team Outer Heaven, WRO 2026.
//
// This file only wires the modules together, so the startup order stays obvious:
//
//   Hardware.h/.cpp         WHICH ROBOT this build is for  <- switch robots here
//                           and its pins, motors, servo, rotor, Move instance
//   Sensors.h/.cpp          encoders, BNO08x heading, camera, microswitches
//   Strategy.h              what a strategy defines, which one is built, its
//                           options; generalStrategy.cpp / controlStrategy.cpp
//                           are the strategies, one per PlatformIO environment
//   lib/move/               Move + WheelRegulator, the drive layer (moves in
//                           millimetres, converted to encoder counts inside)
//
// Several oddities in the routines are load bearing and are marked KNOWN where
// they appear.
//
// SERIAL: Serial is the USB cable, 115200 baud (platformio.ini monitor_speed
// for the competition environments). Serial2 (TX2 pin 16, RX2 pin 17) is the Bluetooth
// module; both carry the telemetry (Sensors.h), Serial also the boot messages.

#include <Arduino.h>

#include "Hardware.h"
#include "Sensors.h"
#include "Strategy.h"

void setup() {

  Serial.begin(115200);

  cameraBegin();

  initHardware();
  initSensors();

  Wire.begin();
  testI2C();   // boot-time diagnostic: prints every device on the bus

  if (headingBegin()) Serial.println("BNO08x heading sensor ready");
  else                Serial.println("BNO08x NOT found - driving without heading hold");

  Serial.print("strategy: ");
  Serial.println(strategyName);

  startTime = millis();

  myservo.write(closedGate);
  digitalWrite(input3, HIGH);
  digitalWrite(input4, LOW);
  enableDrivers();

  // Looks for the purple ball and picks the opening routine (0-3).
  // Leaves routine at its default of 4 if it does not find one.
  selectOpeningRoutine();

  // Opens the Bluetooth port and queues the status block; loop() sends it.
  telemetryBegin(robotSide == LEFT ? "LEFT (wall)" : "RIGHT (ramp)", strategyName,
                 lenght, 3.14159265f * diameter / pulses, routine);
}

void loop() {

  // Inputs first: a switch press can advance the state machine before it runs.
  handleMicroSwitches();

  // Match clock for the endgame kicks (an option in Strategy.h).
  updateEndgameTiming();

  // Fresh BNO08x reading. Rate-limits itself internally.
  headingUpdate();

  // At most one line per pass, and only when it fits the serial buffers.
  telemetryUpdate(routine, state, move.wheelPWM, move.headingCorr());

  // MUST BE LAST
  runRoutines();
}
