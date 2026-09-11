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
// SERIAL: Serial is the USB cable, 115200 baud (platformio.ini monitor_speed
// for megaatmega2560). Serial2 (TX2 pin 16, RX2 pin 17) is the Bluetooth
// module, at BLUETOOTH_BAUD below, and carries only the telemetry line.

#include <Arduino.h>

#include "RobotConfig.h"
#include "Hardware.h"
#include "Sensors.h"
#include "Motion.h"
#include "Routines.h"
#include "Heading.h"

// One line every TELEMETRY_EVERY_MS, identical on Serial (USB) and Serial2
// (Bluetooth):
//   r/s   routine and state
//   deg   heading since power-on, degrees (0 when the sensor is stale)
//   err   heading error the regulator sees, degrees (0 when stale)
//   corr  heading correction being applied, PWM
//   pwm   PWM on motor1..motor4 (rear right, rear left, front left, front
//         right); 0 for a released wheel
//   v     speed of motor1..motor4 in mm/s, signed by encoder direction
//   age   ms since the last sensor report     rst   sensor resets so far
//   hz    loop() passes per second
// The line is shorter than the TX buffer (SERIAL_TX_BUFFER_SIZE, 128 bytes in
// platformio.ini) and is only written into empty buffers, so print() never
// waits for the UART and loop() never stalls. Set TELEMETRY to false to
// silence it.
static const bool          TELEMETRY          = true;
static const unsigned long TELEMETRY_EVERY_MS = 250;

// UART rate of the Bluetooth module. 9600 is what an HC-05 / HC-06 talks from
// the factory, so a new module works with no configuration. At 9600 the
// longest line takes about 120 ms to leave the buffer, inside the 250 ms
// period, so the rule above still holds. If the module is set faster with
// bt_passthrough, change this to match.
static const unsigned long BLUETOOTH_BAUD = 9600;

static void printTelemetryTo(Print& out, float deg, float err, float corr,
                             const int v[], unsigned long age,
                             unsigned long rst, unsigned long hz) {
  out.print(F("r=")); out.print(routine);
  out.print(F(" s=")); out.print(state);
  out.print(F(" deg=")); out.print(deg, 1);
  out.print(F(" err=")); out.print(err, 2);
  out.print(F(" corr=")); out.print(corr, 1);
  out.print(F(" pwm="));
  for (uint8_t i = 0; i < 4; i++) { if (i) out.print(','); out.print(move.wheelPWM[i]); }
  out.print(F(" v="));
  for (uint8_t i = 0; i < 4; i++) { if (i) out.print(','); out.print(v[i]); }
  out.print(F(" age=")); out.print(age);
  out.print(F(" rst=")); out.print(rst);
  out.print(F(" hz=")); out.println(hz);
}

static void printTelemetry() {
  static unsigned long lastPrint = 0;
  static unsigned long loops = 0;
  static long lastCount[4] = {0, 0, 0, 0};
  loops++;

  unsigned long now = millis();
  if (now - lastPrint < TELEMETRY_EVERY_MS) return;

  // Not drained yet (another print got in first): try again next pass rather
  // than let print() block.
  if (Serial.availableForWrite()  < SERIAL_TX_BUFFER_SIZE - 1 ||
      Serial2.availableForWrite() < SERIAL_TX_BUFFER_SIZE - 1) return;

  unsigned long elapsed = now - lastPrint;
  unsigned long hz = loops * 1000UL / elapsed;
  lastPrint = now;
  loops = 0;

  // Wheel speed from the encoder counts gained since the previous line.
  Encoders* enc[4] = {&encoderRearRight, &encoderRearLeft, &encoderLeft, &encoderRight};
  const float mmPerCount = 3.14159265f * diameter / pulses;
  int v[4];
  for (uint8_t i = 0; i < 4; i++) {
    long count = enc[i]->getEncoderCount();
    v[i] = (int)((count - lastCount[i]) * mmPerCount * 1000.0f / elapsed);
    lastCount[i] = count;
  }

  float deg  = headingSinceBoot();
  float err  = headingError();
  float corr = move.regulator.headingCorr();
  unsigned long age = headingAgeMs();
  unsigned long rst = headingResetCount();

  printTelemetryTo(Serial,  deg, err, corr, v, age, rst, hz);
  printTelemetryTo(Serial2, deg, err, corr, v, age, rst, hz);
}

void setup() {

  Serial.begin(115200);
  Serial2.begin(BLUETOOTH_BAUD);   // Bluetooth module, telemetry only

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
