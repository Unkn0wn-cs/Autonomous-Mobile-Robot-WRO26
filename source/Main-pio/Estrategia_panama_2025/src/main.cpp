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
#include <stdio.h>

#include "RobotConfig.h"
#include "Hardware.h"
#include "Sensors.h"
#include "Motion.h"
#include "Routines.h"
#include "Heading.h"

// One line every TELEMETRY_EVERY_MS, identical on Serial (USB) and Serial2
// (Bluetooth). Fixed-width columns, so consecutive lines read as a table:
//
//   r4 s2   MIDDLE  ball 180,25  hdg   -1.2  err  -0.35  corr   3.2  pwm 232 240 228 235  v  310  305  312  300
//
//   r s    routine and state
//   lane   the lane the robot is committed to. Changes when routine 6 picks
//          the next one from the camera.
//   ball   where selectOpeningRoutine() saw the purple ball at boot, Pixy
//          image pixels x,y; "none" if it never did
//   hdg    heading since power-on, degrees (0 when the sensor is stale)
//   err    heading error the regulator sees, degrees (0 when stale)
//   corr   heading correction being applied, PWM
//   pwm    PWM on motor1..motor4 (rear right, rear left, front left, front
//          right); 0 for a released wheel
//   v      speed of motor1..motor4 in mm/s, signed by encoder direction
//
// The line is built in a buffer first and only written when both ports have
// room for all of it, so write() never waits for the UART and loop() never
// stalls. The buffer is smaller than the TX buffers (SERIAL_TX_BUFFER_SIZE,
// 128 bytes in platformio.ini).

static const bool          TELEMETRY          = true; //Set to false to silence telemetry
static const unsigned long TELEMETRY_EVERY_MS = 250;

// 9600 is the HC-05 / HC-06 factory rate; at 9600 the longest line leaves in
// about 115 ms, inside the 250 ms period. Change to match if the module was
// set faster with bt_passthrough.
static const unsigned long BLUETOOTH_BAUD = 9600;

static const char* laneName(rlane l) {
  switch (l) {
    case OUTER: return "OUTER";
    case INNER: return "INNER";
    default:    return "MIDDLE";
  }
}

static void printTelemetry() {
  static unsigned long lastPrint = 0;
  static long lastCount[4] = {0, 0, 0, 0};

  unsigned long now = millis();
  unsigned long elapsed = now - lastPrint;
  if (elapsed < TELEMETRY_EVERY_MS) return;

  // Wheel speed from the encoder counts gained since the previous line.
  Encoders* enc[4] = {&encoderRearRight, &encoderRearLeft, &encoderLeft, &encoderRight};
  const float mmPerCount = 3.14159265f * diameter / pulses;
  long count[4];
  int  v[4];
  for (uint8_t i = 0; i < 4; i++) {
    count[i] = enc[i]->getEncoderCount();
    v[i] = (int)((count[i] - lastCount[i]) * mmPerCount * 1000.0f / elapsed);
  }

  // Floats are formatted separately: avr-libc's snprintf has no %f.
  char hdg[8], err[8], corr[8], ball[8];
  dtostrf(headingSinceBoot(),           6, 1, hdg);
  dtostrf(headingError(),               6, 2, err);
  dtostrf(move.regulator.headingCorr(), 5, 1, corr);
  if (purpleX < 0) strcpy_P(ball, PSTR("none"));
  else             snprintf_P(ball, sizeof ball, PSTR("%d,%d"), purpleX, purpleY);

  char line[120];
  int n = snprintf_P(line, sizeof line,
    PSTR("r%d s%-3d %-6s  ball %-7s hdg %s  err %s  corr %s  pwm %3d %3d %3d %3d  v %4d %4d %4d %4d\r\n"),
    routine, state, laneName(lane), ball, hdg, err, corr,
    move.wheelPWM[0], move.wheelPWM[1], move.wheelPWM[2], move.wheelPWM[3],
    v[0], v[1], v[2], v[3]);
  if (n >= (int)sizeof line) n = sizeof line - 1;

  // No room yet (another print got in first): keep the sample and try again
  // next pass rather than let write() block.
  if (Serial.availableForWrite() < n || Serial2.availableForWrite() < n) return;

  Serial.write(line, n);
  Serial2.write(line, n);

  lastPrint = now;
  for (uint8_t i = 0; i < 4; i++) lastCount[i] = count[i];
}

void setup() {

  Serial.begin(115200);
  Serial2.begin(BLUETOOTH_BAUD);   // Bluetooth module, telemetry only

  pixy.init();


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
