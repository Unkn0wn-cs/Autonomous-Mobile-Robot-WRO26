// Sensors.cpp - encoders, BNO08x heading, Pixy2, microswitches, I2C scan and
// the telemetry. See Sensors.h.

#include "Sensors.h"
#include <stdio.h>
#include <Adafruit_BNO08x.h>

// ---------------------------------------------------------------------------
// Encoders. ORDER IS SIGNIFICANT - see the warning in Sensors.h.
//
// The front pair measures travelled distance and decides when a move is over;
// all four feed the mean speed the deceleration loop tracks. The constructor
// sets the pins as inputs with pull-ups and attaches the interrupts.
// ---------------------------------------------------------------------------

Encoders encoderLeft(A15, A14);      // motor3, front left
Encoders encoderRight(A13, A12);     // motor4, front right
Encoders encoderRearRight(A11, A10); // motor1, rear right
Encoders encoderRearLeft(A9, A8);    // motor2, rear left

// ---------------------------------------------------------------------------
// Heading - BNO08x
// ---------------------------------------------------------------------------

// I2C address is set by the ADR/DI pin on the board: 0x4A low, 0x4B high. Both
// are tried so the jumper can be in either position.
static const uint8_t ADDRESS_A = 0x4A;
static const uint8_t ADDRESS_B = 0x4B;

// Not wired. If connected, the library can hard-reset a wedged sensor.
static const int8_t RESET_PIN = -1;

// Sign applied to every heading difference, so that the reading increases
// under the rotateCW() wheel pattern (see SIGN CONVENTION in Sensors.h). -1
// because, with the sensor chip-up, its raw yaw increases the other way round
// (right-handed about the upward Z axis). square_test prints OK / FLIP after
// each turn.
static const float HEADING_SIGN = -1.0f;

// I2C bus clock. The BNO08x is the only device on the bus (the Pixy2 is on
// SPI). A rotation report read takes ~0.7 ms at 400 kHz. Drop to 100000 if
// headingAgeMs() keeps climbing or resets are counted during a run.
//
// Applied AFTER begin_I2C(): the Adafruit driver calls Wire.begin() inside it,
// and on AVR that resets the bus to 100 kHz.
static const uint32_t I2C_CLOCK_HZ = 400000;

// A bus transaction that waits longer than this has a stuck line. Wire gives
// up, resets the TWI peripheral, and the read fails instead of hanging.
static const uint32_t I2C_TIMEOUT_US = 10000;

// Requested report interval and how often we ask the sensor for a new one.
// The sensor delivers the nearest rate it supports; heading_test prints the
// rate actually delivered.
static const uint32_t REPORT_INTERVAL_US = 2500;   // 400 Hz requested
static const uint8_t  POLL_INTERVAL_MS   = 2;

// Staleness thresholds - see FAIL-SAFES in Sensors.h.
static const unsigned long STALE_FOR_HOLD_MS     = 100;
static const unsigned long STALE_FOR_AVAILABLE_MS = 1000;

static Adafruit_BNO08x   bno(RESET_PIN);
static sh2_SensorValue_t event;

static bool     present      = false;
static bool     haveFirst    = false;
static bool     resetPending = false;  // references invalid until next report
static float    current      = 0.0f;   // degrees, 0-360
static float    target       = 0.0f;   // movement-layer reference
static float    zero         = 0.0f;   // routine-layer reference
static float    boot         = 0.0f;   // telemetry reference
static uint8_t  accuracy     = 0;
static uint8_t  foundAddress = 0;
static unsigned long lastPoll    = 0;
static unsigned long lastReport  = 0;
static unsigned long reportCount = 0;
static unsigned long resetCount  = 0;

// Must be repeated after any sensor reset: the BNO08x drops every subscription
// when it resets, and then stays connected but silent.
static bool subscribe() {
  return bno.enableReport(SH2_GAME_ROTATION_VECTOR, REPORT_INTERVAL_US);
}

static float wrap180(float degrees) {
  while (degrees >  180.0f) degrees -= 360.0f;
  while (degrees < -180.0f) degrees += 360.0f;
  return degrees;
}

bool headingBegin() {
  present      = false;
  foundAddress = 0;
  haveFirst    = false;
  resetPending = false;
  reportCount  = 0;
  resetCount   = 0;

  if (bno.begin_I2C(ADDRESS_A, &Wire)) {
    foundAddress = ADDRESS_A;
  } else if (bno.begin_I2C(ADDRESS_B, &Wire)) {
    foundAddress = ADDRESS_B;
  } else {
    return false;
  }

  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setWireTimeout(I2C_TIMEOUT_US, true);

  if (!subscribe()) {
    foundAddress = 0;
    return false;
  }

  present = true;
  lastReport = millis();

  // Wait briefly for the first quaternion so the initial references are real.
  unsigned long t0 = millis();
  while (!haveFirst && millis() - t0 < 500) headingUpdate();

  headingCaptureTarget();
  headingZero();
  boot = current;
  return true;
}

void headingUpdate() {
  if (!present) return;

  unsigned long now = millis();
  if (now - lastPoll < POLL_INTERVAL_MS) return;
  lastPoll = now;

  if (bno.wasReset()) {
    subscribe();
    resetCount++;
    resetPending = true;
    haveFirst = false;   // nothing below is trusted until it reports again
  }

  if (!bno.getSensorEvent(&event)) return;
  if (event.sensorId != SH2_GAME_ROTATION_VECTOR) return;

  const float qi = event.un.gameRotationVector.i;
  const float qj = event.un.gameRotationVector.j;
  const float qk = event.un.gameRotationVector.k;
  const float qr = event.un.gameRotationVector.real;

  // Yaw about Z from the quaternion.
  float yaw = atan2(2.0f * (qi * qj + qk * qr),
                    (qi * qi - qj * qj - qk * qk + qr * qr)) * RAD_TO_DEG;
  if (yaw < 0.0f) yaw += 360.0f;

  current    = yaw;
  accuracy   = event.status & 0x03;
  haveFirst  = true;
  lastReport = now;
  reportCount++;

  // A reset gives the sensor a new reference frame, so the old references
  // would produce a large, meaningless error. Hold from here instead.
  if (resetPending) {
    resetPending = false;
    target = current;
    zero   = current;
    boot   = current;
  }
}

static bool freshWithin(unsigned long ms) {
  return present && haveFirst && (millis() - lastReport) <= ms;
}

bool     headingAvailable() { return freshWithin(STALE_FOR_AVAILABLE_MS); }
float    headingNow()       { return current; }
uint8_t  headingAddress()   { return foundAddress; }
uint8_t  headingAccuracy()  { return accuracy; }
unsigned long headingAgeMs()       { return present ? (millis() - lastReport) : 0; }
unsigned long headingReportCount() { return reportCount; }
unsigned long headingResetCount()  { return resetCount; }

void  headingCaptureTarget() { target = current; }
float headingError() {
  if (!freshWithin(STALE_FOR_HOLD_MS)) return 0.0f;
  return HEADING_SIGN * wrap180(current - target);
}

void  headingZero() { zero = current; }
float headingSinceZero() {
  if (!headingAvailable()) return 0.0f;
  return HEADING_SIGN * wrap180(current - zero);
}

float headingSinceBoot() {
  if (!headingAvailable()) return 0.0f;
  return HEADING_SIGN * wrap180(current - boot);
}

// ---------------------------------------------------------------------------
// Pixy2 camera
// ---------------------------------------------------------------------------

Pixy2 pixy;

static int8_t   cameraResult  = PIXY_RESULT_ERROR;   // last cameraBegin() result
static uint8_t  cameraFwMajor = 0;
static uint8_t  cameraFwMinor = 0;
static uint16_t cameraFwBuild = 0;

int8_t cameraBegin() {
  cameraResult = pixy.init();
  // pixy.version points into the receive buffer, which the next packet
  // overwrites, so the numbers are copied out now.
  if (cameraResult == PIXY_RESULT_OK && pixy.version) {
    cameraFwMajor = pixy.version->firmwareMajor;
    cameraFwMinor = pixy.version->firmwareMinor;
    cameraFwBuild = pixy.version->firmwareBuild;
  }
  return cameraResult;
}

// ---------------------------------------------------------------------------
// Microswitches
// ---------------------------------------------------------------------------

void initSensors() {
  pinMode(backSwitchPin, INPUT_PULLUP);
  pinMode(sideSwitchPin, INPUT_PULLUP);
  pinMode(switchPin, INPUT_PULLUP);
}

// ---------------------------------------------------------------------------
// I2C bus scan
// ---------------------------------------------------------------------------

int testI2C() {
  byte error, address;
  int nDevices = 0;

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("Scan done\n");
  }

  return nDevices; // return how many devices were found
}

// ---------------------------------------------------------------------------
// Telemetry. Format in Sensors.h.
// ---------------------------------------------------------------------------

static const bool          TELEMETRY         = true;   // false silences both ports
static const unsigned long ROW_EVERY_MS      = 250;
static const uint8_t       HEADER_EVERY_ROWS = 10;     // 2.5 s of rows
static const unsigned long STATUS_REPEAT_MS  = 5000;   // second status block

// 9600 is the HC-05 / HC-06 factory rate, so a new module works with no
// configuration. A row (84 bytes) leaves in about 90 ms at 9600, inside the
// 250 ms row period. Change to match if the module was set faster with
// bt_passthrough.
static const unsigned long BLUETOOTH_BAUD = 9600;

static const uint8_t STATUS_LINES = 8;

static const char*   robotName          = "";
static const char*   strategyLabel      = "";
static int           straightLength     = 0;
static float         mmPerEncoderCount  = 0.0f;
static int           openingRoutineSeen = 4;
static unsigned long telemetryStart     = 0;
static unsigned long lastRow            = 0;
static long          lastCount[4]       = {0, 0, 0, 0};
static uint8_t       statusNext         = STATUS_LINES;   // next status line; == STATUS_LINES when none pending
static bool          statusRepeated     = false;
static bool          headerDue          = false;
static uint8_t       rowsSinceHeader    = 0;

static char line[100];

// motor1..motor4
static Encoders* const encoders[4] = {&encoderRearRight, &encoderRearLeft, &encoderLeft, &encoderRight};

// Sends line[0..n) to both ports, or nothing at all if either cannot take the
// whole line yet - the caller then simply tries again next pass.
static bool sendLine(int n) {
  if (n >= (int)sizeof line) n = sizeof line - 1;
  if (n <= 0) return true;
  if (Serial.availableForWrite() < n || Serial2.availableForWrite() < n) return false;
  Serial.write(line, n);
  Serial2.write(line, n);
  return true;
}

static PGM_P ballPlace(int openingRoutine) {
  switch (openingRoutine) {
    case 0:  return PSTR("upper left");
    case 1:  return PSTR("upper right");
    case 2:  return PSTR("lower left");
    default: return PSTR("lower right");
  }
}

// Builds status line k into line[] and returns its length.
static int statusLine(uint8_t k) {
  const size_t N = sizeof line;
  switch (k) {
    case 0: {
      unsigned long t = millis() - telemetryStart;
      return snprintf_P(line, N, PSTR("\r\n----- status  t %lu.%lu s -----\r\n"),
                        t / 1000, (t % 1000) / 100);
    }
    case 1:
      return snprintf_P(line, N, PSTR("robot     %s   strategy %s   straight %d mm\r\n"),
                        robotName, strategyLabel, straightLength);
    case 2:
      if (headingAddress() == 0)
        return snprintf_P(line, N, PSTR("heading   NOT FOUND   no BNO08x on I2C\r\n"));
      return snprintf_P(line, N, PSTR("heading   %S   BNO08x 0x%02X   reports %lu   resets %lu\r\n"),
                        headingAvailable() ? PSTR("OK   ") : PSTR("STALE"),
                        headingAddress(), headingReportCount(), headingResetCount());
    case 3:
      if (cameraResult == PIXY_RESULT_OK)
        return snprintf_P(line, N, PSTR("camera    OK      Pixy2 firmware %u.%u.%u\r\n"),
                          cameraFwMajor, cameraFwMinor, cameraFwBuild);
      return snprintf_P(line, N, PSTR("camera    FAIL    Pixy2 no response (%d)\r\n"), cameraResult);
    case 4:
      return snprintf_P(line, N, PSTR("switches  back %d  side %d  start %d   (1 = open)\r\n"),
                        digitalRead(backSwitchPin), digitalRead(sideSwitchPin), digitalRead(switchPin));
    case 5:
      return snprintf_P(line, N, PSTR("encoders  %ld %ld %ld %ld   errors %ld %ld %ld %ld\r\n"),
                        encoders[0]->getEncoderCount(),      encoders[1]->getEncoderCount(),
                        encoders[2]->getEncoderCount(),      encoders[3]->getEncoderCount(),
                        encoders[0]->getEncoderErrorCount(), encoders[1]->getEncoderErrorCount(),
                        encoders[2]->getEncoderErrorCount(), encoders[3]->getEncoderErrorCount());
    case 6:
      if (openingRoutineSeen >= 0 && openingRoutineSeen <= 3)
        return snprintf_P(line, N, PSTR("opening   routine %d   purple ball %S\r\n"),
                          openingRoutineSeen, ballPlace(openingRoutineSeen));
      return snprintf_P(line, N, PSTR("opening   routine 4   no purple ball\r\n"));
    case 7: {
      bool headingOK = headingAddress() != 0 && headingAvailable();
      bool cameraOK  = cameraResult == PIXY_RESULT_OK;
      bool encoderOK = true;
      for (uint8_t i = 0; i < 4; i++) if (encoders[i]->getEncoderErrorCount() != 0) encoderOK = false;
      if (headingOK && cameraOK && encoderOK)
        return snprintf_P(line, N, PSTR("check     ALL OK\r\n\r\n"));
      return snprintf_P(line, N, PSTR("check     PROBLEM:%S%S%S\r\n\r\n"),
                        headingOK ? PSTR("") : PSTR(" heading"),
                        cameraOK  ? PSTR("") : PSTR(" camera"),
                        encoderOK ? PSTR("") : PSTR(" encoder-errors"));
    }
  }
  return 0;
}

// Same widths as rowLine() below, so the columns line up.
static int headerLine() {
  return snprintf_P(line, sizeof line,
    PSTR("  r   s |    hdg     err corr | pwm  m1  m2  m3  m4 | mm/s    m1    m2    m3    m4\r\n"));
}

// "+1.2" / "-0.35": a column that only sometimes carries a sign jitters when
// read at speed, so every heading value is written with its sign. dtostrf()
// never writes a '+'.
static void signedStr(float v, uint8_t decimals, char* out) {
  out[0] = (v < 0.0f) ? '-' : '+';
  dtostrf(v < 0.0f ? -v : v, 1, decimals, out + 1);
}

static int rowLine(int routine, int state, const int pwm[4], float headingCorr,
                   const long count[4], unsigned long elapsed) {
  // Floats are formatted separately: avr-libc's snprintf has no %f.
  char hdg[10], err[10];
  signedStr(headingSinceBoot(), 1, hdg);
  signedStr(headingError(),     2, err);

  // The differential is applied as whole PWM, so it is shown as one.
  int corr = (int)(headingCorr + (headingCorr < 0.0f ? -0.5f : 0.5f));

  int v[4];
  for (uint8_t i = 0; i < 4; i++) {
    v[i] = (int)((count[i] - lastCount[i]) * mmPerEncoderCount * 1000.0f / elapsed);
  }

  return snprintf_P(line, sizeof line,
    PSTR("%3d %3d | %6s %7s %+4d |     %3d %3d %3d %3d |      %+5d %+5d %+5d %+5d\r\n"),
    routine, state, hdg, err, corr,
    pwm[0], pwm[1], pwm[2], pwm[3], v[0], v[1], v[2], v[3]);
}

void telemetryBegin(const char* robot, const char* strategy, int straightMM,
                    float mmPerCount, int openingRoutine) {
  if (!TELEMETRY) return;
  Serial2.begin(BLUETOOTH_BAUD);

  robotName          = robot;
  strategyLabel      = strategy;
  straightLength     = straightMM;
  mmPerEncoderCount  = mmPerCount;
  openingRoutineSeen = openingRoutine;

  telemetryStart = millis();
  lastRow        = telemetryStart;
  for (uint8_t i = 0; i < 4; i++) lastCount[i] = encoders[i]->getEncoderCount();

  statusNext = 0;   // the status block goes out first
}

void telemetryUpdate(int routine, int state, const int pwm[4], float headingCorr) {
  if (!TELEMETRY) return;
  unsigned long now = millis();

  // The status block, one line per pass, followed by a fresh header.
  if (statusNext < STATUS_LINES) {
    if (!sendLine(statusLine(statusNext))) return;
    if (++statusNext == STATUS_LINES) headerDue = true;
    return;
  }
  if (!statusRepeated && now - telemetryStart >= STATUS_REPEAT_MS) {
    statusRepeated = true;
    statusNext = 0;
    return;
  }
  if (headerDue) {
    if (!sendLine(headerLine())) return;
    headerDue = false;
    rowsSinceHeader = 0;
    return;
  }

  unsigned long elapsed = now - lastRow;
  if (elapsed < ROW_EVERY_MS) return;

  long count[4];
  for (uint8_t i = 0; i < 4; i++) count[i] = encoders[i]->getEncoderCount();
  if (!sendLine(rowLine(routine, state, pwm, headingCorr, count, elapsed))) return;

  lastRow = now;
  for (uint8_t i = 0; i < 4; i++) lastCount[i] = count[i];
  if (++rowsSinceHeader >= HEADER_EVERY_ROWS) headerDue = true;
}
