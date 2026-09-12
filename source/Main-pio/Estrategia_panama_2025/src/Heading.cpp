// Heading.cpp - heading from the BNO08x. See Heading.h.

#include "Heading.h"

#include <Wire.h>
#include <Adafruit_BNO08x.h>

// I2C address is set by the ADR/DI pin on the board: 0x4A low, 0x4B high. Both
// are tried so the jumper can be in either position.
static const uint8_t ADDRESS_A = 0x4A;
static const uint8_t ADDRESS_B = 0x4B;

// Not wired. If connected, the library can hard-reset a wedged sensor.
static const int8_t RESET_PIN = -1;

// Sign applied to every heading difference, so that the reading increases
// under the rotateCW() wheel pattern (see Heading.h). -1 because, with the
// sensor chip-up, its raw yaw increases the other way round (right-handed
// about the upward Z axis). square_test prints OK / FLIP after each turn.
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

// Staleness thresholds - see FAIL-SAFES in Heading.h.
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
