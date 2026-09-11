// heading_test.cpp - BNO08x readout only. NOT the competition firmware.
//
// THE ROBOT DOES NOT MOVE. This program never touches a motor and does not even
// compile the drivetrain - no AFMotor, no encoders, no Move. If it reads
// sensibly, the sensor and its wiring are good independently of anything the
// movement code is doing.
//
// BUILD AND RUN
//   pio run -e heading_test -t upload
//   pio device monitor -e heading_test        (115200 baud)
//
// WIRING (Arduino Mega)
//   SDA -> pin 20      SCL -> pin 21      power and ground per the module
//   The Mega's dedicated SDA/SCL pins next to AREF are those same two pins.
//   Note A4/A5 are NOT I2C on a Mega - that is an Uno thing.
//
// IT ALWAYS TALKS
// ---------------
// This prints something every second no matter what, including when it cannot
// find the sensor. If the monitor is silent, the board is not running this
// firmware - check the upload targeted heading_test and the COM port is right.
//
// WHAT IT IS FOR
//   1. IS THE SENSOR THERE? It scans the whole I2C bus and names what it finds.
//   2. HOW MUCH DOES IT DRIFT? The 6-axis report has no magnetometer anchoring
//      it, so yaw creeps. Leave the robot still and watch `creep`. Under a
//      degree a minute is irrelevant over a match.
//   3. WHICH WAY IS POSITIVE? `rel` (HEADING_SIGN applied) increases when the
//      robot is turned the way the rotateCW() wheel pattern turns it - see
//      Heading.h. The definitive check is square_test's SIGN verdict, which
//      measures the turn against the actual wheel pattern.
//   4. HOW FAST DOES IT REPORT? `rate` is reports per second actually
//      delivered over the last print interval, against the 400 Hz requested
//      in Heading.cpp. Also confirms the I2C bus is healthy at 400 kHz.
//   5. DOES IT RESET? `rst` counts sensor resets. Any reset during a run is a
//      power problem to chase: each one re-references the heading.
//
// CONTROLS
//   Start switch (pin 14), or any character over serial, re-zeroes.

#include <Arduino.h>
#include <Wire.h>

#include "Heading.h"

// Declared here rather than included from Hardware.h on purpose: including that
// would drag the whole drivetrain into a test whose entire point is to depend on
// none of it. Matches switchPin in Hardware.h.
static const uint8_t START_SWITCH_PIN = 14;

static const unsigned long PRINT_EVERY_MS = 1000;
static const unsigned long RETRY_EVERY_MS = 3000;

static unsigned long zeroedAt = 0;
static float worstWander = 0.0f;
static bool  sensorUp = false;

// Names the addresses this project might plausibly meet, so an unexpected
// device is recognisable instead of just being a number.
static void describeAddress(uint8_t addr) {
  switch (addr) {
    case 0x4A: case 0x4B: Serial.print(F("   <- BNO08x (what we want)")); break;
    case 0x3C: case 0x3D: Serial.print(F("   <- SSD1306 OLED")); break;
    case 0x40: Serial.print(F("   <- INA219 or PCA9685")); break;
    case 0x70: Serial.print(F("   <- TCA9548 I2C multiplexer")); break;
    default: break;
  }
}

// Probes one address and says WHY it did not answer. The distinction matters: a
// clean NACK means the bus is healthy and nothing lives there, whereas a timeout
// or bus error means the wiring itself is in trouble.
static void probe(uint8_t addr) {
  Wire.beginTransmission(addr);
  uint8_t err = Wire.endTransmission();
  Serial.print(F("  0x"));
  Serial.print(addr, HEX);
  Serial.print(F(" -> "));
  switch (err) {
    case 0: Serial.println(F("ACK - device present")); break;
    case 2: Serial.println(F("NACK on address - bus is fine, nothing lives here")); break;
    case 3: Serial.println(F("NACK on data")); break;
    case 4: Serial.println(F("other error - bus may be stuck")); break;
    case 5: Serial.println(F("timeout - SDA or SCL held low")); break;
    default: Serial.print(F("error ")); Serial.println(err); break;
  }
}

static void scanBus() {
  Serial.println(F("scanning I2C bus..."));
  uint8_t found = 0;
  bool sawBno = false;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      found++;
      if (addr == 0x4A || addr == 0x4B) sawBno = true;
      Serial.print(F("  device at 0x"));
      if (addr < 16) Serial.print('0');
      Serial.print(addr, HEX);
      describeAddress(addr);
      Serial.println();
    }
  }

  if (found == 0) {
    Serial.println(F("  NOTHING ON THE BUS."));
    Serial.println(F("  SDA -> pin 20, SCL -> pin 21, power and ground."));
    Serial.println(F("  A bus with nothing on it usually means power or ground."));
    return;
  }

  if (!sawBno) {
    Serial.print(F("  ")); Serial.print(found);
    Serial.println(F(" device(s) answered, but NONE is a BNO08x (0x4A / 0x4B)."));
    probe(0x4A);
    probe(0x4B);
    Serial.println(F("  Something else replied, so the bus works. Check, in order:"));
    Serial.println(F("    1. VIN and GND at the module - an unpowered chip cannot ACK"));
    Serial.println(F("    2. its SDA/SCL reach pin 20/21, same as the device that DID"));
    Serial.println(F("       answer (A4/A5 are NOT I2C on a Mega)"));
    Serial.println(F("    3. 3.3 V vs 5 V - a bare module without a regulator needs 3.3 V"));
  }
}

static void tryStart() {
  sensorUp = headingBegin();
  if (sensorUp) {
    Serial.print(F("BNO08x FOUND at 0x"));
    Serial.print(headingAddress(), HEX);
    Serial.println(F(" - 6-axis GAME rotation vector"));
    Serial.println(F("(accel + gyro; magnetometer deliberately unused so the"));
    Serial.println(F(" motors cannot disturb it)."));
    Serial.println(F("\nTurn the robot BY HAND and watch `rel` follow it."));
    Serial.println(F("Start switch on pin 14, or any character, re-zeroes.\n"));
    Serial.println(F("   time      raw      rel     worst     creep      acc   age    rate   rst"));
    zeroedAt = millis();
    worstWander = 0.0f;
  } else {
    Serial.println(F("BNO08x did not start. Retrying every 3 s."));
    Serial.println(F("(It can answer on the bus but still fail to subscribe -"));
    Serial.println(F(" that usually means a marginal supply or a wedged sensor.)"));
  }
}

static void rezero(const __FlashStringHelper* why) {
  headingCaptureTarget();
  zeroedAt = millis();
  worstWander = 0.0f;
  Serial.print(F("--- zeroed ("));
  Serial.print(why);
  Serial.println(F("): this direction is now 0 deg ---"));
}

void setup() {
  Serial.begin(115200);

  // The Mega resets when the serial monitor opens, and the monitor needs about
  // a second to attach. Without this pause everything below is transmitted into
  // a port nobody is listening to yet, and the screen stays blank.
  delay(1500);

  pinMode(START_SWITCH_PIN, INPUT_PULLUP);

  Serial.println(F("\n\n=== BNO08x heading readout - THE ROBOT DOES NOT MOVE ==="));

  Wire.begin();
  scanBus();
  tryStart();
}

void loop() {
  static unsigned long lastPrint = 0;
  static unsigned long lastRetry = 0;

  // --- no sensor: keep talking, and keep trying ---------------------------
  if (!sensorUp) {
    if (millis() - lastRetry >= RETRY_EVERY_MS) {
      lastRetry = millis();
      Serial.println();
      scanBus();
      tryStart();
    }
    return;
  }

  headingUpdate();

  static bool lastSwitch = HIGH;
  bool nowSwitch = digitalRead(START_SWITCH_PIN);
  if (lastSwitch == HIGH && nowSwitch == LOW) rezero(F("switch"));
  lastSwitch = nowSwitch;

  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    rezero(F("serial"));
  }

  if (millis() - lastPrint < PRINT_EVERY_MS) return;
  unsigned long interval = millis() - lastPrint;
  lastPrint = millis();

  // Reports delivered since the last print, as a rate.
  static unsigned long lastCount = 0;
  unsigned long count = headingReportCount();
  unsigned long rate = (count - lastCount) * 1000UL / interval;
  lastCount = count;

  if (!headingAvailable()) {
    Serial.println(F("  connected, but no rotation report yet..."));
    return;
  }

  float rel = headingError();
  float mag = rel < 0 ? -rel : rel;
  if (mag > worstWander) worstWander = mag;

  float minutes = (millis() - zeroedAt) / 60000.0f;
  float creep = (minutes > 0.05f) ? rel / minutes : 0.0f;

  unsigned long age = headingAgeMs();

  Serial.print(F("  "));
  Serial.print((millis() - zeroedAt) / 1000.0f, 1); Serial.print(F("s "));
  Serial.print(F("   "));  Serial.print(headingNow(), 1);
  Serial.print(F("   "));
  if (rel >= 0) Serial.print('+');
  Serial.print(rel, 2);
  Serial.print(F("    "));  Serial.print(worstWander, 2);
  Serial.print(F("    "));  Serial.print(creep, 2); Serial.print(F("/min"));
  Serial.print(F("     "));  Serial.print(headingAccuracy()); Serial.print(F("/3"));
  Serial.print(F("   "));   Serial.print(age); Serial.print(F("ms"));
  Serial.print(F("   "));   Serial.print(rate); Serial.print(F("Hz"));
  Serial.print(F("   "));   Serial.print(headingResetCount());

  // A report age that keeps climbing means the sensor has gone quiet - the
  // symptom of a stalled bus or a sensor that has stopped reporting.
  if (age > 500) Serial.print(F("   *** SENSOR HAS GONE QUIET ***"));

  Serial.println();
}
