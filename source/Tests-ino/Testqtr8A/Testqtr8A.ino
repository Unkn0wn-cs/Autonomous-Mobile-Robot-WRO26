/*
 * QTR-8A IR Sensor Array - ESP32 Line Follower Test
 * ===================================================
 * Wiring (QTR-8A → ESP32):
 *   OUT1 → GPIO 36 | OUT2 → GPIO 39 | OUT3 → GPIO 34 | OUT4 → GPIO 35
 *   OUT5 → GPIO 32 | OUT6 → GPIO 33 | OUT7 → GPIO 25 | OUT8 → GPIO 26
 *   VCC  → 3.3V    | GND  → GND
 *
 * Position output:
 *   0    = line far LEFT  (under sensor 1)
 *   3500 = line CENTRED
 *   7000 = line far RIGHT (under sensor 8)
 *   -1   = line LOST
 */

// ─── Pin definitions ──────────────────────────────────────────────────────────
const uint8_t NUM_SENSORS = 8;
const uint8_t SENSOR_PINS[NUM_SENSORS] = {36, 39, 34, 35, 32, 33, 25, 26};
const uint8_t LED_PIN = 2;

// ─── Calibration storage ─────────────────────────────────────────────────────
uint16_t sensorMin[NUM_SENSORS];
uint16_t sensorMax[NUM_SENSORS];

// ─── Configuration ───────────────────────────────────────────────────────────
const uint16_t CALIBRATION_SAMPLES = 200;
const uint16_t CALIBRATION_DELAY_MS = 10;
const uint16_t READ_DELAY_MS        = 50;
const uint16_t LINE_THRESHOLD       = 500;

// ─── Read all raw ADC values ──────────────────────────────────────────────────
void readRaw(uint16_t values[NUM_SENSORS]) {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    values[i] = analogRead(SENSOR_PINS[i]);
  }
}

// ─── Normalise raw value to 0-1000 ───────────────────────────────────────────
uint16_t normalise(uint16_t raw, uint8_t idx) {
  if (sensorMax[idx] <= sensorMin[idx]) return 0;

  int32_t scaled = (int32_t)(raw - sensorMin[idx]) * 1000
                   / (sensorMax[idx] - sensorMin[idx]);

  if (scaled < 0)    scaled = 0;
  if (scaled > 1000) scaled = 1000;
  return (uint16_t)scaled;
}

// ─── Calibration ─────────────────────────────────────────────────────────────
void calibrate() {
  Serial.println("\n=== CALIBRATION START ===");
  Serial.println("Slowly move the sensor over the line and white surface...");

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = 4095;
    sensorMax[i] = 0;
  }

  digitalWrite(LED_PIN, HIGH);

  uint16_t raw[NUM_SENSORS];
  for (uint16_t s = 0; s < CALIBRATION_SAMPLES; s++) {
    readRaw(raw);
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      if (raw[i] < sensorMin[i]) sensorMin[i] = raw[i];
      if (raw[i] > sensorMax[i]) sensorMax[i] = raw[i];
    }
    delay(CALIBRATION_DELAY_MS);
  }

  digitalWrite(LED_PIN, LOW);

  Serial.println("\n--- Calibration Results ---");
  Serial.printf("%-8s %-8s %-8s\n", "Sensor", "Min", "Max");
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.printf("S%-7u %-8u %-8u\n", i + 1, sensorMin[i], sensorMax[i]);
  }
  Serial.println("=== CALIBRATION DONE ===\n");
}

// ─── Weighted-average line position ──────────────────────────────────────────
int32_t readLinePosition(uint16_t normValues[NUM_SENSORS]) {
  int32_t weightedSum = 0;
  int32_t sum         = 0;
  bool    lineDetected = false;

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    uint16_t v = normValues[i];
    if (v > LINE_THRESHOLD) lineDetected = true;
    weightedSum += (int32_t)v * (i * 1000);
    sum         += v;
  }

  if (!lineDetected || sum == 0) return -1;
  return weightedSum / sum;
}

// ─── Setup ───────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  analogReadResolution(12);
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    analogSetPinAttenuation(SENSOR_PINS[i], ADC_11db);
  }

  Serial.println("======================================");
  Serial.println("  QTR-8A Line Sensor Test - ESP32");
  Serial.println("======================================");

  Serial.println("Starting calibration in 3 seconds...");
  delay(3000);
  calibrate();
}

// ─── Main loop ───────────────────────────────────────────────────────────────
void loop() {
  uint16_t raw[NUM_SENSORS];
  uint16_t norm[NUM_SENSORS];

  readRaw(raw);

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    norm[i] = normalise(raw[i], i);
  }

  int32_t position = readLinePosition(norm);

  Serial.print("RAW  | ");
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.printf("S%u:%4u ", i + 1, raw[i]);
  }
  Serial.println();

  Serial.print("NORM | ");
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.printf("S%u:%4u ", i + 1, norm[i]);
  }
  Serial.println();

  Serial.print("LINE | [");
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.print(norm[i] > LINE_THRESHOLD ? "##" : "  ");
    if (i < NUM_SENSORS - 1) Serial.print("|");
  }
  Serial.print("]");

  if (position == -1) {
    Serial.println("  Position: LINE LOST");
  } else {
    const char* direction;
    if      (position < 1000) direction = "FAR LEFT";
    else if (position < 2500) direction = "LEFT";
    else if (position < 4500) direction = "CENTRE";
    else if (position < 6000) direction = "RIGHT";
    else                      direction = "FAR RIGHT";

    Serial.printf("  Position: %5ld  (%s)\n", (long)position, direction);
  }

  Serial.println("-------------------------------------------");
  delay(READ_DELAY_MS);
}