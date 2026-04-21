#include <QTRSensors.h>

// --- Motor Configuration (Standard PWM) ---
const int ENA = 44, IN1 = 17, IN2 = 16;  // Left Motor
const int ENB = 45, IN3 = 15, IN4 = 14;  // Right Motor

// --- Sensor Configuration ---
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

const int LED = 12;
const int SWITCH = 11;

// --- PID Constants ---
// Note: Increased Kd slightly to help with steep angle recovery
float Kp = 27.0; 
float Ki = 0.0;  
float Kd = 17.0; 

// --- PID Variables ---
int P = 0;
int I = 0;
int D = 0;
int previousError = 0;
float PID_value = 0;

// --- Speed Configuration ---
int baseSpeed = 85; 

// ==========================================
// Setup
// ==========================================
void setup() {
  Serial.begin(9600);



  // Configure motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stopMotors();

  for (uint16_t i = 0; i < 1000; i++)
  {
    qtr.calibrate();
  }


  bool pinpressed = false;
  pinMode(SWITCH, INPUT_PULLUP);
  while(!pinpressed){
      if ((digitalRead(SWITCH)) == HIGH){
        pinpressed = true;
      }}

  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  // Configure QTR-8RC Sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){3, 4, 5, 6, 7, 8, 9, 10}, SensorCount);
  qtr.setEmitterPin(2);
  
  delay(2000); // 2-second buffer to place robot on track
}

// ==========================================
// Main Loop
// ==========================================
void loop() {
  // delay(10);

  digitalWrite(LED, HIGH);

  int error = readSensors();
  
  // --- 1. HARD RECOVERY MODE ---
  // If the robot is totally lost (error 8 or -8), bypass PID
  // and perform a high-torque pivot turn to find the line.
  if (error == 8) {
    setLeftMotor(120);   // Spin right fast
    setRightMotor(-120);
    return;              
  } else if (error == -8) {
    setLeftMotor(-120);  // Spin left fast
    setRightMotor(120);
    return;              
  }

  // --- 2. DYNAMIC BASE SPEED ---
  // Slow down the forward momentum during sharp turns (error >= 5)
  // to prevent overshooting the curve.
  int currentBaseSpeed = baseSpeed;
  if (abs(error) >= 5) {
    currentBaseSpeed = 45; 
  }

  // --- 3. PID CALCULATION ---
  P = error;
  I = I + error;
  D = error - previousError;
  
  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  // Calculate new motor speeds
  int leftMotorSpeed = currentBaseSpeed + PID_value;
  int rightMotorSpeed = currentBaseSpeed - PID_value;

  // Execute movement
  setLeftMotor(leftMotorSpeed);
  setRightMotor(rightMotorSpeed);
}

// ==========================================
// Sensor Reading (Thresholding & Error Map)
// ==========================================
int readSensors() {
  qtr.read(sensorValues);
  int s[8];

  // Convert raw values to binary based on 1000 threshold
  for (int i = 0; i < 8; i++) {
    s[i] = (sensorValues[i] > 2400) ? 1 : 0;
  }

int error = 0;

  // Count active sensors to easily identify wide lines or lost states
  int activeSensors = s[0] + s[1] + s[2] + s[3] + s[4] + s[5] + s[6] + s[7];

  // ==========================================
  // 1. DEFINITIVE LOST STATE (0 sensors active)
  // ==========================================
  if (activeSensors == 0) {
    // Determine the direction of the search pivot based on the last known error
    return (previousError < 0) ? -8 : 8; 
  }

  // ==========================================
  // 2. STEEP ANGLE / SPLIT LINE CASES
  // ==========================================
  // If a 90+ degree angle causes the line to trigger BOTH far sides of the array:
  // e.g., 10000001 or 11000001
  else if (s[0] == 1 && s[7] == 1) {
    // Follow the momentum of the last known turn to catch the outer edge of the corner
    error = (previousError < 0) ? -7 : 7; 
  }
  else if (s[0] == 1 && s[6] == 1) error = -7;
  else if (s[1] == 1 && s[7] == 1) error = 7;

  // ==========================================
  // 3. WIDE SPACES (4 or 3 sensors active)
  // ==========================================
  else if (s[2]==1 && s[3]==1 && s[4]==1 && s[5]==1) error = 0;
  else if (s[1]==1 && s[2]==1 && s[3]==1 && s[4]==1) error = -2;
  else if (s[3]==1 && s[4]==1 && s[5]==1 && s[6]==1) error = 2;
  
  else if (s[0]==1 && s[1]==1 && s[2]==1) error = -6;
  else if (s[5]==1 && s[6]==1 && s[7]==1) error = 6;
  else if (s[1]==1 && s[2]==1 && s[3]==1) error = -4;
  else if (s[4]==1 && s[5]==1 && s[6]==1) error = 4;
  else if (s[2]==1 && s[3]==1 && s[4]==1) error = -1;
  else if (s[3]==1 && s[4]==1 && s[5]==1) error = 1;

  // ==========================================
  // 4. STANDARD 2-SENSOR AND 1-SENSOR CASES
  // ==========================================
  // Note: We check the OUTER sensors first. If the line is skewed, 
  // we want the robot to prioritize not falling off the track.
  else if (s[0]==1 && s[1]==1) error = -6; 
  else if (s[6]==1 && s[7]==1) error = 6; 
  else if (s[0]==1) error = -7; 
  else if (s[7]==1) error = 7; 

  else if (s[1]==1 && s[2]==1) error = -4; 
  else if (s[5]==1 && s[6]==1) error = 4; 
  else if (s[1]==1) error = -5;
  else if (s[6]==1) error = 5;

  else if (s[2]==1 && s[3]==1) error = -2; 
  else if (s[4]==1 && s[5]==1) error = 2; 
  else if (s[2]==1) error = -3;
  else if (s[5]==1) error = 3;

  else if (s[3]==1 && s[4]==1) error = 0;  
  else if (s[3]==1) error = -1;
  else if (s[4]==1) error = 1;
  
  // ==========================================
  // 5. UNEXPECTED NOISE FALLBACK
  // ==========================================
  // If a bizarre combination triggers (e.g., 01010101) due to track dirt,
  // ignore it and hold the last known good trajectory until the next loop.
  else {
    error = previousError; 
  }

  return error;
}

// ==========================================
// Motor Helper Functions
// ==========================================
void setLeftMotor(int speed) {
  if (speed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    speed = -speed;
  }
  analogWrite(ENA, constrain(speed, 0, 255));
}

void setRightMotor(int speed) {
  if (speed >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    speed = -speed;
  }
  analogWrite(ENB, constrain(speed, 0, 255));
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); analogWrite(ENA, 0);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); analogWrite(ENB, 0);
}