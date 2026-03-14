#include <Wire.h>
#include <AFMotor.h>

// --- I2C Sensor Configuration ---
#define LINE_FOLLOWER_I2C_ADDR 0x78

// --- Motor Configuration ---
AF_DCMotor leftMotor(3);  
AF_DCMotor rightMotor(4); 

// --- PID Constants ---
float Kp = 35.0; 
float Ki = 0.0;  
float Kd = 10.0; 

// --- PID Variables ---
int P = 0;
int I = 0;
int D = 0;
int previousError = 0;
float PID_value = 0;

// --- Speed Configuration ---
int baseSpeed = 80; 

// ==========================================
// I2C Communication Functions (from your code)
// ==========================================
bool WireWriteByte(uint8_t val) {
    Wire.beginTransmission(LINE_FOLLOWER_I2C_ADDR);
    Wire.write(val);
    if(Wire.endTransmission() != 0) {
        // Serial.println("false"); // Commented out to prevent loop lag
        return false;
    }
    // Serial.println("true"); // Commented out to prevent loop lag
    return true;
}

bool WireReadDataByte(uint8_t reg, uint8_t &val) {
    if (!WireWriteByte(reg)) {
        return false;
    }   
    Wire.requestFrom(LINE_FOLLOWER_I2C_ADDR, 1);
    while (Wire.available()) {
        val = Wire.read();
    }   
    return true;
}

// ==========================================
// Setup & Main Loop
// ==========================================
void setup() {
  Serial.begin(9600);
  Wire.begin(); // Initialize I2C bus
  
  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
  leftMotor.run(RELEASE);
  rightMotor.run(RELEASE);
  
  delay(2000); // 2-second delay to place the robot on the track
}

void loop() {
  int error = readSensors();
  
  // Calculate PID
  P = error;
  I = I + error;
  D = error - previousError;
  
  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  // Calculate new motor speeds
  int leftMotorSpeed = baseSpeed + PID_value;
  int rightMotorSpeed = baseSpeed - PID_value;

  // --- DYNAMIC MOTOR CONTROL ---
  
  // Left Motor Control
  if (leftMotorSpeed >= 0) {
    leftMotor.run(FORWARD);
    if (leftMotorSpeed > 255) leftMotorSpeed = 255;
  } else {
    leftMotor.run(BACKWARD); 
    leftMotorSpeed = -leftMotorSpeed; 
    if (leftMotorSpeed > 255) leftMotorSpeed = 255; 
  }
  leftMotor.setSpeed(leftMotorSpeed);

  // Right Motor Control
  if (rightMotorSpeed >= 0) {
    rightMotor.run(FORWARD);
    if (rightMotorSpeed > 255) rightMotorSpeed = 255; 
  } else {
    rightMotor.run(BACKWARD); 
    rightMotorSpeed = -rightMotorSpeed; 
    if (rightMotorSpeed > 255) rightMotorSpeed = 255; 
  }
  rightMotor.setSpeed(rightMotorSpeed);
}

// ==========================================
// Sensor Reading & Error Calculation
// ==========================================
int readSensors() {
  uint8_t data = 0;
  WireReadDataByte(1, data);  // Fetch sensor byte via I2C
  
  // Extract individual sensor states using bitwise shifts from your code
  int s1_val = data & 0x01;
  int s2_val = (data >> 1) & 0x01;
  int s3_val = (data >> 2) & 0x01;
  int s4_val = (data >> 3) & 0x01;

  int error = 0;

  // Evaluate the error based on which sensors see the line
  if      (s1_val == 1 && s2_val == 0 && s3_val == 0 && s4_val == 0) error = -4; // Extreme Left
  else if (s1_val == 1 && s2_val == 1 && s3_val == 0 && s4_val == 0) error = -2; 
  else if (s1_val == 0 && s2_val == 1 && s3_val == 0 && s4_val == 0) error = -1; 
  else if (s1_val == 0 && s2_val == 1 && s3_val == 1 && s4_val == 0) error = 0;  // Centered perfectly
  else if (s1_val == 0 && s2_val == 0 && s3_val == 1 && s4_val == 0) error = 1;  
  else if (s1_val == 0 && s2_val == 0 && s3_val == 1 && s4_val == 1) error = 2;  
  else if (s1_val == 0 && s2_val == 0 && s3_val == 0 && s4_val == 1) error = 4;  // Extreme Right
  else if (s1_val == 0 && s2_val == 0 && s3_val == 0 && s4_val == 0) {
    // Lost the line completely. Use memory to execute a hard pivot turn.
    if (previousError < 0) error = -5; // Spin hard right
    else if (previousError > 0) error = 5; // Spin hard left
  }

  return error;
}