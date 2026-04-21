// Motor A pins
const int ena = 23;
const int in1 = 16;
const int in2 = 4;

// Motor B pins
const int enb = 22;
const int in3 = 2;
const int in4 = 15;

// PWM Settings
const int freq = 30000;
const int resolution = 8; // 8-bit resolution (0-255)

void setup() {
  // Set direction pins as outputs
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // NEW API: ledcAttach(pin, frequency, resolution)
  // This replaces ledcSetup, ledcAttachPin, and channels!
  ledcAttach(ena, freq, resolution);
  ledcAttach(enb, freq, resolution);
}

void moveForward(int speed) {
  // Motor A
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  ledcWrite(ena, speed); // Use the pin number directly now

  // Motor B
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  ledcWrite(enb, speed);
}

void moveBackward(int speed) {
  // Motor A
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  ledcWrite(ena, speed);

  // Motor B
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  ledcWrite(enb, speed);
}

void stopMotors() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  ledcWrite(ena, 0);
  ledcWrite(enb, 0);
}

void loop() {
  moveForward(200);
  delay(2000);
  
  stopMotors();
  delay(1000);
  
  moveBackward(150);
  delay(2000);
  
  stopMotors();
  delay(1000);
}