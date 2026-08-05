// Sensors.cpp - gyroscope, camera, I2C bus and microswitch inputs.

#include "Sensors.h"

MPU6050 sensor;
Pixy2 pixy;

bool cam;  // UNUSED
bool mpu;

int i2cDeviceCount = 0;

// Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z
int ax, ay, az;
int gx, gy, gz;

//Variables usadas por el filtro pasa bajos
long f_ax, f_ay, f_az;
int p_ax, p_ay, p_az;
long f_gx, f_gy, f_gz;
int p_gx, p_gy, p_gz;

//Valor de los offsets
int ax_o, ay_o, az_o;
int gx_o, gy_o, gz_o;

long tiempo_prev;
float dt;
float ang_x, ang_y, ang_z;                 // ang_z is the heading in degrees
float ang_x_prev, ang_y_prev, ang_z_prev;

volatile bool backSwitchPressed = false;
volatile bool sideSwitchPressed = false;  // UNUSED
bool lastBackSwitchState = HIGH;          // for edge detection
bool lastSideSwitchState = HIGH;          // for edge detection

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

int filterGyro(MPU6050_Base sensor) {
  sensor.getRotation(&gx, &gy, &gz);

  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();

  ang_z = ang_z_prev + (gz / 131.0) * dt; // Z-axis rotation using gyroscope only

  ang_z_prev = ang_z;
  return ang_z;
}

void resetGyroAngles() {
  ang_x = ang_y = ang_z = 0;
  ang_x_prev = ang_y_prev = ang_z_prev = 0;
  tiempo_prev = millis();
}

void onSwitchPress() {
  backSwitchPressed = true;
  resetGyroAngles();
}

void updateGyro() {
  int devices = i2cDeviceCount;

  if (devices > 0) {
    const float GYRO_SENSITIVITY = 131.0; // MPU6050 scale factor for ±250°/s

    // Get filtered Z-axis rotation rate
    float gz_filtered = filterGyro(sensor); // Should return raw gyro Z in deg/s

    // Time delta in seconds
    unsigned long now = millis();
    dt = (now - tiempo_prev) / 1000.0;
    tiempo_prev = now;


    // Integrate angular velocity to get angle
    ang_z = ang_z_prev + (gz_filtered / GYRO_SENSITIVITY) * dt;
    ang_z_prev = ang_z;
  } else {
    Serial.println("I2C FAILED or no devices responding!");

    mpu = false;
  }
}
