// Sensors.h - gyroscope, camera, I2C bus and microswitch inputs.
//
// The MPU6050 is used for the Z axis (heading) only. The Pixy2 finds the purple
// ball at the start and weighs orange balls per lane during the run.

#pragma once

#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include <Pixy2.h>

// ---------------------------------------------------------------------------
// Devices
// ---------------------------------------------------------------------------

extern MPU6050 sensor;
extern Pixy2 pixy;

extern bool cam;  // UNUSED. Never read or written anywhere.
extern bool mpu;  // true when the gyro answered at startup

// Number of I2C devices found by the single scan run in setup().
//
// This scan used to run on EVERY pass of loop(). Probing all 126 addresses took
// long enough to make the control period both long and irregular, and the wheel
// regulator cannot be tuned against a loop whose period keeps changing.
extern int i2cDeviceCount;

// ---------------------------------------------------------------------------
// Pixy2 colour signatures
// ---------------------------------------------------------------------------

const int purpleSignature = 2;
const int orangeSignature = 1;
const int redSignature = 3;    // UNUSED
const int ANCHO_IMAGEN = 316;  // UNUSED. Pixy2 horizontal resolution.

// ---------------------------------------------------------------------------
// Gyroscope state
//
// Only gz and the ang_z / ang_z_prev pair are actually used. The accelerometer
// values, the low pass filter values and the offsets are all UNUSED, kept
// because they document the intended full filter that was never finished.
// ---------------------------------------------------------------------------

extern int ax, ay, az;  // UNUSED
extern int gx, gy, gz;  // only gz is read

//Variables usadas por el filtro pasa bajos - all UNUSED
extern long f_ax, f_ay, f_az;
extern int p_ax, p_ay, p_az;
extern long f_gx, f_gy, f_gz;
extern int p_gx, p_gy, p_gz;

//Valor de los offsets - all UNUSED
extern int ax_o, ay_o, az_o;
extern int gx_o, gy_o, gz_o;

extern long tiempo_prev;
extern float dt;
extern float ang_x, ang_y, ang_z;                 // only ang_z is used
extern float ang_x_prev, ang_y_prev, ang_z_prev;  // only ang_z_prev is used

// ---------------------------------------------------------------------------
// Microswitch state
// ---------------------------------------------------------------------------

extern volatile bool backSwitchPressed;
extern volatile bool sideSwitchPressed;  // UNUSED. Declared but never set.
extern bool lastBackSwitchState;         // for edge detection
extern bool lastSideSwitchState;         // for edge detection

// ---------------------------------------------------------------------------
// Operations
// ---------------------------------------------------------------------------

// Scans the whole I2C bus and reports how many devices answered.
// Slow (126 transactions). Call from setup() only, never from loop().
int testI2C();

// Integrates the Z gyro into ang_z and returns it, truncated to int.
//
// Takes the sensor BY VALUE, which copies the object on every call. Wasteful
// but harmless, and preserved exactly as it was.
int filterGyro(MPU6050_Base sensor);

void resetGyroAngles();

// Called on a back microswitch press: zeroes the heading.
void onSwitchPress();

// The per loop gyro update.
//
// KNOWN QUIRK, PRESERVED DELIBERATELY: filterGyro() already integrates ang_z
// AND updates tiempo_prev. This function then integrates a second time using a
// dt computed from the timestamp filterGyro just refreshed, so dt is very close
// to zero and the second integration contributes almost nothing. The heading
// that actually matters comes from inside filterGyro(). It works, it is
// confusing to read, and correcting it would change the robot's rotations.
void updateGyro();
