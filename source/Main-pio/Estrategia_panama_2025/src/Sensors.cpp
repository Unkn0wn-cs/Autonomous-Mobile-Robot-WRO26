// Sensors.cpp - camera, I2C bus scan and microswitch inputs.

#include "Sensors.h"
#include "Heading.h"

Pixy2 pixy;

volatile bool backSwitchPressed = false;
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

void onSwitchPress() {
  backSwitchPressed = true;
  headingZero();
}
