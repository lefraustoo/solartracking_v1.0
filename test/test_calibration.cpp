#include <Arduino.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Wire.h>

// This test is designed to be compiled and run on an Arduino Uno.
// It verifies that the calibrateMPU() function from data_recover.cpp
// can be called and that it sets the MPU6050 offsets to non-zero values.

#include "../src/data_recover.cpp"

void setup() {
  Serial.begin(115200);
  Wire.begin();

  mpuSeguidor.initialize();

  if (mpuSeguidor.testConnection()) {
    Serial.println("MPU6050 connection successful");
    Serial.println("Calibrating MPU6050...");
    calibrateMPU();

    // Check if the offsets have been set to non-zero values
    if (mpuSeguidor.getXAccelOffset() != 0 &&
        mpuSeguidor.getYAccelOffset() != 0 &&
        mpuSeguidor.getZAccelOffset() != 0) {
      Serial.println("Calibration successful: Offsets have been set.");
    } else {
      Serial.println("Calibration failed: Offsets are still zero.");
    }

  } else {
    Serial.println("MPU6050 connection failed");
  }
}

void loop() {
  // Do nothing
}
