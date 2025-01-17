#include <Arduino.h>
#include <Servo.h>

#include <Wire.h>
#include <INA226_WE.h>

#include <SD.h>

#include <RTClib.h>
#include <SPI.h>

#include <Adafruit_I2CDevice.h>

#define SERVOPINH 6 // horizontal servo

Servo horizontal; // horizontal servo

void setup()
{
    // !! initial general settings
    Serial.begin(9600);

    // !! initial servo settings
    horizontal.attach(SERVOPINH);
}

void loop()
{
    for (int i = 0; i < 180; i++)
    {
        horizontal.write(i);
        Serial.println(i);
        delay(10);
    }

    delay(1000);

    for (int i = 180; i > 0; i--)
    {
        horizontal.write(i);
        Serial.println(i);
        delay(10);
    }

    delay(1000);
}