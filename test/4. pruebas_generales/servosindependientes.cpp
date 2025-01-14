#include <Arduino.h>
#include <Servo.h>

#include <Wire.h>
#include <INA226_WE.h>

#include <SD.h>

#include <RTClib.h>
#include <SPI.h>

#include <Adafruit_I2CDevice.h>

#define SERVOPINH 5 // horizontal servo
#define SERVOPINV 6 // vertical servo

Servo horizontal; // horizontal servo
Servo vertical;   // vertical servo

void setup()
{
    // !! initial general settings
    Serial.begin(9600);

    // !! initial servo settings
    horizontal.attach(SERVOPINH);
    vertical.attach(SERVOPINV);
    horizontal.write(180);
    vertical.write(95);
}

void loop()
{
    for (int i = 0; i < 180; i++)
    {
        horizontal.write(i);
        Serial.println(i);
        delay(100);
    }

    for (int i = 180; i > 0; i--)
    {
        horizontal.write(i);
        Serial.println(i);
        delay(100);
    }

    delay(1000);

    for (int i = 0; i < 180; i++)
    {
        vertical.write(i);
        Serial.println(i);
        delay(100);
    }

    for (int i = 180; i > 0; i--)
    {
        vertical.write(i);
        Serial.println(i);
        delay(100);
    }
}