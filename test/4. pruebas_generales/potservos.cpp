#include <Arduino.h>
#include <Servo.h>

#include <Wire.h>
#include <INA226_WE.h>

#include <SD.h>

#include <RTClib.h>
#include <SPI.h>

#include <Adafruit_I2CDevice.h>

#define SERVOPINH 6 // horizontal servo
#define SERVOPINV 5 // horizontal servo

Servo horizontal; // horizontal servo
Servo vertical;   // vertical servo

#define POTPINH A0
#define POTPINV A2

void setup()
{
    // !! initial general settings
    Serial.begin(9600);

    // !! initial servo settings
    horizontal.attach(SERVOPINH);
    vertical.attach(SERVOPINV);

    horizontal.write(90);
    vertical.write(90);

    // !! initial potenciometers settings
    pinMode(POTPINH, INPUT);
    pinMode(POTPINV, INPUT);
}

void loop()
{
    int potValueH = analogRead(POTPINH);
    int potValueV = analogRead(POTPINV);

    int angleH = map(potValueH, 0, 1023, 0, 180);
    int angleV = map(potValueV, 0, 1023, 0, 180);

    Serial.print("Horizontal: ");
    Serial.print(angleH);
    Serial.print(" Vertical: ");
    Serial.println(angleV);

    Serial.print("Potenciometer H: ");
    Serial.print(potValueH);
    Serial.print(" Potenciometer V: ");
    Serial.println(potValueV);

    horizontal.write(angleH);
    vertical.write(angleV);
}