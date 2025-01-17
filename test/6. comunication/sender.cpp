#include <Arduino.h>

int sensorValue = 0;

void setup()
{
    pinMode(A0, INPUT);
    Serial.begin(9600);
}

void loop()
{
    // read the input on analog pin 0:
    sensorValue = analogRead(A0);

    // print out the value you read:
    Serial.println(sensorValue);

    delay(500);
}