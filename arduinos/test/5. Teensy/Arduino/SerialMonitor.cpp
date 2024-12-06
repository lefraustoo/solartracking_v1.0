#include <Arduino.h>

void setup()
{
  Serial.begin(9600);
}
void loop()
{
  Serial.print(128);
  delay(500);
}
