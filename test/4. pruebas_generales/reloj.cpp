#include <Arduino.h>
#include <RTClib.h>
#include <Wire.h>
#include <SPI.h>

// Declaramos un RTC DS3231
RTC_DS3231 rtc;

void setup()
{
  Serial.begin(9600);

  delay(3000);

  // Comprobamos si tenemos el RTC conectado
  if (!rtc.begin())
  {
    Serial.println("No hay un módulo RTC");
    while (1)
      ;
  }

  // Ponemos en hora, solo la primera vez, luego comentar y volver a cargar.
  // Ponemos en hora con los valores de la fecha y la hora en que el sketch ha sido compilado.
  // rtc.adjust(DateTime(2025, 1, 14, 15, 47, 50));
}

void loop()
{
  DateTime now = rtc.now();

  Serial.print(now.day());
  Serial.print('/');
  Serial.print(now.month());
  Serial.print('/');
  Serial.print(now.year());
  Serial.print(" ");
  Serial.print(now.hour());
  Serial.print(':');
  Serial.print(now.minute());
  Serial.print(':');
  Serial.print(now.second());
  Serial.println();
  delay(3000);
}
