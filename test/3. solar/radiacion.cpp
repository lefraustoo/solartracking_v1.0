#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "Adafruit_ADS1X15.h"

Adafruit_ADS1115 ads;
const float multiplier = 0.1875F; 
const float sensitivity = 500.0; 

void setup(void)
{
  Serial.begin(9600);
  ads.begin();
}

void loop(void)
{
  // Lee el valor diferencial entre A0 y A1
  int16_t results = ads.readADC_Differential_0_1();

  // Calcula el voltaje en mV
  float voltage_mV = -(results * multiplier);

  // Convierte el voltaje en irradiancia (W/m²)
  float irradiance = voltage_mV / (sensitivity / 1000.0);

  // Imprime los resultados
  Serial.print("Voltaje (mV): ");
  Serial.print(voltage_mV);
  Serial.print("\tIrradiancia (W/m^2): ");
  Serial.println(irradiance);

  delay(1000);
}

