#include<INA226.h>
#include <Arduino.h>
#include <Wire.h>

INA226 ina(0x40); // INA226 I2C address is 0x40

void setup() {
  Serial.begin(9600);
  Serial.println(__FILE__); // Print the file name
  Wire.begin();

  if(!ina.begin()) {
    Serial.println("Device not found. Fix and Reboot");
  }

  ina.setMaxCurrentShunt(1, 0.002); // Set the maximum current to 1A and the shunt resistor to 0.002 ohms
}

void loop() {
Serial.println("\nBus\tShunt\tCurrent\tPower");
for(int i = 0; i < 20; i++) {
  Serial.print(ina.getBusVoltage_mV());
  Serial.print("\t");
  Serial.print(ina.getShuntVoltage_mV());
  Serial.print("\t");
  Serial.print(ina.getCurrent_mA());
  Serial.print("\t");
  Serial.println(ina.getPower_mW());
  delay(1000);
}
}
