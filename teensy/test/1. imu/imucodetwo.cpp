#include <Arduino.h>
#include "MPU6050.h"

void printMPU6050Values();

MPU6050 accelgyro;
int16_t ax, ay, az, gx, gy, gz;

void setup()
{
    // ... código existente

    // Inicializar el MPU6050
    Wire.begin();
    accelgyro.initialize();
    accelgyro.setXAccelOffset(-3120); // Configura los offsets que mencionaste
    accelgyro.setYAccelOffset(765);
    accelgyro.setZAccelOffset(5748);
    accelgyro.setXGyroOffset(15);
    accelgyro.setYGyroOffset(9);
    accelgyro.setZGyroOffset(129);

    if (accelgyro.testConnection())
    {
        Serial.println("MPU6050 connection successful");
    }
    else
    {
        Serial.println("MPU6050 connection failed");
    }
}

void loop()
{
    // Llamada para imprimir los valores del MPU6050
    printMPU6050Values();

    delay(500); // Ajusta el tiempo de delay si es necesario
}

void printmpu6050values()
{
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    Serial.print("Acelerómetro: ax = ");
    Serial.print(ax);
    Serial.print(" | ay = ");
    Serial.print(ay);
    Serial.print(" | az = ");
    Serial.println(az);

    Serial.print("Giroscopio: gx = ");
    Serial.print(gx);
    Serial.print(" | gy = ");
    Serial.print(gy);
    Serial.print(" | gz = ");
    Serial.println(gz);
}