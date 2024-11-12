#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// Direcciones de los sensores
MPU6050 sensorv(0x68);  // Sensor para inclinación (acelerómetro)
MPU6050 sensorh(0x69);  // Sensor para rotación (giroscopio)

// Variables para almacenar lecturas del acelerómetro y giroscopio
int16_t axv, ayv, azv, axh, ayh, azh, gx, gy, gz;

// Offsets de calibración
int axv_offset = -2573;
int ayv_offset = -937;
int azv_offset = 4657;
int axh_offset= -923;
int ayh_offset = 853;
int azh_offset= 1593;
int gx_offset = 76;
int gy_offset = -35;
int gz_offset = 32;

// Variables para el filtro complementario
long tiempo_prev;
float dt;
float ang_x, ang_y;
float ang_x_prev, ang_y_prev;

void setup() {
  Serial.begin(57600);     // Iniciar puerto serial
  Wire.begin();            // Iniciar I2C  
  sensorv.initialize();    // Inicializar sensor de inclinación
  sensorh.initialize();    // Inicializar sensor de rotación

  if (sensorv.testConnection() && sensorh.testConnection())
    Serial.println("Sensores inicializados correctamente");
  else 
    Serial.println("Error al iniciar los sensores");

  // Aplicar offsets a ambos sensores
  sensorv.setXAccelOffset(axv_offset);
  sensorv.setYAccelOffset(ayv_offset);
  sensorv.setZAccelOffset(azv_offset);

  sensorh.setXAccelOffset(axh_offset);
  sensorh.setYAccelOffset(ayh_offset);
  sensorh.setZAccelOffset(azh_offset);
  sensorh.setXGyroOffset(gx_offset);
  sensorh.setYGyroOffset(gy_offset);
  sensorh.setZGyroOffset(gz_offset);
}

void loop() {
  // Leer las aceleraciones del sensor de inclinación
  sensorv.getAcceleration(&axv, &ayv, &azv);

  // Calcular los ángulos de inclinación con el acelerómetro
  float accel_ang_x = atan(axv / sqrt(pow(ayv, 2) + pow(azv, 2))) * (180.0 / 3.1416);
  float accel_ang_y = atan(ayv / sqrt(pow(axv, 2) + pow(azv, 2))) * (180.0 / 3.1416);

   // Leer las aceleraciones y velocidades angulares

  sensorh.getAcceleration(&axh, &ayh, &azh);
  sensorh.getRotation(&gx, &gy, &gz);
  
  dt = (millis()-tiempo_prev)/1000.0;
  tiempo_prev=millis();
  
  //Calcular los ángulos con acelerometro
  float accel_ang_a=atan(ayh/sqrt(pow(axh,2) + pow(azh,2)))*(180.0/3.14);
  float accel_ang_b=atan(-axh/sqrt(pow(ayh,2) + pow(azh,2)))*(180.0/3.14);
  
  //Calcular angulo de rotación con giroscopio y filtro complemento  
  ang_x = 0.98*(ang_x_prev+(gx/131)*dt) + 0.02*accel_ang_a;
  ang_y = 0.98*(ang_y_prev+(gy/131)*dt) + 0.02*accel_ang_b;
  
  ang_x_prev=ang_x;
  ang_y_prev=ang_y;

  // Mostrar los ángulos de inclinación calculados por el acelerómetro
  Serial.print("Inclinacion en X: ");
  Serial.print(accel_ang_x); 
  Serial.print("\tInclinacion en Y: ");
  Serial.println(accel_ang_y);

  // Mostrar los ángulos de rotación calculados con el filtro complementario
  Serial.print("Rotacion en X:  ");
  Serial.print(ang_x); 
  Serial.print("\tRotacion en Y: ");
  Serial.println(ang_y);
  delay(100);
}


