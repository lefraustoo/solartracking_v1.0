#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <INA226_WE.h>
#include <RTClib.h>
#include <SPI.h>
#include <Adafruit_ADS1X15.h>
#include <I2Cdev.h>
#include <MPU6050.h>

#define SERVOPINH 5
#define SERVOPINV 6

Servo horizontal;
Servo vertical;

int servoh = 90;
int servohLimitHigh = 270;
int servohLimitLow = 0;

int servov = 90;
int servovLimitHigh = 270;
int servovLimitLow = 0;

int tol = 100;
int dtime = 50;

const int ldrPins[4] = {A0, A1, A2, A3};
int ldrValues[4] = {0, 0, 0, 0};
int avgValues[4] = {0, 0, 0, 0};

unsigned long previousServoMillis = 0;
unsigned long previousINAMillis = 0;

const unsigned long servoInterval = 100;
const unsigned long INAInterval = 3000;

#define I2C_ADDRESS 0x40
INA226_WE ina226(I2C_ADDRESS);

float shuntVoltage_mV = 0.0;
float loadVoltage_V = 0.0;
float busVoltage_V = 0.0;
float current_mA = 0.0;
float power_mW = 0.0;

RTC_DS3231 rtc;

Adafruit_ADS1115 ads;
const float multiplier = 0.1875F;
const float sensitivity = 500.0;

MPU6050 sensorv(0x69);
int16_t axv, ayv, azv;

// Offsets de calibración
int axv_offset = -2632;
int ayv_offset = -938;
int azv_offset = 467;
// Variables para el filtro complementario
long tiempo_prev;
float dt;
float ang_x, ang_y;
float ang_x_prev, ang_y_prev;

Adafruit_ADS1115 ads1;

Servo servo;

void ServoMovement();
void INA226multimeter();
void Radiacion();
void Angulos();
void potenciometroext();

void setup()
{
    Serial.begin(9600);

    horizontal.attach(SERVOPINH);
    vertical.attach(SERVOPINV);
    horizontal.write(servoh);
    vertical.write(servov);

    Wire.begin();
    ina226.init();
    ina226.setResistorRange(0.1, 1.3);
    ina226.setCorrectionFactor(0.93);

    ads.begin();

    sensorv.initialize();

    if (!rtc.begin())
    {
        Serial.println("No hay un módulo RTC");
        while (1)
            ;
    }
    else
    {
        Serial.println("RTC inicializado correctamente.");
    }

    if (sensorv.testConnection())
        Serial.println("Sensores inicializados correctamente");
    else
        Serial.println("Error al iniciar los sensores");

    // Aplicar offsets a ambos sensores
    sensorv.setXAccelOffset(axv_offset);
    sensorv.setYAccelOffset(ayv_offset);
    sensorv.setZAccelOffset(azv_offset);

    if (!ads1.begin(0x48))
    {
        Serial.println("Error al inicializar el ADS1115 Potenciometro");
        while (1)
            ;
    }

    if (!ads.begin())
    {
        Serial.println("Error al inicializar el ADS1115 Radiacion");
        while (1)
            ;
    }
}

void loop()
{
    DateTime now = rtc.now();
    Serial.print(now.timestamp(DateTime::TIMESTAMP_FULL));
    Serial.print(F(", "));

    ServoMovement();
    INA226multimeter();
    Radiacion();
    Angulos();
    potenciometroext();
}

void ServoMovement()
{
    for (int i = 0; i < 4; i++)
    {
        ldrValues[i] = analogRead(ldrPins[i]);
    }

    avgValues[0] = (ldrValues[0] + ldrValues[1]) / 2;
    avgValues[1] = (ldrValues[2] + ldrValues[3]) / 2;
    avgValues[2] = (ldrValues[0] + ldrValues[2]) / 2;
    avgValues[3] = (ldrValues[1] + ldrValues[3]) / 2;

    int veg = (avgValues[0] + avgValues[1] + avgValues[2] + avgValues[3]) / 4;

    if (0 < veg && veg < 300)
    {
        tol = map(veg, 10, 300, 5, 100);
        dtime = map(veg, 10, 300, 100, 50);
    }
    else
    {
        tol = 50;
        dtime = 50;
    }

    // Calcular la diferencia entre los valores promedio de los sensores
    int dvert = avgValues[0] - avgValues[1];  // Diferencia vertical (superior - inferior)
    int dhoriz = avgValues[2] - avgValues[3]; // Diferencia horizontal (izquierda - derecha)

    // Ajuste del ángulo vertical si la diferencia supera la tolerancia
    if (abs(dvert) > tol)
    {
        servov += (dvert > 0) ? 1 : -1;                              // Incrementa o decrementa según la diferencia
        servov = constrain(servov, servovLimitLow, servovLimitHigh); // Limita los valores del servo
        vertical.write(servov);                                      // Mueve el servo vertical
    }

    // Ajuste del ángulo horizontal si la diferencia supera la tolerancia
    if (abs(dhoriz) > tol)
    {
        servoh += (dhoriz > 0) ? -1 : 1;                             // Decrementa o incrementa según la diferencia
        servoh = constrain(servoh, servohLimitLow, servohLimitHigh); // Limita los valores del servo
        horizontal.write(servoh);                                    // Mueve el servo horizontal
    }

    Serial.print(F("LDR1: "));
    Serial.print(avgValues[0]);
    Serial.print(F(", LDR2: "));
    Serial.print(avgValues[1]);
    Serial.print(F(", LDR3: "));
    Serial.print(avgValues[2]);
    Serial.print(F(", LDR4: "));
    Serial.print(avgValues[3]);
    Serial.println();

    // Pausa para permitir que los servos se muevan antes de tomar nuevas lecturas
    delay(dtime);
}

void INA226multimeter()
{
    ina226.readAndClearFlags();

    shuntVoltage_mV = ina226.getShuntVoltage_mV();
    busVoltage_V = ina226.getBusVoltage_V();
    current_mA = ina226.getCurrent_mA();
    power_mW = ina226.getBusPower();
    loadVoltage_V = busVoltage_V + (shuntVoltage_mV / 1000);

    Serial.print(F("Shunt Voltage [mV]: "));
    Serial.print(shuntVoltage_mV);
    Serial.print(F(", Bus Voltage [V]: "));
    Serial.print(busVoltage_V);
    Serial.print(F(", Load Voltage [V]: "));
    Serial.print(loadVoltage_V);
    Serial.print(F(", Current [mA]: "));
    Serial.print(current_mA);
    Serial.println();

    // if (!ina226.overflow)
    // {
    //     Serial.println(F("Valores OK - no hay sobrecarga."));
    // }
    // else
    // {
    //     Serial.println(F("¡Sobrecarga! Elija un rango de corriente más alto."));
    // }
}

void Radiacion()
{
    int16_t results = ads.readADC_Differential_0_1();
    float voltage_mV = -(results * multiplier);
    float irradiance = voltage_mV / (sensitivity / 1000.0);

    Serial.print(F("Piranometro.- Voltaje [mV]: "));
    Serial.print(voltage_mV);
    Serial.print(F(", Irradiancia [W/m^2]: "));
    Serial.print(irradiance);
    Serial.println();
}

void Angulos()
{
    sensorv.getAcceleration(&axv, &ayv, &azv);

    float accel_ang_x = atan(axv / sqrt(pow(ayv, 2) + pow(azv, 2))) * (180.0 / 3.1416);
    float accel_ang_y = atan(ayv / sqrt(pow(axv, 2) + pow(azv, 2))) * (180.0 / 3.1416);

    Serial.print(F("Ángulo de Inclinación x: "));
    Serial.print(accel_ang_x);
    Serial.print(F("°, Angulo de Inclinación y: "));
    Serial.print(accel_ang_y);
    Serial.print(F("°, "));
}

void potenciometroext()
{
    int16_t adcValue = ads1.readADC_SingleEnded(0);
    // float voltage = adcValue * (3.0 / 32767.0);
    int actualAngle = map(adcValue, 0, 21845, 0, 180);

    Serial.print("Ángulo de Giro z: ");
    Serial.print(actualAngle);
    Serial.print("°");
    Serial.println();
}