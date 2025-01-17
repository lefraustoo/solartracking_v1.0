#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <INA226_WE.h>
#include <SD.h>
#include <RTClib.h>
#include <SPI.h>

#define SERVOPINH 5
#define SERVOPINV 6
#define POTPINH A0
#define POTPINV A2
#define I2C_ADDRESS 0x40

Servo horizontal, vertical;
int servoh = 90, servov = 90;
float shuntVoltage_mV, loadVoltage_V, busVoltage_V, current_mA, power_mW;
File SeguidorSolar;
INA226_WE ina226(I2C_ADDRESS);
RTC_DS3231 rtc;

unsigned long previousServoMillis = 0;
unsigned long previousINAMillis = 0;
unsigned long previousSDMillis = 0;

const unsigned long servoInterval = 100;
const unsigned long INAInterval = 3000;
const unsigned long SDInterval = 5000;

void ServoMovement();
void INA226multimeter();
void guardadoSD();

void setup()
{
    Serial.begin(9600);
    horizontal.attach(SERVOPINH);
    vertical.attach(SERVOPINV);
    horizontal.write(servoh);
    vertical.write(servov);
    pinMode(POTPINH, INPUT);
    pinMode(POTPINV, INPUT);
    Wire.begin();
    ina226.init();
    ina226.setResistorRange(0.1, 1.3);
    ina226.setCorrectionFactor(0.93);

    if (!SD.begin(10))
    {
        Serial.println(F("Error al inicializar la SD."));
        return;
    }
    SeguidorSolar = SD.open("dataf.txt", FILE_WRITE);
    if (!SeguidorSolar)
    {
        Serial.println(F("Error al abrir el archivo dataf.txt"));
        return;
    }
    Serial.println(F("SD inicializada correctamente."));

    if (!rtc.begin())
    {
        Serial.println(F("No hay un módulo RTC conectado."));
        while (1)
            ;
    }
}

void loop()
{
    unsigned long currentMillis = millis();

    if (currentMillis - previousServoMillis >= servoInterval)
    {
        previousServoMillis = currentMillis;
        ServoMovement();
    }

    if (currentMillis - previousINAMillis >= INAInterval)
    {
        previousINAMillis = currentMillis;
        INA226multimeter();
    }

    if (currentMillis - previousSDMillis >= SDInterval)
    {
        previousSDMillis = currentMillis;
        guardadoSD();
    }
}

void ServoMovement()
{
    int potValueH = analogRead(POTPINH);
    int potValueV = analogRead(POTPINV);

    int angleH = map(potValueH, 0, 1023, 0, 180);
    int angleV = map(potValueV, 0, 1023, 0, 180);

    horizontal.write(angleH);
    vertical.write(angleV);

    Serial.print(F("Horizontal: "));
    Serial.print(angleH);
    Serial.print(F(" | Vertical: "));
    Serial.println(angleV);
}

void INA226multimeter()
{
    ina226.readAndClearFlags();

    shuntVoltage_mV = ina226.getShuntVoltage_mV();
    busVoltage_V = ina226.getBusVoltage_V();
    current_mA = ina226.getCurrent_mA();
    power_mW = ina226.getBusPower();
    loadVoltage_V = busVoltage_V + (shuntVoltage_mV / 1000);

    if (!ina226.overflow)
    {
        Serial.println(F("Valores OK - no hay sobrecarga."));
    }
    else
    {
        Serial.println(F("¡Sobrecarga! Elija un rango de corriente más alto."));
    }
}

void guardadoSD()
{
    SeguidorSolar = SD.open("dataf.txt", FILE_WRITE);

    if (SeguidorSolar)
    {
        DateTime now = rtc.now();
        SeguidorSolar.print(now.timestamp(DateTime::TIMESTAMP_FULL));
        SeguidorSolar.print(F(" | Shunt Voltage [mV]: "));
        SeguidorSolar.print(shuntVoltage_mV);
        SeguidorSolar.print(F(" | Bus Voltage [V]: "));
        SeguidorSolar.print(busVoltage_V);
        SeguidorSolar.print(F(" | Load Voltage [V]: "));
        SeguidorSolar.print(loadVoltage_V);
        SeguidorSolar.print(F(" | Current [mA]: "));
        SeguidorSolar.println(current_mA);

        SeguidorSolar.flush();
        SeguidorSolar.close();
        Serial.println(F("Datos guardados en la SD."));
    }
    else
    {
        Serial.println(F("Error al escribir en la SD."));
    }
}
