/*

TODO: Colocar la fecha y hora actuales.

*/

#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <INA226_WE.h>
#include <RTClib.h>
#include <SPI.h>

#define SERVOPINH 5
#define SERVOPINV 6
#define POTPINH A0
#define POTPINV A2
#define I2C_ADDRESS 0x40

Servo horizontal, vertical;
int servoh = 90, servov = 90, potValueH, potValueV, angleH, angleV;
float shuntVoltage_mV, loadVoltage_V, busVoltage_V, current_mA, power_mW;
INA226_WE ina226(I2C_ADDRESS);
RTC_DS3231 rtc;

unsigned long previousServoMillis = 0;
unsigned long previousINAMillis = 0;

const unsigned long servoInterval = 100;
const unsigned long INAInterval = 3000;

void ServoMovement();
void INA226multimeter();

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

    if (!rtc.begin())
    {
        Serial.println(F(">> No hay un módulo RTC conectado."));
        while (1)
            ;
    }
    // rtc.adjust(DateTime(2025, 1, 14, 16, 10, 10)); // Solo la primera vez (comentar y volver a cargar)
}

void loop()
{

    DateTime now = rtc.now();

    Serial.print(now.timestamp(DateTime::TIMESTAMP_FULL));
    Serial.print(F(" | "));

    ServoMovement();
    INA226multimeter();
}

void ServoMovement()
{
    potValueH = analogRead(POTPINH);
    potValueV = analogRead(POTPINV);

    angleH = map(potValueH, 0, 1023, 0, 180);
    angleV = map(potValueV, 0, 1023, 0, 180);

    horizontal.write(angleH);
    vertical.write(angleV);

    Serial.print(F("| Horizontal: "));
    Serial.print(angleH);
    Serial.print(F("° | Vertical: "));
    Serial.print(angleV);
    Serial.println(F("°"));
}

void INA226multimeter()
{
    ina226.readAndClearFlags();

    shuntVoltage_mV = ina226.getShuntVoltage_mV();
    busVoltage_V = ina226.getBusVoltage_V();
    current_mA = ina226.getCurrent_mA();
    power_mW = ina226.getBusPower();
    loadVoltage_V = busVoltage_V + (shuntVoltage_mV / 1000);

    Serial.print(F(" | Shunt Voltage [mV]: "));
    Serial.print(shuntVoltage_mV);
    Serial.print(F(" | Bus Voltage [V]: "));
    Serial.print(busVoltage_V);
    Serial.print(F(" | Load Voltage [V]: "));
    Serial.print(loadVoltage_V);
    Serial.print(F(" | Current [mA]: "));
    Serial.println(current_mA);

    if (!ina226.overflow)
    {
        Serial.println(F("Valores OK - no hay sobrecarga."));
    }
    else
    {
        Serial.println(F("¡Sobrecarga! Elija un rango de corriente más alto."));
    }
}

// ! End of code.