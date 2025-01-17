#include <Wire.h>
#include <RTClib.h>
#include <SD.h>
#include <Adafruit_INA226.h>
#include <Servo.h>

// Pines y objetos
RTC_DS1307 rtc;
Adafruit_INA226 ina226;
File SeguidorSolar;
Servo servov, servoh;

// Pines de los LDR
const int ldrlt = A0; // LDR top-left
const int ldrrt = A1; // LDR top-right
const int ldrld = A2; // LDR down-left
const int ldrrd = A3; // LDR down-right

// Variables
int lt, rt, ld, rd;
int avt, avd, avl, avr;
int veg;
int servovLimitLow = 0;
int servovLimitHigh = 180;
int servohLimitLow = 0;
int servohLimitHigh = 180;
int dtime = 1000;

unsigned long lastUpdateTime = 0;

// 🔧 **Función para leer los valores de los LDR**
void readLDR()
{
    lt = analogRead(ldrlt);
    rt = analogRead(ldrrt);
    ld = analogRead(ldrld);
    rd = analogRead(ldrrd);

    avt = (lt + rt) / 2;
    avd = (ld + rd) / 2;
    avl = (lt + ld) / 2;
    avr = (rt + rd) / 2;
}

// 🔧 **Función para mover los servos**
void ServoMovement()
{
    if (millis() - lastUpdateTime > dtime)
    {
        readLDR();
        veg = (avt + avd + avl + avr) / 4;

        if (avt > avd)
        {
            servov.write(servov.read() + 1);
        }
        else if (avt < avd)
        {
            servov.write(servov.read() - 1);
        }

        if (avl > avr)
        {
            servoh.write(servoh.read() - 1);
        }
        else if (avl < avr)
        {
            servoh.write(servoh.read() + 1);
        }

        lastUpdateTime = millis();
    }
}

// 🔧 **Función para lectura del INA226**
void INA226multimeter()
{
    if (!ina226.waitUntilConversionCompleted())
    {
        Serial.println("Esperando conversión...");
        return;
    }

    float shuntVoltage_mV = ina226.getShuntVoltage_mV();
    float busVoltage_V = ina226.getBusVoltage_V();
    float current_mA = ina226.getCurrent_mA();
    float power_mW = ina226.getBusPower();
    float loadVoltage_V = busVoltage_V + (shuntVoltage_mV / 1000);

    Serial.print("Shunt Voltage (mV): ");
    Serial.println(shuntVoltage_mV);
    Serial.print("Bus Voltage (V): ");
    Serial.println(busVoltage_V);
    Serial.print("Current (mA): ");
    Serial.println(current_mA);
    Serial.print("Power (mW): ");
    Serial.println(power_mW);
    Serial.print("Load Voltage (V): ");
    Serial.println(loadVoltage_V);
}

// 🔧 **Función para guardar datos en la tarjeta SD**
void guardadoSD()
{
    SeguidorSolar.close();
    SeguidorSolar = SD.open("datas.txt", FILE_WRITE);

    if (SeguidorSolar)
    {
        SeguidorSolar.println("Datos del Seguidor Solar");
        SeguidorSolar.flush();
        SeguidorSolar.close();
    }
    else
    {
        Serial.println("Error al abrir el archivo datas.txt");
    }
}

// 🔧 **Setup del programa**
void setup()
{
    Serial.begin(9600);

    // Inicialización del RTC
    if (!rtc.begin())
    {
        Serial.println("No hay un módulo RTC");
        while (1)
            ;
    }
    else
    {
        DateTime now = rtc.now();
        Serial.print("Fecha y hora actual: ");
        Serial.println(now.timestamp(DateTime::TIMESTAMP_FULL));
    }

    // Inicialización del INA226
    if (!ina226.begin())
    {
        Serial.println("Error al inicializar el INA226");
        while (1)
            ;
    }

    // Inicialización de la SD
    if (!SD.begin(10))
    {
        Serial.println("Error al inicializar la SD");
        while (1)
            ;
    }

    // Inicialización de los servos
    servov.attach(9);
    servoh.attach(10);
    servov.write(90);
    servoh.write(90);
}

// 🔧 **Loop del programa**
void loop()
{
    ServoMovement();
    INA226multimeter();
    guardadoSD();
}
