///////////////////////////////////   LIBRARIES   ////////////////////////////////////
#include <Arduino.h> // Librería principal de Arduino

#include <Servo.h> // Librería para controlar servos

#include <Wire.h>      // Librería para comunicación I2C
#include <INA226_WE.h> // Librería para controlar el módulo INA226

#include <SD.h> // Librería para comunicación con tarjetas SD

#include <RTClib.h> // Librería para controlar el módulo RTC
#include <SPI.h>    // Librería para comunicación SPI

///////////////////////////////////   SERVOS   ////////////////////////////////////

void ServoMovement();

#define SERVOPINH 5 // horizontal servo
#define SERVOPINV 6 // vertical servo

#define POTPINH A0
#define POTPINV A2

// Horizontal servo settings
Servo horizontal;          // horizontal servo
int servoh = 90;           // Initialize angle
int servohLimitHigh = 270; // The maximum angle of rotation in the horizontal direction
int servohLimitLow = 0;    // The minimum angle of rotation in the horizontal direction

// Vertical Servo Settings
Servo vertical;            // vertical servo
int servov = 90;           // Initialize angle
int servovLimitHigh = 270; // The maximum angle of rotation in the vertical direction
int servovLimitLow = 0;    // The minimum angle of rotation in the vertical direction

int Ahoriz;
int Avert;

const int potPinH = A0; // Potenciometro horizontal
const int potPinV = A1; // Potenciometro vertical

///////////////////////////////////   INA226   ////////////////////////////////////
void INA226multimeter();

#define I2C_ADDRESS 0x40
INA226_WE ina226 = INA226_WE(I2C_ADDRESS);

float shuntVoltage_mV = 0.0;
float loadVoltage_V = 0.0;
float busVoltage_V = 0.0;
float current_mA = 0.0;
float power_mW = 0.0;

/////////////////////////////////////   SD   //////////////////////////////////////
File solarfijo;
void guardadoSD();

///////////////////////////////////   RELOJ   ////////////////////////////////////
RTC_DS3231 rtc;
DateTime now = rtc.now();

///////////////////////////////////   SETUP   ////////////////////////////////////
void setup()
{
    // !! initial general settings
    Serial.begin(9600);

    // !! initial servo settings
    horizontal.attach(SERVOPINH);
    vertical.attach(SERVOPINV);

    horizontal.write(90);
    vertical.write(90);

    // !! initial potenciometers settings
    pinMode(POTPINH, INPUT);
    pinMode(POTPINV, INPUT);

    // !! initial INA226 settings
    Wire.begin();
    ina226.init();

    /* Set Number of measurements for shunt and bus voltage which shall be averaged
      Mode *     * Number of samples
      AVERAGE_1            1 (default)
      AVERAGE_4            4
      AVERAGE_16          16
      AVERAGE_64          64
      AVERAGE_128        128
      AVERAGE_256        256
      AVERAGE_512        512
      AVERAGE_1024      1024*/

    // ina226.setAverage(AVERAGE_16); // ? choose mode and uncomment for change of default

    /* Set conversion time in microseconds
       One set of shunt and bus voltage conversion will take:
       number of samples to be averaged x conversion time x 2

         Mode *         * conversion time
       CONV_TIME_140          140 µs
       CONV_TIME_204          204 µs
       CONV_TIME_332          332 µs
       CONV_TIME_588          588 µs
       CONV_TIME_1100         1.1 ms (default)
       CONV_TIME_2116       2.116 ms
       CONV_TIME_4156       4.156 ms
       CONV_TIME_8244       8.244 ms  */

    // ina226.setConversionTime(CONV_TIME_1100); // ? choose conversion time and uncomment for change of default

    /* Set measure mode
      POWER_DOWN - INA226 switched off
      TRIGGERED  - measurement on demand
      CONTINUOUS  - continuous measurements (default)*/

    // ina226.setMeasureMode(CONTINUOUS); // ? choose mode and uncomment for change of default

    /* Set Resistor and Current Range
       if resistor is 5.0 mOhm, current range is up to 10.0 A
       default is 100 mOhm and about 1.3 A*/

    ina226.setResistorRange(0.1, 1.3); // ? choose resistor 0.1 Ohm and gain range up to 1.3A

    /* If the current values delivered by the INA226 differ by a constant factor
       from values obtained with calibrated equipment you can define a correction factor.
       Correction factor = current delivered from calibrated equipment / current delivered by INA226 */

    ina226.setCorrectionFactor(0.93); // ? choose correction factor and uncomment for change of default

    // ina226.waitUntilConversionCompleted(); // ? if you comment this line the first data might be zero

    // !! initial SD settings
    Serial.print(F("Iniciando SD ..."));

    // Inicialización de la SD
    if (!SD.begin(10))
    {
        Serial.println("Error al inicializar la SD.");
        return;
    }

    // Abrir el archivo una vez
    solarfijo = SD.open("dataf.txt", FILE_WRITE);

    if (!solarfijo)
    {
        Serial.println("Error al abrir el archivo datas.txt");
        return;
    }

    Serial.println(F("Iniciado correctamente"));

    // Comprobamos si tenemos el RTC conectado
    if (!rtc.begin())
    {
        Serial.println("No hay un módulo RTC");
        while (1)
            ;
    }

    // Ponemos en hora, solo la primera vez, luego comentar y volver a cargar.
    // Ponemos en hora con los valores de la fecha y la hora en que el sketch ha sido compilado.
    // rtc.adjust(DateTime(2025, 1, 14, 16, 10, 10));
}

///////////////////////////////////   LOOP   ////////////////////////////////////
void loop()
{
    ServoMovement();
    // INA226multimeter();
    // guardadoSD();
}

///////////////////////////////////   FUNCTIONS   ////////////////////////////////////
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
        Serial.println("Values OK - no overflow \n");
    }
    else
    {
        Serial.println("Overflow! Choose higher current rang \n");
    }

    // delay(3000);
}

void ServoMovement()
{
    int potValueH = analogRead(POTPINH);
    int potValueV = analogRead(POTPINV);

    int angleH = map(potValueH, 0, 1023, 0, 180);
    int angleV = map(potValueV, 0, 1023, 0, 180);

    Serial.print("Horizontal: ");
    Serial.print(angleH);
    Serial.print(" Vertical: ");
    Serial.println(angleV);

    Serial.print("Potenciometer H: ");
    Serial.print(potValueH);
    Serial.print(" Potenciometer V: ");
    Serial.println(potValueV);

    horizontal.write(angleH);
    vertical.write(angleV);
}

void guardadoSD()
{

    // solarfijo.close();
    solarfijo = SD.open("dataf.txt", FILE_WRITE);

    if (solarfijo)
    {
        DateTime now = rtc.now();

        // Escribir fecha y hora
        Serial.print(now.day());
        Serial.print('/');
        Serial.print(now.month());
        Serial.print('/');
        Serial.print(now.year());
        Serial.print(' ');

        Serial.print(now.hour());
        Serial.print(':');
        Serial.print(now.minute());
        Serial.print(':');
        Serial.println(now.second());

        // Escribir datos del INA226
        Serial.print("INA226: | Shunt Voltage [mV]: ");
        Serial.print(shuntVoltage_mV);
        Serial.print(" | Bus Voltage [V]: ");
        Serial.print(busVoltage_V);
        Serial.print(" | Load Voltage [V]: ");
        Serial.print(loadVoltage_V);
        Serial.print(" | Current[mA]: ");
        Serial.println(current_mA);

        /////////////////////////////////////// GUARDADO DE DATOS ///////////////////////////////////////

        // Escribir fecha y hora
        solarfijo.print(now.day());
        solarfijo.print('/');
        solarfijo.print(now.month());
        solarfijo.print('/');
        solarfijo.print(now.year());
        solarfijo.print(' ');

        solarfijo.print(now.hour());
        solarfijo.print(':');
        solarfijo.print(now.minute());
        solarfijo.print(':');
        solarfijo.println(now.second());

        // Escribir datos del INA226
        solarfijo.print("INA226: | Shunt Voltage [mV]: ");
        solarfijo.print(shuntVoltage_mV);
        solarfijo.print(" | Bus Voltage [V]: ");
        solarfijo.print(busVoltage_V);
        solarfijo.print(" | Load Voltage [V]: ");
        solarfijo.print(loadVoltage_V);
        solarfijo.print(" | Current[mA]: ");
        solarfijo.println(current_mA);

        solarfijo.flush();
        solarfijo.close();
    }
    else
    {
        Serial.println("Error al escribir en la SD");
    }

    delay(3000);
}

// ! End of code.