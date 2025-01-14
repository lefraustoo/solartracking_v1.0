///////////////////////////////////   LIBRARIES   ////////////////////////////////////
#include <Arduino.h> // Librería principal de Arduino
#include <Servo.h>   // Librería para controlar servos

#include <Wire.h>      // Librería para comunicación I2C
#include <INA226_WE.h> // Librería para controlar el módulo INA226

#include <SD.h> // Librería para comunicación con tarjetas SD

///////////////////////////////////   SERVOS   ////////////////////////////////////
#define SERVOPINH 5 // horizontal servo
#define SERVOPINV 6 // vertical servo

// Horizontal servo settings
Servo horizontal;          // horizontal servo
int servoh = 90;           // Initialize angle
int servohLimitHigh = 270; // The maximum angle of rotation in the horizontal direction
int servohLimitLow = 0;    // The minimum angle of rotation in the horizontal direction

// Vertical Servo Settings
Servo vertical;            // vertical servo
int servov = 95;           // Initialize angle
int servovLimitHigh = 270; // The maximum angle of rotation in the vertical direction
int servovLimitLow = 0;    // The minimum angle of rotation in the vertical direction

///////////////////////////////////   INA226   ////////////////////////////////////
void INA226multimeter();

#define I2C_ADDRESS 0x40
INA226_WE ina226 = INA226_WE(I2C_ADDRESS);

float shuntVoltage_mV = 0.0;
float loadVoltage_V = 0.0;
float busVoltage_V = 0.0;
float current_mA = 0.0;
float power_mW = 0.0;

///////////////////////////////////   MPU6050   ////////////////////////////////////

/////////////////////////////////////   SD   //////////////////////////////////////
File SolarFijo;
void guardadoSD();

///////////////////////////////////   SETUP   ////////////////////////////////////
void setup()
{
    // !! initial general settings
    Serial.begin(9600);

    // !! initial servo settings
    horizontal.attach(SERVOPINH);
    vertical.attach(SERVOPINV);
    horizontal.write(servoh);
    vertical.write(servov);

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

    Serial.println("-- INA226 Current Sensor Example Sketch - Continuous -- \n\n"); // ? uncomment for change of default

    // ina226.waitUntilConversionCompleted(); // ? if you comment this line the first data might be zero

    // !! initial MPU6050 settings

    // !! initial SD settings
    Serial.print(F("Iniciando SD ..."));

    if (!SD.begin(9))
    { // Inicializar SD en pin 9
        Serial.println(F("Error al iniciar"));
        return;
    }

    Serial.println(F("Iniciado correctamente"));
}

///////////////////////////////////   LOOP   ////////////////////////////////////
void loop()
{
    INA226multimeter();
    guardadoSD();
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

    delay(3000);
}

void guardadoSD()
{
    // Abrir el archivo y escribir el valor
    SolarFijo = SD.open("dataf.txt", FILE_WRITE);

    if (SolarFijo)
    {

        SolarFijo.println("INA226: | Shunt Voltage [mV]: " + String(shuntVoltage_mV) + " | Bus Voltage [V]: " + String(busVoltage_V) + " | Load Voltage [V]: " + String(loadVoltage_V) + " | Current[mA]: " + String(current_mA) + " | Bus Power [mW]: " + String(power_mW) + " | \n");
        Serial.println("INA226: | Shunt Voltage [mV]: " + String(shuntVoltage_mV) + " | Bus Voltage [V]: " + String(busVoltage_V) + " | Load Voltage [V]: " + String(loadVoltage_V) + " | Current[mA]: " + String(current_mA) + " | Bus Power [mW]: " + String(power_mW) + " | \n");

        if (!ina226.overflow)
        {
            SolarFijo.println("Values OK - no overflow ");
            Serial.println("Values OK - no overflow ");
        }
        else
        {
            SolarFijo.println("Overflow! Choose higher current rang");
            Serial.println("Overflow! Choose higher current rang");
        }

        SolarFijo.close();
    }
    else
    {
        Serial.println("Error al abrir el archivo");
    }

    delay(500);
}

// ! End of code.