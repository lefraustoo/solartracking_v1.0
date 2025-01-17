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

// Parameters for adjusting the response of the photoresistor module
int tol = 100; // The response range of illuminance, the smaller the value, the more sensitive the response, otherwise it is slow
               //(the value is 10~100, the sensitivity is different depending on the ambient light intensity, the indoor light source changes greatly, and the change is smaller under the sun)

int dtime = 50; // delay parameter. The smaller the value, the faster the response speed.
                // On the contrary, the larger the value, the slower the response speed. Unit: milliseconds General value (10~100)

// 4 connection ports for photoresistor modules
const int ldrlt = A0; // top left
const int ldrrt = A1; // top right
const int ldrld = A2; // down left
const int ldrrd = A3; // down right

// Read the illuminance values ​​of 4 photoresistor modules respectively
int lt;
int rt;
int ld;
int rd;

// Average readings from adjacent photoresistor modules
int avt;
int avd;
int avl;
int avr;

int veg;

int dvert;
int dhoriz;

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
File SeguidorSolar;
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

    // ina226.waitUntilConversionCompleted(); // ? if you comment this line the first data might be zero

    // !! initial SD settings
    Serial.print(F("Iniciando SD ..."));

    if (!SD.begin(10))
    {
        Serial.println("Error al inicializar la SD.");
        return;
    }

    SeguidorSolar = SD.open("datas.txt", FILE_WRITE);

    if (!SeguidorSolar)
    {
        Serial.println("Error al abrir el archivo dataf.txt");
        return;
    }

    Serial.println(F("Iniciado correctamente"));

    // !! initial RTC settings

    // Comprobamos si tenemos el RTC conectado
    if (!rtc.begin())
    {
        Serial.println("No hay un módulo RTC");
        while (1)
            ;
    }

    // rtc.adjust(DateTime(2025, 1, 14, 16, 10, 10)); // Ponemos en hora, solo la primera vez, luego comentar y volver a cargar.

    // !! initial MPU6050 settings
}

///////////////////////////////////   LOOP   ////////////////////////////////////
void loop()
{
    ServoMovement();
    // guardadoSD();
}

///////////////////////////////////   FUNCTIONS   ////////////////////////////////////
void ServoMovement()
{

    // Read the illuminance values ​​of 4 photoresistor modules respectively
    lt = analogRead(ldrlt); // upper left
    rt = analogRead(ldrrt); // top right
    ld = analogRead(ldrld); // down left
    rd = analogRead(ldrrd); // down right

    // Average readings from adjacent photoresistor modules
    avt = (lt + rt) / 2;
    avd = (ld + rd) / 2;
    avl = (lt + ld) / 2;
    avr = (rt + rd) / 2;

    veg = (avt + avd + avl + avr) / 4;

    // Adjust response parameters according to different light intensities
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

    // Then calculate the difference between the upper and lower rows and the average value of the left and right rows
    dvert = avt - avd;  // upper and lower rows
    dhoriz = avl - avr; // left and right rows

    // Check if the difference is within tolerance, otherwise change the vertical angle
    if (-1 * tol > dvert || dvert > tol)
    {
        if (avt > avd)
        {
            servov = ++servov;
            if (servov > servovLimitHigh)
            {
                servov = servovLimitHigh;
            }
        }
        else if (avt < avd)
        {
            servov = --servov;
            if (servov < servovLimitLow)
            {
                servov = servovLimitLow;
            }
        }
        vertical.write(servov); // If the rotation angle of the servo is opposite to the light, use (180- servov) or (servov) to change the direction
    }

    // Check if the difference is within tolerance, otherwise change the horizontal angle
    if (-1 * tol > dhoriz || dhoriz > tol)
    {
        if (avl < avr)
        {
            servoh = --servoh;
            if (servoh < servohLimitLow)
            {
                servoh = servohLimitLow;
            }
        }
        else if (avl > avr)
        {
            servoh = ++servov;
            if (servoh > servohLimitHigh)
            {
                servoh = servohLimitHigh;
            }
        }
        horizontal.write(servoh); // If the rotation angle of the servo is opposite to the light, use (180- servoh) or (servoh) to change the direction
    }

    delay(dtime);
    // delay(50);
    // guardadoSD();
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
        Serial.println("Values OK - no overflow \n");
    }
    else
    {
        Serial.println("Overflow! Choose higher current rang \n");
    }

    // delay(3000);
}

void guardadoSD()
{

    SeguidorSolar.close();
    SeguidorSolar = SD.open("datas.txt", FILE_WRITE);

    if (SeguidorSolar)
    {
        // ServoMovement();
        INA226multimeter();

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

        // Escribir datos de los sensores
        Serial.print("SensoresServo: | lt: ");
        Serial.print(lt);
        Serial.print(" | rt: ");
        Serial.print(rt);
        Serial.print(" | ld: ");
        Serial.print(ld);
        Serial.print(" | rd: ");
        Serial.println(rd);

        // Escribir datos del INA226
        Serial.print("INA226: | Shunt Voltage [mV]: ");
        Serial.print(shuntVoltage_mV);
        Serial.print(" | Bus Voltage [V]: ");
        Serial.print(busVoltage_V);
        Serial.print(" | Load Voltage [V]: ");
        Serial.print(loadVoltage_V);
        Serial.print(" | Current[mA]: ");
        Serial.println(current_mA);

        // Escribir fecha y hora
        SeguidorSolar.print(now.day());
        SeguidorSolar.print('/');
        SeguidorSolar.print(now.month());
        SeguidorSolar.print('/');
        SeguidorSolar.print(now.year());
        SeguidorSolar.print(' ');

        SeguidorSolar.print(now.hour());
        SeguidorSolar.print(':');
        SeguidorSolar.print(now.minute());
        SeguidorSolar.print(':');
        SeguidorSolar.println(now.second());

        /////////////////////////////////////// GUARDADO DE DATOS ///////////////////////////////////////

        // Escribir datos de los sensores
        SeguidorSolar.print("SensoresServo: | lt: ");
        SeguidorSolar.print(lt);
        SeguidorSolar.print(" | rt: ");
        SeguidorSolar.print(rt);
        SeguidorSolar.print(" | ld: ");
        SeguidorSolar.print(ld);
        SeguidorSolar.print(" | rd: ");
        SeguidorSolar.println(rd);

        // Escribir datos del INA226
        SeguidorSolar.print("INA226: | Shunt Voltage [mV]: ");
        SeguidorSolar.print(shuntVoltage_mV);
        SeguidorSolar.print(" | Bus Voltage [V]: ");
        SeguidorSolar.print(busVoltage_V);
        SeguidorSolar.print(" | Load Voltage [V]: ");
        SeguidorSolar.print(loadVoltage_V);
        SeguidorSolar.print(" | Current[mA]: ");
        SeguidorSolar.println(current_mA);

        SeguidorSolar.flush();
        SeguidorSolar.close();
    }
    else
    {
        Serial.println("Error al escribir en la SD");
    }

    delay(3000);
}

// ! End of code.