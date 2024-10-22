///////////////////////////////////   LIBRARIES   ////////////////////////////////////
#include <Arduino.h> // Librería principal de Arduino
#include <Servo.h>   // Librería para controlar servos

#include <Wire.h>
#include <INA226_WE.h>

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
int servov = 95;           // Initialize angle
int servovLimitHigh = 270; // The maximum angle of rotation in the vertical direction
int servovLimitLow = 0;    // The minimum angle of rotation in the vertical direction

// Parameters for adjusting the response of the photoresistor module
int tol = 100; // The response range of illuminance, the smaller the value, the more sensitive the response, otherwise it is slow
               //(the value is 10~100, the sensitivity is different depending on the ambient light intensity, the indoor light source changes greatly, and the change is smaller under the sun)

int dtime = 100; // delay parameter. The smaller the value, the faster the response speed.
                 // On the contrary, the larger the value, the slower the response speed. Unit: milliseconds General value (10~100)

// 4 connection ports for photoresistor modules
const int ldrlt = A0; // top left
const int ldrrt = A1; // top right
const int ldrld = A2; // down left
const int ldrrd = A3; // down right

///////////////////////////////////   INA226   ////////////////////////////////////
void INA226multimeter();

#define I2C_ADDRESS 0x40

INA226_WE ina226 = INA226_WE(I2C_ADDRESS);

///////////////////////////////////   SETUP   ////////////////////////////////////
void setup()
{
  // !! initialize serial communication
  Serial.begin(9600);

  // !! initial servo settings
  horizontal.attach(SERVOPINH);
  vertical.attach(SERVOPINV);
  horizontal.write(servoh);
  vertical.write(servov);

  // !! initial INA226 settings
  while (!Serial)
    ; // wait until serial comes up on Arduino Leonardo or MKR WiFi 1010
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

  // ina226.setAverage(AVERAGE_16); // choose mode and uncomment for change of default

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

  // ina226.setConversionTime(CONV_TIME_1100); //choose conversion time and uncomment for change of default

  /* Set measure mode
    POWER_DOWN - INA226 switched off
    TRIGGERED  - measurement on demand
    CONTINUOUS  - continuous measurements (default)*/

  // ina226.setMeasureMode(CONTINUOUS); // choose mode and uncomment for change of default

  /* Set Resistor and Current Range
     if resistor is 5.0 mOhm, current range is up to 10.0 A
     default is 100 mOhm and about 1.3 A*/

  ina226.setResistorRange(0.1, 1.3); // choose resistor 0.1 Ohm and gain range up to 1.3A

  /* If the current values delivered by the INA226 differ by a constant factor
     from values obtained with calibrated equipment you can define a correction factor.
     Correction factor = current delivered from calibrated equipment / current delivered by INA226 */

  ina226.setCorrectionFactor(0.93);

  Serial.println("INA226 Current Sensor Example Sketch - Continuous");

  ina226.waitUntilConversionCompleted(); // if you comment this line the first data might be zero

  // !! initial MPU6050 settings
  
}

///////////////////////////////////   LOOP   ////////////////////////////////////
void loop()
{
  ServoMovement();
}

///////////////////////////////////   FUNCTIONS   ////////////////////////////////////
void ServoMovement()
{
  // Read the illuminance values ​​of 4 photoresistor modules respectively
  int lt = analogRead(ldrlt); // upper left
  int rt = analogRead(ldrrt); // top right
  int ld = analogRead(ldrld); // down left
  int rd = analogRead(ldrrd); // down right

  Serial.print("lt:");
  Serial.print(lt);
  Serial.print("\trt:");
  Serial.print(rt);
  Serial.print("\tld:");
  Serial.print(ld);
  Serial.print("\trd:");
  Serial.println(rd);

  // Average readings from adjacent photoresistor modules
  int avt = (lt + rt) / 2;
  int avd = (ld + rd) / 2;
  int avl = (lt + ld) / 2;
  int avr = (rt + rd) / 2;

  int veg = (avt + avd + avl + avr) / 4;

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

  Serial.print("veg= ");
  Serial.print(veg);
  Serial.print("\ttol= ");
  Serial.print(tol);
  Serial.print("\tdtime= ");
  Serial.println(dtime);

  // Then calculate the difference between the upper and lower rows and the average value of the left and right rows
  int dvert = avt - avd;  // upper and lower rows
  int dhoriz = avl - avr; // left and right rows

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
      servoh = ++servoh;
      if (servoh > servohLimitHigh)
      {
        servoh = servohLimitHigh;
      }
    }
    horizontal.write(servoh); // If the rotation angle of the servo is opposite to the light, use (180- servoh) or (servoh) to change the direction
  }

  delay(dtime);
}

void INA226multimeter()
{
  float shuntVoltage_mV = 0.0;
  float loadVoltage_V = 0.0;
  float busVoltage_V = 0.0;
  float current_mA = 0.0;
  float power_mW = 0.0;

  ina226.readAndClearFlags();
  shuntVoltage_mV = ina226.getShuntVoltage_mV();
  busVoltage_V = ina226.getBusVoltage_V();
  current_mA = ina226.getCurrent_mA();
  power_mW = ina226.getBusPower();
  loadVoltage_V = busVoltage_V + (shuntVoltage_mV / 1000);

  Serial.print("Shunt Voltage [mV]: ");
  Serial.println(shuntVoltage_mV);
  Serial.print("Bus Voltage [V]: ");
  Serial.println(busVoltage_V);
  Serial.print("Load Voltage [V]: ");
  Serial.println(loadVoltage_V);
  Serial.print("Current[mA]: ");
  Serial.println(current_mA);
  Serial.print("Bus Power [mW]: ");
  Serial.println(power_mW);

  if (!ina226.overflow)
  {
    Serial.println("Values OK - no overflow");
  }
  else
  {
    Serial.println("Overflow! Choose higher current range");
  }
  Serial.println();

  delay(3000);
}

// End of code.
