///////////////////////////////////   LIBRARIES   ////////////////////////////////////
#include <Arduino.h> // Librería principal de Arduino
#include <Servo.h>   // Librería para controlar servos

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

///////////////////////////////////   INA   ////////////////////////////////////

///////////////////////////////////   SETUP   ////////////////////////////////////
void setup()
{
  // initialize serial communication
  Serial.begin(115200);

  // initial servo settings
  horizontal.attach(SERVOPINH);
  vertical.attach(SERVOPINV);
  horizontal.write(servoh);
  vertical.write(servov);
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

// End of code.
