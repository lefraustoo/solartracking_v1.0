/*

 TODO: Inclinacion en x y giro en z


*/

// Librerias
#include <Arduino.h> // Librería principal de Arduino
#include <Servo.h>   // Librería para controlar servos
#include "I2Cdev.h"  // Librería para comunicación I2C
#include "Wire.h"    // Librería para comunicación I2C
#include "MPU6050.h" // Librería para el sensor MPU6050
#include "INA226.h"  // Librería para el sensor INA226

// Calibración del sensor MPU6050
void CalibrationIMU();

void meansensors();
void calibration();

// Change this 3 variables if you want to fine tune the skecth to your needs.
int buffersize = 1000; // Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone = 8; // Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone = 1; // Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

// default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
// MPU6050 accelgyro;
MPU6050 accelgyro(0x68); // <-- use for AD0 high

int16_t ax, ay, az, gx, gy, gz;

int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 0;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

// Movimiento de los ejes
void ServoMovement();

int tol = 100; // The response range of illuminance, the smaller the value, the more sensitive the response, otherwise it is slow
               //(the value is 10~100, the sensitivity is different depending on the ambient light intensity, the indoor light source changes greatly, and the change is smaller under the sun)

int dtime = 100; // delay parameter. The smaller the value, the faster the response speed.
                 // On the contrary, the larger the value, the slower the response speed. Unit: milliseconds General value (10~100)

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

// 4 connection ports for photoresistor modules
const int ldrlt = A0; // top left
const int ldrrt = A1; // top right
const int ldrld = A2; // down left
const int ldrrd = A3; // down right

void ina226reading();

INA226 ina(0x40); // INA226 I2C address is 0x40

void inclinaciones();
// La dirección del MPU6050 puede ser 0x68 o 0x69, dependiendo
// del estado de AD0. Si no se especifica, 0x68 estará implicito
MPU6050 sensor;

long tiempo_prev, dt;
float girosc_ang_x, girosc_ang_y;
float girosc_ang_x_prev, girosc_ang_y_prev;

void setup()
{

    // initialize serial communication
    Serial.begin(115200);

    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // Movimiento de los servos
    horizontal.attach(SERVOPINH);
    vertical.attach(SERVOPINV);
    horizontal.write(servoh);
    vertical.write(servov);

    // Inicialización del sensor MPU6050

    // COMMENT NEXT LINE IF YOU ARE USING ARDUINO DUE
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Leonardo measured 250kHz.

    // initialize device
    accelgyro.initialize();

    // start message
    Serial.println("\nMPU6050 Calibration Sketch");
    delay(2000);
    Serial.println("\nYour MPU6050 should be placed in horizontal position, with package letters facing up. \nDon't touch it until you see a finish message.\n");
    delay(3000);
    // verify connection
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    delay(1000);
    // reset offsets
    accelgyro.setXAccelOffset(0);
    accelgyro.setYAccelOffset(0);
    accelgyro.setZAccelOffset(0);
    accelgyro.setXGyroOffset(0);
    accelgyro.setYGyroOffset(0);
    accelgyro.setZGyroOffset(0);

    // Inicialización del sensor INA226

    Serial.println(__FILE__); // Print the file name

    if (!ina.begin())
    {
        Serial.println("Device not found. Fix and Reboot");
    }

    ina.setMaxCurrentShunt(1, 0.002); // Set the maximum current to 1A and the shunt resistor to 0.002 ohms

    // Inicialización del sensor MPU6050

    if (sensor.testConnection())
        Serial.println("Sensor iniciado correctamente");
    else
        Serial.println("Error al iniciar el sensor");

    if (sensor.testConnection())
        Serial.println("Sensor iniciado correctamente");
    else
        Serial.println("Error al iniciar el sensor");
    tiempo_prev = millis();
}

void loop()
{
    // CalibrationIMU();
    ServoMovement();
    ina226reading();
    inclinaciones();
}

// Definición de funciones
void ServoMovement()
{
    // Read the illuminance values ​​of 4 photoresistor modules respectively
    int lt = analogRead(ldrlt); // upper left
    int rt = analogRead(ldrrt); // top right
    int ld = analogRead(ldrld); // down left
    int rd = analogRead(ldrrd); // down right

    // Print the illuminance values ​​of 4 photoresistor modules
    Serial.print("lt:");
    Serial.print(lt);
    Serial.print(" ");
    Serial.print("rt:");
    Serial.print(rt);
    Serial.print(" ");
    Serial.print("ld:");
    Serial.print(ld);
    Serial.print(" ");
    Serial.print("rd:");
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
        dtime = 10;
    }

    // Print the average value of the illuminance of the 4 photoresistor modules
    Serial.print("veg= ");
    Serial.println(veg);
    Serial.print("tol= ");
    Serial.println(tol);
    Serial.print("dtime= ");
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

void CalibrationIMU()
{
    if (state == 0)
    {
        Serial.println("\nReading sensors for first time...");
        meansensors();
        state++;
        delay(1000);
    }

    if (state == 1)
    {
        Serial.println("\nCalculating offsets...");
        calibration();
        state++;
        delay(1000);
    }

    if (state == 2)
    {
        meansensors();
        Serial.println("\nFINISHED!");
        Serial.print("\nSensor readings with offsets:\t");
        Serial.print(mean_ax);
        Serial.print("\t");
        Serial.print(mean_ay);
        Serial.print("\t");
        Serial.print(mean_az);
        Serial.print("\t");
        Serial.print(mean_gx);
        Serial.print("\t");
        Serial.print(mean_gy);
        Serial.print("\t");
        Serial.println(mean_gz);
        Serial.print("Your offsets:\t");
        Serial.print(ax_offset);
        Serial.print("\t");
        Serial.print(ay_offset);
        Serial.print("\t");
        Serial.print(az_offset);
        Serial.print("\t");
        Serial.print(gx_offset);
        Serial.print("\t");
        Serial.print(gy_offset);
        Serial.print("\t");
        Serial.println(gz_offset);
        Serial.println("\nData is printed as: acelX acelY acelZ giroX giroY giroZ");
        Serial.println("Check that your sensor readings are close to 0 0 16384 0 0 0");
        Serial.println("If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)");
        while (1)
            ;
    }
}

void meansensors()
{
    long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;

    while (i < (buffersize + 101))
    {
        // read raw accel/gyro measurements from device
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        if (i > 100 && i <= (buffersize + 100))
        { // First 100 measures are discarded
            buff_ax = buff_ax + ax;
            buff_ay = buff_ay + ay;
            buff_az = buff_az + az;
            buff_gx = buff_gx + gx;
            buff_gy = buff_gy + gy;
            buff_gz = buff_gz + gz;
        }
        if (i == (buffersize + 100))
        {
            mean_ax = buff_ax / buffersize;
            mean_ay = buff_ay / buffersize;
            mean_az = buff_az / buffersize;
            mean_gx = buff_gx / buffersize;
            mean_gy = buff_gy / buffersize;
            mean_gz = buff_gz / buffersize;
        }
        i++;
        delay(2); // Needed so we don't get repeated measures
    }
}

void calibration()
{
    ax_offset = -mean_ax / 8;
    ay_offset = -mean_ay / 8;
    az_offset = (16384 - mean_az) / 8;

    gx_offset = -mean_gx / 4;
    gy_offset = -mean_gy / 4;
    gz_offset = -mean_gz / 4;
    while (1)
    {
        int ready = 0;
        accelgyro.setXAccelOffset(ax_offset);
        accelgyro.setYAccelOffset(ay_offset);
        accelgyro.setZAccelOffset(az_offset);

        accelgyro.setXGyroOffset(gx_offset);
        accelgyro.setYGyroOffset(gy_offset);
        accelgyro.setZGyroOffset(gz_offset);

        meansensors();
        Serial.println("...");

        if (abs(mean_ax) <= acel_deadzone)
            ready++;
        else
            ax_offset = ax_offset - mean_ax / acel_deadzone;

        if (abs(mean_ay) <= acel_deadzone)
            ready++;
        else
            ay_offset = ay_offset - mean_ay / acel_deadzone;

        if (abs(16384 - mean_az) <= acel_deadzone)
            ready++;
        else
            az_offset = az_offset + (16384 - mean_az) / acel_deadzone;

        if (abs(mean_gx) <= giro_deadzone)
            ready++;
        else
            gx_offset = gx_offset - mean_gx / (giro_deadzone + 1);

        if (abs(mean_gy) <= giro_deadzone)
            ready++;
        else
            gy_offset = gy_offset - mean_gy / (giro_deadzone + 1);

        if (abs(mean_gz) <= giro_deadzone)
            ready++;
        else
            gz_offset = gz_offset - mean_gz / (giro_deadzone + 1);

        if (ready == 6)
            break;
    }
}

void ina226reading()
{
    Serial.println("\nBus\tShunt\tCurrent\tPower");
    Serial.print(ina.getBusVoltage_mV());
    Serial.print("\t");
    Serial.print(ina.getShuntVoltage_mV());
    Serial.print("\t");
    Serial.print(ina.getCurrent_mA());
    Serial.print("\t");
    Serial.println(ina.getPower_mW());
}

void inclinaciones()
{
    // Leer las aceleraciones
    sensor.getAcceleration(&ax, &ay, &az);
    // Calcular los angulos de inclinacion:
    float accel_ang_x = atan(ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180.0 / 3.14);
    float accel_ang_y = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * (180.0 / 3.14);
    // Mostrar los angulos separadas por un [tab]
    Serial.print("Inclinacion en X: ");
    Serial.print(accel_ang_x);
    Serial.print("\tInclinacion en Y:");
    Serial.println(accel_ang_y);
    delay(10);
}

void giros()
{
    // Leer las velocidades angulares
    sensor.getRotation(&gx, &gy, &gz);

    // Calcular los angulos rotacion:

    dt = millis() - tiempo_prev;
    tiempo_prev = millis();

    girosc_ang_x = (gx / 131) * dt / 1000.0 + girosc_ang_x_prev;
    girosc_ang_y = (gy / 131) * dt / 1000.0 + girosc_ang_y_prev;

    girosc_ang_x_prev = girosc_ang_x;
    girosc_ang_y_prev = girosc_ang_y;

    // Mostrar los angulos separadas por un [tab]
    Serial.print("Rotacion en X:  ");
    Serial.print(girosc_ang_x);
    Serial.print("tRotacion en Y: ");
    Serial.println(girosc_ang_y);

    delay(100);
}