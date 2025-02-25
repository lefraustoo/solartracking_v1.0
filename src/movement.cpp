#include <Arduino.h>
#include <Servo.h>

#define SERVOPINH 5
#define SERVOPINV 6
#define POTPINH A0
#define POTPINV A2

Servo horizontal, vertical;

int servoh = 90, servov = 90, potValueH, potValueV, angleH, angleV;

int servohLimitHigh = 270, servohLimitLow = 0, servovLimitHigh = 270, servovLimitLow = 0;

int tol = 100, dtime = 50;

const int ldrPins[4] = {A0, A1, A2, A3};
int ldrValues[4] = {0, 0, 0, 0};
int avgValues[4] = {0, 0, 0, 0};

void ServoMovement();
void PotMovement();

void setup()
{
    Serial.begin(9600);

    horizontal.attach(SERVOPINH);
    vertical.attach(SERVOPINV);
    horizontal.write(servoh);
    vertical.write(servov);

    pinMode(POTPINH, INPUT);
    pinMode(POTPINV, INPUT);
}

void loop()
{
    ServoMovement();
    PotMovement();
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

    delay(dtime);
}

void PotMovement()
{
    potValueH = analogRead(POTPINH);
    potValueV = analogRead(POTPINV);

    angleH = map(potValueH, 0, 1023, 0, 180);
    angleV = map(potValueV, 0, 1023, 0, 180);

    horizontal.write(angleH);
    vertical.write(angleV);
}