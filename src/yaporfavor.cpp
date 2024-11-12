#include <Arduino.h>

int numero1 = 100; // contador
int numero2 = 999;
void setup()
{

    Serial.begin(57600); // Velocidad de la transmisión y apertura del puerto
}

void loop()
{

    Serial.println(numero1);
    numero1++;
    Serial.println("0");
    Serial.println(numero2);
    numero2++;
    Serial.println("1");
    delay(100);
}