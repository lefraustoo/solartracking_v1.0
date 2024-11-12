#include <Arduino.h>

int datoserial1 = 10; // variable para guardar el dato que se envía
int datoserial2 = 10;
int dato = 0;

void setup()
{

    Serial.begin(57600); // Velocidad de la transmisión y apertura del puerto
}

void loop()
{

    if (Serial.available())
    {
        dato = Serial.parseInt();
        if (dato == 1)
        {
            datoserial1 = Serial.parseInt();
        }
        if (dato == 0)
        {
            datoserial2 = Serial.parseInt();
        }

        Serial.println(datoserial1);
        Serial.println(datoserial2);
    }
}
