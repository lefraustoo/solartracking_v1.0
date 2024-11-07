#include <Wire.h>
#include <Arduino.h>

void setup()
{
    Wire.begin();
    Serial.begin(9600);
    while (!Serial)
        ; // Espera a que el puerto serie esté listo
    Serial.println("Escaneando direcciones I2C...");
}

void loop()
{
    byte error, address;
    int dispositivos = 0;

    for (address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("Dispositivo I2C encontrado en dirección 0x");
            Serial.println(address, HEX);
            dispositivos++;
        }
        else if (error == 4)
        {
            Serial.print("Error desconocido en dirección 0x");
            Serial.println(address, HEX);
        }
    }

    if (dispositivos == 0)
        Serial.println("No se encontraron dispositivos I2C");
    else
        Serial.println("Escaneo finalizado");

    delay(1000); // Vuelve a escanear cada segundo
}
