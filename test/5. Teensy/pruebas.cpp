#include <SD.h>

File DatosSolar;

void setup()
{
    Serial.begin(9600); // Comunicación Serial
    Serial.print(F("Iniciando SD ..."));

    if (!SD.begin(9))
    { // Inicializar SD en pin 9
        Serial.println(F("Error al iniciar"));
        return;
    }

    Serial.println(F("Iniciado correctamente"));
}

void loop()
{
    // Verificar si hay datos disponibles por Serial
    if (Serial.available() > 0)
    {
        // Leer el valor completo como una cadena (String)
        String receivedValue = Serial.readStringUntil('\n');

        // Convertir la cadena a un número entero
        int valor = receivedValue.toInt();

        // Abrir el archivo y escribir el valor
        DatosSolar = SD.open("datalog.txt", FILE_WRITE);

        if (DatosSolar)
        {
            DatosSolar.print("Time (ms): ");
            DatosSolar.println(millis());
            DatosSolar.print("Potenciometro: ");
            DatosSolar.println(valor);

            Serial.print("Time (ms): ");
            Serial.println(millis());
            Serial.print("Potenciometro: ");
            Serial.println(valor);

            DatosSolar.close();
        }
        else
        {
            Serial.println("Error al abrir el archivo");
        }

        delay(500);
    }
}
