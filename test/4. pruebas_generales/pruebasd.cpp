#include <Arduino.h>
#include <SD.h>

#include <Adafruit_I2CDevice.h>

File logFile;

void setup()
{
    Serial.begin(9600);
    Serial.print(F("Iniciando SD ..."));
    if (!SD.begin(9))
    {
        Serial.println(F("Error al iniciar"));
        return;
    }
    Serial.println(F("Iniciado correctamente"));
}

// Funcion que simula la lectura de un sensor
int readSensor()
{
    return random(300);
}

void loop()
{
    // Abrir archivo y escribir valor
    logFile = SD.open("datalog.txt", FILE_WRITE);

    if (logFile)
    {
        int value = readSensor();
        logFile.print("Time(ms)=");
        logFile.print(millis());
        logFile.print(", value=");
        logFile.println(value);

        Serial.print("Time(ms)=");
        Serial.print(millis());
        Serial.print(", value=");
        Serial.println(value);

        logFile.close();
    }
    else
    {
        Serial.println("Error al abrir el archivo");
    }
    delay(500);
}