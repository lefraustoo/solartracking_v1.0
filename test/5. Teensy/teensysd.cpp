#include <Arduino.h> //librería principal de Arduino
#include <SD.h>      //librería para el manejo de una tarjeta SD, es la misma que utilizan las tarjetas Arduino
#include <SPI.h>     //librería para la comunicación vía protocolo SPI

#ifndef BUILTIN_SDCARD
#define BUILTIN_SDCARD 10 // Pin CS embebido para Teensy 4.1 (modificar según tu placa)
#endif

const int chipSelect = BUILTIN_SDCARD; // declaración del pin CS embebido de la tarjeta

String dataStringA0 = ""; // cadena de datos para almacenar el valor del primer sensor análogo
String dataStringA1 = ""; // cadena de datos para almacenar el valor del segundo sensor análogo
String dataStringD6 = ""; // cadena de datos para almacenar el valor del primer sensor digital
String dataStringD7 = ""; // cadena de datos para almacenar el valor del segundo sensor digital

void setup()
{

  Serial.begin(9600); // iniciamos la comunicación serial
  pinMode(6, INPUT);  // declaramos al pin 6 y 7 como entradas digitales
  pinMode(7, INPUT);

  while (!Serial)
  {
    ; // esperamos a que el puerto serial responda
  }

  Serial.print("Initializing SD card..."); // inicializamos la comunicación con la tarjeta micro SD

  if (!SD.begin(chipSelect))
  { // en caso de algún fallo en la inicialización desplegamos un mensaje de alerta
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
}

void loop()
{
  int sensorA0 = analogRead(A0);   // leemos datos del primer sensor analógico en el pin A0
  dataStringA0 = String(sensorA0); // almacenamos estos datos en una cadena
  int sensorA1 = analogRead(A1);   // leemos datos del segundo sensor analógico en el pin A1
  dataStringA1 = String(sensorA1); // almacenamos estos datos en una cadena

  int sensorD6 = digitalRead(6);   // leemos datos del primer sensor digital en el pin 6
  dataStringD6 = String(sensorD6); // almacenamos los datos en una cadena
  int sensorD7 = digitalRead(7);   // leemos datos del primer sensor digital en el pin 7
  dataStringD7 = String(sensorD7); // almacenamos los datos en una cadena

  File dataFile = SD.open("datos1.txt", FILE_WRITE); // creamos y abrimos un archivo donde guardaremos los datos de todos los sensores

  // Si el archivo esta disponible escribiremos datos en el
  if (dataFile)
  {
    dataFile.print(dataStringA0); // Escribimos los datos provenientes de todos los sensores
    dataFile.print(",");          // los datos estarán reparados por comas y cada lectura será escrita en un renglón
    dataFile.print(dataStringA1);
    dataFile.print(",");
    dataFile.print(dataStringD6);
    dataFile.print(",");
    dataFile.println(dataStringD7);
    dataFile.close();
    // Imprimimos los valores también en el monitor serial
    Serial.print(dataStringA0);
    Serial.print(",");
    Serial.print(dataStringA1);
    Serial.print(",");
    Serial.print(dataStringD6);
    Serial.print(",");
    Serial.println(dataStringD7);
  }
  else
  {
    // Si el archivo presento un error al abrirlo imprimimos un mensaje de alerta
    Serial.println("error opening datalog.txt");
  }
  delay(500); // haremos lecturas cada 500 ms
}