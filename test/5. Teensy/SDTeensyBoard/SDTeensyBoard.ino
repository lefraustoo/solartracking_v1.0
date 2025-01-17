///////////////////////////////////   LIBRARIES   ////////////////////////////////////
#include <Arduino.h>        // librería principal de Arduino
#include <SoftwareSerial.h> // librería para la comunicación serial por software
#include <SD.h>             //librería para el manejo de una tarjeta SD, es la misma que utilizan las tarjetas Arduino
#include <SdFat.h>          //librería para el manejo de una tarjeta SD, es la misma que utilizan las tarjetas Arduino
#include <SPI.h>            //librería para la comunicación vía protocolo SPI

///////////////////////////////   GUARDADO DE DATOS   ////////////////////////////////

const int chipSelect = BUILTIN_SDCARD; // declaración del pin CS embebido de la tarjeta

//////////////////////////////////   COMUNICACION   //////////////////////////////////
SoftwareSerial ArduinoMaster(7, 8); // RX, TX
String msg;

//////////////////////////////////   RECEPCION   /////////////////////////////////////
void readMasterPort()
{
  while (ArduinoMaster.available())
  {
    delay(10);
    if (ArduinoMaster.available() > 0)
    {
      char c = ArduinoMaster.read(); // leer un byte desde el buffer
      // Serial.print(c);               // imprimir el byte en el monitor serial
      // concatenar
      if (c != '\n')
      {
        msg += c;
      }
      else
      {
        break;
      }
    }
    ArduinoMaster.flush(); // limpiar el buffer
  }
}

/////////////////////////////////////   SETUP   //////////////////////////////////////
void setup()
{
  Serial.begin(115200);
  ArduinoMaster.begin(115200);

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

/////////////////////////////////////   LOOP   //////////////////////////////////////
void loop()
{
  readMasterPort();

  // Leer del maestro el mensaje
  if (msg != "")
  {
    File dataFile = SD.open("datalog.txt", FILE_WRITE); // creamos y abrimos un archivo donde guardaremos los datos de todos los sensores

    // Si el archivo esta disponible escribiremos datos en el archivo
    if (dataFile)
    {
      dataFile.println(msg); // Escribimos los datos provenientes de todos los sensores
      dataFile.close();
    }
    else
    {
      // Si el archivo presento un error al abrirlo imprimimos un mensaje de alerta
      Serial.println("error opening datalog.txt");
    }

    Serial.print("\n");
    Serial.print(msg);
    msg = "";
  }
}
