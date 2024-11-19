#include <Arduino.h>
#include <SoftwareSerial.h>

SoftwareSerial ArduinoSlave(2, 3);
String msg;

void setup()
{

  Serial.begin(9600);
  Serial.println("ENTER Commands:");
  ArduinoSlave.begin(9600);
}

void loop()
{
  readSlave();      // Leer el puerto
  readSerialPort(); // Leer el monitor

  if (msg != "")
  {
    Serial.print("Master sent : "); // Enviar data slave
    Serial.println(msg);
    ArduinoSlave.print(msg);
    msg = "";
  }
}

void readSerialPort()
{
  while (Serial.available())
  {
    delay(10);
    if (Serial.available() > 0)
    {
      char c = Serial.read(); // leer un byte desde el buffer
      msg += c;               // concatenar al String
    }
  }
  Serial.flush(); // limpiar buffer
}
void readSlave()
{
  while (ArduinoSlave.available())
  {
    delay(10);
    if (ArduinoSlave.available() > 0)
    {
      char c = ArduinoSlave.read(); // leer un byte desde el buffer
      msg += c;                     // concatenar al String
    }
  }
  if (msg != "") // saber si hay retorno del esclavo
  {
    Serial.println("Slave return: " + msg);
    msg = "";
  }

  ArduinoSlave.flush();
}
