#include <Arduino.h>
#include <SoftwareSerial.h>

SoftwareSerial ArduinoMaster(2, 3);
String msg;

void setup()
{
  Serial.begin(9600);
  ArduinoMaster.begin(9600);
}

void loop()
{
  readMasterPort();

  // Leer del maestro el mensaje
  if (msg != "")
  {
    ArduinoMaster.print(msg);
    Serial.print("Master sent : ");
    Serial.println(msg);
    msg = "";
  }
}

void readMasterPort()
{
  while (ArduinoMaster.available())
  {
    delay(10);
    if (ArduinoMaster.available() > 0)
    {
      char c = ArduinoMaster.read(); // leer un byte desde el buffer
      msg += c;                      // concatenar al String
    }
  }
  ArduinoMaster.flush();
}
