/*
 * Código Optimizado para Data Logger de Paneles Solares con guardado en Tarjeta SD
 * Genera una salida en formato CSV (Valores Separados por Comas) para un fácil análisis
 * y la guarda en un archivo "DATALOG.CSV" en una tarjeta SD.
 *
 * Autor de la optimización: Gemini
 * Fecha: 21 de julio de 2025
 *
 * Mejoras clave:
 * - Añadida funcionalidad para guardar datos en una tarjeta SD.
 * - Salida de datos estructurada en formato CSV.
 * - Temporizador no bloqueante (millis) para un muestreo a intervalos fijos.
 * - Funciones refactorizadas para devolver valores en lugar de imprimir texto.
 * - Código más limpio, modular y listo para el análisis de datos.
 */

#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <SPI.h>
#include <Adafruit_ADS1X15.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <INA226_WE.h>
#include <SD.h> // <-- LIBRERÍA PARA LA TARJETA SD

// --- CONFIGURACIÓN DE SENSORES Y PARÁMETROS ---

// --- NUEVO: Configuración de la Tarjeta SD ---
#define SD_CS_PIN 4 // Pin Chip Select para el módulo SD.
const char *filename = "DATALOG.CSV";
File dataFile; // Objeto para manejar el archivo de datos.

// Intervalo de muestreo en milisegundos (ej. 3000ms = 3 segundos)
const unsigned long SAMPLING_INTERVAL = 3000;
unsigned long previousMillis = 0;

// Sensores de potencia INA226 (uno para cada panel)
#define I2C_ADDRESS_SEGUIDOR 0x40
#define I2C_ADDRESS_FIJO 0x41
INA226_WE inaSeguidor(I2C_ADDRESS_SEGUIDOR);
INA226_WE inaFijo(I2C_ADDRESS_FIJO);

// Reloj de tiempo real (RTC)
RTC_DS3231 rtc;

// Conversores ADC ADS1115
Adafruit_ADS1115 adsPiranometro;   // Para el piranómetro
Adafruit_ADS1115 adsPotenciometro; // Para el potenciómetro del panel fijo

// Constantes para el piranómetro
const float ADS_MULTIPLIER = 0.1875F;
const float PIRANOMETRO_SENSITIVITY = 0.02; // Sensibilidad en mV por W/m^2, ¡ajusta este valor!

// Sensor de Inclinación MPU6050 (para el panel seguidor)
MPU6050 mpuSeguidor(0x69);
// Offsets de calibración para el MPU6050 (ajusta según tu sensor)
int ax_offset = -2632;
int ay_offset = -938;
int az_offset = 467;

// --- DECLARACIÓN DE FUNCIONES ---
float getPowerSeguidor();
float getPowerFijo();
float getIrradiancia();
void getAngulosSeguidor(float &angleX, float &angleY);
int getAnguloFijo();
void printCSVHeader(Stream &output); // Modificada para aceptar cualquier salida (Serial o File)
void logData();

// =================================================================
// --- SETUP ---
// =================================================================
void setup()
{
    Serial.begin(9600);
    Wire.begin();

    // Esperar a que el puerto serie se conecte
    while (!Serial)
        ;
    Serial.println("Iniciando Data Logger...");

    // --- Inicializar Sensores ---
    // (El código de inicialización de sensores permanece igual)
    if (!inaSeguidor.init())
        Serial.println("Fallo al iniciar INA Seguidor");
    if (!inaFijo.init())
        Serial.println("Fallo al iniciar INA Fijo");
    if (!rtc.begin())
        Serial.println("Fallo al iniciar RTC");
    if (!adsPiranometro.begin(0x48))
        Serial.println("Fallo al iniciar ADS Piranometro");
    if (!adsPotenciometro.begin(0x49))
        Serial.println("Fallo al iniciar ADS Potenciometro");
    mpuSeguidor.initialize();
    if (mpuSeguidor.testConnection())
    {
        mpuSeguidor.setXAccelOffset(ax_offset);
        mpuSeguidor.setYAccelOffset(ay_offset);
        mpuSeguidor.setZAccelOffset(az_offset);
    }
    else
    {
        Serial.println("Fallo al iniciar MPU6050");
    }

    // --- NUEVO: Inicializar la Tarjeta SD ---
    Serial.print("Iniciando tarjeta SD...");
    if (!SD.begin(SD_CS_PIN))
    {
        Serial.println("¡Fallo en la inicializacion! Verifique conexiones y formato de la tarjeta.");
        // Podrías detener el programa aquí si la SD es esencial, o dejar que continúe solo con Serial.
        while (true)
            ;
    }
    Serial.println("Tarjeta SD inicializada.");

    // --- NUEVO: Escribir la cabecera en el archivo si no existe ---
    if (!SD.exists(filename))
    {
        Serial.println("Archivo de datos no encontrado. Creando cabecera...");
        dataFile = SD.open(filename, FILE_WRITE);
        if (dataFile)
        {
            printCSVHeader(dataFile); // Escribe la cabecera en el archivo
            dataFile.close();
        }
        else
        {
            Serial.println("Error al abrir el archivo para escribir la cabecera.");
        }
    }

    // Imprimir la cabecera en el Monitor Serie para la visualización en tiempo real.
    printCSVHeader(Serial);
}

// =================================================================
// --- LOOP ---
// =================================================================
void loop()
{
    // Usar millis() para tomar una muestra de datos a intervalos regulares
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= SAMPLING_INTERVAL)
    {
        previousMillis = currentMillis; // Actualizar el tiempo de la última muestra
        logData();                      // Llamar a la función que lee y registra los datos
    }
}

// =================================================================
// --- FUNCIONES DE REGISTRO Y LECTURA ---
// =================================================================

/**
 * @brief Imprime la cabecera del archivo CSV al flujo de salida proporcionado.
 * @param output El flujo de salida (puede ser Serial o un objeto File).
 */
void printCSVHeader(Stream &output)
{
    output.println("Timestamp,Potencia_Seguidor_mW,Potencia_Fijo_mW,Irradiancia_W/m2,Angulo_Seguidor_X,Angulo_Seguidor_Y,Angulo_Fijo_Z");
}

/**
 * @brief Recopila los datos de todos los sensores y los imprime en una sola línea CSV.
 */
void logData()
{
    DateTime now = rtc.now();

    // 1. Obtener datos de cada sensor
    float potenciaS = getPowerSeguidor();
    float potenciaF = getPowerFijo();
    float irradiancia = getIrradiancia();
    float anguloX, anguloY;
    getAngulosSeguidor(anguloX, anguloY);
    int anguloZ = getAnguloFijo();

    // --- NUEVO: Construir la cadena de datos ---
    String dataString = "";
    dataString += String(now.timestamp(DateTime::TIMESTAMP_FULL));
    dataString += ",";
    dataString += String(potenciaS);
    dataString += ",";
    dataString += String(potenciaF);
    dataString += ",";
    dataString += String(irradiancia);
    dataString += ",";
    dataString += String(anguloX);
    dataString += ",";
    dataString += String(anguloY);
    dataString += ",";
    dataString += String(anguloZ);

    // --- NUEVO: Escribir la cadena en el archivo SD ---
    dataFile = SD.open(filename, FILE_WRITE);
    if (dataFile)
    {
        dataFile.println(dataString);
        dataFile.close();
        // Imprimir la misma cadena en el monitor serie para depuración en tiempo real
        Serial.println(dataString);
    }
    else
    {
        Serial.println("Error al abrir el archivo de datos.");
    }
}

// --- Las funciones de lectura de sensores permanecen sin cambios ---

float getPowerSeguidor()
{
    if (inaSeguidor.readAndClearFlags())
    {
        return inaSeguidor.getBusPower();
    }
    return 0.0;
}

float getPowerFijo()
{
    if (inaFijo.readAndClearFlags())
    {
        return inaFijo.getBusPower();
    }
    return 0.0;
}

float getIrradiancia()
{
    int16_t results = adsPiranometro.readADC_Differential_0_1();
    float voltage_mV = results * ADS_MULTIPLIER;
    float irradiance = voltage_mV / PIRANOMETRO_SENSITIVITY;
    return irradiance > 0 ? irradiance : 0;
}

void getAngulosSeguidor(float &angleX, float &angleY)
{
    int16_t ax, ay, az;
    mpuSeguidor.getAcceleration(&ax, &ay, &az);
    angleX = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * (180.0 / PI);
    angleY = atan(-ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180.0 / PI);
}

int getAnguloFijo()
{
    int16_t adcValue = adsPotenciometro.readADC_SingleEnded(0);
    int angle = map(adcValue, 0, 21845, 0, 180);
    return constrain(angle, 0, 180);
}
