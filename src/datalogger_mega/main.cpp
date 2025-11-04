/*
 * CÓDIGO DE DATA LOGGER (PARA ARDUINO MEGA)
 * Basado en la lógica de sensores de tus archivos .cpp.
 *
 * TAREA:
 * Mide 6 cosas y las guarda en una tarjeta SD en formato CSV.
 * - Potencia Panel 1 (INA)
 * - Potencia Panel 2 (INA)
 * - Ángulo Panel 1 (MPU6050)
 * - Ángulo Panel 2 (ADS1115)
 * - Irradiancia (ADS1115)
 * - Timestamp (RTC)
 *
 * MEJORAS:
 * - NO USA 'String' para evitar cuelgues.
 * - ABRE LA SD UNA SOLA VEZ para evitar corrupción.
 * - USA 'millis()' para un muestreo estable.
 * - Separa las direcciones I2C que estaban en conflicto.
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <Adafruit_ADS1X15.h>
#include <MPU6050.h>
#include <INA226_WE.h>

// --- Configuración del Logger ---
#define SD_CS_PIN 53 // Pin CS en MEGA es 53
const char *filename = "DATALOG.CSV";
File dataFile;
const unsigned long SAMPLING_INTERVAL = 3000; // Muestreo cada 3 seg
const int FLUSH_INTERVAL = 10;                // Guardar en SD cada 10 muestras
unsigned long previousMillis = 0;
int logCounter = 0;

// --- Configuración de Sensores (CON DIRECCIONES CORREGIDAS) ---
RTC_DS3231 rtc;

// Panel 1 (Seguidor)
INA226_WE inaSeguidor(0x40); // Dirección I2C Panel 1
MPU6050 mpuSeguidor(0x69);   // Dirección I2C MPU

// Panel 2 (Fijo/Manual)
INA226_WE inaFijo(0x41);           // ¡NUEVA DIRECCIÓN I2C! Panel 2
Adafruit_ADS1115 adsPotFijo(0x49); // ¡NUEVA DIRECCIÓN I2C! Potenciómetro Panel 2

// Entorno
Adafruit_ADS1115 adsPiranometro(0x48); // Dirección I2C Piranómetro

// --- Constantes de Calibración (de tu código) ---
const float ADS_MULTIPLIER = 0.1875F;
const float PIRANOMETRO_SENSITIVITY = 0.02; // ¡USÉ ESTE VALOR, NO 500! 500 era un error.
const int POT_ADC_MAX = 21845;              // De tu código

// Offsets MPU (de tu código)
int ax_offset = -2632;
int ay_offset = -938;
int az_offset = 467;

// --- Declaración de Funciones ---
void initializeSensors();
void printCSVHeader(Stream &output);
void logData();
float getPotenciaSeguidor();
float getPotenciaFijo();
float getIrradiancia();
void getAngulosSeguidor(float &angleX, float &angleY);
int getAnguloFijo();

// ==========================================
//  SETUP
// ==========================================
void setup()
{
    Serial.begin(9600);
    Wire.begin();
    while (!Serial)
        ;
    Serial.println(F("Iniciando Data Logger (MEGA)..."));

    initializeSensors();

    // --- Inicializar la Tarjeta SD ---
    Serial.print(F("Iniciando tarjeta SD..."));
    if (!SD.begin(SD_CS_PIN))
    {
        Serial.println(F("¡Fallo en la inicializacion!"));
        while (true)
            ; // Detener
    }
    Serial.println(F("Tarjeta SD inicializada."));

    // --- Abrir archivo (una sola vez) ---
    bool fileExists = SD.exists(filename);
    dataFile = SD.open(filename, FILE_WRITE);

    if (!dataFile)
    {
        Serial.println(F("Error al abrir el archivo de datos."));
        while (true)
            ; // Detener
    }

    if (!fileExists)
    {
        Serial.println(F("Escribiendo cabecera en SD..."));
        printCSVHeader(dataFile);
        dataFile.flush();
    }
    else
    {
        Serial.println(F("Archivo encontrado. Añadiendo datos..."));
    }

    printCSVHeader(Serial);
}

// ==========================================
//  LOOP (NO BLOQUEANTE)
// ==========================================
void loop()
{
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= SAMPLING_INTERVAL)
    {
        previousMillis = currentMillis;
        logData(); // Leer sensores y guardar datos

        logCounter++;
        if (logCounter % FLUSH_INTERVAL == 0)
        {
            dataFile.flush(); // Guardar en la SD
            Serial.println(F("-> Datos guardados en SD (flush)"));
        }
    }
}

// ==========================================
//  FUNCIONES DE LÓGICA
// ==========================================

void initializeSensors()
{
    if (!rtc.begin())
        Serial.println(F("Fallo al iniciar RTC"));
    if (!inaSeguidor.init())
        Serial.println(F("Fallo al iniciar INA Seguidor (0x40)"));
    if (!inaFijo.init())
        Serial.println(F("Fallo al iniciar INA Fijo (0x41)"));
    if (!adsPiranometro.begin())
        Serial.println(F("Fallo al iniciar ADS Piranometro (0x48)"));
    if (!adsPotFijo.begin())
        Serial.println(F("Fallo al iniciar ADS Pot Fijo (0x49)"));

    mpuSeguidor.initialize();
    if (mpuSeguidor.testConnection())
    {
        mpuSeguidor.setXAccelOffset(ax_offset);
        mpuSeguidor.setYAccelOffset(ay_offset);
        mpuSeguidor.setZAccelOffset(az_offset);
    }
    else
    {
        Serial.println(F("Fallo al iniciar MPU6050 (0x69)"));
    }
}

void printCSVHeader(Stream &output)
{
    output.println(F("Timestamp,Pot_Seguidor_mW,Pot_Fijo_mW,Irradiancia_W/m2,Ang_Seg_X,Ang_Seg_Y,Ang_Fijo_Z"));
}

/*
 * Recopila datos y los escribe (pieza por pieza) en la SD y el Serial.
 * NO USA 'String'
 */
void logData()
{
    DateTime now = rtc.now();

    // 1. Obtener datos
    float potS = getPotenciaSeguidor();
    float potF = getPotenciaFijo();
    float irrad = getIrradiancia();
    float angX, angY;
    getAngulosSeguidor(angX, angY);
    int angZ = getAnguloFijo();

    // 2. Escribir en archivo SD (pieza por pieza)
    dataFile.print(now.timestamp(DateTime::TIMESTAMP_FULL));
    dataFile.print(F(","));
    dataFile.print(potS);
    dataFile.print(F(","));
    dataFile.print(potF);
    dataFile.print(F(","));
    dataFile.print(irrad);
    dataFile.print(F(","));
    dataFile.print(angX);
    dataFile.print(F(","));
    dataFile.print(angY);
    dataFile.print(F(","));
    dataFile.println(angZ);

    // 3. Escribir en Monitor Serie (para depuración)
    Serial.print(now.timestamp(DateTime::TIMESTAMP_FULL));
    Serial.print(F(","));
    Serial.print(potS);
    Serial.print(F(","));
    Serial.print(potF);
    Serial.print(F(","));
    Serial.print(irrad);
    Serial.print(F(","));
    Serial.print(angX);
    Serial.print(F(","));
    Serial.print(angY);
    Serial.print(F(","));
    Serial.println(angZ);
}

// --- Funciones de Lectura de Sensores (Basadas en tu código) ---

float getPotenciaSeguidor()
{
    if (inaSeguidor.isConversionReady()) // API Moderna
    {
        return inaSeguidor.getBusPower();
    }
    return NAN;
}

float getPotenciaFijo()
{
    if (inaFijo.isConversionReady()) // API Moderna
    {
        return inaFijo.getBusPower();
    }
    return NAN;
}

float getIrradiancia()
{
    // De tu función 'Radiacion()' en ASeguidorSolar.cpp
    int16_t results = adsPiranometro.readADC_Differential_0_1();
    float voltage_mV = results * ADS_MULTIPLIER; // Tu código tenía un '-(results...)''. Lo quité. Si da negativo, vuelve a ponerlo.

    if (PIRANOMETRO_SENSITIVITY == 0)
        return 0.0;
    float irradiance = voltage_mV / PIRANOMETRO_SENSITIVITY;
    return irradiance > 0 ? irradiance : 0;
}

void getAngulosSeguidor(float &angleX, float &angleY)
{
    // De tu función 'Angulos()' en ASeguidorSolar.cpp
    int16_t ax, ay, az;
    mpuSeguidor.getAcceleration(&ax, &ay, &az);

    float f_ax = (float)ax;
    float f_ay = (float)ay;
    float f_az = (float)az;

    // Usar atan2 es más robusto y PI (no 3.1416) es más preciso
    angleX = atan2(f_ay, sqrt(f_ax * f_ax + f_az * f_az)) * (180.0 / PI);
    angleY = atan2(-f_ax, sqrt(f_ay * f_ay + f_az * f_az)) * (180.0 / PI);
}

int getAnguloFijo()
{
    // De tu función 'potenciometroext()' en ASeguidorSolar.cpp
    int16_t adcValue = adsPotFijo.readADC_SingleEnded(0);
    int angle = map(adcValue, 0, POT_ADC_MAX, 0, 180);
    return constrain(angle, 0, 180);
}
