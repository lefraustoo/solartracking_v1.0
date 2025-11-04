/*
 * CÓDIGO DE DATA LOGGER (PARA ARDUINO MEGA)
 *
 * Tarea: Leer 6 sensores I2C y guardar los datos en formato CSV en una
 * tarjeta SD a intervalos fijos (no bloqueante).
 *
 * MEJORAS CRÍTICAS DE ESTABILIDAD:
 * 1. SIN CLASE String: Se eliminó la concatenación de 'String' para
 * prevenir la fragmentación de RAM y cuelgues del sistema.
 * 2. MANEJO EFICIENTE DE SD: El archivo de datos se abre UNA SOLA VEZ en
 * setup() y se usa 'dataFile.flush()' periódicamente para guardar.
 * Esto previene el desgaste y corrupción de la tarjeta SD.
 * 3. DATOS LIMPIOS: Las funciones de sensor devuelven 'NAN' (Not a Number)
 * si la lectura falla, mejorando la integridad del análisis de datos.
 * 4. OPTIMIZACIÓN: Se usa 'x*x' en lugar de 'pow(x, 2)' y 'atan2' para
 * cálculos de ángulo más rápidos y robustos.
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <Adafruit_ADS1X15.h>
#include <MPU6050.h>
#include <INA226_WE.h>

// --- CONFIGURACIÓN DE PARÁMETROS ---
#define SD_CS_PIN 53 // Pin Chip Select para la SD en Arduino MEGA
const char *filename = "DATALOG.CSV";

// Intervalos (no bloqueantes)
const unsigned long SAMPLING_INTERVAL = 3000; // Tomar muestra cada 3 segundos
const int FLUSH_INTERVAL = 10;                // Guardar en SD cada 10 muestras (30 seg)

unsigned long previousMillis = 0;
int logCounter = 0; // Contador para el flush

// --- OBJETOS DE SENSORES ---

// Tarjeta SD
File dataFile;

// Sensores de potencia INA226
#define I2C_ADDRESS_SEGUIDOR 0x40
#define I2C_ADDRESS_FIJO 0x41
INA226_WE inaSeguidor(I2C_ADDRESS_SEGUIDOR);
INA226_WE inaFijo(I2C_ADDRESS_FIJO);

// Reloj de tiempo real (RTC)
RTC_DS3231 rtc;

// Conversores ADC ADS1115
// Adafruit_ADS1115 adsPiranometro(0x48);   // Para el piranómetro
// Adafruit_ADS1115 adsPotenciometro(0x49); // Para el potenciómetro del panel fijo
Adafruit_ADS1115 adsPiranometro;   // ¡CORREGIDO! Declaración vacía
Adafruit_ADS1115 adsPotenciometro; // ¡CORREGIDO! Declaración vacía

// Sensor de Inclinación MPU6050
MPU6050 mpuSeguidor(0x69);
// Offsets de calibración (¡ajusta según tu sensor!)
int ax_offset = -2632;
int ay_offset = -938;
int az_offset = 467;

// --- CONSTANTES DE CALIBRACIÓN ---
// Piranómetro
const float ADS_MULTIPLIER_PIRANOMETRO = 0.1875F; // Para ADS1115
const float PIRANOMETRO_SENSITIVITY = 0.02;       // Sensibilidad en mV por W/m^2

// Potenciómetro (Panel Fijo)
const int POT_ADC_MIN = 0;     // Valor ADC para 0 grados
const int POT_ADC_MAX = 21845; // Valor ADC calibrado para 180 grados
const int POT_ANGLE_MIN = 0;
const int POT_ANGLE_MAX = 180;

// --- DECLARACIÓN DE FUNCIONES ---
float getPowerSeguidor();
float getPowerFijo();
float getIrradiancia();
void getAngulosSeguidor(float &angleX, float &angleY);
int getAnguloFijo();
void printCSVHeader(Stream &output);
void logData();
void initializeSensors();

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
    Serial.println(F("Iniciando Data Logger..."));

    // --- Inicializar todos los sensores ---
    initializeSensors();

    // --- Inicializar la Tarjeta SD ---
    Serial.print(F("Iniciando tarjeta SD..."));
    if (!SD.begin(SD_CS_PIN))
    {
        Serial.println(F("¡Fallo en la inicializacion! Verifique conexiones."));
        while (true)
            ; // Detener el programa
    }
    Serial.println(F("Tarjeta SD inicializada."));

    // --- Lógica de Archivo (Abrir UNA SOLA VEZ) ---
    bool fileExists = SD.exists(filename);
    dataFile = SD.open(filename, FILE_WRITE); // Abrir en modo de añadir (append)

    if (!dataFile)
    {
        Serial.println(F("Error al abrir el archivo de datos."));
        while (true)
            ; // Detener
    }

    if (!fileExists)
    {
        Serial.println(F("Archivo no existe. Escribiendo cabecera en SD..."));
        printCSVHeader(dataFile);
        dataFile.flush(); // Asegurarse de que la cabecera se escriba
    }
    else
    {
        Serial.println(F("Archivo encontrado. Añadiendo datos..."));
    }

    // Imprimir la cabecera en el Monitor Serie para visualización.
    printCSVHeader(Serial);
}

// =================================================================
// --- LOOP (NO BLOQUEANTE) ---
// =================================================================
void loop()
{
    unsigned long currentMillis = millis();

    // Usar millis() para tomar una muestra de datos a intervalos regulares
    if (currentMillis - previousMillis >= SAMPLING_INTERVAL)
    {
        previousMillis = currentMillis; // Actualizar el tiempo de la última muestra

        logData(); // Llamar a la función que lee y registra los datos

        logCounter++;

        // --- Lógica de Guardado (Flush) Periódico ---
        if (logCounter % FLUSH_INTERVAL == 0)
        {
            dataFile.flush(); // Guardar el búfer en la tarjeta SD
            Serial.println(F("-> Datos guardados en SD (flush)"));
        }
    }
}

// =================================================================
// --- FUNCIONES DE REGISTRO Y LECTURA ---
// =================================================================

/**
 * @brief Inicializa todos los sensores I2C y reporta fallos.
 */
void initializeSensors()
{
    if (!inaSeguidor.init())
        Serial.println(F("Fallo al iniciar INA Seguidor"));
    if (!inaFijo.init())
        Serial.println(F("Fallo al iniciar INA Fijo"));
    if (!rtc.begin())
        Serial.println(F("Fallo al iniciar RTC"));
    // if (!adsPiranometro.begin()) Serial.println(F("Fallo al iniciar ADS Piranometro"));
    // if (!adsPotenciometro.begin()) Serial.println(F("Fallo al iniciar ADS Potenciometro"));
    if (!adsPiranometro.begin(0x48))
        Serial.println(F("Fallo al iniciar ADS Piranometro (0x48)"));
    if (!adsPotenciometro.begin(0x49))
        Serial.println(F("Fallo al iniciar ADS Potenciometro (0x49)"));

    mpuSeguidor.initialize();
    if (mpuSeguidor.testConnection())
    {
        mpuSeguidor.setXAccelOffset(ax_offset);
        mpuSeguidor.setYAccelOffset(ay_offset);
        mpuSeguidor.setZAccelOffset(az_offset);
    }
    else
    {
        Serial.println(F("Fallo al iniciar MPU6050"));
    }
}

/**
 * @brief Imprime la cabecera del archivo CSV al flujo de salida.
 * @param output El flujo de salida (Serial o un objeto File).
 */
void printCSVHeader(Stream &output)
{
    output.println(F("Timestamp,Potencia_Seguidor_mW,Potencia_Fijo_mW,Irradiancia_W/m2,Angulo_Seguidor_X,Angulo_Seguidor_Y,Angulo_Fijo_Z"));
}

/**
 * @brief Recopila datos y los escribe (pieza por pieza) en la SD y el Serial.
 * ¡¡¡NO USA LA CLASE STRING!!!
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

    // 2. Escribir en el archivo SD (que ya está abierto)
    // Usar F() macro para ahorrar RAM en los strings fijos
    dataFile.print(now.timestamp(DateTime::TIMESTAMP_FULL));
    dataFile.print(F(","));
    dataFile.print(potenciaS);
    dataFile.print(F(","));
    dataFile.print(potenciaF);
    dataFile.print(F(","));
    dataFile.print(irradiancia);
    dataFile.print(F(","));
    dataFile.print(anguloX);
    dataFile.print(F(","));
    dataFile.print(anguloY);
    dataFile.print(F(","));
    dataFile.println(anguloZ); // println() para la última pieza (salto de línea)

    // 3. Escribir la misma línea en el monitor serie para depuración
    Serial.print(now.timestamp(DateTime::TIMESTAMP_FULL));
    Serial.print(F(","));
    Serial.print(potenciaS);
    Serial.print(F(","));
    Serial.print(potenciaF);
    Serial.print(F(","));
    Serial.print(irradiancia);
    Serial.print(F(","));
    Serial.print(anguloX);
    Serial.print(F(","));
    Serial.print(anguloY);
    Serial.print(F(","));
    Serial.println(anguloZ);
}

// --- Funciones de Lectura de Sensores (Optimizadas) ---

float getPowerSeguidor()
{
    // if (inaSeguidor.readAndClearFlags())
    // {
    //     return inaSeguidor.getBusPower(); // Retorna potencia en mW
    // }
    // return NAN; // Retorna "Not a Number" si la lectura falla

    // ¡CORREGIDO! Usando la API moderna de la librería (v1.3.0+)
    if (inaSeguidor.isConversionReady())
    {
        return inaSeguidor.getBusPower();
    }
    return NAN; // Retorna NAN si la conversión no está lista (o sensor desconectado)
}

float getPowerFijo()
{
    // if (inaFijo.readAndClearFlags())
    // {
    //     return inaFijo.getBusPower(); // Retorna potencia en mW
    // }
    // return NAN; // Retorna "Not a Number" si la lectura falla

    // ¡CORREGIDO! Usando la API moderna de la librería (v1.3.0+)
    if (inaFijo.isConversionReady())
    {
        return inaFijo.getBusPower();
    }
    return NAN; // Retorna NAN si la conversión no está lista (o sensor desconectado)
}

float getIrradiancia()
{
    int16_t results = adsPiranometro.readADC_Differential_0_1();
    float voltage_mV = results * ADS_MULTIPLIER_PIRANOMETRO;

    if (PIRANOMETRO_SENSITIVITY == 0)
        return 0.0; // Evitar división por cero

    float irradiance = voltage_mV / PIRANOMETRO_SENSITIVITY;
    return irradiance > 0 ? irradiance : 0; // No retornar valores negativos
}

void getAngulosSeguidor(float &angleX, float &angleY)
{
    int16_t ax, ay, az;
    mpuSeguidor.getAcceleration(&ax, &ay, &az);

    // Convertir a float ANTES de multiplicar para evitar desbordamiento
    float f_ax = (float)ax;
    float f_ay = (float)ay;
    float f_az = (float)az;

    // Usar x*x (más rápido) y atan2 (más robusto que atan)
    angleX = atan2(f_ay, sqrt(f_ax * f_ax + f_az * f_az)) * (180.0 / PI);
    angleY = atan2(-f_ax, sqrt(f_ay * f_ay + f_az * f_az)) * (180.0 / PI);
}

int getAnguloFijo()
{
    int16_t adcValue = adsPotenciometro.readADC_SingleEnded(0);
    // Usar constantes en lugar de "números mágicos"
    int angle = map(adcValue, POT_ADC_MIN, POT_ADC_MAX, POT_ANGLE_MIN, POT_ANGLE_MAX);
    return constrain(angle, POT_ANGLE_MIN, POT_ANGLE_MAX);
}
