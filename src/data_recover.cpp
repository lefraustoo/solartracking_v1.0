/*
 * Código Optimizado para Data Logger de Paneles Solares
 * Genera una salida en formato CSV (Valores Separados por Comas) para un fácil análisis.
 *
 * Autor de la optimización: lefraustoo ft. los expertos
 * Fecha: 21 de julio de 2025
 *
 * Mejoras clave:
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

// --- CONFIGURACIÓN DE SENSORES Y PARÁMETROS ---

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
void printCSVHeader();
void logData();

// =================================================================
// --- SETUP ---
// =================================================================
void setup()
{
    Serial.begin(9600);
    Wire.begin();

    // --- Inicializar Sensores con Verificación ---
    if (!inaSeguidor.init())
        Serial.println("Fallo al iniciar INA Seguidor");
    if (!inaFijo.init())
        Serial.println("Fallo al iniciar INA Fijo");
    if (!rtc.begin())
        Serial.println("Fallo al iniciar RTC");
    if (!adsPiranometro.begin(0x48))
        Serial.println("Fallo al iniciar ADS Piranometro"); // Dirección I2C por defecto
    if (!adsPotenciometro.begin(0x49))
        Serial.println("Fallo al iniciar ADS Potenciometro"); // Cambia si usas otra dirección

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

    // Esperar a que el puerto serie se conecte
    while (!Serial)
        ;

    // Imprimir la cabecera del CSV una sola vez
    printCSVHeader();
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
 * @brief Imprime la cabecera del archivo CSV al puerto serie.
 */
void printCSVHeader()
{
    Serial.println("Timestamp,Potencia_Seguidor_mW,Potencia_Fijo_mW,Irradiancia_W/m2,Angulo_Seguidor_X,Angulo_Seguidor_Y,Angulo_Fijo_Z");
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

    // 2. Imprimir la línea de datos CSV
    Serial.print(now.timestamp(DateTime::TIMESTAMP_FULL));
    Serial.print(",");
    Serial.print(potenciaS);
    Serial.print(",");
    Serial.print(potenciaF);
    Serial.print(",");
    Serial.print(irradiancia);
    Serial.print(",");
    Serial.print(anguloX);
    Serial.print(",");
    Serial.print(anguloY);
    Serial.print(",");
    Serial.println(anguloZ);
}

/**
 * @brief Lee la potencia del panel seguidor.
 * @return Potencia en miliwatts (mW).
 */
float getPowerSeguidor()
{
    if (inaSeguidor.readAndClearFlags())
    {
        return inaSeguidor.getBusPower();
    }
    return 0.0; // Retorna 0 si hay error de lectura
}

/**
 * @brief Lee la potencia del panel fijo.
 * @return Potencia en miliwatts (mW).
 */
float getPowerFijo()
{
    if (inaFijo.readAndClearFlags())
    {
        return inaFijo.getBusPower();
    }
    return 0.0;
}

/**
 * @brief Lee la irradiancia desde el piranómetro.
 * @return Irradiancia en W/m^2.
 */
float getIrradiancia()
{
    int16_t results = adsPiranometro.readADC_Differential_0_1();
    float voltage_mV = results * ADS_MULTIPLIER;
    // La irradiancia es el voltaje medido dividido por la sensibilidad del sensor.
    // ¡DEBES CALIBRAR ESTE VALOR DE SENSIBILIDAD!
    float irradiance = voltage_mV / PIRANOMETRO_SENSITIVITY;
    return irradiance > 0 ? irradiance : 0; // Evitar valores negativos
}

/**
 * @brief Lee los ángulos de inclinación del panel seguidor.
 * @param angleX Referencia para guardar el ángulo en el eje X.
 * @param angleY Referencia para guardar el ángulo en el eje Y.
 */
void getAngulosSeguidor(float &angleX, float &angleY)
{
    int16_t ax, ay, az;
    mpuSeguidor.getAcceleration(&ax, &ay, &az);
    angleX = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * (180.0 / PI);
    angleY = atan(-ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180.0 / PI);
}

/**
 * @brief Lee el ángulo del panel fijo desde el potenciómetro.
 * @return Ángulo en grados (0-180).
 */
int getAnguloFijo()
{
    int16_t adcValue = adsPotenciometro.readADC_SingleEnded(0);
    // Mapear el valor del ADC al rango de 0-180 grados.
    // El valor máximo (21845) debe ser calibrado según tu potenciómetro.
    int angle = map(adcValue, 0, 21845, 0, 180);
    return constrain(angle, 0, 180); // Asegurar que el valor esté en el rango
}
