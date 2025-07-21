/*
 * Código Optimizado para Control de Dos Sistemas de Paneles Solares
 * Sistema 1: Seguidor solar automático con 4 LDRs.
 * Sistema 2: Panel con posicionamiento manual mediante 2 potenciómetros.
 *
 * Autor de la optimización: lefraustoo ft. los expertos
 * Fecha: 21 de julio de 2025
 *
 * Descripción de la mejora:
 * - Se han separado por completo la lógica y los pines para cada sistema.
 * - Se utilizan 4 objetos Servo para controlar 4 motores de forma independiente.
 * - Se han asignado pines de hardware sin conflictos.
 * - Los nombres de las variables y funciones son descriptivos para mayor claridad.
 * - Se han corregido los límites de los servos al rango estándar (0-180 grados).
 * - El código es más legible, mantenible y, lo más importante, funcional.
 */

#include <Arduino.h>
#include <Servo.h>

// --- PINES Y CONFIGURACIÓN DEL SISTEMA 1: SEGUIDOR SOLAR AUTOMÁTICO ---
// Servos para el panel seguidor (conecta a pines PWM ~)
#define SERVO_SEGUIDOR_H_PIN 5 // Servo Horizontal del Seguidor
#define SERVO_SEGUIDOR_V_PIN 6 // Servo Vertical del Seguidor

// Fotorresistencias (LDRs) para el seguidor (conecta a pines Analógicos A)
// ¡Asegúrate de que no se repitan con los pines de los potenciómetros!
#define LDR_TOP_LEFT_PIN A0     // LDR Superior Izquierda
#define LDR_TOP_RIGHT_PIN A1    // LDR Superior Derecha
#define LDR_BOTTOM_LEFT_PIN A2  // LDR Inferior Izquierda
#define LDR_BOTTOM_RIGHT_PIN A3 // LDR Inferior Derecha

// --- PINES Y CONFIGURACIÓN DEL SISTEMA 2: PANEL DE CONTROL MANUAL ---
// Servos para el panel manual (conecta a pines PWM ~)
#define SERVO_MANUAL_H_PIN 9  // Servo Horizontal del panel Manual
#define SERVO_MANUAL_V_PIN 10 // Servo Vertical del panel Manual

// Potenciómetros para el control manual (conecta a pines Analógicos A)
#define POT_MANUAL_H_PIN A4 // Potenciómetro para el eje Horizontal
#define POT_MANUAL_V_PIN A5 // Potenciómetro para el eje Vertical

// --- OBJETOS SERVO (UNO PARA CADA MOTOR) ---
Servo servoSeguidorH, servoSeguidorV;
Servo servoManualH, servoManualV;

// --- VARIABLES GLOBALES ---
// Posiciones iniciales para los servos del seguidor
int posSeguidorH = 90;
int posSeguidorV = 90;

// Límites de movimiento para los servos (0-180 grados es el estándar)
const int SERVO_LIMITE_BAJO = 0;
const int SERVO_LIMITE_ALTO = 180;

// Parámetros de sensibilidad y velocidad para el seguidor (tu lógica original)
int tolerancia = 100;
int tiempoDelaySeguidor = 50;

// --- DECLARACIÓN DE FUNCIONES (BUENA PRÁCTICA) ---
void actualizarSistemaSeguidor();
void actualizarSistemaManual();

// =================================================================
// --- SETUP: INICIALIZACIÓN DEL SISTEMA ---
// =================================================================
void setup()
{
    Serial.begin(9600); // Iniciar comunicación serial para depuración

    // --- Inicialización del Sistema 1: Seguidor Automático ---
    servoSeguidorH.attach(SERVO_SEGUIDOR_H_PIN);
    servoSeguidorV.attach(SERVO_SEGUIDOR_V_PIN);
    // Mover los servos a su posición inicial
    servoSeguidorH.write(posSeguidorH);
    servoSeguidorV.write(posSeguidorV);
    // Los pines analógicos (A0-A5) no necesitan pinMode cuando se usan con analogRead.

    // --- Inicialización del Sistema 2: Control Manual ---
    servoManualH.attach(SERVO_MANUAL_H_PIN);
    servoManualV.attach(SERVO_MANUAL_V_PIN);
    // La posición inicial del sistema manual será definida por los potenciómetros en el primer loop.

    Serial.println("Sistemas de paneles solares inicializados.");
    Serial.println("Sistema 1: Seguidor automatico. Sistema 2: Control manual.");
}

// =================================================================
// --- LOOP PRINCIPAL ---
// =================================================================
void loop()
{
    // El loop principal ahora es muy limpio. Llama a las funciones que
    // gestionan cada sistema de forma independiente.
    actualizarSistemaSeguidor();
    actualizarSistemaManual();

    // Una pequeña pausa general es buena para la estabilidad,
    // pero la lógica de temporización principal está dentro de cada función.
    delay(10);
}

// =================================================================
// --- FUNCIONES DE CONTROL ---
// =================================================================

/**
 * @brief Gestiona toda la lógica del seguidor solar automático.
 * Lee los LDRs, calcula las diferencias de luz y mueve los servos correspondientes.
 */
void actualizarSistemaSeguidor()
{
    // 1. Leer los valores de las 4 fotorresistencias
    int ldrTopLeft = analogRead(LDR_TOP_LEFT_PIN);
    int ldrTopRight = analogRead(LDR_TOP_RIGHT_PIN);
    int ldrBottomLeft = analogRead(LDR_BOTTOM_LEFT_PIN);
    int ldrBottomRight = analogRead(LDR_BOTTOM_RIGHT_PIN);

    // 2. Calcular los promedios de luz por eje para determinar la dirección de la fuente de luz
    int avgTop = (ldrTopLeft + ldrTopRight) / 2;
    int avgBottom = (ldrBottomLeft + ldrBottomRight) / 2;
    int avgLeft = (ldrTopLeft + ldrBottomLeft) / 2;
    int avgRight = (ldrTopRight + ldrBottomRight) / 2;

    // 3. Calcular la diferencia de luz entre los ejes opuestos
    // Una diferencia positiva significa que la luz es más intensa en el primer término (ej. avgTop)
    int diffVertical = avgTop - avgBottom;
    int diffHorizontal = avgLeft - avgRight;

    // 4. Ajustar la sensibilidad (tolerancia) y velocidad de respuesta (delay)
    // basado en la cantidad de luz general. Esto hace que el sistema sea menos "nervioso" con poca luz.
    int luzGeneral = (avgTop + avgBottom) / 2; // Un promedio general de la luz vertical
    if (luzGeneral > 10 && luzGeneral < 300)
    {
        tolerancia = map(luzGeneral, 10, 300, 5, 100);
        tiempoDelaySeguidor = map(luzGeneral, 10, 300, 100, 50);
    }
    else
    {
        tolerancia = 50;
        tiempoDelaySeguidor = 50;
    }

    // 5. Mover el servo vertical si la diferencia de luz supera la tolerancia
    if (abs(diffVertical) > tolerancia)
    {
        // Si hay más luz arriba (diffVertical > 0), el servo sube (incrementa el ángulo)
        posSeguidorV += (diffVertical > 0) ? 1 : -1;
    }

    // 6. Mover el servo horizontal si la diferencia de luz supera la tolerancia
    if (abs(diffHorizontal) > tolerancia)
    {
        // Si hay más luz a la izquierda (diffHorizontal > 0), el servo se mueve a la izquierda.
        // NOTA: El incremento o decremento (+1 o -1) puede necesitar ser invertido
        // dependiendo de cómo hayas montado físicamente tu servo.
        posSeguidorH += (diffHorizontal > 0) ? 1 : -1;
    }

    // 7. Limitar las posiciones para que no excedan los límites físicos del servo
    posSeguidorV = constrain(posSeguidorV, SERVO_LIMITE_BAJO, SERVO_LIMITE_ALTO);
    posSeguidorH = constrain(posSeguidorH, SERVO_LIMITE_BAJO, SERVO_LIMITE_ALTO);

    // 8. Enviar la nueva posición a los servos del seguidor
    servoSeguidorV.write(posSeguidorV);
    servoSeguidorH.write(posSeguidorH);

    // 9. Esperar un breve momento para dar tiempo a que el movimiento se complete
    // y para que el sistema se estabilice antes de la siguiente lectura.
    delay(tiempoDelaySeguidor);
}

/**
 * @brief Gestiona el panel de control manual.
 * Lee los potenciómetros y ajusta la posición de sus servos correspondientes.
 */
void actualizarSistemaManual()
{
    // 1. Leer el valor de los dos potenciómetros (rango 0-1023)
    int potValorH = analogRead(POT_MANUAL_H_PIN);
    int potValorV = analogRead(POT_MANUAL_V_PIN);

    // 2. Mapear (convertir) el rango del potenciómetro al rango del servo (0-180)
    int anguloH = map(potValorH, 0, 1023, SERVO_LIMITE_BAJO, SERVO_LIMITE_ALTO);
    int anguloV = map(potValorV, 0, 1023, SERVO_LIMITE_BAJO, SERVO_LIMITE_ALTO);

    // 3. Escribir el ángulo directamente en los servos del sistema manual
    servoManualH.write(anguloH);
    servoManualV.write(anguloV);
}
