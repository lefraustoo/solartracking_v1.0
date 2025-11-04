/*
 * CÓDIGO DE CONTROLADOR (PARA ARDUINO UNO)
 * * Tarea: Mover dos sistemas de servos.
 * 1. Seguidor Automático: 2 servos (H/V) controlados por 4 LDRs.
 * 2. Panel Manual: 2 servos (H/V) controlados por 2 Potenciómetros.
 *
 * MEJORA CLAVE:
 * - Se eliminó todo el código bloqueante (delay()).
 * - Se usan dos temporizadores 'millis()' independientes para cada sistema.
 * - El control manual ahora es 100% responsivo e instantáneo,
 * incluso mientras el seguidor automático se está moviendo.
 */

#include <Arduino.h>
#include <Servo.h>

// --- PINES Y CONFIGURACIÓN DEL SISTEMA 1: SEGUIDOR SOLAR AUTOMÁTICO ---
#define SERVO_SEGUIDOR_H_PIN 5 // Servo Horizontal del Seguidor
#define SERVO_SEGUIDOR_V_PIN 6 // Servo Vertical del Seguidor

#define LDR_TOP_LEFT_PIN A0     // LDR Superior Izquierda
#define LDR_TOP_RIGHT_PIN A1    // LDR Superior Derecha
#define LDR_BOTTOM_LEFT_PIN A2  // LDR Inferior Izquierda
#define LDR_BOTTOM_RIGHT_PIN A3 // LDR Inferior Derecha

// --- PINES Y CONFIGURACIÓN DEL SISTEMA 2: PANEL DE CONTROL MANUAL ---
#define SERVO_MANUAL_H_PIN 9  // Servo Horizontal del panel Manual
#define SERVO_MANUAL_V_PIN 10 // Servo Vertical del panel Manual

#define POT_MANUAL_H_PIN A4 // Potenciómetro para el eje Horizontal
#define POT_MANUAL_V_PIN A5 // Potenciómetro para el eje Vertical

// --- OBJETOS SERVO (UNO PARA CADA MOTOR) ---
Servo servoSeguidorH, servoSeguidorV;
Servo servoManualH, servoManualV;

// --- VARIABLES GLOBALES: SEGUIDOR AUTOMÁTICO ---
int posSeguidorH = 90;
int posSeguidorV = 90;
int tolerancia = 100;
unsigned long intervaloSeguidor = 50; // Intervalo de actualización (en lugar de delay)
unsigned long prevMillisSeguidor = 0;

// --- VARIABLES GLOBALES: CONTROL MANUAL ---
const unsigned long INTERVALO_MANUAL = 25; // Actualizar control manual cada 25ms (súper responsivo)
unsigned long prevMillisManual = 0;

// --- LÍMITES DE MOVIMIENTO ---
const int SERVO_LIMITE_BAJO = 0;
const int SERVO_LIMITE_ALTO = 180;

// --- DECLARACIÓN DE FUNCIONES ---
void actualizarSistemaSeguidor();
void actualizarSistemaManual();

// =================================================================
// --- SETUP: INICIALIZACIÓN DEL SISTEMA ---
// =================================================================
void setup()
{
    Serial.begin(9600);

    // --- Inicialización del Sistema 1: Seguidor Automático ---
    servoSeguidorH.attach(SERVO_SEGUIDOR_H_PIN);
    servoSeguidorV.attach(SERVO_SEGUIDOR_V_PIN);
    servoSeguidorH.write(posSeguidorH);
    servoSeguidorV.write(posSeguidorV);

    // --- Inicialización del Sistema 2: Control Manual ---
    servoManualH.attach(SERVO_MANUAL_H_PIN);
    servoManualV.attach(SERVO_MANUAL_V_PIN);

    // Los pines analógicos (A0-A5) no necesitan pinMode.

    Serial.println("Controlador de paneles (UNO) inicializado.");
}

// =================================================================
// --- LOOP PRINCIPAL (NO BLOQUEANTE) ---
// =================================================================
void loop()
{
    unsigned long currentMillis = millis(); // Obtener el tiempo actual UNA VEZ

    // --- Gestor del Sistema 1: Seguidor Automático ---
    if (currentMillis - prevMillisSeguidor >= intervaloSeguidor)
    {
        prevMillisSeguidor = currentMillis; // Actualizar el tiempo
        actualizarSistemaSeguidor();        // Ejecutar la lógica del seguidor
    }

    // --- Gestor del Sistema 2: Control Manual ---
    if (currentMillis - prevMillisManual >= INTERVALO_MANUAL)
    {
        prevMillisManual = currentMillis; // Actualizar el tiempo
        actualizarSistemaManual();        // Ejecutar la lógica manual
    }
}

// =================================================================
// --- FUNCIONES DE CONTROL ---
// =================================================================

/**
 * @brief Gestiona toda la lógica del seguidor solar automático.
 * Lee LDRs, calcula diferencias y mueve los servos (SIN DELAY).
 */
void actualizarSistemaSeguidor()
{
    // 1. Leer los valores de las 4 fotorresistencias
    int ldrTopLeft = analogRead(LDR_TOP_LEFT_PIN);
    int ldrTopRight = analogRead(LDR_TOP_RIGHT_PIN);
    int ldrBottomLeft = analogRead(LDR_BOTTOM_LEFT_PIN);
    int ldrBottomRight = analogRead(LDR_BOTTOM_RIGHT_PIN);

    // 2. Calcular los promedios de luz
    int avgTop = (ldrTopLeft + ldrTopRight) / 2;
    int avgBottom = (ldrBottomLeft + ldrBottomRight) / 2;
    int avgLeft = (ldrTopLeft + ldrBottomLeft) / 2;
    int avgRight = (ldrTopRight + ldrBottomRight) / 2;

    // 3. Calcular la diferencia (error)
    int diffVertical = avgTop - avgBottom;
    int diffHorizontal = avgLeft - avgRight;

    // 4. Ajustar la sensibilidad (tolerancia) y velocidad (intervalo)
    int luzGeneral = (avgTop + avgBottom) / 2;
    if (luzGeneral > 10 && luzGeneral < 300)
    {
        tolerancia = map(luzGeneral, 10, 300, 5, 100);
        // En lugar de un delay, cambiamos el intervalo de la PRÓXIMA ejecución
        intervaloSeguidor = map(luzGeneral, 10, 300, 100, 50); // Más luz = más rápido (menos intervalo)
    }
    else
    {
        tolerancia = 50;
        intervaloSeguidor = 50;
    }

    // 5. Mover el servo vertical si el error supera la tolerancia
    if (abs(diffVertical) > tolerancia)
    {
        posSeguidorV += (diffVertical > 0) ? 1 : -1;
    }

    // 6. Mover el servo horizontal
    if (abs(diffHorizontal) > tolerancia)
    {
        // NOTA: Invierte el +1/-1 si tu servo se mueve al revés
        posSeguidorH += (diffHorizontal > 0) ? 1 : -1;
    }

    // 7. Limitar las posiciones
    posSeguidorV = constrain(posSeguidorV, SERVO_LIMITE_BAJO, SERVO_LIMITE_ALTO);
    posSeguidorH = constrain(posSeguidorH, SERVO_LIMITE_BAJO, SERVO_LIMITE_ALTO);

    // 8. Enviar la nueva posición a los servos
    servoSeguidorV.write(posSeguidorV);
    servoSeguidorH.write(posSeguidorH);

    // 9. ¡No hay delay! El loop principal se encarga del tiempo.
}

/**
 * @brief Gestiona el panel de control manual.
 * Lee los potenciómetros y ajusta la posición de sus servos.
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
