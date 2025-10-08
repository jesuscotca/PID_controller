/*
 * Ejemplo 01: Basic PID
 * 
 * Ejemplo básico de uso de la clase PID_controller
 * Control simple de una variable con lectura analógica
 * 
 * Hardware:
 * - Potenciómetro en A0 (input)
 * - LED PWM en pin 9 (output)
 * - Potenciómetro en A1 (setpoint) [opcional]
 */

#include <PID_Control.h>

// Crear controlador PID
// Parámetros: Kp, Ki, Kd, tiempo de windup (ms)
PID_controller myPID(2.0, 0.5, 1.0, 5000);

// Variables
float setpoint = 512.0;  // Valor deseado (0-1023)
float input = 0.0;       // Valor actual del sensor
float output = 0.0;      // Salida del PID (0-255 para PWM)

void setup() {
    Serial.begin(115200);
    Serial.println("=== PID Controller - Basic Example ===");
    
    // Configurar límites de salida para PWM (0-255)
    myPID.setOutputLimits(0, 255);
    
    // Configurar filtro derivativo (0.7 = filtrado moderado)
    myPID.setDerivativeFilter(0.7);
    
    // Configurar pin de salida
    pinMode(9, OUTPUT);
    
    Serial.println("Setup completo!");
    Serial.println("Setpoint: " + String(setpoint));
}

void loop() {
    // Leer entrada (sensor/potenciómetro)
    input = analogRead(A0);
    
    // Opcionalmente leer setpoint desde otro potenciómetro
    // setpoint = analogRead(A1);
    
    // Calcular salida PID
    output = myPID.compute(setpoint, input);
    
    // Aplicar salida al actuador (LED PWM)
    analogWrite(9, (int)output);
    
    // Mostrar información cada 500ms
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 500) {
        lastPrint = millis();
        
        Serial.println("------------------");
        Serial.println("Setpoint: " + String(setpoint, 2));
        Serial.println("Input:    " + String(input, 2));
        Serial.println("Output:   " + String(output, 2));
        Serial.println("Error:    " + String(myPID.getError(), 2));
        Serial.println("P: " + String(myPID.getError() * 2.0, 2) + 
                     " I: " + String(myPID.getIntegralError(), 2) + 
                     " D: " + String(myPID.getDerivativeError(), 2));
    }
    
    delay(10);  // 100 Hz de actualización
}