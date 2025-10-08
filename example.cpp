/*
 * Ejemplo de uso del controlador PID mejorado
 * 
 * Este ejemplo muestra cómo usar todas las funcionalidades
 * de la librería PID_controller mejorada
 */

#include <Arduino.h>
#include <PID_controller.h>

// Crear controlador PID con:
// Kp = 2.0, Ki = 0.5, Kd = 1.0, windup_time = 5000ms
PID_controller myPID(2.0, 0.5, 1.0, 5000);

float setpoint = 100.0;  // Valor deseado
float input = 0.0;       // Valor actual del sensor
float output = 0.0;      // Salida del PID

void setup() {
    Serial.begin(115200);
    
    // ===== CONFIGURACIÓN BÁSICA =====
    
    // Establecer límites de salida (ej: PWM de 0 a 255)
    myPID.setOutputLimits(0, 255);
    
    // Configurar filtro derivativo (0.7 = filtrado moderado)
    // 0.0 = máximo filtrado, 1.0 = sin filtro
    myPID.setDerivativeFilter(0.7);
    
    
    // ===== CONFIGURACIÓN AVANZADA (OPCIONAL) =====
    
    // Cambiar tiempo de anti-windup
    myPID.setWindupTime(10000); // 10 segundos
    
    // Deshabilitar anti-windup por tiempo (solo usar por saturación)
    // myPID.enableWindupLimit(false);
    
    // Cambiar ganancias en tiempo de ejecución
    // myPID.setTunings(3.0, 0.8, 1.5);
    
    // Establecer error máximo esperado (para cálculos)
    myPID.setMaxError(150.0);
    
    Serial.println("PID Controller iniciado");
    Serial.println("Setpoint: " + String(setpoint));
}

void loop() {
    // Leer sensor (simular con ruido)
    input = analogRead(A0) * (100.0 / 1023.0);
    
    // Calcular salida PID
    output = myPID.compute(setpoint, input);
    
    // Aplicar salida (ej: PWM a un motor)
    analogWrite(9, (int)output);
    
    // ===== MONITOREO Y DEBUG =====
    
    // Imprimir información cada 500ms
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 500) {
        lastPrint = millis();
        
        Serial.println("=== PID Status ===");
        Serial.println("Input: " + String(input, 2));
        Serial.println("Setpoint: " + String(setpoint, 2));
        Serial.println("Output: " + String(output, 2));
        Serial.println("Error: " + String(myPID.getError(), 2));
        Serial.println("Integral: " + String(myPID.getIntegralError(), 2));
        Serial.println("Derivative: " + String(myPID.getDerivativeError(), 2));
        Serial.println();
    }
    
    // ===== CAMBIO DE SETPOINT =====
    
    // Cambiar setpoint cada 10 segundos
    static unsigned long lastChange = 0;
    if (millis() - lastChange > 10000) {
        lastChange = millis();
        setpoint = (setpoint == 100.0) ? 150.0 : 100.0;
        Serial.println(">>> Nuevo Setpoint: " + String(setpoint));
    }
    
    // ===== RESET MANUAL (EJEMPLO) =====
    
    // Si presionas un botón conectado al pin 2
    if (digitalRead(2) == HIGH) {
        myPID.reset();
        Serial.println(">>> PID Reset!");
        delay(1000); // Debounce simple
    }
    
    delay(10); // Esperar 10ms (100 Hz de control)
}


// ===== EJEMPLO: MODO MANUAL/AUTOMÁTICO =====

void ejemplo_modo_manual() {
    // Cambiar a modo manual
    myPID.setMode(false);
    
    // En modo manual, el PID retorna el último valor calculado
    // Útil para control manual sin saltos al volver a automático
    
    // Control manual...
    output = 150; // Valor manual
    
    // Volver a automático (hace reset automático para evitar saltos)
    myPID.setMode(true);
}


// ===== EJEMPLO: CONTROL DE TEMPERATURA =====

void ejemplo_control_temperatura() {
    // PID para control de temperatura
    // Kp=8, Ki=0.2, Kd=2, windup=30s
    PID_controller tempPID(8.0, 0.2, 2.0, 30000);
    
    // Límites PWM del calentador (0-255)
    tempPID.setOutputLimits(0, 255);
    
    // Filtrado agresivo del derivativo (para reducir ruido del sensor)
    tempPID.setDerivativeFilter(0.3);
    
    float targetTemp = 25.0; // °C
    float currentTemp = readTemperature(); // Tu función
    
    float heaterPWM = tempPID.compute(targetTemp, currentTemp);
    analogWrite(HEATER_PIN, (int)heaterPWM);
}


// ===== EJEMPLO: CONTROL DE VELOCIDAD DE MOTOR =====

void ejemplo_control_motor() {
    // PID para velocidad de motor
    // Kp=1.5, Ki=0.3, Kd=0.05, windup=5s
    PID_controller motorPID(1.5, 0.3, 0.05, 5000);
    
    // Límites del motor (-255 a 255 para control bidireccional)
    motorPID.setOutputLimits(-255, 255);
    
    // Poco filtrado (la velocidad es menos ruidosa)
    motorPID.setDerivativeFilter(0.9);
    
    float targetRPM = 1000.0;
    float currentRPM = readEncoder(); // Tu función
    
    float motorPWM = motorPID.compute(targetRPM, currentRPM);
    
    // Controlar motor (dirección + PWM)
    if (motorPWM >= 0) {
        digitalWrite(DIR_PIN, HIGH);
        analogWrite(PWM_PIN, abs(motorPWM));
    } else {
        digitalWrite(DIR_PIN, LOW);
        analogWrite(PWM_PIN, abs(motorPWM));
    }
}


// ===== FUNCIÓN AUXILIAR (SIMULACIÓN) =====

float readTemperature() {
    // Simular lectura de temperatura
    return 22.0 + random(-10, 10) / 10.0;
}

float readEncoder() {
    // Simular lectura de encoder
    return 900.0 + random(-50, 50);
}