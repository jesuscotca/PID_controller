/*
 * Ejemplo 02: Control de Temperatura
 * 
 * Control PID de temperatura usando un sensor NTC/DHT/DS18B20
 * y un elemento calefactor controlado por PWM
 * 
 * Hardware:
 * - Sensor de temperatura en A0 (o digital según sensor)
 * - MOSFET/Relé SSR en pin 9 para calefactor
 * - LED indicador en pin 13
 * 
 * Este ejemplo usa un sensor analógico simple (NTC)
 * Adaptar según tu sensor específico
 */

#include <PID_Control.h>

// Configuración del sensor NTC
const float BETA = 3950;           // Coeficiente Beta del NTC
const float R_SERIES = 10000;      // Resistencia en serie (ohms)
const float R_NTC_NOMINAL = 10000; // Resistencia del NTC a 25°C
const float T_NOMINAL = 25.0;      // Temperatura nominal (°C)

// PID para temperatura
// Ajustado para respuesta lenta de temperatura
PID_controller tempPID(8.0, 0.2, 2.0, 30000);  // Windup de 30s

// Variables
float targetTemp = 25.0;    // Temperatura objetivo (°C)
float currentTemp = 0.0;    // Temperatura actual
float heaterPWM = 0.0;      // Salida PWM (0-255)

// Configuración
const int HEATER_PIN = 9;
const int LED_PIN = 13;
const int TEMP_SENSOR_PIN = A0;

void setup() {
    Serial.begin(115200);
    Serial.println("=== PID Temperature Controller ===");
    
    // Configurar pines
    pinMode(HEATER_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    
    // Configurar PID
    tempPID.setOutputLimits(0, 255);  // PWM 0-255
    
    // Filtrado agresivo para temperatura (señal ruidosa)
    tempPID.setDerivativeFilter(0.3);
    
    // Anti-windup largo (temperatura cambia lento)
    tempPID.setWindupTime(30000);  // 30 segundos
    
    Serial.println("Target: " + String(targetTemp) + "°C");
    Serial.println("Sistema iniciado!");
    
    // Parpadear LED para indicar inicio
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(200);
        digitalWrite(LED_PIN, LOW);
        delay(200);
    }
}

void loop() {
    // Leer temperatura
    currentTemp = readTemperature();
    
    // Calcular salida PID
    heaterPWM = tempPID.compute(targetTemp, currentTemp);
    
    // Aplicar PWM al calefactor
    analogWrite(HEATER_PIN, (int)heaterPWM);
    
    // LED indicador (ON si está calentando)
    digitalWrite(LED_PIN, heaterPWM > 50 ? HIGH : LOW);
    
    // Mostrar información cada 2 segundos
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 2000) {
        lastPrint = millis();
        
        Serial.println("================================");
        Serial.println("Target:   " + String(targetTemp, 2) + " °C");
        Serial.println("Current:  " + String(currentTemp, 2) + " °C");
        Serial.println("Error:    " + String(tempPID.getError(), 2) + " °C");
        Serial.println("Heater:   " + String((heaterPWM/255.0)*100, 1) + " %");
        Serial.println("Integral: " + String(tempPID.getIntegralError(), 2));
        
        // Advertencia si está muy lejos
        if (abs(tempPID.getError()) > 5.0) {
            Serial.println("⚠ Temperature deviation > 5°C");
        }
    }
    
    // Cambiar setpoint con comandos serial (opcional)
    if (Serial.available() > 0) {
        float newTarget = Serial.parseFloat();
        if (newTarget >= 15.0 && newTarget <= 50.0) {
            targetTemp = newTarget;
            Serial.println("✓ New target: " + String(targetTemp) + "°C");
        }
    }
    
    delay(100);  // 10 Hz de actualización (adecuado para temperatura)
}

// Función para leer temperatura desde NTC
float readTemperature() {
    // Leer ADC
    int adcValue = analogRead(TEMP_SENSOR_PIN);
    
    // Convertir a resistencia
    float resistance = R_SERIES / ((1023.0 / adcValue) - 1.0);
    
    // Ecuación de Steinhart-Hart simplificada (Beta)
    float steinhart;
    steinhart = resistance / R_NTC_NOMINAL;              // (R/Ro)
    steinhart = log(steinhart);                          // ln(R/Ro)
    steinhart /= BETA;                                   // 1/B * ln(R/Ro)
    steinhart += 1.0 / (T_NOMINAL + 273.15);            // + (1/To)
    steinhart = 1.0 / steinhart;                        // Invertir
    steinhart -= 273.15;                                // Convertir a Celsius
    
    return steinhart;
}

/*
 * NOTAS DE CALIBRACIÓN:
 * 
 * 1. Ajusta las constantes BETA, R_SERIES, R_NTC_NOMINAL según tu NTC
 * 2. Si usas DHT11/DHT22, reemplaza readTemperature() con:
 *    #include <DHT.h>
 *    DHT dht(TEMP_SENSOR_PIN, DHT22);
 *    return dht.readTemperature();
 * 
 * 3. Para DS18B20:
 *    #include <OneWire.h>
 *    #include <DallasTemperature.h>
 *    
 * 4. Ajustar ganancias PID según tu sistema:
 *    - Kp alto = respuesta rápida pero puede oscilar
 *    - Ki bajo = acumulación lenta, bueno para offset
 *    - Kd ayuda a suavizar cambios bruscos
 */