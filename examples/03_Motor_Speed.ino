/*
 * Ejemplo 03: Control de Velocidad de Motor
 * 
 * Control PID de velocidad usando encoder óptico
 * Motor DC con puente H (L298N, TB6612, etc)
 * 
 * Hardware:
 * - Motor DC con encoder
 * - Puente H conectado a:
 *   - PWM pin 9
 *   - DIR pin 8
 * - Encoder en pines 2 y 3 (interrupciones)
 * - Botón en pin 4 para cambiar velocidad
 */

#include <PID_Control.h>

// PID para velocidad de motor
// Ganancias ajustadas para respuesta rápida
PID_controller motorPID(1.5, 0.3, 0.05, 5000);

// Pines
const int PWM_PIN = 9;
const int DIR_PIN = 8;
const int ENCODER_A = 2;
const int ENCODER_B = 3;
const int BUTTON_PIN = 4;

// Variables del encoder
volatile long encoderCount = 0;
volatile int lastEncoded = 0;

// Variables de control
float targetRPM = 100.0;      // RPM objetivo
float currentRPM = 0.0;       // RPM actual
float motorOutput = 0.0;      // Salida del PID

// Configuración del encoder
const int PULSES_PER_REV = 360;  // Pulsos por revolución (ajustar según tu encoder)
const int SAMPLE_TIME_MS = 50;    // 20 Hz

// Variables para cálculo de RPM
unsigned long lastTime = 0;
long lastEncoderCount = 0;

void setup() {
    Serial.begin(115200);
    Serial.println("=== PID Motor Speed Controller ===");
    
    // Configurar pines
    pinMode(PWM_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    
    // Configurar interrupciones del encoder
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B), updateEncoder, CHANGE);
    
    // Configurar PID
    motorPID.setOutputLimits(-255, 255);  // Bidireccional
    motorPID.setDerivativeFilter(0.9);    // Poco filtrado (velocidad es menos ruidosa)
    
    Serial.println("Target RPM: " + String(targetRPM));
    Serial.println("Sistema iniciado!");
    
    lastTime = millis();
}

void loop() {
    // Calcular RPM cada SAMPLE_TIME_MS
    static unsigned long lastSample = 0;
    if (millis() - lastSample >= SAMPLE_TIME_MS) {
        lastSample = millis();
        
        // Calcular RPM
        currentRPM = calculateRPM();
        
        // Calcular salida PID
        motorOutput = motorPID.compute(targetRPM, currentRPM);
        
        // Aplicar salida al motor
        setMotorSpeed(motorOutput);
    }
    
    // Verificar botón para cambiar velocidad
    static unsigned long lastButtonPress = 0;
    static bool lastButtonState = HIGH;
    bool buttonState = digitalRead(BUTTON_PIN);
    
    if (buttonState == LOW && lastButtonState == HIGH && 
        millis() - lastButtonPress > 500) {
        lastButtonPress = millis();
        
        // Ciclar entre velocidades
        if (targetRPM == 100.0) targetRPM = 200.0;
        else if (targetRPM == 200.0) targetRPM = 300.0;
        else if (targetRPM == 300.0) targetRPM = 0.0;
        else targetRPM = 100.0;
        
        Serial.println(">>> New Target: " + String(targetRPM) + " RPM");
    }
    lastButtonState = buttonState;
    
    // Mostrar información cada 500ms
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 500) {
        lastPrint = millis();
        
        Serial.println("---------------------------");
        Serial.println("Target:  " + String(targetRPM, 1) + " RPM");
        Serial.println("Current: " + String(currentRPM, 1) + " RPM");
        Serial.println("Error:   " + String(motorPID.getError(), 1) + " RPM");
        Serial.println("Output:  " + String(motorOutput, 1) + " / 255");
        Serial.println("P: " + String(motorPID.getError() * 1.5, 1) + 
                     " I: " + String(motorPID.getIntegralError(), 1) + 
                     " D: " + String(motorPID.getDerivativeError(), 2));
    }
    
    // Comandos serial
    if (Serial.available() > 0) {
        float newRPM = Serial.parseFloat();
        if (newRPM >= -500.0 && newRPM <= 500.0) {
            targetRPM = newRPM;
            Serial.println("✓ New target: " + String(targetRPM) + " RPM");
        }
    }
}

// Interrupción del encoder
void updateEncoder() {
    int MSB = digitalRead(ENCODER_A);
    int LSB = digitalRead(ENCODER_B);
    
    int encoded = (MSB << 1) | LSB;
    int sum = (lastEncoded << 2) | encoded;
    
    if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
        encoderCount++;
    }
    if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
        encoderCount--;
    }
    
    lastEncoded = encoded;
}

// Calcular RPM desde encoder
float calculateRPM() {
    unsigned long currentTime = millis();
    long currentCount = encoderCount;
    
    // Calcular diferencia de tiempo y pulsos
    float deltaTime = (currentTime - lastTime) / 1000.0;  // segundos
    long deltaPulses = currentCount - lastEncoderCount;
    
    // Calcular RPM
    float rpm = 0.0;
    if (deltaTime > 0) {
        float rps = (deltaPulses / (float)PULSES_PER_REV) / deltaTime;
        rpm = rps * 60.0;
    }
    
    // Actualizar variables
    lastTime = currentTime;
    lastEncoderCount = currentCount;
    
    return rpm;
}

// Aplicar velocidad al motor
void setMotorSpeed(float speed) {
    // Determinar dirección
    if (speed >= 0) {
        digitalWrite(DIR_PIN, HIGH);
    } else {
        digitalWrite(DIR_PIN, LOW);
        speed = -speed;  // Convertir a positivo
    }
    
    // Aplicar PWM
    int pwmValue = constrain((int)speed, 0, 255);
    analogWrite(PWM_PIN, pwmValue);
}

/*
 * NOTAS DE CALIBRACIÓN:
 * 
 * 1. Ajustar PULSES_PER_REV según tu encoder:
 *    - Típicamente 360, 600, 1024 PPR
 *    - Si usas encoder con reducción: PPR x Reducción
 * 
 * 2. Ajustar ganancias PID según inercia del motor:
 *    - Motor pequeño/rápido: Kp bajo, Kd bajo
 *    - Motor grande/lento: Kp alto, Ki más alto
 * 
 * 3. Si el motor oscila:
 *    - Reducir Kp
 *    - Aumentar filtro derivativo (reducir alpha)
 * 
 * 4. Si tarda en alcanzar velocidad:
 *    - Aumentar Kp
 *    - Aumentar Ki (cuidado con overshoot)
 * 
 * 5. Para motores brushless, adaptar según tu ESC
 */