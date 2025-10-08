/*
 * Ejemplo 04: UAV Control con PID_Manager
 * 
 * Sistema de control en cascada para quadcopter
 * Usa PID_Manager para gestión eficiente de múltiples PIDs
 * 
 * Arquitectura:
 * - Lazos externos: Roll, Pitch, Yaw (actitud) -> comandos de rate
 * - Lazos internos: Roll rate, Pitch rate, Yaw rate -> outputs a motores
 * - Altitude control: Altitude -> Vertical speed -> Throttle adjustment
 * 
 * Hardware simulado:
 * - IMU (MPU6050, BMI088, etc)
 * - Barómetro/Sonar
 * - 4 ESCs para motores
 * 
 * NOTA: Este es un ejemplo educativo. Para UAV real, usar librerías
 * especializadas como Betaflight o ArduPilot
 */

#include <PID_Control.h>

// Crear PID Manager a 100 Hz (10ms)
PID_Manager flightController(10);

// IDs de los PIDs
enum PID_IDS {
    // Actitud (lazos externos)
    PID_ROLL = 0,
    PID_PITCH,
    PID_YAW,
    
    // Velocidad angular (lazos internos)
    PID_ROLL_RATE,
    PID_PITCH_RATE,
    PID_YAW_RATE,
    
    // Altitud
    PID_ALTITUDE,
    PID_VERTICAL_SPEED
};

// Variables de estado (simuladas - reemplazar con IMU real)
struct FlightState {
    // Actitud (grados)
    float roll;
    float pitch;
    float yaw;
    
    // Velocidad angular (grados/s)
    float rollRate;
    float pitchRate;
    float yawRate;
    
    // Altitud (metros)
    float altitude;
    float verticalSpeed;
} state;

// Comandos del piloto (RC o autónomo)
struct FlightCommands {
    float rollTarget;
    float pitchTarget;
    float yawTarget;
    float throttle;
    float altitudeTarget;
} commands;

// Outputs de motores (1000-2000 µs para ESCs)
struct MotorOutputs {
    int motor1;  // Front right
    int motor2;  // Rear left
    int motor3;  // Front left
    int motor4;  // Rear right
} motors;

void setup() {
    Serial.begin(115200);
    Serial.println("=== UAV Flight Controller - PID Manager ===");
    
    // ========== CONFIGURAR PIDs ==========
    
    // Lazos de ACTITUD (externos)
    // Output: comando de rate en grados/s (-250 a 250)
    flightController.addPID(4.0, 0.0, 0.5, -250, 250);  // ROLL
    flightController.addPID(4.0, 0.0, 0.5, -250, 250);  // PITCH
    flightController.addPID(2.0, 0.0, 0.3, -250, 250);  // YAW
    
    // Lazos de RATE (internos)
    // Output: ajuste PWM a motores (-400 a 400)
    flightController.addPID(1.5, 0.3, 0.05, -400, 400);  // ROLL_RATE
    flightController.addPID(1.5, 0.3, 0.05, -400, 400);  // PITCH_RATE
    flightController.addPID(1.2, 0.2, 0.0, -400, 400);   // YAW_RATE (sin D)
    
    // Control de ALTITUD
    flightController.addPID(2.5, 0.1, 1.0, -200, 200);  // ALTITUDE -> vspeed
    flightController.addPID(3.0, 0.5, 0.1, -500, 500);  // VSPEED -> throttle
    
    // Filtros derivativos agresivos para actitud (IMU ruidoso)
    flightController.setDerivativeFilter(PID_ROLL, 0.3);
    flightController.setDerivativeFilter(PID_PITCH, 0.3);
    flightController.setDerivativeFilter(PID_YAW, 0.3);
    
    // Menos filtrado para rates (giroscopio menos ruidoso)
    flightController.setDerivativeFilter(PID_ROLL_RATE, 0.7);
    flightController.setDerivativeFilter(PID_PITCH_RATE, 0.7);
    
    Serial.println("✓ PIDs configurados: " + String(flightController.getPIDCount()));
    
    // Inicializar comandos
    commands.rollTarget = 0.0;
    commands.pitchTarget = 0.0;
    commands.yawTarget = 0.0;
    commands.throttle = 1000;  // Mínimo
    commands.altitudeTarget = 0.0;
    
    Serial.println("✓ Sistema inicializado");
    Serial.println("Esperando activación...");
}

void loop() {
    // Esperar tiempo de muestreo (10ms = 100Hz)
    if (!flightController.update()) {
        return;  // No es momento de actualizar
    }
    
    // 1. LEER SENSORES
    readSensors();
    
    // 2. LEER COMANDOS RC (simulados aquí)
    readCommands();
    
    // 3. CONTROL EN CASCADA
    
    // --- ACTITUD ---
    // Lazos externos: actitud -> comando de rate
    float rollRateCmd = flightController.compute(PID_ROLL, 
                                                  commands.rollTarget, 
                                                  state.roll);
    
    float pitchRateCmd = flightController.compute(PID_PITCH, 
                                                   commands.pitchTarget, 
                                                   state.pitch);
    
    float yawRateCmd = flightController.compute(PID_YAW, 
                                                 commands.yawTarget, 
                                                 state.yaw);
    
    // Lazos internos: rate -> output de motor
    float rollOutput = flightController.compute(PID_ROLL_RATE, 
                                                 rollRateCmd, 
                                                 state.rollRate);
    
    float pitchOutput = flightController.compute(PID_PITCH_RATE, 
                                                  pitchRateCmd, 
                                                  state.pitchRate);
    
    float yawOutput = flightController.compute(PID_YAW_RATE, 
                                                yawRateCmd, 
                                                state.yawRate);
    
    // --- ALTITUD ---
    float vSpeedCmd = flightController.compute(PID_ALTITUDE, 
                                                commands.altitudeTarget, 
                                                state.altitude);
    
    float throttleAdj = flightController.compute(PID_VERTICAL_SPEED, 
                                                  vSpeedCmd, 
                                                  state.verticalSpeed);
    
    // 4. MEZCLAR OUTPUTS Y ENVIAR A MOTORES
    mixAndSendToMotors(rollOutput, pitchOutput, yawOutput, 
                       commands.throttle + throttleAdj);
    
    // 5. TELEMETRÍA
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 200) {  // 5 Hz
        lastPrint = millis();
        printTelemetry();
    }
}

// Simular lectura de sensores (reemplazar con IMU real)
void readSensors() {
    // En un UAV real:
    // - Leer acelerómetro/giroscopio (MPU6050, BMI088, etc)
    // - Fusionar con Complementary/Kalman/Madgwick filter
    // - Leer barómetro/sonar para altitud
    
    // Simulación simple
    state.roll = random(-100, 100) / 10.0;
    state.pitch = random(-100, 100) / 10.0;
    state.yaw = random(-1800, 1800) / 10.0;
    
    state.rollRate = random(-500, 500) / 10.0;
    state.pitchRate = random(-500, 500) / 10.0;
    state.yawRate = random(-500, 500) / 10.0;
    
    state.altitude = random(0, 1000) / 100.0;
    state.verticalSpeed = random(-200, 200) / 100.0;
}

// Leer comandos del RC (simulados)
void readCommands() {
    // En UAV real: leer PPM/SBUS/CRSF del receptor
    
    // Simulación: comandos fijos
    commands.rollTarget = 5.0;      // 5° roll
    commands.pitchTarget = 0.0;     // 0° pitch
    commands.yawTarget = 0.0;       // Mantener heading
    commands.throttle = 1500;       // Hover throttle
    commands.altitudeTarget = 2.0;  // 2 metros
}

// Motor mixer para quadcopter en X
void mixAndSendToMotors(float roll, float pitch, float yaw, float throttle) {
    // Configuración X:
    //     M3   M1
    //       \ /
    //       / \
    //     M2   M4
    
    motors.motor1 = throttle - roll + pitch + yaw;  // Front Right
    motors.motor2 = throttle + roll - pitch + yaw;  // Rear Left
    motors.motor3 = throttle + roll + pitch - yaw;  // Front Left
    motors.motor4 = throttle - roll - pitch - yaw;  // Rear Right
    
    // Limitar a rango ESC (1000-2000 µs)
    motors.motor1 = constrain(motors.motor1, 1000, 2000);
    motors.motor2 = constrain(motors.motor2, 1000, 2000);
    motors.motor3 = constrain(motors.motor3, 1000, 2000);
    motors.motor4 = constrain(motors.motor4, 1000, 2000);
    
    // Enviar a ESCs (usar Servo library o directamente PWM)
    // servo1.writeMicroseconds(motors.motor1);
    // servo2.writeMicroseconds(motors.motor2);
    // servo3.writeMicroseconds(motors.motor3);
    // servo4.writeMicroseconds(motors.motor4);
}

// Imprimir telemetría
void printTelemetry() {
    Serial.println("========================================");
    Serial.println("ACTITUD:");
    Serial.println("  Roll:  " + String(state.roll, 1) + "° (cmd: " + 
                   String(commands.rollTarget, 1) + "°)");
    Serial.println("  Pitch: " + String(state.pitch, 1) + "° (cmd: " + 
                   String(commands.pitchTarget, 1) + "°)");
    Serial.println("  Yaw:   " + String(state.yaw, 1) + "°");
    
    Serial.println("RATES:");
    Serial.println("  Roll:  " + String(state.rollRate, 1) + "°/s");
    Serial.println("  Pitch: " + String(state.pitchRate, 1) + "°/s");
    Serial.println("  Yaw:   " + String(state.yawRate, 1) + "°/s");
    
    Serial.println("ALTITUD:");
    Serial.println("  Alt: " + String(state.altitude, 2) + " m (cmd: " + 
                   String(commands.altitudeTarget, 1) + " m)");
    Serial.println("  VSpd: " + String(state.verticalSpeed, 2) + " m/s");
    
    Serial.println("MOTORES:");
    Serial.println("  M1: " + String(motors.motor1) + " µs");
    Serial.println("  M2: " + String(motors.motor2) + " µs");
    Serial.println("  M3: " + String(motors.motor3) + " µs");
    Serial.println("  M4: " + String(motors.motor4) + " µs");
    
    Serial.println("PID STATS:");
    Serial.println("  Roll Error: " + String(flightController.getError(PID_ROLL), 2));
    Serial.println("  Pitch Error: " + String(flightController.getError(PID_PITCH), 2));
    Serial.println("  dt: " + String(flightController.getDeltaTime() * 1000, 2) + " ms");
}

/*
 * NOTAS IMPORTANTES:
 * 
 * 1. Este es un EJEMPLO EDUCATIVO - NO volar con este código sin:
 *    - Implementar sensor fusion (Complementary/Kalman/Madgwick)
 *    - Failsafes apropiados
 *    - Calibración completa
 *    - Pruebas exhaustivas en banco
 * 
 * 2. Para UAV real, considerar frameworks establecidos:
 *    - Betaflight (racing/acro)
 *    - ArduPilot (autónomo)
 *    - PX4 (profesional)
 * 
 * 3. Ajustar ganancias PID mediante:
 *    - Método Ziegler-Nichols
 *    - Auto-tuning
 *    - Prueba y error en simulador
 * 
 * 4. Librería recomendada para IMU:
 *    - MPU6050_tockn
 *    - Adafruit_MPU6050
 *    - Madgwick filter
 */