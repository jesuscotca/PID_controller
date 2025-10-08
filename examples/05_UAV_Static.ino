/*
 * Ejemplo 05: UAV Control con PID_Static
 * 
 * Sistema de control de máxima performance usando templates
 * Ideal para racing drones o sistemas con alta frecuencia de control
 * 
 * Ventajas vs PID_Manager:
 * - Inline completo (más rápido)
 * - Menos memoria RAM
 * - Perfecto para loops >500 Hz
 * 
 * Desventajas:
 * - Número de PIDs fijo en compile-time
 * - Menos flexibilidad dinámica
 */

#include <PID_Control.h>

// Crear controlador estático con 8 PIDs
PIDController<8> flightController;

// IDs de PIDs (enum para claridad)
enum {
    PID_ROLL = 0,
    PID_PITCH,
    PID_YAW,
    PID_ROLL_RATE,
    PID_PITCH_RATE,
    PID_YAW_RATE,
    PID_ALT,
    PID_VSPEED
};

// Estado del UAV
struct {
    float roll, pitch, yaw;
    float rollRate, pitchRate, yawRate;
    float altitude, vSpeed;
} state;

// Comandos
struct {
    float rollCmd, pitchCmd, yawCmd;
    float throttle;
    float altCmd;
} cmd;

// Outputs
struct {
    int16_t m1, m2, m3, m4;
} motors;

// Timing para alta frecuencia
uint32_t lastLoop = 0;
float dt = 0.0;

void setup() {
    Serial.begin(115200);
    Serial.println("=== UAV Static PID - High Performance ===");
    
    // ========== INICIALIZAR PIDs ==========
    
    // Actitud (lazos externos)
    flightController.init(PID_ROLL,       4.0,  0.0, 0.5, -250, 250);
    flightController.init(PID_PITCH,      4.0,  0.0, 0.5, -250, 250);
    flightController.init(PID_YAW,        2.0,  0.0, 0.3, -250, 250);
    
    // Rate (lazos internos) - ganancias más agresivas para racing
    flightController.init(PID_ROLL_RATE,  2.0,  0.5, 0.08, -500, 500);
    flightController.init(PID_PITCH_RATE, 2.0,  0.5, 0.08, -500, 500);
    flightController.init(PID_YAW_RATE,   1.5,  0.3, 0.0,  -500, 500);
    
    // Altitud
    flightController.init(PID_ALT,        2.5,  0.1, 1.0,  -200, 200);
    flightController.init(PID_VSPEED,     3.0,  0.5, 0.1,  -500, 500);
    
    // Filtros derivativos
    flightController.setFilter(PID_ROLL,       0.3);
    flightController.setFilter(PID_PITCH,      0.3);
    flightController.setFilter(PID_ROLL_RATE,  0.8);  // Menos filtrado para racing
    flightController.setFilter(PID_PITCH_RATE, 0.8);
    
    // Inicializar comandos
    cmd.rollCmd = 0.0;
    cmd.pitchCmd = 0.0;
    cmd.yawCmd = 0.0;
    cmd.throttle = 1000;
    cmd.altCmd = 0.0;
    
    Serial.println("✓ PIDs: " + String(flightController.getNumPIDs()));
    Serial.println("✓ Sistema listo");
    
    lastLoop = micros();
}

void loop() {
    // ========== TIMING DE ALTA PRECISIÓN ==========
    uint32_t currentTime = micros();
    dt = (currentTime - lastLoop) / 1000000.0;  // Convertir a segundos
    lastLoop = currentTime;
    
    // Establecer dt una sola vez para todos los PIDs
    flightController.setDeltaTime(dt);
    
    // ========== LECTURA RÁPIDA DE SENSORES ==========
    readSensors();
    
    // ========== LECTURA DE COMANDOS ==========
    readCommands();
    
    // ========== CONTROL EN CASCADA ==========
    
    // Actitud -> Rate command
    float rollRateCmd = flightController.compute(PID_ROLL, cmd.rollCmd, state.roll);
    float pitchRateCmd = flightController.compute(PID_PITCH, cmd.pitchCmd, state.pitch);
    float yawRateCmd = flightController.compute(PID_YAW, cmd.yawCmd, state.yaw);
    
    // Rate -> Motor output
    float rollOut = flightController.compute(PID_ROLL_RATE, rollRateCmd, state.rollRate);
    float pitchOut = flightController.compute(PID_PITCH_RATE, pitchRateCmd, state.pitchRate);
    float yawOut = flightController.compute(PID_YAW_RATE, yawRateCmd, state.yawRate);
    
    // Altitud (opcional, deshabilitar para racing puro)
    float vSpeedCmd = flightController.compute(PID_ALT, cmd.altCmd, state.altitude);
    float throttleAdj = flightController.compute(PID_VSPEED, vSpeedCmd, state.vSpeed);
    
    // ========== MOTOR MIXING ==========
    mixMotors(rollOut, pitchOut, yawOut, cmd.throttle + throttleAdj);
    
    // ========== ENVIAR A ESCs ==========
    // writeMotors();
    
    // ========== TELEMETRÍA (BAJA FRECUENCIA) ==========
    static uint32_t lastTelem = 0;
    if (micros() - lastTelem > 100000) {  // 10 Hz
        lastTelem = micros();
        printTelemetry();
    }
}

void readSensors() {
    // Simulación - reemplazar con IMU real
    state.roll = random(-100, 100) / 10.0;
    state.pitch = random(-100, 100) / 10.0;
    state.yaw = random(-1800, 1800) / 10.0;
    state.rollRate = random(-500, 500) / 10.0;
    state.pitchRate = random(-500, 500) / 10.0;
    state.yawRate = random(-500, 500) / 10.0;
    state.altitude = random(0, 1000) / 100.0;
    state.vSpeed = random(-200, 200) / 100.0;
}

void readCommands() {
    // Simulación - reemplazar con RC real
    cmd.rollCmd = 10.0;     // 10° roll
    cmd.pitchCmd = 5.0;     // 5° pitch forward
    cmd.yawCmd = 0.0;       // No yaw
    cmd.throttle = 1500;    // Hover
    cmd.altCmd = 2.0;       // 2m altitude
}

void mixMotors(float roll, float pitch, float yaw, float throttle) {
    // Motor mixer X-configuration
    motors.m1 = throttle - roll + pitch + yaw;  // FR
    motors.m2 = throttle + roll - pitch + yaw;  // RL
    motors.m3 = throttle + roll + pitch - yaw;  // FL
    motors.m4 = throttle - roll - pitch - yaw;  // RR
    
    // Constrain to ESC range
    motors.m1 = constrain(motors.m1, 1000, 2000);
    motors.m2 = constrain(motors.m2, 1000, 2000);
    motors.m3 = constrain(motors.m3, 1000, 2000);
    motors.m4 = constrain(motors.m4, 1000, 2000);
}

void printTelemetry() {
    // Frecuencia de loop
    static uint32_t loopCount = 0;
    static uint32_t lastFreqCalc = 0;
    static float loopFreq = 0;
    
    loopCount++;
    if (millis() - lastFreqCalc > 1000) {
        loopFreq = loopCount;
        loopCount = 0;
        lastFreqCalc = millis();
    }
    
    Serial.println("========================================");
    Serial.println("PERFORMANCE:");
    Serial.println("  Loop: " + String(loopFreq, 0) + " Hz");
    Serial.println("  dt: " + String(dt * 1000000, 0) + " µs");
    
    Serial.println("ACTITUD:");
    Serial.println("  R/P/Y: " + String(state.roll, 1) + "° / " + 
                   String(state.pitch, 1) + "° / " + String(state.yaw, 1) + "°");
    
    Serial.println("RATES:");
    Serial.println("  R/P/Y: " + String(state.rollRate, 0) + " / " + 
                   String(state.pitchRate, 0) + " / " + 
                   String(state.yawRate, 0) + " °/s");
    
    Serial.println("MOTORES:");
    Serial.println("  M1-4: " + String(motors.m1) + " / " + String(motors.m2) + 
                   " / " + String(motors.m3) + " / " + String(motors.m4));
    
    Serial.println("ERRORS:");
    Serial.println("  Roll: " + String(flightController.getError(PID_ROLL), 2));
    Serial.println("  Pitch: " + String(flightController.getError(PID_PITCH), 2));
}

/*
 * OPTIMIZACIONES PARA MÁXIMA PERFORMANCE:
 * 
 * 1. Usar micros() en vez de millis() para timing preciso
 * 
 * 2. Minimizar Serial.print (solo para debug, no en producción)
 * 
 * 3. Usar DMA para lectura de IMU si está disponible
 * 
 * 4. Considerar usar PROGMEM para constantes
 * 
 * 5. Habilitar optimizaciones del compilador:
 *    En platformio.ini:
 *    build_flags = -O3 -march=native
 * 
 * 6. Para STM32/ESP32, usar FreeRTOS tasks:
 *    - Task de alta prioridad para control (1kHz+)
 *    - Task de baja prioridad para telemetría
 * 
 * 7. Usar IMU con hardware filter (gyro LPF) para reducir carga
 * 
 * FRECUENCIAS TÍPICAS:
 * - Racing drone: 4-8 kHz
 * - Cinematic: 500-1000 Hz
 * - Photo/video: 200-400 Hz
 * - Autónomo: 100-200 Hz
 */