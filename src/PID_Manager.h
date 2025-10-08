/*
 * PID_Manager.h
 * 
 * Sistema centralizado para múltiples PIDs
 * Optimizado para UAVs y sistemas de control en tiempo real
 * 
 * Características:
 * - Una sola llamada a millis() por ciclo
 * - Memoria compartida eficiente (~32 bytes por PID)
 * - Sincronización perfecta entre todos los PIDs
 * - Hasta 12 PIDs simultáneos (configurable)
 * 
 * Uso típico:
 *   PID_Manager pidManager(10);  // 100 Hz
 *   int8_t pid_id = pidManager.addPID(kp, ki, kd, min, max);
 *   
 *   void loop() {
 *     if (pidManager.update()) {
 *       float output = pidManager.compute(pid_id, setpoint, input);
 *     }
 *   }
 */

#ifndef PID_MANAGER_H
#define PID_MANAGER_H

#include <Arduino.h>

// Configuración del sistema
#define MAX_PID_CONTROLLERS 12  // Máximo número de PIDs
#define DEFAULT_SAMPLE_TIME 10  // ms (100 Hz)

// Estructura compacta para cada PID (32 bytes)
struct PIDConfig {
    // Ganancias (12 bytes)
    float kp;
    float ki;
    float kd;
    
    // Estado (16 bytes)
    float prevError;
    float integral;
    float prevDerivative;  // Para filtro
    float reserved;        // Alineación/futuro uso
    
    // Límites (4 bytes)
    int16_t outMin;
    int16_t outMax;
    
    // Flags (1 byte)
    uint8_t flags;  // bit 0: enabled, bit 1: anti-windup, bits 2-7: reservados
    
    // Padding (1 byte para alineación)
    uint8_t padding;
};

class PID_Manager {
public:
    // Constructor
    PID_Manager(uint16_t sampleTimeMs = DEFAULT_SAMPLE_TIME);
    
    // Gestión de PIDs
    int8_t addPID(float kp, float ki, float kd, int16_t outMin, int16_t outMax);
    void removePID(uint8_t id);
    void enablePID(uint8_t id, bool enable);
    
    // Actualización centralizada
    bool update();  // Retorna true si es momento de actualizar
    float compute(uint8_t id, float setpoint, float input);
    
    // Configuración
    void setTunings(uint8_t id, float kp, float ki, float kd);
    void setOutputLimits(uint8_t id, int16_t min, int16_t max);
    void setDerivativeFilter(uint8_t id, float alpha);
    void setSampleTime(uint16_t ms);
    
    // Reset
    void reset(uint8_t id);
    void resetAll();
    
    // Diagnóstico
    float getError(uint8_t id);
    float getIntegral(uint8_t id);
    float getDerivative(uint8_t id);
    uint32_t getLastUpdateTime();
    float getDeltaTime();
    uint8_t getPIDCount();
    
private:
    // Arrays de datos
    PIDConfig _pids[MAX_PID_CONTROLLERS];
    float _errors[MAX_PID_CONTROLLERS];
    float _derivativeAlpha[MAX_PID_CONTROLLERS];
    
    // Variables de control
    uint8_t _pidCount;
    uint32_t _lastTime;
    uint16_t _sampleTime;
    float _dt;  // Delta time en segundos
    
    // Métodos privados
    inline float constrainOutput(float value, int16_t min, int16_t max);
    inline bool isPIDValid(uint8_t id);
};

#endif // PID_MANAGER_H