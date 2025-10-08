/*
 * PID_Manager.cpp
 * 
 * Implementación del gestor centralizado de PIDs
 */

#include "PID_Manager.h"

PID_Manager::PID_Manager(uint16_t sampleTimeMs) {
    _pidCount = 0;
    _sampleTime = sampleTimeMs;
    _lastTime = millis();
    _dt = sampleTimeMs / 1000.0;
    
    // Inicializar arrays
    for (uint8_t i = 0; i < MAX_PID_CONTROLLERS; i++) {
        _pids[i].flags = 0;  // Deshabilitado
        _pids[i].prevError = 0;
        _pids[i].integral = 0;
        _pids[i].prevDerivative = 0;
        _errors[i] = 0;
        _derivativeAlpha[i] = 1.0;  // Sin filtro por defecto
    }
}

int8_t PID_Manager::addPID(float kp, float ki, float kd, int16_t outMin, int16_t outMax) {
    if (_pidCount >= MAX_PID_CONTROLLERS) {
        return -1;  // No hay espacio
    }
    
    uint8_t id = _pidCount++;
    
    // Configurar PID
    _pids[id].kp = kp;
    _pids[id].ki = ki;
    _pids[id].kd = kd;
    _pids[id].prevError = 0;
    _pids[id].integral = 0;
    _pids[id].prevDerivative = 0;
    _pids[id].outMin = outMin;
    _pids[id].outMax = outMax;
    _pids[id].flags = 0b00000001;  // Habilitado por defecto
    
    _errors[id] = 0;
    
    return id;
}

void PID_Manager::removePID(uint8_t id) {
    if (!isPIDValid(id)) return;
    _pids[id].flags = 0;  // Deshabilitar
}

void PID_Manager::enablePID(uint8_t id, bool enable) {
    if (!isPIDValid(id)) return;
    
    if (enable) {
        _pids[id].flags |= 0b00000001;
    } else {
        _pids[id].flags &= 0b11111110;
    }
}

bool PID_Manager::update() {
    uint32_t currentTime = millis();
    
    // Verificar si es momento de actualizar
    if (currentTime - _lastTime < _sampleTime) {
        return false;
    }
    
    // Calcular delta time real
    _dt = (currentTime - _lastTime) / 1000.0;
    _lastTime = currentTime;
    
    return true;
}

float PID_Manager::compute(uint8_t id, float setpoint, float input) {
    // Validaciones
    if (!isPIDValid(id)) return 0;
    if (!(_pids[id].flags & 0b00000001)) return 0;  // PID deshabilitado
    
    PIDConfig &pid = _pids[id];
    
    // Calcular error
    float error = setpoint - input;
    _errors[id] = error;
    
    // Término derivativo con filtro
    float derivative = (error - pid.prevError) / _dt;
    pid.prevDerivative = _derivativeAlpha[id] * derivative + 
                         (1.0 - _derivativeAlpha[id]) * pid.prevDerivative;
    
    // Calcular salida preliminar
    float output = pid.kp * error + 
                   pid.ki * pid.integral + 
                   pid.kd * pid.prevDerivative;
    
    // Anti-windup: solo integrar si no estamos saturados
    if (output >= pid.outMin && output <= pid.outMax) {
        // Dentro de límites, acumular normalmente
        pid.integral += error * _dt;
    } else if ((output > pid.outMax && error < 0) || 
               (output < pid.outMin && error > 0)) {
        // Saturado pero el error ayuda a desaturar
        pid.integral += error * _dt;
    }
    // Si saturado y el error empeora la saturación, no acumular
    
    // Recalcular con integral actualizada
    output = pid.kp * error + 
             pid.ki * pid.integral + 
             pid.kd * pid.prevDerivative;
    
    // Aplicar límites
    output = constrainOutput(output, pid.outMin, pid.outMax);
    
    // Actualizar estado
    pid.prevError = error;
    
    return output;
}

void PID_Manager::setTunings(uint8_t id, float kp, float ki, float kd) {
    if (!isPIDValid(id)) return;
    
    _pids[id].kp = kp;
    _pids[id].ki = ki;
    _pids[id].kd = kd;
}

void PID_Manager::setOutputLimits(uint8_t id, int16_t min, int16_t max) {
    if (!isPIDValid(id)) return;
    if (min >= max) return;
    
    _pids[id].outMin = min;
    _pids[id].outMax = max;
}

void PID_Manager::setDerivativeFilter(uint8_t id, float alpha) {
    if (!isPIDValid(id)) return;
    _derivativeAlpha[id] = constrain(alpha, 0.0, 1.0);
}

void PID_Manager::setSampleTime(uint16_t ms) {
    if (ms < 1) return;  // Validación
    _sampleTime = ms;
    _dt = ms / 1000.0;
}

void PID_Manager::reset(uint8_t id) {
    if (!isPIDValid(id)) return;
    
    _pids[id].integral = 0;
    _pids[id].prevError = 0;
    _pids[id].prevDerivative = 0;
    _errors[id] = 0;
}

void PID_Manager::resetAll() {
    for (uint8_t i = 0; i < _pidCount; i++) {
        reset(i);
    }
}

float PID_Manager::getError(uint8_t id) {
    if (!isPIDValid(id)) return 0;
    return _errors[id];
}

float PID_Manager::getIntegral(uint8_t id) {
    if (!isPIDValid(id)) return 0;
    return _pids[id].integral;
}

float PID_Manager::getDerivative(uint8_t id) {
    if (!isPIDValid(id)) return 0;
    return _pids[id].prevDerivative;
}

uint32_t PID_Manager::getLastUpdateTime() {
    return _lastTime;
}

float PID_Manager::getDeltaTime() {
    return _dt;
}

uint8_t PID_Manager::getPIDCount() {
    return _pidCount;
}

inline float PID_Manager::constrainOutput(float value, int16_t min, int16_t max) {
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

inline bool PID_Manager::isPIDValid(uint8_t id) {
    return (id < _pidCount);
}