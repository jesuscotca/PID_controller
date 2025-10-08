/*
 * PID_Static.h
 * 
 * Controlador PID estático con templates C++
 * Zero overhead abstraction para máxima performance
 * 
 * Características:
 * - Cero overhead de punteros/vtables
 * - Inline completo en compilación
 * - Tamaño conocido en compile-time (~24 bytes por PID)
 * - Perfecto para sistemas críticos en tiempo real
 * 
 * Limitaciones:
 * - Número de PIDs fijo en compile-time
 * - No se pueden añadir/remover PIDs dinámicamente
 * 
 * Uso:
 *   PIDController<8> controller;  // 8 PIDs
 *   
 *   void setup() {
 *     controller.init(0, kp, ki, kd, min, max);
 *   }
 *   
 *   void loop() {
 *     controller.setDeltaTime(dt);
 *     float output = controller.compute(0, setpoint, input);
 *   }
 */

#ifndef PID_STATIC_H
#define PID_STATIC_H

#include <Arduino.h>

// Estructura compacta de estado PID (24 bytes)
struct PIDState {
    // Estado (12 bytes)
    float prevError;
    float integral;
    float prevDerivative;
    
    // Ganancias (12 bytes)
    float kp, ki, kd;
    
    // Límites (4 bytes) - usar int16 en vez de float ahorra memoria
    int16_t outMin, outMax;
    
    // Flags (1 byte)
    uint8_t flags;  // bit 0: enabled
    
    // Padding (3 bytes para alineación a 32-bit)
    uint8_t padding[3];
};

// Clase template para PID estático
template<uint8_t NUM_PIDS>
class PIDController {
public:
    // Constructor
    PIDController() : _dt(0.01f) {
        // Inicializar todos los PIDs
        for (uint8_t i = 0; i < NUM_PIDS; i++) {
            _states[i].prevError = 0;
            _states[i].integral = 0;
            _states[i].prevDerivative = 0;
            _states[i].kp = 0;
            _states[i].ki = 0;
            _states[i].kd = 0;
            _states[i].outMin = -1000;
            _states[i].outMax = 1000;
            _states[i].flags = 0b00000001;  // Habilitado
            _derivAlpha[i] = 1.0f;  // Sin filtro
        }
    }
    
    // Inicializar un PID específico
    inline void init(uint8_t id, float kp, float ki, float kd, int16_t outMin, int16_t outMax) {
        if (id >= NUM_PIDS) return;
        
        _states[id].kp = kp;
        _states[id].ki = ki;
        _states[id].kd = kd;
        _states[id].outMin = outMin;
        _states[id].outMax = outMax;
        _states[id].prevError = 0;
        _states[id].integral = 0;
        _states[id].prevDerivative = 0;
        _states[id].flags = 0b00000001;  // Habilitado
    }
    
    // Establecer dt para todos (llamar una vez por ciclo)
    inline void setDeltaTime(float dt) {
        _dt = dt;
    }
    
    // Compute inline - se optimiza completamente en compilación
    inline float compute(uint8_t id, float setpoint, float input) {
        if (id >= NUM_PIDS) return 0;
        if (!(_states[id].flags & 0b00000001)) return 0;  // Deshabilitado
        
        PIDState &state = _states[id];
        
        // Calcular error
        float error = setpoint - input;
        
        // Derivativo con filtro
        float derivative = (error - state.prevError) / _dt;
        state.prevDerivative = _derivAlpha[id] * derivative + 
                               (1.0f - _derivAlpha[id]) * state.prevDerivative;
        
        // Output preliminar
        float output = state.kp * error + 
                       state.ki * state.integral + 
                       state.kd * state.prevDerivative;
        
        // Anti-windup por saturación
        if (output >= state.outMin && output <= state.outMax) {
            state.integral += error * _dt;
        } else if ((output > state.outMax && error < 0) || 
                   (output < state.outMin && error > 0)) {
            state.integral += error * _dt;
        }
        
        // Recalcular con integral actualizada
        output = state.kp * error + 
                 state.ki * state.integral + 
                 state.kd * state.prevDerivative;
        
        // Saturar salida
        if (output > state.outMax) output = state.outMax;
        else if (output < state.outMin) output = state.outMin;
        
        // Actualizar estado
        state.prevError = error;
        
        return output;
    }
    
    // Reset de un PID
    inline void reset(uint8_t id) {
        if (id >= NUM_PIDS) return;
        _states[id].integral = 0;
        _states[id].prevError = 0;
        _states[id].prevDerivative = 0;
    }
    
    // Reset de todos los PIDs
    inline void resetAll() {
        for (uint8_t i = 0; i < NUM_PIDS; i++) {
            reset(i);
        }
    }
    
    // Configurar filtro derivativo
    inline void setFilter(uint8_t id, float alpha) {
        if (id >= NUM_PIDS) return;
        _derivAlpha[id] = constrain(alpha, 0.0f, 1.0f);
    }
    
    // Cambiar ganancias
    inline void setTunings(uint8_t id, float kp, float ki, float kd) {
        if (id >= NUM_PIDS) return;
        _states[id].kp = kp;
        _states[id].ki = ki;
        _states[id].kd = kd;
    }
    
    // Cambiar límites
    inline void setOutputLimits(uint8_t id, int16_t min, int16_t max) {
        if (id >= NUM_PIDS) return;
        if (min >= max) return;
        _states[id].outMin = min;
        _states[id].outMax = max;
    }
    
    // Habilitar/deshabilitar PID
    inline void enable(uint8_t id, bool enable) {
        if (id >= NUM_PIDS) return;
        if (enable) {
            _states[id].flags |= 0b00000001;
        } else {
            _states[id].flags &= 0b11111110;
        }
    }
    
    // Getters
    inline float getError(uint8_t id) {
        if (id >= NUM_PIDS) return 0;
        return _states[id].prevError;  // Error guardado
    }
    
    inline float getIntegral(uint8_t id) {
        if (id >= NUM_PIDS) return 0;
        return _states[id].integral;
    }
    
    inline float getDerivative(uint8_t id) {
        if (id >= NUM_PIDS) return 0;
        return _states[id].prevDerivative;
    }
    
    // Obtener número de PIDs
    inline uint8_t getNumPIDs() {
        return NUM_PIDS;
    }
    
private:
    PIDState _states[NUM_PIDS];
    float _derivAlpha[NUM_PIDS];
    float _dt;
};

#endif // PID_STATIC_H