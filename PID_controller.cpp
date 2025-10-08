#include <Arduino.h>
#include <PID_controller.h>

PID_controller::PID_controller(double kp, double ki, double kd, unsigned long windup_time)
{
    // Ganancias
    _kp = kp; 
    _ki = ki;
    _kd = kd; 
    
    // Tiempo de anti-windup
    _windup_time = windup_time;
    _windupLimitEnabled = true;
    
    // Inicializar tiempos
    _previousTime = millis();
    _prev_int_time = millis();
    _currentTime = millis();
    
    // Inicializar errores
    _prevError = 0;
    _intError = 0;
    _Error = 0;
    _devError = 0;
    _intError_current = 0;
    
    // Límites de salida (deshabilitados por defecto)
    _outputMin = -1000000;
    _outputMax = 1000000;
    _limitsEnabled = false;
    
    // Filtro derivativo (sin filtro por defecto)
    _derivativeAlpha = 1.0;
    _filteredDerivative = 0;
    
    // Modo automático por defecto
    _automatic = true;
    _lastOutput = 0;
    
    // Max error
    _maxError = 100;
    _maxCorrection = 0;
}

float PID_controller::compute(float setpoint, float input)
{
    // Si está en modo manual, retornar última salida
    if (!_automatic) {
        return _lastOutput;
    }
    
    // Actualizar tiempo actual
    _currentTime = millis();
    _elapsedTime = (_currentTime - _previousTime) / 1000.0; // Convertir a segundos
    
    // Protección: si el tiempo es muy pequeño o negativo, retornar última salida
    if (_elapsedTime <= 0.001) {
        return _lastOutput;
    }
    
    // Calcular error actual
    _Error = setpoint - input;
    
    // Término integral (método trapezoidal)
    _intError_current = (_Error + _prevError) * _elapsedTime / 2.0;
    
    // Término derivativo (sin filtrar)
    float rawDerivative = (_Error - _prevError) / _elapsedTime;
    
    // Aplicar filtro al derivativo si está configurado
    _filteredDerivative = _derivativeAlpha * rawDerivative + (1.0 - _derivativeAlpha) * _filteredDerivative;
    _devError = _filteredDerivative;
    
    // Calcular salida ANTES de actualizar integral (para anti-windup)
    float output = _kp * _Error + _ki * _intError + _kd * _devError;
    
    // Anti-windup por saturación: solo acumular si no estamos saturados
    if (_limitsEnabled) {
        // Si la salida está dentro de límites, acumular integral
        if (output >= _outputMin && output <= _outputMax) {
            _intError += _intError_current;
        }
        // Si está saturada, solo acumular si ayuda a desaturar
        else if ((output > _outputMax && _intError_current < 0) || 
                 (output < _outputMin && _intError_current > 0)) {
            _intError += _intError_current;
        }
    } else {
        // Sin límites, siempre acumular
        _intError += _intError_current;
    }
    
    // Anti-windup por tiempo: resetear integral periódicamente
    if (_windupLimitEnabled && (_currentTime - _prev_int_time > _windup_time)) {
        _prev_int_time = _currentTime;
        _intError = 0;
    }
    
    // Recalcular salida con integral actualizada
    output = _kp * _Error + _ki * _intError + _kd * _devError;
    
    // Aplicar límites de saturación
    output = constrainOutput(output);
    
    // Guardar valores para la siguiente iteración
    _prevError = _Error;
    _previousTime = _currentTime;
    _lastOutput = output;
    
    return output;
}

void PID_controller::reset()
{
    _intError = 0;
    _prevError = 0;
    _filteredDerivative = 0;
    _previousTime = millis();
    _prev_int_time = millis();
    _lastOutput = 0;
}

void PID_controller::setOutputLimits(float min, float max)
{
    if (min >= max) return; // Validación
    
    _outputMin = min;
    _outputMax = max;
    _limitsEnabled = true;
    
    // Ajustar integral si está fuera de límites
    float maxIntegral = (_outputMax - _kp * _Error - _kd * _devError) / _ki;
    float minIntegral = (_outputMin - _kp * _Error - _kd * _devError) / _ki;
    
    if (_intError > maxIntegral) _intError = maxIntegral;
    if (_intError < minIntegral) _intError = minIntegral;
}

void PID_controller::setMaxError(float maxError)
{
    _maxError = maxError;
}

void PID_controller::setWindupTime(unsigned long time)
{
    _windup_time = time;
}

void PID_controller::enableWindupLimit(bool enable)
{
    _windupLimitEnabled = enable;
    if (enable) {
        _prev_int_time = millis();
    }
}

void PID_controller::setDerivativeFilter(float alpha)
{
    // Limitar alpha entre 0 y 1
    _derivativeAlpha = constrain(alpha, 0.0, 1.0);
}

void PID_controller::setTunings(double kp, double ki, double kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PID_controller::setMode(bool automatic)
{
    // Si cambiamos de manual a automático, hacer un bumpless transfer
    if (!_automatic && automatic) {
        reset(); // Resetear para evitar saltos
    }
    _automatic = automatic;
}

bool PID_controller::getMode()
{
    return _automatic;
}

float PID_controller::getError()
{
    return _Error;
}

float PID_controller::getIntegralError()
{
    return _intError;
}

float PID_controller::getDerivativeError()
{
    return _devError;
}

float PID_controller::getMaxCorrection()
{
    _maxCorrection = _kp * _maxError + _ki * _maxError + _kd * _maxError;
    return _maxCorrection;
}

float PID_controller::constrainOutput(float output)
{
    if (_limitsEnabled) {
        return constrain(output, _outputMin, _outputMax);
    }
    return output;
}