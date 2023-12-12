#include <Arduino.h>
#include <PID_controller.h>

PID_controller::PID_controller(double kp, double ki, double kd, unsigned long windup_time)
{
    _kp = kp; 
    _ki = ki;
    _kd = kd; 
    _int_time = millis(); 
    _windup_time = windup_time; 
}

float PID_controller::compute(float setpoint, float input)
{
    _currentTime = millis();
    _elapsedTime = _currentTime - _previousTime; 
    _Error = setpoint - input; 
    _intError_current = (_Error + _prevError)*_elapsedTime/2; //Integral calculated as first degree regresion
    _intError += _intError_current; //Acumulated error 
    _devError = (_Error - _prevError)/_elapsedTime; //First degree regresion slope 
    if (_int_time - _prev_int_time> _windup_time){
        _prev_int_time = _int_time; 
        _intError = 0; 
    }
    float output = _kp*_Error + _ki*_intError + _kd*_devError; 
    _prevError = _Error; 
    _previousTime = _currentTime; 
    return output; 
}

float PID_controller::getError(){
    return _Error;
}

void PID_controller::setMaxError(float maxError){
    _maxError = maxError; 
}

float PID_controller::getMax_correction(){
    _maxCorrection = _kp*_maxError + _ki*_maxError + _kd*_maxError; 
    return _maxError; 
}
