#ifndef PID_controller_h 
#define PID_controller_h 

#include <Arduino.h>

class PID_controller
{
    public: 
        PID_controller(double kp, double ki, double kd, unsigned long windup_time); 
        float compute(float setpoint, float input); 
        float getError(); 
        float getMax_correction(); 
        void setMaxError(float maxError); 
    private: 
        unsigned long _currentTime, _previousTime, _int_time, _prev_int_time, _windup_time; 
        float _elapsedTime; 
        double _kp, _ki, _kd; 
        float _Error, _prevError, _intError_current, _intError, _devError, _maxError, _maxCorrection; 
}; 


#endif 
