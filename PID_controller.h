#ifndef PID_controller_h 
#define PID_controller_h 

#include <Arduino.h>

class PID_controller
{
    public: 
        // Constructor
        PID_controller(double kp, double ki, double kd, unsigned long windup_time = 5000); 
        
        // Métodos principales
        float compute(float setpoint, float input); 
        void reset(); 
        
        // Configuración de límites
        void setOutputLimits(float min, float max); 
        void setMaxError(float maxError); 
        
        // Configuración de anti-windup
        void setWindupTime(unsigned long time);
        void enableWindupLimit(bool enable);
        
        // Configuración de filtro derivativo
        void setDerivativeFilter(float alpha); // 0.0 a 1.0 (0 = máximo filtrado, 1 = sin filtro)
        
        // Configuración de ganancias
        void setTunings(double kp, double ki, double kd);
        
        // Getters
        float getError(); 
        float getIntegralError();
        float getDerivativeError();
        float getMaxCorrection(); 
        
        // Modo manual/automático
        void setMode(bool automatic);
        bool getMode();
        
    private: 
        // Variables de tiempo
        unsigned long _currentTime, _previousTime, _prev_int_time, _windup_time; 
        float _elapsedTime; 
        
        // Ganancias PID
        double _kp, _ki, _kd; 
        
        // Variables de error
        float _Error, _prevError, _intError_current, _intError, _devError; 
        float _maxError, _maxCorrection; 
        
        // Límites de salida
        float _outputMin, _outputMax;
        bool _limitsEnabled;
        
        // Anti-windup
        bool _windupLimitEnabled;
        
        // Filtro derivativo
        float _derivativeAlpha;
        float _filteredDerivative;
        
        // Modo
        bool _automatic;
        float _lastOutput;
        
        // Métodos privados
        float constrainOutput(float output);
}; 

#endif