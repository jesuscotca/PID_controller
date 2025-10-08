# 🎛️ PID_Control

> Librería completa de control PID para Arduino - De principiante a experto

[![Arduino](https://img.shields.io/badge/Arduino-Compatible-00979D?logo=arduino)](https://www.arduino.cc/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Version](https://img.shields.io/badge/version-2.0.0-blue.svg)](https://github.com/tuusuario/PID_Control)

Una librería de control PID diseñada tanto para estudiantes que están aprendiendo como para proyectos profesionales de robótica y automatización. Incluye tres implementaciones optimizadas para diferentes niveles de complejidad.

---

## 📖 Tabla de Contenidos

- [¿Qué es un Controlador PID?](#-qué-es-un-controlador-pid)
- [¿Por qué usar esta librería?](#-por-qué-usar-esta-librería)
- [Instalación](#-instalación)
- [Inicio Rápido](#-inicio-rápido)
- [Las Tres Implementaciones](#-las-tres-implementaciones)
- [Guía Educativa](#-guía-educativa)
- [Ejemplos Incluidos](#-ejemplos-incluidos)
- [API Completa](#-api-completa)
- [Ajuste de Parámetros PID](#-ajuste-de-parámetros-pid)
- [Preguntas Frecuentes](#-preguntas-frecuentes)
- [Troubleshooting](#-troubleshooting)
- [Contribuir](#-contribuir)
- [Licencia](#-licencia)

---

## 🤔 ¿Qué es un Controlador PID?

### Explicación Simple

Imagina que quieres mantener la temperatura de tu habitación en 22°C:

- Si hace 18°C (muy frío), enciendes mucho la calefacción
- Si hace 21°C (casi perfecto), enciendes poco la calefacción
- Si hace 22°C (perfecto), apagas la calefacción

**Un controlador PID hace exactamente esto, pero de forma mucho más inteligente.**

### ¿Qué significa PID?

PID son las siglas de **Proporcional**, **Integral** y **Derivativo**. Son tres formas diferentes de reaccionar al error (la diferencia entre lo que quieres y lo que tienes):

#### 🔴 **P - Proporcional** (El presente)
> "¿Qué tan lejos estoy del objetivo AHORA?"

```
Acción = Kp × Error
```

- **Error grande** → Acción fuerte
- **Error pequeño** → Acción suave
- **Sin error** → Sin acción

**Ejemplo:** Si tu temperatura está 10°C por debajo del objetivo, calientas mucho. Si está 1°C por debajo, calientas poco.

**Problema:** Solo con P, nunca llegas exactamente al objetivo (queda un error residual).

#### 🟡 **I - Integral** (El pasado)
> "¿Cuánto tiempo llevo con error?"

```
Acción = Ki × Suma_de_Errores_Acumulados
```

- Acumula todos los errores del pasado
- Si llevas mucho tiempo con un pequeño error, eventualmente reacciona
- Elimina el error residual que deja el término P

**Ejemplo:** Si llevas 5 minutos con la temperatura 0.5°C por debajo, la integral acumula este error y aumenta gradualmente la calefacción hasta eliminar esa diferencia.

**Problema:** Si se acumula demasiado, puede causar sobrepaso (overshoot).

#### 🟢 **D - Derivativo** (El futuro)
> "¿Qué tan rápido está cambiando el error?"

```
Acción = Kd × Velocidad_de_Cambio_del_Error
```

- Predice hacia dónde va el sistema
- Si el error está disminuyendo rápido, reduce la acción para evitar sobrepaso
- Suaviza la respuesta

**Ejemplo:** Si la temperatura está subiendo rápido hacia el objetivo, el término D reduce la calefacción antes de tiempo para evitar pasarse.

**Problema:** Muy sensible al ruido en las mediciones.

### Combinándolos: El PID Completo

```
Output = Kp×Error + Ki×∫Error + Kd×(dError/dt)
         ↑           ↑            ↑
      Presente    Pasado       Futuro
```

**Kp, Ki, Kd** son las "ganancias" que ajustas para que el controlador funcione bien en tu sistema específico.

---

## 🌟 ¿Por qué usar esta librería?

### ✅ Educativa
- Documentación extensa con explicaciones paso a paso
- 6 ejemplos completos desde básico hasta avanzado
- Comentarios educativos en el código
- Sin conocimientos previos de control necesarios

### ✅ Completa
- Anti-windup dual (evita que la integral "explote")
- Filtro derivativo configurable (reduce ruido)
- Límites de salida
- Modo manual/automático
- Diagnóstico completo (acceso a P, I, D por separado)

### ✅ Eficiente
- Tres implementaciones optimizadas para diferentes casos de uso
- Hasta 3.7× más rápida que implementaciones tradicionales
- Ahorro de memoria de hasta 70%
- Código probado y optimizado

### ✅ Flexible
- Compatible con todos los Arduinos
- Sin dependencias externas
- Fácil de integrar en proyectos existentes
- Open source (MIT License)

---

## 📦 Instalación

### Opción 1: Arduino Library Manager (Recomendado)

1. Abrir Arduino IDE
2. `Sketch` → `Include Library` → `Manage Libraries...`
3. Buscar "PID_Control"
4. Click en `Install`

### Opción 2: Instalación Manual

1. Descargar el ZIP desde [releases](https://github.com/tuusuario/PID_Control/releases)
2. Arduino IDE → `Sketch` → `Include Library` → `Add .ZIP Library`
3. Seleccionar el archivo descargado

### Opción 3: PlatformIO

Agregar a `platformio.ini`:

```ini
lib_deps = 
    https://github.com/tuusuario/PID_Control.git
```

### Opción 4: Git Clone

```bash
cd ~/Arduino/libraries/
git clone https://github.com/tuusuario/PID_Control.git
```

---

## 🚀 Inicio Rápido

### Tu Primer PID en 5 Minutos

**Hardware necesario:**
- Arduino (cualquiera)
- Potenciómetro en A0
- LED en pin 9

**Código:**

```cpp
#include <PID_Control.h>

// Crear controlador PID con ganancias Kp=2.0, Ki=0.5, Kd=1.0
PID_controller myPID(2.0, 0.5, 1.0);

float setpoint = 512.0;  // Valor objetivo (0-1023)
float input = 0.0;       // Valor actual
float output = 0.0;      // Salida del PID

void setup() {
    Serial.begin(115200);
    
    // Configurar límites de salida (PWM: 0-255)
    myPID.setOutputLimits(0, 255);
    
    pinMode(9, OUTPUT);
}

void loop() {
    // Leer entrada
    input = analogRead(A0);
    
    // Calcular salida PID
    output = myPID.compute(setpoint, input);
    
    // Aplicar salida
    analogWrite(9, output);
    
    // Mostrar en serial
    Serial.println("SP:" + String(setpoint) + 
                   " IN:" + String(input) + 
                   " OUT:" + String(output));
    
    delay(10);  // 100 Hz
}
```

**¿Qué hace este código?**

1. Lee un potenciómetro (0-1023)
2. Compara con el valor deseado (512)
3. Calcula cuánto debe encender un LED para compensar la diferencia
4. El LED brilla más cuando estás lejos del objetivo

**Experimenta:**
- Cambia `setpoint` a diferentes valores
- Gira el potenciómetro y observa cómo el LED reacciona
- Ajusta Kp, Ki, Kd y observa los cambios

---

## 🎯 Las Tres Implementaciones

Esta librería incluye tres formas diferentes de usar PID, cada una optimizada para diferentes situaciones:

### 1️⃣ PID_controller - Para Aprender y Prototipar

**¿Cuándo usar?**
- Eres nuevo en control PID
- Solo necesitas 1-3 PIDs
- Estás prototipando
- Quieres máxima flexibilidad

**Ventajas:**
- ✅ Fácil de entender
- ✅ Todas las características disponibles
- ✅ Cambios dinámicos de parámetros
- ✅ Modo manual/automático

**Desventajas:**
- ❌ Más lento (~450 µs por PID)
- ❌ Usa más memoria (~100 bytes por PID)

**Ejemplo:**

```cpp
#include <PID_controller.h>

PID_controller tempPID(8.0, 0.2, 2.0);

void setup() {
    tempPID.setOutputLimits(0, 255);
    tempPID.setDerivativeFilter(0.7);
}

void loop() {
    float output = tempPID.compute(targetTemp, currentTemp);
    analogWrite(heaterPin, output);
}
```

---

### 2️⃣ PID_Manager - Para Robots y UAVs

**¿Cuándo usar?**
- Necesitas 3-12 PIDs simultáneos
- UAV/Quadcopter (roll, pitch, yaw, altitude...)
- Robot con múltiples articulaciones
- Sistema de control complejo

**Ventajas:**
- ✅ Sincronización perfecta entre PIDs
- ✅ Una sola llamada a `millis()` por ciclo
- ✅ Ahorro de memoria (~32 bytes por PID)
- ✅ 2.5× más rápido que individual

**Desventajas:**
- ❌ Requiere planificación inicial
- ❌ Límite de 12 PIDs (configurable)

**Ejemplo - Quadcopter:**

```cpp
#include <PID_Manager.h>

PID_Manager flight(10);  // 100 Hz
int8_t pidRoll, pidPitch, pidYaw;

void setup() {
    // Crear PIDs
    pidRoll = flight.addPID(4.0, 0.0, 0.5, -250, 250);
    pidPitch = flight.addPID(4.0, 0.0, 0.5, -250, 250);
    pidYaw = flight.addPID(2.0, 0.0, 0.3, -250, 250);
}

void loop() {
    if (flight.update()) {  // Solo cuando pase el tiempo de muestreo
        float rollOut = flight.compute(pidRoll, rollTarget, rollActual);
        float pitchOut = flight.compute(pidPitch, pitchTarget, pitchActual);
        float yawOut = flight.compute(pidYaw, yawTarget, yawActual);
        
        // Enviar a motores...
    }
}
```

---

### 3️⃣ PID_Static - Para Máxima Performance

**¿Cuándo usar?**
- Racing drones (necesitas >1000 Hz)
- Sistemas críticos en tiempo real
- Arduino con poca RAM (ATmega328)
- Quieres el último bit de performance

**Ventajas:**
- ✅ 3.7× más rápido que individual
- ✅ Inline completo (cero overhead)
- ✅ Memoria mínima (~24 bytes por PID)
- ✅ Ideal para loops ultra-rápidos

**Desventajas:**
- ❌ Número de PIDs fijo en compile-time
- ❌ Menos flexible

**Ejemplo - Racing Drone:**

```cpp
#include <PID_Static.h>

PIDController<6> racing;  // 6 PIDs fijos

enum { ROLL, PITCH, YAW, ROLL_RATE, PITCH_RATE, YAW_RATE };

void setup() {
    racing.init(ROLL, 4.0, 0.0, 0.5, -500, 500);
    racing.init(PITCH, 4.0, 0.0, 0.5, -500, 500);
    // ...
}

void loop() {
    uint32_t start = micros();
    float dt = (start - lastTime) / 1000000.0;
    lastTime = start;
    
    racing.setDeltaTime(dt);  // Una vez por ciclo
    
    // Todos los computes son inline - súper rápido
    float r = racing.compute(ROLL, rollCmd, rollActual);
    float p = racing.compute(PITCH, pitchCmd, pitchActual);
    // ...
}
```

---

## 📚 Guía Educativa

### Parte 1: Entendiendo el Error

El **error** es la diferencia entre lo que quieres y lo que tienes:

```cpp
Error = Setpoint - Input
```

**Ejemplo práctico:**
- Quieres 25°C (setpoint)
- Tienes 22°C (input)
- Error = 25 - 22 = **3°C**

```cpp
float output = myPID.compute(25.0, 22.0);
// Internamente calcula: Error = 25 - 22 = 3
```

---

### Parte 2: El Término Proporcional (Kp)

El término **P** multiplica el error por una constante **Kp**:

```cpp
P_output = Kp × Error
```

**¿Qué hace Kp?**

- **Kp alto** (ej: 10.0):
  - ✅ Respuesta rápida
  - ✅ Reacciona fuerte a cambios
  - ❌ Puede oscilar
  - ❌ Sobrepaso (overshoot)

- **Kp bajo** (ej: 0.5):
  - ✅ Respuesta suave
  - ✅ No oscila
  - ❌ Respuesta lenta
  - ❌ Puede no llegar al objetivo

**Ejemplo práctico - Control de temperatura:**

```cpp
PID_controller tempPID(5.0, 0.0, 0.0);  // Solo Kp=5

// Objetivo: 25°C, Actual: 20°C
// Error = 5°C
// Output = 5 × 5 = 25 (PWM al calentador)

// Cuando llega a 24°C:
// Error = 1°C
// Output = 5 × 1 = 5 (menos calentamiento)
```

**Problema del término P solo:**

Siempre queda un **error residual**. Si solo usas P, nunca llegas exactamente al objetivo.

---

### Parte 3: El Término Integral (Ki)

El término **I** acumula todos los errores pasados:

```cpp
Integral = Integral + (Error × Δt)
I_output = Ki × Integral
```

**¿Qué hace Ki?**

- **Ki alto** (ej: 1.0):
  - ✅ Elimina error residual rápido
  - ❌ Puede causar sobrepaso
  - ❌ Windup (acumulación excesiva)

- **Ki bajo** (ej: 0.1):
  - ✅ Corrección suave
  - ✅ No causa sobrepaso
  - ❌ Corrección lenta

**Ejemplo - Eliminar error residual:**

```cpp
PID_controller pid(2.0, 0.5, 0.0);  // Kp=2, Ki=0.5

// Temperatura estabilizada en 24.5°C (objetivo: 25°C)
// Error constante = 0.5°C

// Ciclo 1: Integral = 0 + (0.5 × 0.01) = 0.005
// Ciclo 2: Integral = 0.005 + (0.5 × 0.01) = 0.010
// Ciclo 3: Integral = 0.010 + (0.5 × 0.01) = 0.015
// ...después de 100 ciclos: Integral = 0.5
// I_output = 0.5 × 0.5 = 0.25 (empuja gradualmente hasta 25°C)
```

**Problema: Windup**

Si el actuador está saturado (ej: calentador al 100%), el error sigue acumulándose sin efecto. Cuando finalmente se alcanza el objetivo, hay tanta integral acumulada que causa un gran sobrepaso.

**Solución en esta librería:**

```cpp
// Anti-windup automático
myPID.setOutputLimits(0, 255);  // No acumula si está saturado
```

---

### Parte 4: El Término Derivativo (Kd)

El término **D** reacciona a qué tan rápido está cambiando el error:

```cpp
Derivative = (Error - PreviousError) / Δt
D_output = Kd × Derivative
```

**¿Qué hace Kd?**

- Actúa como un "freno"
- Si el error está disminuyendo rápido, reduce la acción
- Previene sobrepaso
- Suaviza la respuesta

**¿Cuándo usar Kd?**

- ✅ Sistema con mucha inercia (motores grandes, temperatura)
- ✅ Prevenir oscilaciones
- ❌ Señales muy ruidosas (amplifica el ruido)

**Ejemplo - Motor con inercia:**

```cpp
PID_controller motorPID(1.5, 0.3, 0.05);  // Incluye Kd=0.05

// Motor acelerando hacia el objetivo
// Velocidad: 900 RPM → 950 RPM → 980 RPM (objetivo: 1000)
// Error disminuye rápido: 100 → 50 → 20

// Sin Kd: El motor se pasaría a 1050 RPM (overshoot)
// Con Kd: Detecta que el error baja rápido y reduce el PWM
//         antes de llegar, estabilizando en 1000 RPM
```

**Problema: Ruido**

El derivativo amplifica el ruido. Si tu sensor tiene lecturas ruidosas:

```
Lectura real: 25.0 → 25.0 → 25.0
Sensor ruidoso: 25.0 → 25.3 → 24.8
```

El derivativo reacciona fuertemente a estos saltos.

**Solución - Filtro derivativo:**

```cpp
myPID.setDerivativeFilter(0.3);  // Filtrado agresivo (0-1)
// 0.0 = máximo filtrado (muy suave)
// 1.0 = sin filtro (reacciona a todo)
```

---

### Parte 5: Combinándolos - El PID Completo

```cpp
PID_controller myPID(Kp, Ki, Kd);

Output = Kp×Error + Ki×∫Error + Kd×(dError/dt)
```

**Analogía del conductor:**

Imagina que conduces un auto hacia un destino:

- **P (Proporcional)**: Giras el volante proporcionalmente a qué tan lejos estás del camino
- **I (Integral)**: Si llevas tiempo ligeramente desviado, corriges gradualmente
- **D (Derivativo)**: Si estás girando rápido hacia el camino, suavizas el giro para no pasarte

---

### Parte 6: ¿Cómo Ajustar Kp, Ki, Kd?

#### Método 1: Empezar de Cero (Recomendado para principiantes)

**Paso 1: Solo Kp**

1. Poner Ki=0, Kd=0
2. Aumentar Kp gradualmente hasta que el sistema oscile
3. Reducir Kp a la mitad de ese valor

```cpp
// Prueba 1: Kp=1 → Muy lento
// Prueba 2: Kp=5 → Todavía lento
// Prueba 3: Kp=10 → Oscila!
// Resultado: Usar Kp=5
PID_controller pid(5.0, 0.0, 0.0);
```

**Paso 2: Añadir Ki**

1. Con Kp fijo, aumentar Ki gradualmente
2. Parar cuando elimine el error residual
3. Si causa sobrepaso, reducir

```cpp
// Con Kp=5 fijo
// Prueba 1: Ki=0.1 → Error residual desaparece lentamente
// Prueba 2: Ki=0.5 → Perfecto!
// Prueba 3: Ki=1.0 → Sobrepaso
PID_controller pid(5.0, 0.5, 0.0);
```

**Paso 3: Añadir Kd (opcional)**

1. Solo si hay oscilaciones o sobrepaso
2. Aumentar gradualmente
3. Parar cuando suavice la respuesta

```cpp
// Con Kp=5, Ki=0.5
// Prueba 1: Kd=0.5 → Un poco mejor
// Prueba 2: Kd=1.0 → Suave!
PID_controller pid(5.0, 0.5, 1.0);
```

#### Método 2: Ziegler-Nichols (Avanzado)

1. Ki=0, Kd=0
2. Aumentar Kp hasta oscilación sostenida
3. Anotar Kp crítico (Ku) y período de oscilación (Tu)
4. Calcular:
   - Kp = 0.6 × Ku
   - Ki = 1.2 × Ku / Tu
   - Kd = 0.075 × Ku × Tu

**Valores típicos para diferentes sistemas:**

| Sistema | Kp | Ki | Kd |
|---------|----|----|-----|
| Temperatura | 5-15 | 0.1-0.5 | 1-3 |
| Motor DC | 1-5 | 0.1-1.0 | 0.01-0.1 |
| Servo posición | 3-10 | 0-0.5 | 0.5-2 |
| Nivel de líquido | 2-8 | 0.05-0.2 | 0.5-1.5 |
| UAV (actitud) | 3-6 | 0-0.1 | 0.3-1 |
| UAV (rate) | 1-3 | 0.2-0.5 | 0.01-0.1 |

---

## 💡 Ejemplos Incluidos

La librería incluye 6 ejemplos progresivos:

### 📘 Ejemplo 1: Basic PID
**Nivel:** Principiante  
**Hardware:** Potenciómetro + LED  
**Aprenderás:**
- Crear un PID básico
- Configurar límites
- Leer error

```cpp
File → Examples → PID_Control → 01_Basic_PID
```

---

### 📗 Ejemplo 2: Temperature Control
**Nivel:** Intermedio  
**Hardware:** Sensor NTC/DHT + Calentador  
**Aprenderás:**
- Control real de temperatura
- Filtrado de ruido
- Anti-windup para sistemas lentos

```cpp
File → Examples → PID_Control → 02_Temperature_Control
```

---

### 📙 Ejemplo 3: Motor Speed
**Nivel:** Intermedio  
**Hardware:** Motor DC + Encoder  
**Aprenderás:**
- Control de velocidad
- Usar interrupciones
- Control bidireccional

```cpp
File → Examples → PID_Control → 03_Motor_Speed
```

---

### 📕 Ejemplo 4: UAV Manager
**Nivel:** Avanzado  
**Hardware:** IMU + 4 ESCs  
**Aprenderás:**
- Control en cascada
- Gestión de múltiples PIDs
- Arquitectura profesional

```cpp
File → Examples → PID_Control → 04_UAV_Manager
```

---

### 📔 Ejemplo 5: UAV Static
**Nivel:** Avanzado  
**Hardware:** IMU + 4 ESCs  
**Aprenderás:**
- Optimización extrema
- Control de alta frecuencia (>1kHz)
- Templates C++

```cpp
File → Examples → PID_Control → 05_UAV_Static
```

---

### 📓 Ejemplo 6: Comparison
**Nivel:** Intermedio  
**Hardware:** Ninguno (benchmark)  
**Aprenderás:**
- Medir performance
- Comparar implementaciones
- Análisis de memoria

```cpp
File → Examples → PID_Control → 06_Comparison
```

---

## 📖 API Completa

### PID_controller (Clase Individual)

#### Constructor

```cpp
PID_controller(double kp, double ki, double kd, unsigned long windup_time = 5000)
```

**Parámetros:**
- `kp`: Ganancia proporcional
- `ki`: Ganancia integral
- `kd`: Ganancia derivativa
- `windup_time`: Tiempo en ms para resetear integral (default: 5000)

**Ejemplo:**
```cpp
PID_controller myPID(2.0, 0.5, 1.0);        // Windup por defecto
PID_controller myPID(2.0, 0.5, 1.0, 10000); // Windup de 10 segundos
```

#### Métodos Principales

##### compute()
```cpp
float compute(float setpoint, float input)
```
Calcula la salida del PID.

**Parámetros:**
- `setpoint`: Valor objetivo
- `input`: Valor actual medido

**Retorna:** Salida del controlador

**Ejemplo:**
```cpp
float temperature = readSensor();
float output = myPID.compute(25.0, temperature);
analogWrite(heaterPin, output);
```

##### reset()
```cpp
void reset()
```
Resetea el estado del PID (integral, derivativo, error previo).

**Cuándo usar:**
- Cambio brusco de setpoint
- Cambio de modo manual a automático
- Reinicio del sistema

**Ejemplo:**
```cpp
myPID.reset();  // Empezar desde cero
```

#### Configuración de Límites

##### setOutputLimits()
```cpp
void setOutputLimits(float min, float max)
```
Establece límites de salida y habilita anti-windup por saturación.

**Ejemplo:**
```cpp
myPID.setOutputLimits(0, 255);      // Para PWM
myPID.setOutputLimits(-100, 100);   // Control bidireccional
myPID.setOutputLimits(1000, 2000);  // Para servos (µs)
```

##### setMaxError()
```cpp
void setMaxError(float maxError)
```
Define el error máximo esperado (para cálculos).

#### Anti-Windup

##### setWindupTime()
```cpp
void setWindupTime(unsigned long time)
```
Configura tiempo para resetear integral automáticamente.

**Ejemplo:**
```cpp
myPID.setWindupTime(30000);  // Reset cada 30 segundos
```

##### enableWindupLimit()
```cpp
void enableWindupLimit(bool enable)
```
Habilita/deshabilita anti-windup por tiempo.

**Ejemplo:**
```cpp
myPID.enableWindupLimit(false);  // Solo usar anti-windup por saturación
```

#### Filtro Derivativo

##### setDerivativeFilter()
```cpp
void setDerivativeFilter(float alpha)
```
Configura filtro pasa-bajos para el término derivativo.

**Parámetros:**
- `alpha`: 0.0 (máximo filtrado) a 1.0 (sin filtro)

**Guía de valores:**
- `0.1-0.3`: Señal muy ruidosa (temperatura, presión)
- `0.5-0.7`: Señal moderadamente ruidosa (posición, velocidad)
- `0.9-1.0`: Señal limpia (giroscopio filtrado, encoder)

**Ejemplo:**
```cpp
myPID.setDerivativeFilter(0.3);  // Filtrado agresivo para temperatura
```

#### Cambio de Parámetros

##### setTunings()
```cpp
void setTunings(double kp, double ki, double kd)
```
Cambia las ganancias PID en tiempo de ejecución.

**Ejemplo:**
```cpp
// Auto-tuning o cambio de modo
if (agressiveMode) {
    myPID.setTunings(5.0, 1.0, 2.0);
} else {
    myPID.setTunings(2.0, 0.5, 1.0);
}
```

#### Modo Manual/Automático

##### setMode()
```cpp
void setMode(bool automatic)
```
Cambia entre modo automático (PID activo) y manual.

**Ejemplo:**
```cpp
myPID.setMode(false);  // Modo manual
// ... control manual ...
myPID.setMode(true);   // Volver a PID (hace reset automático)
```

##### getMode()
```cpp
bool getMode()
```
Retorna el modo actual (true = automático).

#### Diagnóstico

##### getError()
```cpp
float getError()
```
Retorna el error actual (setpoint - input).

##### getIntegralError()
```cpp
float getIntegralError()
```
Retorna el valor acumulado de la integral.

##### getDerivativeError()
```cpp
float getDerivativeError()
```
Retorna el valor del término derivativo.

**Ejemplo - Monitoreo:**
```cpp
Serial.println("Error: " + String(myPID.getError()));
Serial.println("P: " + String(myPID.getError() * Kp));
Serial.println("I: " + String(myPID.getIntegralError()));
Serial.println("D: " + String(myPID.getDerivativeError()));
```

---

### PID_Manager (Gestor Centralizado)

#### Constructor

```cpp
PID_Manager(uint16_t sampleTimeMs = 10)
```

**Parámetros:**
- `sampleTimeMs`: Tiempo de muestreo en ms (default: 10 = 100 Hz)

**Ejemplo:**
```cpp
PID_Manager flight(10);   // 100 Hz
PID_Manager slow(100);    // 10 Hz para temperatura
```

#### Gestión de PIDs

##### addPID()
```cpp
int8_t addPID(float kp, float ki, float kd, int16_t outMin, int16_t outMax)
```
Añade un nuevo PID al gestor.

**Retorna:** ID del PID (-1 si está lleno)

**Ejemplo:**
```cpp
int8_t pidRoll = flight.addPID(4.0, 0.0, 0.5, -250, 250);
int8_t pidPitch = flight.addPID(4.0, 0.0, 0.5, -250, 250);

if (pidRoll == -1) {
    Serial.println("Error: No hay espacio para más PIDs");
}
```

##### removePID()
```cpp
void removePID(uint8_t id)
```
Deshabilita un PID (no libera memoria).

##### enablePID()
```cpp
void enablePID(uint8_t id, bool enable)
```
Habilita/deshabilita un PID dinámicamente.

**Ejemplo:**
```cpp
// Deshabilitar control de altitud en modo manual
flight.enablePID(pidAltitude, false);
```

#### Control

##### update()
```cpp
bool update()
```
Verifica si es momento de actualizar (según sampleTime).

**Retorna:** true si debe actualizar

**Ejemplo:**
```cpp
void loop() {
    if (flight.update()) {  // Solo cada 10ms
        // Hacer control PID
    }
    // Otras tareas...
}
```

##### compute()
```cpp
float compute(uint8_t id, float setpoint, float input)
```
Calcula salida de un PID específico.

**Ejemplo:**
```cpp
if (flight.update()) {
    float out1 = flight.compute(pidRoll, rollCmd, rollActual);
    float out2 = flight.compute(pidPitch, pitchCmd, pitchActual);
}
```

#### Configuración

##### setTunings()
```cpp
void setTunings(uint8_t id, float kp, float ki, float kd)
```

##### setOutputLimits()
```cpp
void setOutputLimits(uint8_t id, int16_t min, int16_t max)
```

##### setDerivativeFilter()
```cpp
void setDerivativeFilter(uint8_t id, float alpha)
```

##### setSampleTime()
```cpp
void setSampleTime(uint16_t ms)
```
Cambia la frecuencia de actualización.

**Ejemplo:**
```cpp
flight.setSampleTime(5);  // Cambiar a 200 Hz
```

#### Reset

##### reset()
```cpp
void reset(uint8_t id)
```
Resetea un PID específico.

##### resetAll()
```cpp
void resetAll()
```
Resetea todos los PIDs.

#### Diagnóstico

##### getError()
```cpp
float getError(uint8_t id)
```

##### getIntegral()
```cpp
float getIntegral(uint8_t id)
```

##### getDerivative()
```cpp
float getDerivative(uint8_t id)
```

##### getDeltaTime()
```cpp
float getDeltaTime()
```
Retorna el tiempo real entre actualizaciones (en segundos).

##### getPIDCount()
```cpp
uint8_t getPIDCount()
```
Retorna el número de PIDs activos.

---

### PIDController (Template Estático)

#### Constructor

```cpp
PIDController<NUM_PIDS>()
```

**Template parameter:**
- `NUM_PIDS`: Número fijo de PIDs (compile-time)

**Ejemplo:**
```cpp
PIDController<8> flight;  // 8 PIDs fijos
PIDController<4> robot;   // 4 PIDs fijos
```

#### Inicialización

##### init()
```cpp
void init(uint8_t id, float kp, float ki, float kd, int16_t outMin, int16_t outMax)
```

**Ejemplo:**
```cpp
enum { ROLL, PITCH, YAW };  // IDs claros

void setup() {
    flight.init(ROLL, 4.0, 0.0, 0.5, -500, 500);
    flight.init(PITCH, 4.0, 0.0, 0.5, -500, 500);
    flight.init(YAW, 2.0, 0.0, 0.3, -500, 500);
}
```

#### Control

##### setDeltaTime()
```cpp
void setDeltaTime(float dt)
```
Establece dt para todos los PIDs (llamar una vez por ciclo).

**Ejemplo:**
```cpp
void loop() {
    uint32_t now = micros();
    float dt = (now - lastTime) / 1000000.0;
    lastTime = now;
    
    flight.setDeltaTime(dt);  // UNA VEZ
    
    // Todos los computes usan el mismo dt
    float r = flight.compute(ROLL, ...);
    float p = flight.compute(PITCH, ...);
}
```

##### compute()
```cpp
float compute(uint8_t id, float setpoint, float input)
```
Completamente inline - máxima velocidad.

#### Resto de Métodos

Similares a PID_Manager:
- `reset(id)`, `resetAll()`
- `setFilter(id, alpha)`
- `setTunings(id, kp, ki, kd)`
- `setOutputLimits(id, min, max)`
- `enable(id, enable)`
- `getError(id)`, `getIntegral(id)`, `getDerivative(id)`

---

## 🎓 Ajuste de Parámetros PID

### Síntomas y Soluciones

| Problema | Causa Probable | Solución |
|----------|----------------|----------|
| 🐌 Respuesta muy lenta | Kp muy bajo | Aumentar Kp |
| 🌊 Oscilaciones constantes | Kp muy alto | Reducir Kp |
| 📊 No llega al objetivo (offset) | Ki muy bajo o cero | Aumentar Ki |
| 🚀 Sobrepaso grande | Ki muy alto | Reducir Ki |
| 📈 Sobrepaso con oscilaciones | Kp alto + Ki alto | Reducir ambos |
| 😵 Salida muy nerviosa | Kd muy alto o ruido | Reducir Kd o filtrar señal |
| 🌀 Integral "se dispara" | Windup sin control | Habilitar límites de salida |
| ⚡ Reacción lenta a cambios | Kd muy bajo | Aumentar Kd |

### Proceso de Ajuste Visual

```
1. SOLO Kp (Ki=0, Kd=0)
   ┌─────────────────────────┐
   │ Setpoint ─────────────  │
   │           /````````     │  ← Kp muy bajo (lento)
   │         /               │
   └─────────────────────────┘

   ┌─────────────────────────┐
   │          /\  /\  /\     │  ← Kp muy alto (oscila)
   │        /    \/  \/      │
   │ Setpoint ──────────     │
   └─────────────────────────┘

   ┌─────────────────────────┐
   │        /```````         │  ← Kp correcto
   │      /   ↑ offset       │     pero hay error residual
   │ Setpoint ──────────     │
   └─────────────────────────┘

2. AÑADIR Ki (Kp fijo)
   ┌─────────────────────────┐
   │        /`````/`````     │  ← Ki elimina offset
   │      /  Setpoint ─────  │
   │    /                    │
   └─────────────────────────┘

3. AÑADIR Kd (Kp, Ki fijos)
   ┌─────────────────────────┐
   │        /```````````     │  ← Kd suaviza respuesta
   │      / ← sin overshoot  │
   │ Setpoint ──────────     │
   └─────────────────────────┘
```

---

## ❓ Preguntas Frecuentes

### ¿Cuál implementación debo usar?

**Usa PID_controller si:**
- Eres nuevo en PID
- Solo necesitas 1-2 controles
- Estás aprendiendo
- Valoras flexibilidad sobre velocidad

**Usa PID_Manager si:**
- Tienes 3+ PIDs
- UAV, robot multi-articulado
- Necesitas sincronización
- Proyecto semi-profesional

**Usa PID_Static si:**
- Racing drone
- Necesitas >1kHz de control
- Memoria muy limitada
- Sabes exactamente cuántos PIDs necesitas

### ¿Por qué mi PID oscila?

**Causas comunes:**
1. **Kp muy alto** → Reducir Kp
2. **Kd muy bajo** → Aumentar Kd
3. **Ruido en sensor** → Filtrar o reducir Kd
4. **Delay en actuador** → Reducir todas las ganancias
5. **Frecuencia de muestreo muy baja** → Aumentar velocidad de loop

### ¿Por qué no llega al setpoint?

**Causas:**
1. **Solo usas Kp (sin Ki)** → Añadir Ki
2. **Límites muy restrictivos** → Revisar setOutputLimits()
3. **Actuador saturado** → Aumentar potencia máxima
4. **Anti-windup muy agresivo** → Aumentar tiempo de windup

### ¿Cuándo usar el filtro derivativo?

**Siempre que tengas:**
- Sensor ruidoso (NTC, presión, etc.)
- Salida nerviosa/temblorosa
- Kd>0 y señal sin filtro de hardware

**No necesitas filtro si:**
- Sensor muy limpio (encoder, giroscopio filtrado)
- Kd=0
- Ya tienes filtro en hardware

### ¿Cómo sé si mi frecuencia de muestreo es correcta?

**Regla general:** Al menos **10× más rápido que la dinámica del sistema**

Ejemplos:
- Temperatura (cambia en segundos) → 10-50 Hz OK
- Motor DC (cambia en 100ms) → 50-200 Hz
- Servo posición (cambia en 20ms) → 200-500 Hz
- UAV attitude (cambia en 10ms) → 500-2000 Hz

### ¿Puedo cambiar el setpoint bruscamente?

**Sí, pero considera:**

```cpp
// Cambio brusco
setpoint = 100;  // Estaba en 0
// Causa gran error instantáneo → posible sobrepaso

// Mejor: Rampa suave
float targetSetpoint = 100;
setpoint += (targetSetpoint - setpoint) * 0.1;  // 10% por ciclo
```

O resetear el PID:
```cpp
setpoint = 100;
myPID.reset();  // Empezar limpio
```

### Mi integral crece sin control (windup)

**Soluciones implementadas:**

1. **Anti-windup por saturación** (automático con límites):
```cpp
myPID.setOutputLimits(0, 255);  // Activa anti-windup
```

2. **Anti-windup por tiempo**:
```cpp
myPID.setWindupTime(10000);  // Reset cada 10s
```

3. **Deshabilitar windup temporal**:
```cpp
myPID.enableWindupLimit(false);  // Solo saturación
```

---

## 🔧 Troubleshooting

### Problema: El código no compila

**Error:** `PID_controller was not declared`

**Solución:**
```cpp
// Asegúrate de incluir:
#include <PID_Control.h>
// O específicamente:
#include <PID_controller.h>
```

**Error:** `no matching function for call to 'PIDController'`

**Solución:**
```cpp
// Template necesita número de PIDs:
PIDController<8> flight;  // No olvidar el <8>
```

### Problema: El PID no hace nada

**Checklist:**

1. **¿Llamas a compute()?**
```cpp
void loop() {
    output = myPID.compute(setpoint, input);  // ✓ Correcto
    // No: float output = 0;  // ✗ Nunca llamas compute!
}
```

2. **¿PID_Manager.update() retorna true?**
```cpp
if (manager.update()) {  // Solo TRUE cada sampleTime
    output = manager.compute(...);
}
```

3. **¿Está habilitado?**
```cpp
manager.enablePID(id, true);  // Verificar
```

4. **¿Límites muy restrictivos?**
```cpp
myPID.setOutputLimits(0, 255);  // Verificar que son apropiados
```

### Problema: Salida siempre al máximo/mínimo

**Causas:**

1. **Error muy grande**
```cpp
// Setpoint: 1000, Input: 10
// Error = 990 → Output saturado
// Solución: Verificar unidades y rangos
```

2. **Límites incorrectos**
```cpp
myPID.setOutputLimits(0, 10);  // Muy bajo para PWM
myPID.setOutputLimits(0, 255);  // ✓ Correcto para PWM
```

3. **Ganancias muy altas**
```cpp
PID_controller pid(1000, 500, 100);  // ✗ Demasiado alto
PID_controller pid(2.0, 0.5, 1.0);   // ✓ Empezar bajo
```

### Problema: Consumo excesivo de RAM

**Soluciones:**

1. **Usar PID_Manager en vez de array:**
```cpp
// Antes: ~800 bytes
PID_controller pids[8];

// Después: ~320 bytes
PID_Manager manager;
```

2. **Usar PID_Static para mínima RAM:**
```cpp
PIDController<8> pids;  // ~240 bytes
```

3. **Reducir MAX_PID_CONTROLLERS** en PID_Manager.h:
```cpp
#define MAX_PID_CONTROLLERS 6  // En vez de 12
```

### Problema: Loop muy lento

**Diagnóstico:**

```cpp
void loop() {
    uint32_t start = micros();
    
    // Tu código PID...
    
    uint32_t elapsed = micros() - start;
    Serial.println("Loop: " + String(elapsed) + " µs");
}
```

**Optimizaciones:**

1. **Usar PID_Static** (3× más rápido)
2. **Reducir Serial.print** (muy lento)
3. **Optimizar lectura de sensores**
4. **Verificar no hay delay() innecesarios**

---

## 🤝 Contribuir

¡Las contribuciones son bienvenidas!

### Cómo Contribuir

1. **Fork** el repositorio
2. Crear una **branch** (`git checkout -b feature/MiMejora`)
3. **Commit** cambios (`git commit -m 'Añadir nueva característica'`)
4. **Push** a la branch (`git push origin feature/MiMejora`)
5. Abrir un **Pull Request**

### Guidelines

- Seguir el estilo de código existente
- Añadir ejemplos para nuevas características
- Actualizar documentación
- Probar en al menos 2 plataformas Arduino diferentes

### Reportar Bugs

Abrir un issue con:
- Descripción del problema
- Código mínimo para reproducir
- Plataforma (Arduino Uno, ESP32, etc.)
- Versión de la librería

---

## 📜 Licencia

Este proyecto está bajo licencia MIT. Ver archivo [LICENSE](LICENSE) para más detalles.

```
MIT License - Puedes:
✓ Usar comercialmente
✓ Modificar
✓ Distribuir
✓ Uso privado

Con la condición de:
! Incluir licencia y copyright
! Sin garantías
```

---

## 🙏 Agradecimientos

- Comunidad Arduino por el ecosistema increíble
- Brett Beauregard por su librería PID original que inspiró este proyecto
- Todos los contribuidores y testers

---

## 📞 Soporte

- **Issues:** [GitHub Issues](https://github.com/tuusuario/PID_Control/issues)
- **Discusiones:** [GitHub Discussions](https://github.com/tuusuario/PID_Control/discussions)
- **Email:** tu@email.com

---

## 🎓 Recursos Adicionales

### Aprender Más sobre PID

- [PID Controller Explained](https://www.youtube.com/watch?v=wkfEZmsQqiA) - Video educativo
- [PID Control - Wikipedia](https://es.wikipedia.org/wiki/Controlador_PID)
- [Control Systems Engineering - Norman Nise](https://www.amazon.com/Control-Systems-Engineering-Norman-Nise/dp/1118170519)

### Librerías Relacionadas

- [Adafruit_MPU6050](https://github.com/adafruit/Adafruit_MPU6050) - Para IMU
- [Servo](https://www.arduino.cc/reference/en/libraries/servo/) - Control de servos
- [Encoder](https://www.pjrc.com/teensy/td_libs_Encoder.html) - Lectura de encoders

---

## 📊 Comparación con Otras Librerías

| Característica | PID_Control | Arduino-PID-Library | FastPID |
|----------------|-------------|---------------------|---------|
| Anti-windup dual | ✅ | ❌ | ⚠️ Básico |
| Filtro derivativo | ✅ Configurable | ❌ | ✅ Fijo |
| Múltiples PIDs optimizados | ✅ PID_Manager | ❌ | ❌ |
| Template estático | ✅ PID_Static | ❌ | ✅ |
| Modo manual/auto | ✅ | ✅ | ❌ |
| Documentación educativa | ✅ Extensa | ⚠️ Básica | ⚠️ Básica |
| Ejemplos | ✅ 6 completos | ⚠️ 2 básicos | ⚠️ 1 básico |
| Tamaño RAM (por PID) | 32-100 bytes | ~60 bytes | ~24 bytes |
| Velocidad | ⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ |

---

<div align="center">

**¿Te gusta este proyecto? Dale una ⭐ en GitHub!**

Hecho con ❤️ para la comunidad Arduino

[⬆ Volver arriba](#-pid_control)

</div>