/*
 * PID_Control - Librería completa de control PID para Arduino
 * 
 * Incluye tres implementaciones optimizadas:
 * 
 * 1. PID_controller: Clase individual con todas las características
 *    - Anti-windup dual (por tiempo y saturación)
 *    - Filtro derivativo configurable
 *    - Límites de salida
 *    - Modo manual/automático
 *    - Ideal para proyectos simples o aprendizaje
 * 
 * 2. PID_Manager: Gestor centralizado para múltiples PIDs
 *    - Sincronización perfecta entre controladores
 *    - Memoria optimizada (~32 bytes por PID)
 *    - Una sola llamada a millis() por ciclo
 *    - Perfecto para UAVs y robots
 * 
 * 3. PID_Static: Template estático de máxima performance
 *    - Zero overhead abstraction
 *    - Inline completo en compilación
 *    - Memoria mínima (~24 bytes por PID)
 *    - Ideal para sistemas críticos en tiempo real
 * 
 * Autor: Tu Nombre
 * Versión: 2.0.0
 * Licencia: MIT
 * 
 * Uso:
 *   #include <PID_Control.h>  // Incluye las 3 implementaciones
 * 
 * O incluir solo lo necesario:
 *   #include <PID_controller.h>  // Solo clase individual
 *   #include <PID_Manager.h>     // Solo manager
 *   #include <PID_Static.h>      // Solo estático
 */

#ifndef PID_CONTROL_H
#define PID_CONTROL_H

// Versión de la librería
#define PID_CONTROL_VERSION "2.0.0"
#define PID_CONTROL_VERSION_MAJOR 2
#define PID_CONTROL_VERSION_MINOR 0
#define PID_CONTROL_VERSION_PATCH 0

// Incluir todas las implementaciones
#include "PID_controller.h"
#include "PID_Manager.h"
#include "PID_Static.h"

#endif // PID_CONTROL_H