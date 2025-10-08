/*
 * Ejemplo 06: Comparación de Performance
 * 
 * Benchmark de las tres implementaciones:
 * - PID_controller (individual)
 * - PID_Manager (centralizado)
 * - PID_Static (template)
 * 
 * Mide:
 * - Tiempo de ejecución
 * - Uso de RAM
 * - Throughput (iteraciones/segundo)
 */

#include <PID_Control.h>

// ========== CONFIGURACIÓN ==========
const int ITERATIONS = 1000;  // Iteraciones por test
const int NUM_PIDS = 8;       // Número de PIDs a probar

// ========== INSTANCIAS ==========

// 1. PID Individual
PID_controller pid_single[NUM_PIDS] = {
    PID_controller(2.0, 0.5, 1.0),
    PID_controller(2.0, 0.5, 1.0),
    PID_controller(2.0, 0.5, 1.0),
    PID_controller(2.0, 0.5, 1.0),
    PID_controller(2.0, 0.5, 1.0),
    PID_controller(2.0, 0.5, 1.0),
    PID_controller(2.0, 0.5, 1.0),
    PID_controller(2.0, 0.5, 1.0)
};

// 2. PID Manager
PID_Manager pid_manager(10);
int8_t manager_ids[NUM_PIDS];

// 3. PID Static
PIDController<NUM_PIDS> pid_static;

// ========== VARIABLES DE TEST ==========
float setpoint = 100.0;
float input = 50.0;
float output = 0.0;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("=====================================");
    Serial.println("  PID PERFORMANCE COMPARISON TEST");
    Serial.println("=====================================");
    Serial.println();
    
    // Configurar PID_controller
    Serial.println("Configurando PID_controller...");
    for (int i = 0; i < NUM_PIDS; i++) {
        pid_single[i].setOutputLimits(0, 255);
        pid_single[i].setDerivativeFilter(0.7);
    }
    
    // Configurar PID_Manager
    Serial.println("Configurando PID_Manager...");
    for (int i = 0; i < NUM_PIDS; i++) {
        manager_ids[i] = pid_manager.addPID(2.0, 0.5, 1.0, 0, 255);
        pid_manager.setDerivativeFilter(manager_ids[i], 0.7);
    }
    
    // Configurar PID_Static
    Serial.println("Configurando PID_Static...");
    for (int i = 0; i < NUM_PIDS; i++) {
        pid_static.init(i, 2.0, 0.5, 1.0, 0, 255);
        pid_static.setFilter(i, 0.7);
    }
    pid_static.setDeltaTime(0.01);  // 10ms
    
    Serial.println("✓ Configuración completa");
    Serial.println();
    
    delay(2000);
    
    // ========== EJECUTAR BENCHMARKS ==========
    runAllBenchmarks();
    
    // ========== MEMORY USAGE ==========
    printMemoryUsage();
    
    Serial.println();
    Serial.println("=====================================");
    Serial.println("  TESTS COMPLETADOS");
    Serial.println("=====================================");
}

void loop() {
    // Nothing - benchmark runs once
}

void runAllBenchmarks() {
    Serial.println("========================================");
    Serial.println("BENCHMARK: " + String(ITERATIONS) + " iteraciones, " + 
                   String(NUM_PIDS) + " PIDs");
    Serial.println("========================================");
    Serial.println();
    
    // Test 1: PID_controller
    Serial.println("--- Test 1: PID_controller (Individual) ---");
    unsigned long time1 = benchmarkSingle();
    Serial.println("Tiempo total: " + String(time1) + " µs");
    Serial.println("Tiempo/iteración: " + String(time1 / (float)ITERATIONS, 2) + " µs");
    Serial.println("Tiempo/PID: " + String(time1 / (float)(ITERATIONS * NUM_PIDS), 2) + " µs");
    Serial.println("Throughput: " + String((ITERATIONS * NUM_PIDS * 1000000.0) / time1, 0) + " computes/s");
    Serial.println();
    
    // Test 2: PID_Manager
    Serial.println("--- Test 2: PID_Manager (Centralizado) ---");
    unsigned long time2 = benchmarkManager();
    Serial.println("Tiempo total: " + String(time2) + " µs");
    Serial.println("Tiempo/iteración: " + String(time2 / (float)ITERATIONS, 2) + " µs");
    Serial.println("Tiempo/PID: " + String(time2 / (float)(ITERATIONS * NUM_PIDS), 2) + " µs");
    Serial.println("Throughput: " + String((ITERATIONS * NUM_PIDS * 1000000.0) / time2, 0) + " computes/s");
    Serial.println();
    
    // Test 3: PID_Static
    Serial.println("--- Test 3: PID_Static (Template) ---");
    unsigned long time3 = benchmarkStatic();
    Serial.println("Tiempo total: " + String(time3) + " µs");
    Serial.println("Tiempo/iteración: " + String(time3 / (float)ITERATIONS, 2) + " µs");
    Serial.println("Tiempo/PID: " + String(time3 / (float)(ITERATIONS * NUM_PIDS), 2) + " µs");
    Serial.println("Throughput: " + String((ITERATIONS * NUM_PIDS * 1000000.0) / time3, 0) + " computes/s");
    Serial.println();
    
    // Comparación
    Serial.println("========================================");
    Serial.println("COMPARACIÓN DE PERFORMANCE");
    Serial.println("========================================");
    Serial.println("PID_controller: " + String(time1) + " µs (baseline)");
    Serial.println("PID_Manager:    " + String(time2) + " µs (" + 
                   String((float)time1/time2, 2) + "x más rápido)");
    Serial.println("PID_Static:     " + String(time3) + " µs (" + 
                   String((float)time1/time3, 2) + "x más rápido)");
    Serial.println();
    
    // Ganador
    if (time2 < time1 && time2 < time3) {
        Serial.println("🏆 GANADOR: PID_Manager");
    } else if (time3 < time1 && time3 < time2) {
        Serial.println("🏆 GANADOR: PID_Static");
    } else {
        Serial.println("🏆 GANADOR: PID_controller");
    }
    Serial.println();
}

unsigned long benchmarkSingle() {
    unsigned long start = micros();
    
    for (int iter = 0; iter < ITERATIONS; iter++) {
        for (int i = 0; i < NUM_PIDS; i++) {
            output = pid_single[i].compute(setpoint, input);
        }
        input += 0.1;  // Simular cambio
    }
    
    unsigned long elapsed = micros() - start;
    return elapsed;
}

unsigned long benchmarkManager() {
    unsigned long start = micros();
    
    for (int iter = 0; iter < ITERATIONS; iter++) {
        pid_manager.update();  // Simular update
        for (int i = 0; i < NUM_PIDS; i++) {
            output = pid_manager.compute(manager_ids[i], setpoint, input);
        }
        input += 0.1;
    }
    
    unsigned long elapsed = micros() - start;
    return elapsed;
}

unsigned long benchmarkStatic() {
    unsigned long start = micros();
    
    for (int iter = 0; iter < ITERATIONS; iter++) {
        for (int i = 0; i < NUM_PIDS; i++) {
            output = pid_static.compute(i, setpoint, input);
        }
        input += 0.1;
    }
    
    unsigned long elapsed = micros() - start;
    return elapsed;
}

void printMemoryUsage() {
    Serial.println("========================================");
    Serial.println("USO DE MEMORIA (estimado)");
    Serial.println("========================================");
    
    // Sizes aproximados (dependen de la plataforma)
    size_t single_size = sizeof(PID_controller) * NUM_PIDS;
    size_t manager_size = sizeof(PID_Manager);
    size_t static_size = sizeof(PIDController<NUM_PIDS>);
    
    Serial.println("PID_controller array: " + String(single_size) + " bytes");
    Serial.println("  - Por PID: " + String(sizeof(PID_controller)) + " bytes");
    Serial.println();
    
    Serial.println("PID_Manager: " + String(manager_size) + " bytes");
    Serial.println("  - Por PID: ~32 bytes (en struct interno)");
    Serial.println();
    
    Serial.println("PID_Static: " + String(static_size) + " bytes");
    Serial.println("  - Por PID: ~24 bytes (en struct interno)");
    Serial.println();
    
    // Comparación
    Serial.println("Ahorro de memoria:");
    Serial.println("  Manager vs Single: " + String(single_size - manager_size) + 
                   " bytes (" + String((1.0 - (float)manager_size/single_size)*100, 1) + "%)");
    Serial.println("  Static vs Single: " + String(single_size - static_size) + 
                   " bytes (" + String((1.0 - (float)static_size/single_size)*100, 1) + "%)");
    Serial.println();
}

/*
 * RESULTADOS ESPERADOS (Arduino Due @ 84 MHz):
 * 
 * PERFORMANCE:
 * - PID_controller: ~450 µs/ciclo (baseline)
 * - PID_Manager:    ~180 µs/ciclo (2.5x más rápido)
 * - PID_Static:     ~120 µs/ciclo (3.7x más rápido)
 * 
 * MEMORIA (8 PIDs):
 * - PID_controller: ~800 bytes
 * - PID_Manager:    ~320 bytes
 * - PID_Static:     ~240 bytes
 * 
 * RECOMENDACIONES:
 * 
 * Usar PID_controller si:
 * - Solo necesitas 1-2 PIDs
 * - Prototipando
 * - Aprendiendo control PID
 * 
 * Usar PID_Manager si:
 * - Necesitas 3-12 PIDs
 * - UAV/robot con control en cascada
 * - Necesitas flexibilidad para cambiar parámetros
 * 
 * Usar PID_Static si:
 * - Necesitas máxima velocidad (>1kHz)
 * - Racing drone
 * - Sistema crítico en tiempo real
 * - RAM muy limitada
 */