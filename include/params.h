#ifndef PARAMS_H
#define PARAMS_H

// ======= Configurações =======

// Encoder
constexpr int PULSES_PER_REV = 20;   // número de pulsos por volta do motor

 // PWM
constexpr int PWM_FREQ = 20000; // 20 kHz
constexpr int PWM_RES  = 10;    // 10 bits (0..1023)

// Medição de RPM
const unsigned long intervalMs = 1000; // 1 s

#endif // PARAMS_H