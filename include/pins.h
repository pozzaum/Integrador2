#ifndef DEFINES_H

#define DEFINES_H

// ======= Configurações =======
#define ENCODER_PIN 17      // encoder

// Motor A (M1)
#define M1_EN   14  // ENA - PWM
#define M1_IN1  27  // IN1
#define M1_IN2  26  // IN2

// Motor B (M2)
#define M2_EN   32  // ENB - PWM
#define M2_IN1  25  // IN3
#define M2_IN2  33  // IN4

// Canais PWM do ESP32
#define CH_M1   0
#define CH_M2   1

// HCSR04

#define TRIG_PIN 21
#define ECHO_PIN 22

#endif // DEFINES_H