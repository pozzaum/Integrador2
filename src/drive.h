#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>
#include "motor.h"
#include "encoder.h"


#define DEG_PER_PULSE_YAW 18.0f

// ===== Navegação por heading error (eψ) =====
static const float YAW_ALIGN_DEG   = 45.0f;   // acima disso: gira no lugar
static const float YAW_CURVE_DEG   = 12.0f;   // abaixo disso: segue quase reto
static const float YAW_DEADBAND_DEG= 3.0f;    // não oscilar por ruído

// Ganho para converter eψ (graus) -> steer (−1..+1)
static const float KP_STEER = 0.02f;          // 0.02 → 100° vira steer ~2.0→satura em ±1

// Velocidades
static const uint8_t PWM_TURN   = 240;        // giro no lugar
static const uint8_t PWM_BASE   = 240;        // base pra seguir em frente
static const uint8_t PWM_MIN    = 150;        // base mínima quando perto
static const float   SLOW_DIST1 = 25.0f;      // <25 m: desacelera
static const float   SLOW_DIST2 = 10.0f;      // <10 m: desacelera mais


void driveArc(uint8_t base, float steer);
void turnInPlace(int sign, uint8_t pwm);
bool turnByAngle(float graus, bool sentidoHorario, uint8_t pwm = 240, unsigned long timeout_ms = 5000);


#endif // DRIVE_H