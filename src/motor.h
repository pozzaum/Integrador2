#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "pins.h"
#include "params.h"

void setMotorRaw(uint8_t motor, int16_t pwm);

inline void setMotor(uint8_t motor, uint8_t pwm_0_255) { setMotorRaw(motor, (int16_t)pwm_0_255); }
inline void stopMotors() { setMotor(1, 0); setMotor(2, 0); }
inline void runForward(uint8_t pwm) { setMotor(1, pwm); setMotor(2, pwm); }


void setMotor(int motor, int speed);


#endif // MOTOR_H
