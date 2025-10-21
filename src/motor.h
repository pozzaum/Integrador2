#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "pins.h"
#include "params.h"

inline void setMotor(uint8_t motor, uint8_t pwm_0_255);
inline void stopMotors();
inline void runForward(uint8_t pwm);

static void setMotorRaw(uint8_t motor, int16_t pwm);

void setMotor(int motor, int speed);


#endif // MOTOR_H
