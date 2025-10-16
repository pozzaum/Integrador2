#include "motor.h"

void setMotor(int motor, int speed) {
  int in1, in2, ch;
  if (motor == 1) { in1 = M1_IN1; in2 = M1_IN2; ch = CH_M1; }
  else            { in1 = M2_IN1; in2 = M2_IN2; ch = CH_M2; }

  // Satura -255..255
  if (speed < -255) speed = -255;
  if (speed >  255) speed =  255;

  if (speed == 0) {
    // Livre (costando): IN1=LOW, IN2=LOW
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(ch, 0);
    return;
  }

  if (speed > 0) {
    // Frente: IN1=HIGH, IN2=LOW
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    // Ré: IN1=LOW, IN2=HIGH
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }

  int duty = map(abs(speed), 0, 255, 0, (1 << PWM_RES) - 1);
  ledcWrite(ch, duty);
}