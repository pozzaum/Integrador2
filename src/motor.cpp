#include "motor.h"

void setMotorRaw(uint8_t motor, int16_t pwm)
{
  uint8_t in1, in2, en, ch;

  // Seleção de pinos/canais
  if (motor == 1) {
    in1 = M1_IN1; in2 = M1_IN2; en = M1_EN; ch = CH_M1;
  } else { // motor == 2
    in1 = M2_IN1; in2 = M2_IN2; en = M2_EN; ch = CH_M2;
  }

  // Satura interface em -255..+255
  if (pwm >  255) pwm =  255;
  if (pwm < -255) pwm = -255;

  // Parado (coast): IN1=LOW, IN2=LOW, duty=0
  if (pwm == 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(ch, 0);
    return;
  }

  // Direção: positivo = frente, negativo = ré
  const bool forward = (pwm > 0);

  // Magnitude 0..255
  uint16_t mag8 = (pwm > 0) ? (uint16_t)pwm : (uint16_t)(-pwm);

  // Converte 0..255 -> 0..(2^PWM_RES - 1)
  const uint16_t maxDuty = (1u << PWM_RES) - 1u;
  uint16_t duty = (uint32_t)mag8 * maxDuty / 255u;

  // Seta direção (mantém sua convenção atual)
  digitalWrite(in1, forward ? LOW  : HIGH);
  digitalWrite(in2, forward ? HIGH : LOW);

  // Aplica PWM
  ledcWrite(ch, duty);
}


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
    // Frente: IN1=LOW, IN2=HIGH
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    // Ré: IN1=HIGH, IN2=LOW
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }

  int duty = map(abs(speed), 0, 255, 0, (1 << PWM_RES) - 1);
  ledcWrite(ch, duty);
}