#include "motor.h"

static void setMotorRaw(uint8_t motor, int16_t pwm)
{
  uint8_t in1, in2, en;
  uint8_t ch;

  // Seleção de pinos conforme o motor
  if (motor == 1) {
    in1 = M1_IN1; in2 = M1_IN2; en = M1_EN; ch = CH_M1;
  } else { // motor == 2
    in1 = M2_IN1; in2 = M2_IN2; en = M2_EN; ch = CH_M2;
  }

  // Caso PWM zero → para o motor
  if (pwm == 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(ch, 0);
    return;
  }

  // Direção: positivo = frente, negativo = ré
  bool forward = (pwm > 0);

  // Calcula magnitude (sem usar abs/min pra evitar ambiguidade)
  uint16_t duty = (pwm < 0) ? (uint16_t)(-pwm) : (uint16_t)pwm;

  // Garante limite de 0–255
  if (duty > 255u) duty = 255u;

  // Ajusta direção e aplica PWM
  digitalWrite(in1, forward ? LOW : HIGH);
  digitalWrite(in2, forward ? HIGH  : LOW);
  ledcWrite(ch, duty);
}


// atalho específico: frente/parado
inline void setMotor(uint8_t motor, uint8_t pwm_0_255) { setMotorRaw(motor, (int16_t)pwm_0_255); }
inline void stopMotors() { setMotor(1, 0); setMotor(2, 0); }
inline void runForward(uint8_t pwm) { setMotor(1, pwm); setMotor(2, pwm); }

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
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    // Ré: IN1=LOW, IN2=HIGH
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }

  int duty = map(abs(speed), 0, 255, 0, (1 << PWM_RES) - 1);
  ledcWrite(ch, duty);
}