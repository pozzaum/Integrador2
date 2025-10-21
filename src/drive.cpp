#include "drive.h"

// ===== Controle diferencial básico =====

// Curva com diferencial: base ∈ [0..255]; steer ∈ [-1..+1] (esq negativo, dir positivo)
void driveArc(uint8_t base, float steer) {
  if (steer < -1.f) steer = -1.f;
  if (steer >  1.f) steer =  1.f;

  // left = base * (1 - steer), right = base * (1 + steer)
  // steer = +1 -> gira para a direita (reduz esquerda / aumenta direita)
  float left  = base * (1.f - steer);
  float right = base * (1.f + steer);

  // Satura na faixa 0..255 (vamos pra frente; valores negativos viram zero)
  int l_pwm = (int)(left  < 0 ? 0 : (left  > 255 ? 255 : left));
  int r_pwm = (int)(right < 0 ? 0 : (right > 255 ? 255 : right));

  setMotor(1, +l_pwm);
  setMotor(2, +r_pwm);
}

// Giro no lugar: sinal ∈ {-1, +1}
void turnInPlace(int sign, uint8_t pwm) {
  int left  = (sign > 0) ? +pwm : -pwm;  // +: gira pra direita
  int right = (sign > 0) ? -pwm : +pwm;
  setMotor(1, left);
  setMotor(2, right);
}

bool turnByAngle(float graus, bool sentidoHorario, uint8_t pwm, unsigned long timeout_ms)
{
  if (graus < 0) graus = -graus;          // trabalha com módulo
  if (graus < 1.0f) return true;          // ângulos muito pequenos: ignore

  // Quantidade alvo de pulsos (arredonda pra cima)
  const unsigned long targetPulses = (unsigned long)ceilf(graus / (float)DEG_PER_PULSE_YAW);
  if (targetPulses == 0) return true;

  // Prepara contagem
  encoder_reset();

  // Sinal do giro no lugar: +1 = direita (horário), -1 = esquerda (anti-horário)
  const int sign = sentidoHorario ? +1 : -1;

  // Começa girando
  turnInPlace(sign, pwm);

  const unsigned long t0 = millis();
  unsigned long lastCheckMs = t0;
  unsigned long lastPulses  = 0;

  while (true) {
    // Timeout de segurança
    if (millis() - t0 > timeout_ms) {
      setMotorRaw(1, 0);
      setMotorRaw(2, 0);
      return false;  // não atingiu a meta a tempo
    }

    // Lê contadores (máximo dos dois para robustez)
    unsigned long l = encoder_readLeft();
    unsigned long r = encoder_readRight();
    unsigned long pulses = (l > r) ? l : r;

    // Atingiu meta?
    if (pulses >= targetPulses) {
      setMotorRaw(1, 0);
      setMotorRaw(2, 0);
      return true;
    }

    // Anti-travamento simples: se não gerou pulsos em ~300 ms, incrementa PWM (até 255)
    unsigned long now = millis();
    if (now - lastCheckMs >= 300) {
      if (pulses <= lastPulses) {
        if (pwm < 255) {
          pwm = (uint8_t)min(255, (int)pwm + 15);
          turnInPlace(sign, pwm);  // reaplica com mais torque
        }
      }
      lastPulses  = pulses;
      lastCheckMs = now;
    }

    // Pequeno yield para a CPU/ISR
    delay(1);
  }
}