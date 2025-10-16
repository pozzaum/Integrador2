#include "main.h"

// Variáveis globais
volatile unsigned long pulseCount = 0;
unsigned long previousMillis = 0;

// ISR do encoder
void IRAM_ATTR countPulse() {
  pulseCount++;
}

// HCSR04
HCSR04 hc(TRIG_PIN, ECHO_PIN); //initialization class HCSR04 (trig pin , echo pin)

void setup() {
  Serial.begin(9600);

  // Direção (H-bridge)
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);

  // PWM
  ledcSetup(CH_M1, PWM_FREQ, PWM_RES);
  ledcAttachPin(M1_EN, CH_M1);
  ledcSetup(CH_M2, PWM_FREQ, PWM_RES);
  ledcAttachPin(M2_EN, CH_M2);

  // Encoder
  pinMode(ENCODER_PIN, INPUT_PULLUP); // ajuste conforme seu sensor
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), countPulse, RISING);

  // Liga motores a 100% para frente
  setMotor(1, 255);
  setMotor(2, 255);

}

void loop() {
  unsigned long now = millis();

  if (now - previousMillis >= intervalMs) {
    previousMillis = now;

    noInterrupts();
    unsigned long pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    // rpm = (pulses / pulsos_por_volta) * (60 s / janela_s)
    float window_s = intervalMs / 1000.0f;
    float rpm = (pulses / (float)PULSES_PER_REV) * (60.0f / window_s);

    Serial.print("Pulses: ");
    Serial.print(pulses);
    Serial.print(" | Km / h: ");
    Serial.println(rpm * 60 * 2 * 3.1416*3e-5);
  }

  // Distância Ultrassônica
  Serial.println(hc.dist()); // return curent distance in serial
  delay(200);
}
