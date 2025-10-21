#include "encoder.h"

volatile unsigned long encLeftCount  = 0;
volatile unsigned long encRightCount = 0;

// --- ISRs ---
void IRAM_ATTR encLeftISR()  { encLeftCount++;  }
void IRAM_ATTR encRightISR() { encRightCount++; }

// --- Inicialização ---
void encoder_begin() {
  pinMode(ENC_LEFT_PIN, INPUT_PULLUP);
  pinMode(ENC_RIGHT_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_PIN),  encLeftISR,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_PIN), encRightISR, RISING);
}

// --- Reset ---
void encoder_reset() {
  noInterrupts();
  encLeftCount  = 0;
  encRightCount = 0;
  interrupts();
}

// --- Leituras seguras ---
unsigned long encoder_readLeft() {
  noInterrupts();
  unsigned long val = encLeftCount;
  interrupts();
  return val;
}

unsigned long encoder_readRight() {
  noInterrupts();
  unsigned long val = encRightCount;
  interrupts();
  return val;
}
