#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

#define ENC_LEFT_PIN   5
#define ENC_RIGHT_PIN  18

// Contadores globais
extern volatile unsigned long encLeftCount;
extern volatile unsigned long encRightCount;

// Inicializa interrupções
void encoder_begin();

// Zera contadores
void encoder_reset();

// Lê contadores de forma atômica
unsigned long encoder_readLeft();
unsigned long encoder_readRight();

#endif
