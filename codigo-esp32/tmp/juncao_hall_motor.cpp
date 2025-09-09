#include <Arduino.h>

// Make IRAM_ATTR safe on any board:
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

// ==== User settings ====
const uint8_t HALL_PIN = 27;        // change to your pin
const float   R = 0.03f;            // wheel radius [m]
const uint8_t MAGNETS_PER_REV = 1;  // number of magnets on the wheel
const unsigned long TIMEOUT_US = 500000; // 0.5 seconds in microseconds

// Derived
const float PERIMETER = 2.0f * PI * R;
const float DIST_PER_PULSE = PERIMETER / MAGNETS_PER_REV;

// ISR-shared
volatile unsigned long lastPulseTimeUs = 0;
volatile unsigned long periodUs = 0;

void IRAM_ATTR hallISR() {
  unsigned long now = micros();
  periodUs = now - lastPulseTimeUs;
  lastPulseTimeUs = now;
}

void setup() {
  Serial.begin(115200);
  pinMode(HALL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, FALLING);
}

void loop() {
  float kmh = 0.0f;
  unsigned long timeSinceLastPulse;

  // Safely read the shared variables
  noInterrupts();
  unsigned long latestPeriod = periodUs;
  unsigned long lastTime = lastPulseTimeUs;
  interrupts();

  // If a significant amount of time has passed since the last pulse, assume speed is zero.
  timeSinceLastPulse = micros() - lastTime;
  if (timeSinceLastPulse > TIMEOUT_US) {
    kmh = 0.0f;
  } else {
    // If we have a recent pulse, calculate the speed.
    if (latestPeriod > 0) {
      float s = latestPeriod * 1e-6f; // microseconds -> seconds
      kmh = (DIST_PER_PULSE / s) * 3.6f; // m/s -> km/h
    }
  }

  Serial.println(kmh, 2);
  delay(200);
}