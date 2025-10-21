#ifndef PARAMS_H
#define PARAMS_H

// ======= Configurações =======

// Encoder
constexpr int PULSES_PER_REV = 20;   // número de pulsos por volta do motor

 // PWM
constexpr int PWM_FREQ = 20000; // 20 kHz
constexpr int PWM_RES  = 10;    // 10 bits (0..1023)

// Medição de RPM
const unsigned long intervalMs = 1000; // 1 s

// --------- Ajustes do alvo (graus decimais) ---------
// 26°14'07.9"S 48°53'04.8"W  -> lat ≈ -26.235528, lon ≈ -48.884667
static const double TARGET_LAT = -26.235528;
static const double TARGET_LON = -48.884667;
// Histerese de chegada (m) — ajuste conforme erro do seu GPS
static const double TARGET_INNER_M = 8.0;
static const double TARGET_OUTER_M = 12.0;

// Pinos/baud do GPS
static const int GPS_RX_PIN = 16;   // RX do ESP32 (liga no TX do GPS)
static const int GPS_TX_PIN = 17;   // TX do ESP32 (liga no RX do GPS, se usar)
static const uint32_t GPS_BAUD = 9600;

// Temporização simples para logs do GPS
static unsigned long prevGpsLogMs = 0;
static const unsigned long gpsLogPeriodMs = 500;

// Flag de bloqueio por GPS (sem fix = true)
static bool gpsBlocked = true;
// Flag de chegada ao alvo (via histerese do módulo GPS)
static bool goalArrived = false;

// ========= Parâmetros de parada =========
static const float THRESH_STOP_CM   = 50.0f;  // ≤ 50 cm -> parar
static const float THRESH_RESUME_CM = 60.0f;  // ≥ 60 cm -> retomar

#endif // PARAMS_H