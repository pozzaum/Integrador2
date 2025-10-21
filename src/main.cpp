#include "main.h"

// ========= Variáveis globais =========
volatile unsigned long pulseCount = 0;
unsigned long previousMillisRPM   = 0;
unsigned long previousMillisSonar = 0;
const unsigned long sonarPeriodMs = 80;   // período de leitura do sonar

bool stopped = false;      // estado atual por SONAR (true = parado)
uint8_t runPWM = 255;      // PWM "andar pra frente" (0–255)

// ========= HCSR04 =========
HCSR04 hc(TRIG_PIN, ECHO_PIN); // (trig, echo)

// ========== Helpers de motor ==========
// (assumindo que já existem em main.h: runForward(uint8_t), stopMotors(), etc.)

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
  encoder_begin();

  // ====== GPS: inicialização ======
  // OBS: use Serial1 no ESP32 para a porta 1
  gps_begin(&Serial1, GPS_BAUD, GPS_RX_PIN, GPS_TX_PIN);
  gps_set_target(TARGET_LAT, TARGET_LON, TARGET_INNER_M, TARGET_OUTER_M);
  gps_set_report_interval_ms(1000);

  // Começa PARADO até ter fix do GPS (gate de segurança)
  stopMotors();
  gpsBlocked = true;
  goalArrived = false;

  previousMillisRPM   = millis();
  previousMillisSonar = millis();

  Serial.println("[BOOT] Sistema iniciado. Aguardando FIX do GPS para liberar movimento.");
}

void loop() {
  unsigned long now = millis();

  // ======= Estado de navegação (último comando válido do GPS) =======
  static GpsNavCommand lastCmd;
  static bool          lastCmdValid = false;

  // ======= GPS feed contínuo =======
  gps_feed();

  // ======= Diagnóstico básico de GPS + liberação de movimento =======
  if (now - prevGpsLogMs >= gpsLogPeriodMs) {
    prevGpsLogMs = now;

    if (!gps_has_data(10)) {
      Serial.println("[GPS] Sem dados. Cheque RX/TX, GND e baud.");
    }

    if (!gps_has_fix()) {
      if (!gpsBlocked) {
        stopMotors();
        gpsBlocked = true;
      }
      Serial.println("[GPS] Sem FIX. Movimento BLOQUEADO.");
      lastCmdValid = false; // sem fix, invalida comando
    } else {
      // Temos fix: calcula navegação
      gpsBlocked = false;

      double lat, lon;
      if (gps_current_location(&lat, &lon)) {
        double dist, brg;
        if (gps_distance_bearing_to_target(&dist, &brg)) {

          // TODO: substitua por heading da IMU quando tiver (COG do GPS só presta com velocidade > ~0.7 m/s)
          const double heading_atual_deg = 0.0;

          GpsNavCommand cmd;
          if (gps_compute_go_to_heading(heading_atual_deg, &cmd)) {
            goalArrived  = cmd.arrived;
            lastCmd      = cmd;
            lastCmdValid = true;

            int sats   = gps_satellites();
            double hdop = gps_hdop();
            Serial.printf("[GPS] Lat: %.6f Lon: %.6f | Dist: %.1f m | Brg: %.1f° | eψ: %.1f° | Sats: %d | HDOP: %.2f | Arrived: %s\n",
                          lat, lon, cmd.distance_m, cmd.bearing_to_goal_deg,
                          cmd.heading_error_deg, sats, hdop,
                          (cmd.arrived ? "YES" : "NO"));
          } else {
            lastCmdValid = false;
          }
        } else {
          lastCmdValid = false;
        }
      } else {
        lastCmdValid = false;
      }
    }
  }

  // ======= Janela de RPM =======
  if (now - previousMillisRPM >= intervalMs) {
    previousMillisRPM = now;

    noInterrupts();
    unsigned long pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    float window_s = intervalMs / 1000.0f;
    float rpm = (pulses / (float)PULSES_PER_REV) * (60.0f / window_s);

    // Ajuste este fator conforme o diâmetro efetivo da roda
    float kmh = rpm * 60.0f * 2.0f * 3.1416f * 3e-5f;

    Serial.print("Pulses: "); Serial.print(pulses);
    Serial.print(" | RPM: "); Serial.print(rpm, 1);
    Serial.print(" | Km/h: "); Serial.println(kmh, 2);
  }

  // ======= Leitura do sonar (não-bloqueante) + lógica de parada =======
  if (now - previousMillisSonar >= sonarPeriodMs) {
    previousMillisSonar = now;

    float dist_cm = hc.dist();
    bool valid = (dist_cm > 0.0f && dist_cm < 1000.0f);

    if (valid) {
      Serial.print("Dist (cm): ");
      Serial.println(dist_cm, 1);

      if (!stopped && dist_cm <= THRESH_STOP_CM) {
        stopMotors();
        stopped = true;
        Serial.println("[INFO] Obstáculo próximo: PARAR");
      } else if (stopped && dist_cm >= THRESH_RESUME_CM) {
        // Libera o gate de obstáculo; quem decide andar é a política final
        stopped = false;
        Serial.println("[INFO] Livre novamente (gate obstáculo liberado).");
      }
    } else {
      Serial.println("Dist: leitura inválida");
    }
  }

  // ======= Política final de movimento com direção por eψ =======
  bool shouldMove = (!stopped) && (!gpsBlocked) && (!goalArrived) && lastCmdValid;

  static bool wasMoving = false;

  if (!shouldMove) {
    if (wasMoving) {
      stopMotors();
      wasMoving = false;
      if (gpsBlocked)        Serial.println("[STATE] PARAR: Sem FIX.");
      else if (stopped)      Serial.println("[STATE] PARAR: Obstáculo.");
      else if (goalArrived)  Serial.println("[STATE] PARAR: Chegou ao alvo.");
      else                   Serial.println("[STATE] PARAR: Gate.");
    }
  } else {
    // Temos um comando válido: aplicar navegação
    float epsi = (float)lastCmd.heading_error_deg;   // graus
    float dist = (float)lastCmd.distance_m;          // metros

    // Deadband
    if (fabs(epsi) < YAW_DEADBAND_DEG) epsi = 0.f;

    // Perfil de velocidade base pela distância
    uint8_t base = PWM_BASE;
    if (dist < SLOW_DIST2)      base = max<uint8_t>(PWM_MIN, PWM_BASE - 80);
    else if (dist < SLOW_DIST1) base = max<uint8_t>(PWM_MIN, PWM_BASE - 40);

    if (fabs(epsi) >= YAW_ALIGN_DEG) {
      // Giro no lugar até alinhar
      int sgn = (epsi > 0) ? +1 : -1; // eψ>0: alvo está à direita → girar para a direita
      turnInPlace(sgn, PWM_TURN);
      //turnInPlace(1, 240);
      Serial.println("[NAV] Giro no lugar para alinhar.");
    } else {
      // Curva/seguimento com diferencial
      float steer = KP_STEER * epsi;
      // driveArc já satura steer em [-1, +1] (conforme sua implementação)
      driveArc(base, steer);

      // Log opcional (comente se ficar verboso)
      Serial.printf("[NAV] Steer=%.2f | eψ=%.1f° | base=%u | dist=%.1f m\n",
                    steer, epsi, base, dist);
    }

    if (!wasMoving) {
      Serial.println("[STATE] Movimento LIBERADO (GPS OK, sem obstáculo, não-chegou).");
      wasMoving = true;
    }
  }
}

