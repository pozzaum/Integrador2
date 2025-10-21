#include "main.h"

// ========= Parâmetros de parada =========
static const float THRESH_STOP_CM   = 50.0f;  // ≤ 50 cm -> parar
static const float THRESH_RESUME_CM = 60.0f;  // ≥ 60 cm -> retomar

// ========= Variáveis globais =========
volatile unsigned long pulseCount = 0;
unsigned long previousMillisRPM   = 0;
unsigned long previousMillisSonar = 0;
const unsigned long sonarPeriodMs = 80;   // período de leitura do sonar

bool stopped = false;      // estado atual (true = parado)
uint8_t runPWM = 255;      // PWM "andar pra frente" (0–255)

// ========= Encoder ISR =========
void IRAM_ATTR countPulse() {
  pulseCount++;
}

// ========= HCSR04 =========
HCSR04 hc(TRIG_PIN, ECHO_PIN); // (trig, echo)

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
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), countPulse, RISING);

  // Começa andando para frente
  runForward(runPWM);

  previousMillisRPM   = millis();
  previousMillisSonar = millis();
}

void loop() {
  unsigned long now = millis();

  // ======= Janela de RPM =======
  if (now - previousMillisRPM >= intervalMs) {
    previousMillisRPM = now;

    noInterrupts();
    unsigned long pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    // rpm = (pulses / pulsos_por_volta) * (60 s / janela_s)
    float window_s = intervalMs / 1000.0f;
    float rpm = (pulses / (float)PULSES_PER_REV) * (60.0f / window_s);

    // Exemplo de conversão para km/h (ajuste o fator conforme o teu raio de roda)
    float kmh = rpm * 60.0f * 2.0f * 3.1416f * 3e-5f;

    Serial.print("Pulses: "); Serial.print(pulses);
    Serial.print(" | RPM: "); Serial.print(rpm, 1);
    Serial.print(" | Km/h: "); Serial.println(kmh, 2);
  }

  // ======= Leitura do sonar (não-bloqueante) + lógica de parada =======
  if (now - previousMillisSonar >= sonarPeriodMs) {
    previousMillisSonar = now;

    float dist_cm = hc.dist();  // depende da tua lib; normalmente retorna em cm
    // Algumas libs retornam <=0 quando sem leitura válida. Protege:
    bool valid = (dist_cm > 0.0f && dist_cm < 1000.0f);

    if (valid) {
      Serial.print("Dist (cm): ");
      Serial.println(dist_cm, 1);

      if (!stopped && dist_cm <= THRESH_STOP_CM) {
        // Detectou obstáculo perto -> parar
        stopMotors();
        stopped = true;
        Serial.println("[INFO] Obstáculo próximo: PARAR");
      } else if (stopped && dist_cm >= THRESH_RESUME_CM) {
        // Ficou livre novamente -> retomar
        runForward(runPWM);
        stopped = false;
        Serial.println("[INFO] Livre novamente: ANDAR");
      }
    } else {
      // Leitura inválida: não muda estado dos motores, só informa
      Serial.println("Dist: leitura inválida");
    }
  }
}
