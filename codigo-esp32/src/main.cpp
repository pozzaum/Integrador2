#include <Arduino.h>

// ======= Configurações =======
#define ENCODER_PIN 17      // Troque para o pino que você usa de fato
#define PULSES_PER_REV 20   // nº de pulsos por volta do encoder

// Motor A (M1)
#define M1_EN   14  // ENA - PWM
#define M1_IN1  27  // IN1
#define M1_IN2  26  // IN2

// Motor B (M2)
#define M2_EN   32  // ENB - PWM
#define M2_IN1  25  // IN3
#define M2_IN2  33  // IN4

// Canais PWM do ESP32
#define CH_M1   0
#define CH_M2   1

// PWM
const int PWM_FREQ = 20000; // 20 kHz
const int PWM_RES  = 10;    // 10 bits (0..1023)

// Medição de RPM
volatile unsigned long pulseCount = 0;
unsigned long previousMillis = 0;
const unsigned long intervalMs = 1000; // 1 s

// ISR do encoder
void IRAM_ATTR countPulse() {
  pulseCount++;
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
    // Frente: IN1=HIGH, IN2=LOW
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    // Ré: IN1=LOW, IN2=HIGH
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }

  int duty = map(abs(speed), 0, 255, 0, (1 << PWM_RES) - 1);
  ledcWrite(ch, duty);
}

void setup() {
  Serial.begin(115200);
  delay(200);

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
    Serial.print(" | RPM: ");
    Serial.println(rpm);
  }

  delay(5);
}
