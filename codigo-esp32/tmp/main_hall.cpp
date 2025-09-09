#include <Arduino.h>

#define TRIG_PIN 5   // Pino TRIG conectado ao GPIO 5
#define ECHO_PIN 18  // Pino ECHO conectado ao GPIO 18
#define HALL_PIN 27

signed long T1 = 0;
signed long T2 = 0;
signed long time_seconds = 0;

float get_speed(long* t_delta) {
  T2 = millis();
  time_seconds = (T2 - T1);
  *t_delta = time_seconds;
  float s = (PERIMETER / time_seconds) * 1000 * 3.6;
  T1 = T2;

  return s;
}

void TaskSpeed(void *pvParameters) {
  pinMode(HALL_PIN, INPUT);

  float speedVal = 0.0;
  long time_delta = 0;
  T1 = millis();
  
  for (;;) {
    if (digitalRead(HALL_PIN) == LOW) {
      time_delta = 0;
      speedVal = get_speed(&time_delta);
      if (speedVal < 65) {
        if (xQueueSend(speedQueue, &speedVal, portMAX_DELAY) != pdTRUE) {
          Serial.println("[ERROR] Failed to send speed to PID task.");    
        }
        if (xQueueSend(timeQueue, &time_delta, portMAX_DELAY) != pdTRUE) {
          Serial.println("[ERROR] Failed to send time to PID task.");    
        }
        if (abs(speedVal - setpoint) < 2) {
          timesPassedThroughSetpoint++;
        }
        if (timesPassedThroughSetpoint > 10 && speedVal > 35) {
          is_at_setpoint = true;
        } 
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(HALL_PIN, INPUT);
}

void loop() {

  Serial.println(digitalRead(HALL_PIN));

  // // Envia um pulso de 10 microssegundos
  // digitalWrite(TRIG_PIN, LOW);
  // delayMicroseconds(2);
  // digitalWrite(TRIG_PIN, HIGH);
  // delayMicroseconds(10);
  // digitalWrite(TRIG_PIN, LOW);

  // // Mede o tempo de resposta
  // long duracao = pulseIn(ECHO_PIN, HIGH);

  // // Calcula a distância (em cm)
  // float distancia = duracao * 0.034 / 2;

  // Serial.print("Distância: ");
  // Serial.print(distancia);
  // Serial.println(" cm");

  // delay(500);
}
