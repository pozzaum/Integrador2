
/*
#define TRIG_PIN 5   // Pino TRIG conectado ao GPIO 5
#define ECHO_PIN 18  // Pino ECHO conectado ao GPIO 18

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  // Envia um pulso de 10 microssegundos
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Mede o tempo de resposta
  long duracao = pulseIn(ECHO_PIN, HIGH);

  // Calcula a distância (em cm)
  float distancia = duracao * 0.034 / 2;

  Serial.print("Distância: ");
  Serial.print(distancia);
  Serial.println(" cm");

  delay(500);
}
  */