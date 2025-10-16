# Ligações ESP32 ↔ Ponte H L298N

Este documento descreve as conexões entre o **ESP32** e a **ponte H L298N**, de acordo com o último código utilizado.

---

## Motor A (conectado em OUT1/OUT2 da L298N)
- **ESP32 GPIO 14** → **ENA (Enable A / PWM)**
- **ESP32 GPIO 27** → **IN1**
- **ESP32 GPIO 26** → **IN2**

## Motor B (conectado em OUT3/OUT4 da L298N)
- **ESP32 GPIO 32** → **ENB (Enable B / PWM)**
- **ESP32 GPIO 25** → **IN3**
- **ESP32 GPIO 33** → **IN4**

---

## Alimentação
- **Vs (L298N)** → Fonte dos motores (**6–9 V** recomendados para motores 3–6 V)  
- **GND (L298N)** → GND da fonte dos motores e **GND do ESP32** (terra comum obrigatório)  
- **5V (L298N)** → Alimentação lógica da ponte.  
  - Se o **jumper 5V_EN** estiver **ligado**, o L298N regula 5 V a partir de Vs (funciona bem com Vs ≥ 7 V).  
  - Se o **jumper 5V_EN** estiver **removido**, você deve fornecer **5 V externos** neste pino.

---

## Observações
- Os motores devem ser ligados diretamente nos bornes **OUT1/OUT2** (Motor A) e **OUT3/OUT4** (Motor B).  
- Sempre mantenha **terra comum** entre ESP32, L298N e fonte dos motores.  
- Para testes de PWM, remova os jumpers ENA/ENB; para funcionamento fixo 100%, deixe-os conectados.
