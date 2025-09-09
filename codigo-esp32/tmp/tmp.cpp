// #include <AccelStepper.h>
// #include <Arduino_FreeRTOS.h>
// #include <queue.h>
// #include <semphr.h>

#include <Arduino.h>


#define HALL_PIN 12
#define RELAY_PIN 11
#define R 0.3429
#define PERIMETER 2 * PI * R

#define motorInterfaceType 1
#define dirPin 8
#define stepPin 9
#define enablePin 4

#define NUM_OF_CYCLES 10
#define MAX_TEMP_SENSOR 35

AccelStepper motor(motorInterfaceType, stepPin, dirPin);

signed long T1 = 0;
signed long T2 = 0;
signed long time_seconds = 0;

float setpoint = 35;
float kp = 0.58;
float kd = 0.00083;

float integral_term = 0.0;
float error_prev = 0.0;

bool is_at_setpoint = false;

int count_cycles = 0;
int timesPassedThroughSetpoint = 0;

double pid_controller(double avg, long dt){
  double error = setpoint - avg;
  float derivative_term = (error - error_prev) / dt;
  error_prev = error;

  return (kp * error) + (kd * derivative_term);
}

float get_speed(long* t_delta) {
    T2 = millis();
    time_seconds = (T2 - T1);
    *t_delta = time_seconds;
    float s = (PERIMETER / time_seconds) * 1000 * 3.6;
    T1 = T2;

    return s;
}

void handle_step(double st) {
    motor.move(st);
    motor.runToPosition();
}

QueueHandle_t speedQueue;
QueueHandle_t pidQueue;
QueueHandle_t timeQueue;
QueueHandle_t breakingQueue;

TaskHandle_t handleTaskSpeed;
TaskHandle_t handleTaskPid;
TaskHandle_t handleTaskStep;

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

void TaskPid(void *pvParameters) {

  float speedReceived = 0.0;
  double stepPID = 0.0;
  long time_delta = 0;
  
  for (;;) {
    if (xQueueReceive(speedQueue, &speedReceived, portMAX_DELAY) == pdPASS && 
        xQueueReceive(timeQueue, &time_delta, portMAX_DELAY) == pdPASS) {
          stepPID = pid_controller(speedReceived, time_delta);
          Serial.print("INFO");
          Serial.print(speedReceived);
          Serial.print(",");
          Serial.println(stepPID);
      if (xQueueSend(pidQueue, &stepPID, portMAX_DELAY) != pdTRUE) {
        Serial.println("[ERROR] Failed to send stepPID to STEP task.");    
      }
    }
  }
}

void TaskStep(void *pvParameters) {
  double stepPID = 0.0;
  
  motor.setMaxSpeed(1000);
  motor.setAcceleration(1000);

  float multiplicationFactor = 1.0;
  int breakingPart = 0;
  int prevBreakingPart = 0;
  int count = 3;

  for (;;) {
    if (xQueueReceive(pidQueue, &stepPID, portMAX_DELAY) == pdPASS) {
      if (breakingPart == 1 && count > 0) { // More torque
        multiplicationFactor = 0.6;
        count--;   
      }
      // If there was a way to override the steps..........
      if (breakingPart == -1 && count > 0) { // Less torque
        multiplicationFactor = 0.2;
        count--;   
      }
      handle_step(stepPID * multiplicationFactor);
    }
    if (xQueueReceive(breakingQueue, &breakingPart, 20 / portTICK_PERIOD_MS) == pdPASS) {
      if (breakingPart != prevBreakingPart) {
        count = 3;
        prevBreakingPart = breakingPart;
      }
    };
  }
}

void TaskValve(void *pvParameters) {
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);

  int breakingPart = 0;

  for (;;) {
    if (is_at_setpoint) {
      if (count_cycles == NUM_OF_CYCLES) {
        vTaskSuspend( handleTaskSpeed );
        vTaskSuspend( handleTaskPid );
        vTaskSuspend( handleTaskStep );

        digitalWrite(enablePin, HIGH); // Turn off motor


        Serial.println("end_cycle"); // Keep in mind this will keep printing 'end_cycle'

        if (Serial.readStringUntil('\n') == "continue") {
            count_cycles = 0;
            is_at_setpoint = false;
            timesPassedThroughSetpoint = 0;
            
            vTaskResume( handleTaskSpeed );
            vTaskResume( handleTaskPid );
            vTaskResume( handleTaskStep );

            digitalWrite(enablePin, LOW); // Turn on motor
        }

      } else {
        digitalWrite(RELAY_PIN, LOW);
        breakingPart = 1; // More torque
        if (xQueueSend(breakingQueue, &breakingPart, portMAX_DELAY) != pdTRUE) {
          Serial.println("[ERROR] Failed to send breakingQueue to STEP task.");    
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        digitalWrite(RELAY_PIN, HIGH);
        breakingPart = -1; // Less torque
        if (xQueueSend(breakingQueue, &breakingPart, portMAX_DELAY) != pdTRUE) {
          Serial.println("[ERROR] Failed to send breakingQueue to STEP task.");    
        }
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        count_cycles++;
      }
    } else {
      vTaskDelay(20 / portTICK_PERIOD_MS);
    }
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    vTaskDelay(1);
  }

  Serial.println("Starting to create tasks...");

  speedQueue = xQueueCreate(1, sizeof(float));
  pidQueue = xQueueCreate(1, sizeof(double));
  timeQueue = xQueueCreate(1, sizeof(long));
  breakingQueue = xQueueCreate(1, sizeof(int));
  
  if (speedQueue != NULL && pidQueue != NULL) {
    xTaskCreate(TaskSpeed, "TaskSpeed", 128, NULL, 3, &handleTaskSpeed);
    xTaskCreate(TaskPid, "TaskPid", 150, NULL, 1, &handleTaskPid);
    xTaskCreate(TaskStep, "TaskStep", 100, NULL, 2, &handleTaskStep);
    xTaskCreate(TaskValve, "TaskValve", 80, NULL, 3, NULL);
    Serial.println("After creating tasks...");
  }

  Serial.println("Tasks should start now!!");

}

void loop() {}