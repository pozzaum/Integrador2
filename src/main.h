#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <HCSR04.h>
#include <algorithm>
#include <cstdlib>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Variáveis de tempo de compilação e pinos
#include "pins.h"
#include "params.h"


// Headers das funcionalidades
#include "motor.h"
#include "gps.h"
#include "distance_tracking.h"


#endif // MAIN_H