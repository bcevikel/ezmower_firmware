#ifndef TASK_SONAR_H
#define TASK_SONAR_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include"sonar_sensor.h"

void task_sonar_func(void* pvParams);
void task_sonar_get_distances(float* left, float* center, float* right);


#endif