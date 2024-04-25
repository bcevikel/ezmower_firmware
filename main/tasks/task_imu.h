#ifndef TASK_IMU_H
#define TASK_IMU_H
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include"esp_packets_types.h"



void task_imu_func(void* pvParams);
void task_imu_get_data(esp_packets_data_frame_imu_rot* rot, esp_packets_data_frame_imu_trans* trans, float* temp);




#endif