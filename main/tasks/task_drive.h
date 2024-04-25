#ifndef TASK_DRIVE_H
#define TASK_DRIVE_H
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include "pid_ctrl.h"
#include"encoders.h"
#include"l298n_driver.h"

/**
 * This header file defines interfaces for the task that will handle motor drive via
 * the L298N driver
*/


void task_drive_motors_func(void* pvParameters);
void task_drive_set_left_motor_cps(float left);
void task_drive_set_right_motor_cps(float right);
void task_drive_set_both_motor_cps(float left,float right);
void task_drive_set_pid_parameters(float p, float i, float d);
void task_drive_get_left_encoder_counts(int* left_enc);
void task_drive_get_right_encoder_counts(int* right_enc);
void task_drive_get_both_encoder_counts(int* left_enc, int* right_enc);





#endif