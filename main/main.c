#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <stdio.h>
#include"tasks/task_drive.h"
#include"tasks/task_sonar.h"
#include"tasks/task_uart.h"
#include"tasks/task_imu.h"


static const char* TAG = "MAIN";
SemaphoreHandle_t barrier_sem;

void app_main()
{
    barrier_sem = xSemaphoreCreateBinary();

    // start the motor drive task 
    xTaskCreatePinnedToCore(task_drive_motors_func,"motor_drive_task",4000,barrier_sem,1,NULL,1);
    BaseType_t did_drv_init = xSemaphoreTake(barrier_sem,pdMS_TO_TICKS(300));
    if(did_drv_init == pdFALSE){
        ESP_LOGE(TAG,"Drive function init failed.");
        esp_restart();
    }
    ESP_LOGI(TAG,"Motor drive service started.");
    task_drive_set_both_motor_cps(0,0);

    // start the sonar task 
    xTaskCreatePinnedToCore(task_sonar_func,"sonar_sample_task",4000,barrier_sem,2,NULL,1);
    BaseType_t did_sonar_init = xSemaphoreTake(barrier_sem,pdMS_TO_TICKS(300));
    if(did_sonar_init == pdFALSE){
        ESP_LOGE(TAG,"Sonar service cannot start.");
        esp_restart();
    }
    ESP_LOGI(TAG,"Sonar sampling service started.");

    // start the imu task, bind it to core 0
    xTaskCreatePinnedToCore(task_imu_func,"imu_sample_task",4000,barrier_sem,1,NULL,0);
    BaseType_t did_imu_init = xSemaphoreTake(barrier_sem,pdMS_TO_TICKS(5000));
       if(did_imu_init == pdFALSE){
        ESP_LOGE(TAG,"IMU service cannot start.");
        esp_restart();
    } 
    ESP_LOGI(TAG,"IMU sampling service started.");
    
    // start the UART link
    xTaskCreatePinnedToCore(task_uart_func,"uart_rxtx_task",4000,NULL,1,NULL,1);



    
}   