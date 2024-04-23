#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pid_ctrl.h"
#include"encoders.h"
#include"sonar_sensor.h"


#include"esp_log.h"
#include "esp_timer.h"
#include"l298n_driver.h"

#define EXAMPLE_PCNT_HIGH_LIMIT 100
#define EXAMPLE_PCNT_LOW_LIMIT  -100


#define GPIO_ENA 18
#define GPIO_ENB 19
#define GPIO_IN1 17
#define GPIO_IN2 5
#define GPIO_IN3 4
#define GPIO_IN4 16

#define HCSR_04_LEFT_ECHO 36 
#define HCSR_04_LEFT_TRIG 25 
#define HCSR_04_LEFT_ENUM 1

#define HCSR_04_CENTER_ECHO 39
#define HCSR_04_CENTER_TRIG 33 
#define HCSR_04_CENTER_ENUM 2

#define HCSR_04_RIGHT_ECHO 34 
#define HCSR_04_RIGHT_TRIG 32 
#define HCSR_04_RIGHT_ENUM 3




void app_main() {
    l298n_driver_config_t motor_drv_conf = {
        .mpcwm_timer_resolution_hz = 10000000, // 10MHz, 0.1us per tick 
        .mcpwm_pwm_freq_hz = 25000, // 25KHz,
        .mcpwm_group_id = 0,
        .mcpwm_min_pwm_ticks = 220,
        .gpio_ena_num = GPIO_ENA,
        .gpio_enb_num = GPIO_ENB,
        .gpio_in1_num = GPIO_IN1,
        .gpio_in2_num = GPIO_IN2,
        .gpio_in3_num = GPIO_IN3,
        .gpio_in4_num = GPIO_IN4,
    };
    l298n_driver_handle_t motor_driver_handle = NULL;
    l298n_driver_new(&motor_drv_conf,&motor_driver_handle);
    l298n_driver_enable(motor_driver_handle);
    l298n_driver_start_motors(motor_driver_handle);


    // set up sphase enc 
    sphase_encoder_handle_t sphase_encoder = NULL;
    sphase_encoder_config_t sphase_conf = {
        .edge_gpio_num = 22,
        .high_limit = 1000,
        .low_limit = -1000,
        .max_glitch_ns = 1000,
    };
    sphase_encoder_new(&sphase_conf,&sphase_encoder);
    sphase_encoder_change_direction(sphase_encoder,SPHASE_ENCODER_INC_ON_RISING_EDGE_INC_ON_FALLING_EDGE);
    sphase_encoder_enable(sphase_encoder);
    sphase_encoder_start(sphase_encoder);
    hcsr_04_timer_handle_t hcsr_04_timer_handle = NULL;
    hcsr_04_timer_new(&hcsr_04_timer_handle,0);

    QueueHandle_t sonar_data_queue = xQueueCreate(6,sizeof(hcsr_04_capture_data_t));

    hcsr_04_handle_t sonar_left_handle = NULL;
    hcsr_04_config_t sonar_conf_left = {
        .gpio_num_echo = HCSR_04_LEFT_ECHO,
        .gpio_num_trig = HCSR_04_LEFT_TRIG,
        .group_id = 1,
        .meas_queue = sonar_data_queue,
        .sensor_enum = HCSR_04_LEFT_ENUM,
    };

    hcsr_04_handle_t sonar_center_handle = NULL;
    hcsr_04_config_t sonar_conf_center = {
        .gpio_num_echo = HCSR_04_CENTER_ECHO,
        .gpio_num_trig = HCSR_04_CENTER_TRIG,
        .group_id = 1,
        .meas_queue = sonar_data_queue,
        .sensor_enum = HCSR_04_CENTER_ENUM,
    };

    hcsr_04_handle_t sonar_right_handle = NULL;
    hcsr_04_config_t sonar_conf_right = {
        .gpio_num_echo = HCSR_04_RIGHT_ECHO,
        .gpio_num_trig = HCSR_04_RIGHT_TRIG,
        .group_id = 1,
        .meas_queue = sonar_data_queue,
        .sensor_enum = HCSR_04_RIGHT_ENUM,
    };
    ESP_ERROR_CHECK(hcsr_04_driver_new(&sonar_conf_left,&sonar_left_handle,hcsr_04_timer_handle));
    ESP_ERROR_CHECK(hcsr_04_driver_new(&sonar_conf_center,&sonar_center_handle,hcsr_04_timer_handle));
    ESP_ERROR_CHECK(hcsr_04_driver_new(&sonar_conf_right,&sonar_right_handle,hcsr_04_timer_handle));
    ESP_ERROR_CHECK(hcsr_04_timer_enable(hcsr_04_timer_handle));
    ESP_ERROR_CHECK(hcsr_04_driver_enable(sonar_left_handle));
    ESP_ERROR_CHECK(hcsr_04_driver_enable(sonar_center_handle));
    ESP_ERROR_CHECK(hcsr_04_driver_enable(sonar_right_handle));


    //   // PID 
    //     pid_ctrl_parameter_t pid_runtime_param = {
    //     .kp = 1.3,
    //     .ki = 1,
    //     .kd = 0.4,
    //     .cal_type = PID_CAL_TYPE_INCREMENTAL,
    //     .max_output   = BDC_MCPWM_DUTY_TICK_MAX - 1,
    //     .min_output   = 220,
    //     .max_integral = 1000,
    //     .min_integral = -1000,
    // };
    // pid_ctrl_block_handle_t pid_ctrl = NULL;
    // pid_ctrl_config_t pid_config = {
    //     .init_param = pid_runtime_param,
    // };
    // ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &pid_ctrl));
    
    while (1)
    {
        l298n_driver_set_direction(motor_driver_handle,L298N_TARGET_BOTH_MOTORS,L298N_ACTION_FORWARD);

        l298n_driver_set_speed(motor_driver_handle,L298N_TARGET_MOTOR_A,250);
        l298n_driver_set_speed(motor_driver_handle,L298N_TARGET_MOTOR_B,400);
        //l298n_driver_calibration_check(motor_driver_handle);
        ESP_ERROR_CHECK(hcsr_04_start_capture(sonar_left_handle));
        ESP_ERROR_CHECK(hcsr_04_start_capture(sonar_center_handle));
        ESP_ERROR_CHECK(hcsr_04_start_capture(sonar_right_handle));
        hcsr_04_capture_data_t new_data = {0};
        new_data.sensor_enum = -1;
        for (size_t i = 0; i < 3; i++)
        {
            BaseType_t ret = xQueueReceive(sonar_data_queue,&new_data,pdMS_TO_TICKS(HCSR_04_MAX_TIME_TO_RETURN_MS));
            if(ret == pdFALSE){
                ESP_LOGE("MAIN","Bad meas.");
                xQueueReset(sonar_data_queue);
                continue;
            }
            uint32_t timer_res_hz;
            ESP_ERROR_CHECK(hcsr_04_get_timer_resolution(hcsr_04_timer_handle,&timer_res_hz));

            float pulse_width_us = new_data.time_of_flight_ticks * (1000000.0 /((float) timer_res_hz) );
            float distance = (float) pulse_width_us / 58;
            if(distance > 150) {
                distance = 150;
            }
            int64_t cur_time = esp_timer_get_time();
            int64_t diff = cur_time - new_data.timestamp;
            float ts = diff / 1000;
            ESP_LOGI("MAIN", "Measured distance: %.2fcm on device %d, stale: %0.1f", distance,new_data.sensor_enum,ts);
            xQueueReset(sonar_data_queue);
            //float tof_ms =  (1000 / timer_res_hz) * new_data.time_of_flight_ticks;
            //ESP_LOGI("MAIN","ToF: %0.3fms, device:%d",tof_ms,new_data.sensor_enum);
            //int a = new_data.time_of_flight_ticks;
            //int b = timer_res_hz;
            //ESP_LOGI("MAIN","ToF in ticks :%d , device:%d hz:%d",a,new_data.sensor_enum,b);
        }
        


        vTaskDelay(pdMS_TO_TICKS(100));
    }
    

   
}
