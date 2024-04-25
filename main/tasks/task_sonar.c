








#include"task_sonar.h"
#include"robot_configuration.h"

static hcsr_04_handle_t sonar_left_handle;
static hcsr_04_handle_t sonar_center_handle;
static hcsr_04_handle_t sonar_right_handle;

static float sonar_left_cur_sample;
static float sonar_center_cur_sample;
static float sonar_right_cur_sample;
static const char* TAG = "SONAR TASK";

void task_sonar_get_distances(float* left, float* center, float* right)
{
    *left = sonar_left_cur_sample;
    *center = sonar_center_cur_sample;
    *right = sonar_right_cur_sample;
}




void task_sonar_func(void *pvParams)
{
    sonar_left_cur_sample = 0;
    sonar_center_cur_sample = 0;
    sonar_right_cur_sample = 0;

    // save barrier sem for later use
    SemaphoreHandle_t barrier_sem = pvParams;
    hcsr_04_timer_handle_t hcsr_04_timer_handle = NULL;
    ESP_ERROR_CHECK(hcsr_04_timer_new(&hcsr_04_timer_handle,0));

    QueueHandle_t sonar_data_queue = xQueueCreate(6,sizeof(hcsr_04_capture_data_t));

    sonar_left_handle = NULL;
    hcsr_04_config_t sonar_conf_left = {
        .gpio_num_echo = HCSR_04_LEFT_ECHO_GPIO,
        .gpio_num_trig = HCSR_04_LEFT_TRIG_GPIO,
        .group_id = 0,
        .meas_queue = sonar_data_queue,
        .sensor_enum = HCSR_04_LEFT_ENUM,
    };

    sonar_center_handle = NULL;
    hcsr_04_config_t sonar_conf_center = {
        .gpio_num_echo = HCSR_04_CENTER_ECHO_GPIO,
        .gpio_num_trig = HCSR_04_CENTER_TRIG_GPIO,
        .group_id = 0,
        .meas_queue = sonar_data_queue,
        .sensor_enum = HCSR_04_CENTER_ENUM,
    };

    sonar_right_handle = NULL;
    hcsr_04_config_t sonar_conf_right = {
        .gpio_num_echo = HCSR_04_RIGHT_ECHO_GPIO,
        .gpio_num_trig = HCSR_04_RIGHT_TRIG_GPIO,
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
    // release the barrier sem 
    if(barrier_sem){
        xSemaphoreGive(barrier_sem);
    }
    while (true)
    {
        // start capturing 
        ESP_ERROR_CHECK(hcsr_04_start_capture(sonar_left_handle));
        ESP_ERROR_CHECK(hcsr_04_start_capture(sonar_center_handle));
        ESP_ERROR_CHECK(hcsr_04_start_capture(sonar_right_handle));
        hcsr_04_capture_data_t new_data = {0};
        new_data.sensor_enum = -1;
        // wait for sensors to return 
        vTaskDelay(pdMS_TO_TICKS(40));
        // try to exhaust the queue
        while(true){
            BaseType_t state = xQueueReceive(sonar_data_queue,&new_data,0);
            if(state == pdFALSE){
                break;
            }

            uint32_t timer_res_hz;
            ESP_ERROR_CHECK(hcsr_04_get_timer_resolution(hcsr_04_timer_handle,&timer_res_hz));
            // calculate the pulse width in uS
            float pulse_width_us = new_data.time_of_flight_ticks * (1000000.0 /((float) timer_res_hz) );
            // calculate the distance 
            float distance = (float) pulse_width_us / 58;
            // ceil the distance 
            if(distance > HCSR_04_MAX_MEAS_CM) {
                distance = HCSR_04_MAX_MEAS_CM;
            }   
            // dispatch distance based on sensor enum
            switch (new_data.sensor_enum)
            {
            case HCSR_04_LEFT_ENUM:
                sonar_left_cur_sample = distance;
                break;
            case HCSR_04_CENTER_ENUM:
                sonar_center_cur_sample = distance;
                break;
            case HCSR_04_RIGHT_ENUM:
                sonar_right_cur_sample = distance;
                break;
            default:
                ESP_LOGW(TAG,"HCSR sample with bad enum of %d",new_data.sensor_enum);
                break;
            }
            // calculate some other parameters for optional debug
            int64_t cur_time = esp_timer_get_time();
            int64_t diff = cur_time - new_data.timestamp;
            float ts = diff / 1000;
            ESP_LOGD(TAG, "Measured distance: %.2fcm on device %d, stale: %0.1f", distance,new_data.sensor_enum,ts);
        }
    }

    hcsr_04_timer_del(hcsr_04_timer_handle);
    hcsr_04_driver_del(sonar_left_handle);
    hcsr_04_driver_del(sonar_center_handle);
    hcsr_04_driver_del(sonar_right_handle);
    vTaskDelete(NULL);
}


