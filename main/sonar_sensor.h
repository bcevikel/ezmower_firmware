#ifndef SONAR_SENSOR_H
#define SONAR_SENSOR_H
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/mcpwm_cap.h"

#include"esp_err.h"
#include"esp_timer.h"
#include "esp_check.h"
#define HCSR_04_SPEED_US_PER_METER 58 
#define HCSR_04_MAX_TIME_TO_RETURN_MS 40 

/*
* Wrapper struct for MCPWM timer resource 
*/
typedef struct 
{
    mcpwm_cap_timer_handle_t mcpwm_capture_timer;
    int group_id;
} hcsr_04_timer_resource_t;
typedef hcsr_04_timer_resource_t* hcsr_04_timer_handle_t;

/*
Configuration struct for the HCSR04 Sonar Sensor.
*/
typedef struct
{
    int gpio_num_echo; // ECHO pin of the sensor
    int gpio_num_trig; // TRIG pin of the sensor
    int group_id; // MCPWM Group id to allocate resources from
    QueueHandle_t meas_queue; // The FreeRTOS queue to which we will publish the measurements
    int sensor_enum; // This int can be used to identify different sensors, meas packets will contain this int.
} hcsr_04_config_t;
/*
Data packet struct for the HCSR04 Sensor. It is used to 
pass data around in queues between ISR and tasks.
*/
typedef struct
{
    uint32_t time_of_flight_ticks; // Sonar time of flight 
    int64_t timestamp; // timestamp of measurement in local hw time
    int sensor_enum; // from which sensor 
} hcsr_04_capture_data_t;
/*
HCSR04 Sensor context struct
*/
typedef struct 
{
    hcsr_04_timer_handle_t mcpwm_capture_timer;
    mcpwm_cap_channel_handle_t mcpwm_capture_channel;
    QueueHandle_t data_queue;
    uint32_t capture_timestamp_begin;
    uint32_t capture_timestamp_end;
    int gpio_num_trig;
    int sensor_enum;

} hcsr_04_context_t;
typedef hcsr_04_context_t* hcsr_04_handle_t;


esp_err_t hcsr_04_timer_new(hcsr_04_timer_handle_t* resource, int group_id);
esp_err_t hcsr_04_timer_del(hcsr_04_timer_handle_t resource);
esp_err_t hcsr_04_get_timer_resolution(hcsr_04_timer_handle_t timer_handle, uint32_t *timer_res);
esp_err_t hcsr_04_timer_enable(hcsr_04_timer_handle_t timer_handle);
esp_err_t hcsr_04_timer_disable(hcsr_04_timer_handle_t timer_handle);
esp_err_t hcsr_04_driver_new(hcsr_04_config_t* config, hcsr_04_handle_t* handle, hcsr_04_timer_handle_t timer_handle);
esp_err_t hcsr_04_driver_del(hcsr_04_handle_t handle);
esp_err_t hcsr_04_driver_enable(hcsr_04_handle_t handle);
esp_err_t hcsr_04_driver_disable(hcsr_04_handle_t handle);
esp_err_t hcsr_04_start_capture(hcsr_04_handle_t handle);
esp_err_t hcsr_04_get_timer_resolution(hcsr_04_timer_handle_t handle, uint32_t *timer_res);

#endif