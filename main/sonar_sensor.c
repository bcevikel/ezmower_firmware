#include"sonar_sensor.h"

#include "esp_private/esp_clk.h"
#include "driver/gpio.h"
const static char *TAG = "SONAR";


/*
Internal function that will be executed by the ISR on ECHO pin high and low edge events. 
*/
static bool hc_sr04_echo_callback(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data)
{
    hcsr_04_context_t* sensor_ctx = (hcsr_04_context_t*) user_data;
    //calculate the interval in the ISR,
    //so that the interval will be always correct even when capture_queue is not handled in time and overflow.
    if (edata->cap_edge == MCPWM_CAP_EDGE_POS) {
        // store the timestamp when pos edge is detected
        sensor_ctx->capture_timestamp_begin = edata->cap_value;
        sensor_ctx->capture_timestamp_end = sensor_ctx->capture_timestamp_begin;
        return true;
    } else {
        sensor_ctx->capture_timestamp_end = edata->cap_value;
        uint32_t tof_ticks = sensor_ctx->capture_timestamp_end - sensor_ctx->capture_timestamp_begin;
        // publish to queue 
        int64_t ts = esp_timer_get_time();
        hcsr_04_capture_data_t packet = {
            .time_of_flight_ticks = tof_ticks,
            .timestamp = ts,
            .sensor_enum = sensor_ctx->sensor_enum,
        };
        // if we caused a task to unblock, yield cpu to that task
        BaseType_t did_unblock;
        BaseType_t is_ok;
        is_ok = xQueueSendFromISR(sensor_ctx->data_queue,&packet,&did_unblock);
        portYIELD_FROM_ISR(did_unblock);
        // return the queue full status 
        return is_ok;
    }
}

/**
 * Allocates a new HCSR04 timer resource (MCPWM capture timer) for timers to use.
 * @param *resource the new resource handle that will be created 
 * @param group_id which group id to take the timer resource from 
*/
esp_err_t hcsr_04_timer_new(hcsr_04_timer_handle_t* resource, int group_id)
{
    hcsr_04_timer_handle_t timer = (hcsr_04_timer_handle_t) malloc(sizeof(hcsr_04_timer_resource_t));
    if(!timer) {return ESP_ERR_NOT_FOUND;}

    mcpwm_cap_timer_handle_t cap_timer = NULL;
    mcpwm_capture_timer_config_t cap_conf = {
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
        .group_id = group_id,
    };
    ESP_RETURN_ON_ERROR(mcpwm_new_capture_timer(&cap_conf,&cap_timer),TAG,"Cannot find free capture timer resource on group %d.",group_id);
    timer->group_id = group_id;
    timer->mcpwm_capture_timer = cap_timer;
    *resource = timer;
    return ESP_OK;
}

/**
 * Deallocates the timer resource for the HCSR04 sensors.
 * @param *resource the timer resource to deallocate
*/
esp_err_t hcsr_04_timer_del(hcsr_04_timer_handle_t resource){
    ESP_RETURN_ON_ERROR(mcpwm_del_capture_timer(resource->mcpwm_capture_timer),TAG,"Cannot delete capture timer.");
    free(resource);
    return ESP_OK;
}

/**
 * Returns the timer resolution in Hertz so that you can calculate the tick per second value.
 * @param 
 * @param *timer_res timer resolution, in Hz
*/
esp_err_t hcsr_04_get_timer_resolution(hcsr_04_timer_handle_t timer_handle, uint32_t *timer_res){
    return mcpwm_capture_timer_get_resolution(timer_handle->mcpwm_capture_timer,timer_res);
}

/**
* Enable the internal hardware of the HCSR04 timer.
* @param timer_handle the timer handle to start
*/
esp_err_t hcsr_04_timer_enable(hcsr_04_timer_handle_t timer_handle){
    ESP_RETURN_ON_ERROR(mcpwm_capture_timer_enable(timer_handle->mcpwm_capture_timer),TAG,"Cannot enable capture timer");
    ESP_RETURN_ON_ERROR(mcpwm_capture_timer_start(timer_handle->mcpwm_capture_timer),TAG,"Cannot start capture timer.");
    return ESP_OK;
}

/**
* Disable the internal hardware of the HCSR04 timer.
* @param timer_handle the timer handle to disable
*/
esp_err_t hcsr_04_timer_disable(hcsr_04_timer_handle_t timer_handle){
    ESP_RETURN_ON_ERROR(mcpwm_capture_timer_disable(timer_handle->mcpwm_capture_timer),TAG,"Cannot disable capture timer");
    ESP_RETURN_ON_ERROR(mcpwm_capture_timer_stop(timer_handle->mcpwm_capture_timer),TAG,"Cannot stop capture timer.");
    return ESP_OK;
}



/**
 * Allocates a new HCSR04 driver.
 * @param config the configuration struct
 * @param handle the to be created handle 
 * @param hcsr_timer the timer resource to use 
*/
esp_err_t hcsr_04_driver_new(hcsr_04_config_t* config, hcsr_04_handle_t* handle, hcsr_04_timer_handle_t hcsr_timer){
    if(!handle || !config) {return ESP_ERR_NOT_FOUND; }
    if(!config->meas_queue){
        ESP_LOGE(TAG,"Invalid FreeRTOS queue.");
        return ESP_ERR_INVALID_ARG;
    }
    // attempt to allocate the internal ctx 
    hcsr_04_handle_t hcsr_04_handle = (hcsr_04_handle_t) malloc(sizeof(hcsr_04_context_t));
    // handle no mem case
    if(!hcsr_04_handle){
        return ESP_ERR_NO_MEM;
    }
    // create the channel resource
    mcpwm_cap_channel_handle_t cap_chan = NULL;
    mcpwm_capture_channel_config_t cap_ch_conf = {
        .gpio_num = config->gpio_num_echo,
        .prescale = 1,
        // capture on both edge
        .flags.neg_edge = true,
        .flags.pos_edge = true,
        // pull up internally
        .flags.pull_up = true,
    };
    ESP_RETURN_ON_ERROR(mcpwm_new_capture_channel(hcsr_timer->mcpwm_capture_timer,&cap_ch_conf,&cap_chan),TAG,"Cannot acquire capture channel on group %d.",config->group_id);
    // register the callback func
    mcpwm_capture_event_callbacks_t cbs = {
        .on_cap = hc_sr04_echo_callback,
    };
    ESP_RETURN_ON_ERROR(mcpwm_capture_channel_register_event_callbacks(cap_chan, &cbs, (void*)hcsr_04_handle),TAG,"Cannot register the ISR callback.");
    // configure the trig pin
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << config->gpio_num_trig,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf),TAG,"Cannot configure trig, GPIO%d",config->gpio_num_trig);
    // drive low by default
    ESP_RETURN_ON_ERROR(gpio_set_level(config->gpio_num_trig, 0),TAG,"Cannot set level on trig pin of GPIO%d.",config->gpio_num_trig);

    hcsr_04_handle->mcpwm_capture_timer = hcsr_timer;
    hcsr_04_handle->mcpwm_capture_channel = cap_chan;
    hcsr_04_handle->capture_timestamp_begin = 0;
    hcsr_04_handle->capture_timestamp_end = 0;
    hcsr_04_handle->data_queue = config->meas_queue;
    hcsr_04_handle->gpio_num_trig = config->gpio_num_trig;
    hcsr_04_handle->sensor_enum = config->sensor_enum;

    *handle = hcsr_04_handle; // write our fresh handle
    return ESP_OK;
}

/**
 * Deletes an allocated HCSR04 driver.
 * Take note that the queue will not be deleted.
 * @param handle the to be deleted handle 
*/
esp_err_t hcsr_04_driver_del(hcsr_04_handle_t handle){
    ESP_RETURN_ON_ERROR(mcpwm_del_capture_channel(handle->mcpwm_capture_channel),TAG,"Cannot delete capture channel.");
    ESP_RETURN_ON_ERROR(gpio_reset_pin(handle->gpio_num_trig),TAG,"Cannot unregister trigger pin.");
    free(handle);
    return ESP_OK;
}

/**
 * Starts the hardware related with the HCSR04 sensor.
 * @param handle the to be enabled handle 
*/
esp_err_t hcsr_04_driver_enable(hcsr_04_handle_t handle){
    ESP_RETURN_ON_ERROR(mcpwm_capture_channel_enable(handle->mcpwm_capture_channel),TAG,"Cannot enable capture channel.");
    return ESP_OK;
}
/**
 * Stops the hardware related with the HCSR04 sensor.
 * @param handle the to be disabled handle 
*/
esp_err_t hcsr_04_driver_disable(hcsr_04_handle_t handle){
    ESP_RETURN_ON_ERROR(mcpwm_capture_channel_disable(handle->mcpwm_capture_channel),TAG,"Cannot disable capture channel.");
    return ESP_OK;
}


/**
 * Starts the capture by pulsing the TRIG pin of the sensor.
 * Read your resuls in hcsr_04_capture_data_t format from the queue you supplied.
 * @param handle the handle to capture from 
*/
esp_err_t hcsr_04_start_capture(hcsr_04_handle_t handle){
    ESP_RETURN_ON_ERROR(gpio_set_level(handle->gpio_num_trig, 1),TAG,"Cannot set GPIO%d HIGH to capture.",handle->gpio_num_trig); // set high
    esp_rom_delay_us(10); // unstable API, watch out for this 
    ESP_RETURN_ON_ERROR(gpio_set_level(handle->gpio_num_trig, 0),TAG,"Cannot set GPIO%d LOW to capture.",handle->gpio_num_trig); // set low
    return ESP_OK;
}



