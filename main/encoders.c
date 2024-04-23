#include"encoders.h"


static const char* TAG = "SPHASE ENC";


/*
*   PCNT unit overflow callback function.
*/
static bool sphase_encoder_ovf_callback(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void* sphase_encoder_handle){
    // do accumulation operation on the passed context 
    sphase_encoder_handle_t encoder_handle = (sphase_encoder_handle_t) sphase_encoder_handle;
    // determine type of ovf
    if(edata->watch_point_value > 0){
        encoder_handle->encoder_accum += encoder_handle->high_limit;
    }
    else if(edata->watch_point_value < 0){
        encoder_handle->encoder_accum += encoder_handle->low_limit; // low limit is already negative, so add 
    }
    return true;
}

/**
 * Create a new single phase encoder object. 
 * 
 * @param config The config to configure the encoder.
 * @param handle The handle which will be returned by the encoder on success.
 * @returns esp_err_t Whether the operation was successfull or not.
 */
esp_err_t sphase_encoder_new(sphase_encoder_config_t *config, sphase_encoder_handle_t* handle){
    // attempt to create our own context
    sphase_encoder_handle_t sphase_encoder_handle = (sphase_encoder_handle_t) malloc(sizeof(sphase_encoder_ctx_t));
    if(!sphase_encoder_handle){
        ESP_LOGE(TAG,"Cannot allocate sphase encoder object.");
        return ESP_ERR_NO_MEM; // handle no heap case
    }

    sphase_encoder_handle->pcnt_unit = NULL;
    sphase_encoder_handle->pcnt_channel = NULL;
    sphase_encoder_handle->encoder_accum = 0;
    sphase_encoder_handle->cur_dir = SPHASE_ENCODER_COUNT_INIT;
    sphase_encoder_handle->high_limit = config->high_limit;
    sphase_encoder_handle->low_limit = config->low_limit;
    

    // Create config for pcnt unit
    pcnt_unit_config_t pcnt_unit_config = {
        .high_limit = config->high_limit,
        .low_limit = config->low_limit,
    };

    pcnt_unit_handle_t pcnt_unit = NULL;
    // attempt to create the new unit
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&pcnt_unit_config, &pcnt_unit),TAG,"Cannot create new pcnt unit.");
    sphase_encoder_handle->pcnt_unit = pcnt_unit; // register to our ctx

    // add the watchpoints to handle overflow cases
    ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(pcnt_unit,config->high_limit),TAG,"Cannot create new watchpoint");
    ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(pcnt_unit,config->low_limit),TAG,"Cannot create new watchpoint");
    // register our overflow callback function 
    pcnt_event_callbacks_t callback_ovf = {
        .on_reach = sphase_encoder_ovf_callback,
    };
    // register the ovf event callback with our ctx 
    ESP_RETURN_ON_ERROR(pcnt_unit_register_event_callbacks(pcnt_unit, &callback_ovf, sphase_encoder_handle),TAG,"Cannot register callbacks.");

    // set up the glitch filter as configured
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = config->max_glitch_ns,
    };
    ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config),TAG,"Cannot set up glitch filter");
    // set up the channel as configured
    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = config->edge_gpio_num,
        .level_gpio_num = -1, // We will not use level since it is single phase
        .flags.virt_level_io_level = 1, // default will count up, we will fix no init case later
    };
    // create the channel
    pcnt_channel_handle_t pcnt_channel = NULL;
    ESP_RETURN_ON_ERROR(pcnt_new_channel(pcnt_unit,&chan_config,&pcnt_channel),TAG,"Cannot create the new pcnt channel.");
    sphase_encoder_handle->pcnt_channel = pcnt_channel;
    // we will not use the level channel, so keep it in a determined state 
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_channel, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // write to our handle and return ok
    *handle = sphase_encoder_handle;
    return ESP_OK;
}
/**
 * Delete an existing single phase encoder object. 
 * 
 * @param handle The handle which will be deleted
 * @returns esp_err_t Whether the operation was successfull or not.
 */
esp_err_t sphase_encoder_del(sphase_encoder_handle_t handle){
    if(handle == NULL) {return ESP_ERR_NOT_FOUND;}
    sphase_encoder_disable(handle);
    ESP_RETURN_ON_ERROR(pcnt_del_unit(handle->pcnt_unit),TAG,"Cannot delete the pcnt unit.");
    ESP_RETURN_ON_ERROR(pcnt_del_channel(handle->pcnt_channel),TAG,"Cannot delete the pcnt channel.");
    free(handle);
    return ESP_OK;
}

/**
 * Enable the single phase encoder.
 * 
 * @param handle The handle which will be enabled
 * @returns esp_err_t Whether the operation was successfull or not.
 */
esp_err_t sphase_encoder_enable(sphase_encoder_handle_t handle){
    if(handle == NULL) {return ESP_ERR_NOT_FOUND;}

    if(handle->cur_dir == SPHASE_ENCODER_COUNT_INIT){
        ESP_LOGW(TAG,"Direction not set. Using default direction.");
    }
    return pcnt_unit_enable(handle->pcnt_unit);
}
/**
 * Disable the single phase encoder.
 * 
 * @param handle The handle which will be disable
 * @returns esp_err_t Whether the operation was successfull or not.
 */
esp_err_t sphase_encoder_disable(sphase_encoder_handle_t handle){
    if(handle == NULL) {return ESP_ERR_NOT_FOUND;}

    return pcnt_unit_disable(handle->pcnt_unit);
}

/**
 * Start the single phase encoder. It will retain count in this state.
 * 
 * @param handle The handle which will start 
 * @returns esp_err_t Whether the operation was successfull or not.
 */
esp_err_t sphase_encoder_start(sphase_encoder_handle_t handle){
    if(handle == NULL) {return ESP_ERR_NOT_FOUND;}

    return pcnt_unit_start(handle->pcnt_unit);
}
/**
 * Stop the single phase encoder. It will retain count in this state.
 * 
 * @param handle The handle which will stop 
 * @returns esp_err_t Whether the operation was successfull or not.
 */
esp_err_t sphase_encoder_stop(sphase_encoder_handle_t handle){
    if(handle == NULL) {return ESP_ERR_NOT_FOUND;}

    return pcnt_unit_stop(handle->pcnt_unit);
}
/**
 * Change the encoder counting direction.
 * 
 * @param handle The handle which the new direction will be applied to. 
 * @param dir The direction that will be applied.
 * @returns esp_err_t Whether the operation was successfull or not.
 */
esp_err_t sphase_encoder_change_direction(sphase_encoder_handle_t handle, sphase_encoder_action_t dir){
    if(handle == NULL) {return ESP_ERR_NOT_FOUND;}

    switch (dir)
    {
    case SPHASE_ENCODER_INC_ON_RISING_EDGE_INC_ON_FALLING_EDGE:
        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(handle->pcnt_channel, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
        break;
    case SPHASE_ENCODER_INC_ON_RISING_EDGE_DEC_ON_FALLING_EDGE:
        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(handle->pcnt_channel, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
        break;
    case SPHASE_ENCODER_DEC_ON_RISING_EDGE_INC_ON_FALLING_EDGE:
        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(handle->pcnt_channel, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
        break;
    case SPHASE_ENCODER_DEC_ON_RISING_EDGE_DEC_ON_FALLING_EDGE:
        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(handle->pcnt_channel, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
        break;    
    default:
        ESP_LOGE(TAG,"Unknown action.");
        return ESP_FAIL; // hope this will not jump the stack or smth, watch out for this line think i had problems with return from switch-case before
        //break;
    }
    handle->cur_dir = dir;
    return ESP_OK;
}
/**
 * Resets the internal accumulator and the hardware counter.
 * 
 * @param handle The handle which the reset will be applied to. 
 * @returns esp_err_t Whether the operation was successfull or not.
 */
esp_err_t sphase_encoder_reset_count(sphase_encoder_handle_t handle){
    if(handle == NULL) {return ESP_ERR_NOT_FOUND;}

    handle->encoder_accum = 0;
    return pcnt_unit_clear_count(handle->pcnt_unit);
}

/**
 * This will return the current count from the hardware counter.
 * This means that it can and will overflow within the bounds you set with the config at allocation.
 * 
 * @param handle The handle to get the count from. 
 * @returns esp_err_t Whether the operation was successfull or not.
 */
esp_err_t sphase_encoder_get_count_from_counter(sphase_encoder_handle_t handle, int *val){
    if(handle == NULL) {return ESP_ERR_NOT_FOUND;}
   return pcnt_unit_get_count(handle->pcnt_unit,val);
}

/**
 * This will return the current count from the hardware counter and the internal software accumulator.
 * This will still overflow, but its bound by int64_t limits.
 * 
 * @param handle The handle to get the count from. 
 * @returns esp_err_t Whether the operation was successfull or not.
 */
esp_err_t sphase_encoder_get_count_from_accumulator(sphase_encoder_handle_t handle, int64_t *value){
    if(handle == NULL) {return ESP_ERR_NOT_FOUND;}

    int hw_count;
    esp_err_t err = pcnt_unit_get_count(handle->pcnt_unit,&hw_count);
    *value = ((int64_t) hw_count) + handle->encoder_accum;

    return err;
}

