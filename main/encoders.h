#ifndef ENCODERS_H
#define ENCODERS_H
#include <stdint.h>
#include "driver/pulse_cnt.h"
#include"esp_err.h"
#include "esp_check.h"


typedef struct
{
    int edge_gpio_num; // The input capture GPIO pin number
    uint32_t max_glitch_ns; // The maximum glitch interval in ns 
    int high_limit; // upper limit for the encoder
    int low_limit; // lower limit for the encoder

} sphase_encoder_config_t;

typedef enum {
    SPHASE_ENCODER_INC_ON_RISING_EDGE_INC_ON_FALLING_EDGE,
    SPHASE_ENCODER_INC_ON_RISING_EDGE_DEC_ON_FALLING_EDGE,
    SPHASE_ENCODER_DEC_ON_RISING_EDGE_INC_ON_FALLING_EDGE,
    SPHASE_ENCODER_DEC_ON_RISING_EDGE_DEC_ON_FALLING_EDGE,
    SPHASE_ENCODER_COUNT_INIT,
} sphase_encoder_action_t;


typedef struct
{
    pcnt_unit_handle_t pcnt_unit; 
    pcnt_channel_handle_t pcnt_channel;
    int64_t encoder_accum; // internal accumulator to bypass hardware rollover due to hw counter of pcnt unit
    sphase_encoder_action_t cur_dir;
    int high_limit; // upper limit for the encoder
    int low_limit; // lower limit for the encoder
    
} sphase_encoder_ctx_t;

typedef sphase_encoder_ctx_t* sphase_encoder_handle_t; // define the handle, which is a ptr

// Function declares
esp_err_t sphase_encoder_new(sphase_encoder_config_t *config, sphase_encoder_handle_t* handle);
esp_err_t sphase_encoder_del(sphase_encoder_handle_t handle);
esp_err_t sphase_encoder_enable(sphase_encoder_handle_t handle);
esp_err_t sphase_encoder_disable(sphase_encoder_handle_t handle);
esp_err_t sphase_encoder_start(sphase_encoder_handle_t handle);
esp_err_t sphase_encoder_stop(sphase_encoder_handle_t handle);
esp_err_t sphase_encoder_change_direction(sphase_encoder_handle_t handle, sphase_encoder_action_t dir);
esp_err_t sphase_encoder_reset_count(sphase_encoder_handle_t handle);
esp_err_t sphase_encoder_get_count_from_counter(sphase_encoder_handle_t handle, int *val);
esp_err_t sphase_encoder_get_count_from_accumulator(sphase_encoder_handle_t handle, int64_t *value);



#endif