#ifndef L298N_DRIVER_H
#define L298N_DRIVER_H
#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include"esp_err.h"
#include "esp_check.h"
#include <stdio.h>


typedef struct 
{
    int mcpwm_group_id;
    int mpcwm_timer_resolution_hz;
    int mcpwm_pwm_freq_hz;
    int mcpwm_min_pwm_ticks; // at which % power does motors start to move ? The user will need to tune this.

    int gpio_ena_num;
    int gpio_enb_num;
    int gpio_in1_num;
    int gpio_in2_num;
    int gpio_in3_num;
    int gpio_in4_num;
       
} l298n_driver_config_t;

typedef enum {
    L298N_TARGET_MOTOR_A,    
    L298N_TARGET_MOTOR_B,   
    L298N_TARGET_BOTH_MOTORS,   
         
} l298n_driver_motor_targets_t;

typedef enum {
    L298N_ACTION_FORWARD,
    L298N_ACTION_BACKWARD,
    L298N_ACTION_COAST,
    L298N_ACTION_BRAKE,
    
} l298n_driver_actions_t;

typedef enum {
    L298N_STATE_MOTOR_FORWARD,
    L298N_STATE_MOTOR_BACKWARD,
    L298N_STATE_MOTOR_COAST,
    L298N_STATE_MOTOR_BRAKE,     
} l298n_driver_motor_states_t;

typedef struct 
{
    // Configuration related 
    int mcpwm_group_id;
    int mpcwm_timer_resolution_hz;
    int mcpwm_pwm_freq_hz;
    int mcpwm_max_pwm_ticks; 
    int mcpwm_min_pwm_ticks; // at which % power does motors start to move ? The user will need to tune this.
    // MCPWM Resources
    mcpwm_timer_handle_t mcpwm_timer;
    mcpwm_oper_handle_t mcpwm_operator;
    mcpwm_gen_handle_t mcpwm_gen_a;
    mcpwm_gen_handle_t mcpwm_gen_b;
    mcpwm_cmpr_handle_t mcpwm_comparator_a;
    mcpwm_cmpr_handle_t mcpwm_comparator_b;
    // Internal State
    l298n_driver_motor_states_t motor_state_a;
    l298n_driver_motor_states_t motor_state_b;
    int motor_speed_a;
    int motor_speed_b;
    // Pins
    int gpio_in1_num;
    int gpio_in2_num;
    int gpio_in3_num;
    int gpio_in4_num;

} l298n_driver_context_t;

typedef l298n_driver_context_t* l298n_driver_handle_t;


esp_err_t l298n_driver_new(l298n_driver_config_t *config, l298n_driver_handle_t* handle);
esp_err_t l298n_driver_del(l298n_driver_handle_t handle);
esp_err_t l298n_driver_enable(l298n_driver_handle_t handle);
esp_err_t l298n_driver_disable(l298n_driver_handle_t handle);
esp_err_t l298n_driver_start_motors(l298n_driver_handle_t handle);
esp_err_t l298n_driver_stop_motors(l298n_driver_handle_t handle);
esp_err_t l298n_driver_get_min_speed(l298n_driver_handle_t handle, int *min_speed);
esp_err_t l298n_driver_get_max_speed(l298n_driver_handle_t handle, int *max_speed);
esp_err_t l298n_driver_get_speed(l298n_driver_handle_t handle,l298n_driver_motor_targets_t target, int * motor_speed);
esp_err_t l298n_driver_set_speed(l298n_driver_handle_t handle,l298n_driver_motor_targets_t target, int motor_speed);
esp_err_t l298n_driver_get_direction(l298n_driver_handle_t handle,l298n_driver_motor_targets_t target, l298n_driver_motor_states_t *current_dir);
esp_err_t l298n_driver_set_direction(l298n_driver_handle_t handle, l298n_driver_motor_targets_t target, l298n_driver_motor_states_t target_dir);
esp_err_t l298n_driver_print_motor_info(l298n_driver_handle_t handle);
esp_err_t l298n_driver_calibration_check(l298n_driver_handle_t handle);


#endif