#include"l298n_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "L298N Driver";
/**
 * Allocates a new driver object and writes the newly allocated handle to the given handle pointer.
 * 
 * @param config the configuration of the new driver 
 * @param handle the newly created driver
 * @returns esp_err_t whether the operation was successfull or not
 */
esp_err_t l298n_driver_new(l298n_driver_config_t *config, l298n_driver_handle_t* handle){
    if(!handle) {return ESP_ERR_NOT_FOUND;}
    if(!config) {return ESP_ERR_NOT_FOUND;}
    // attempt to acquire memory for the internal context we will keep
    l298n_driver_handle_t l298n_driver_handle = (l298n_driver_handle_t) calloc(1,sizeof(l298n_driver_context_t));
    if(!l298n_driver_handle){
        ESP_LOGE(TAG,"Cannot acquire memory for internal context.");
        return ESP_ERR_NO_MEM;
    }
    // initiliaze internal context
    l298n_driver_handle->mcpwm_group_id = config->mcpwm_group_id;
    l298n_driver_handle->mpcwm_timer_resolution_hz = config->mpcwm_timer_resolution_hz;
    l298n_driver_handle->mcpwm_pwm_freq_hz = config->mcpwm_pwm_freq_hz;
    l298n_driver_handle->mcpwm_min_pwm_ticks = config->mcpwm_min_pwm_ticks;

    l298n_driver_handle->motor_state_a = L298N_STATE_MOTOR_COAST;
    l298n_driver_handle->motor_state_b = L298N_STATE_MOTOR_COAST;
    l298n_driver_handle->motor_speed_a = 0;
    l298n_driver_handle->motor_speed_b = 0;
    l298n_driver_handle->gpio_in1_num = config->gpio_in1_num;
    l298n_driver_handle->gpio_in2_num = config->gpio_in2_num;
    l298n_driver_handle->gpio_in3_num = config->gpio_in3_num;
    l298n_driver_handle->gpio_in4_num = config->gpio_in4_num;
    
    l298n_driver_handle->mcpwm_max_pwm_ticks = config->mpcwm_timer_resolution_hz / config->mcpwm_pwm_freq_hz;   // calculate the maximum pwm timer ticks 
 

    ESP_LOGI(TAG,"Configuring GPIO pads for direction.");
    gpio_config_t io_conf;
    // Disable interrupt for all direction pins
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // Set them as output
    io_conf.mode = GPIO_MODE_OUTPUT;
    // Disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    // Enable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    // Set GPIO pins mask
    io_conf.pin_bit_mask = (1ULL<<config->gpio_in1_num) | (1ULL<<config->gpio_in2_num) | (1ULL<<config->gpio_in3_num) | (1ULL<<config->gpio_in4_num);
    // Configure GPIO with the given settings
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf),TAG,"Cannot configure GPIO pads");
    
    // Start allocating MCPWM objects
    // acquire the timer
    mcpwm_timer_config_t mcpwm_timer_config = {
        .group_id = config->mcpwm_group_id,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = config->mpcwm_timer_resolution_hz,
        .period_ticks = l298n_driver_handle->mcpwm_max_pwm_ticks,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP
    };
    mcpwm_timer_handle_t mcpwm_timer_handle = NULL;
    ESP_RETURN_ON_ERROR(mcpwm_new_timer(&mcpwm_timer_config,&mcpwm_timer_handle),TAG,"Cannot acquire new MCPWM timer.");
    l298n_driver_handle->mcpwm_timer = mcpwm_timer_handle;

    // acquire the operator now 
    mcpwm_operator_config_t mcpwm_operator_config = {
        .group_id = config->mcpwm_group_id,
    };
    mcpwm_oper_handle_t mcpwm_operator = NULL;
    ESP_RETURN_ON_ERROR(mcpwm_new_operator(&mcpwm_operator_config,&mcpwm_operator),TAG,"Cannot acquire new MCPWM operator.");
    ESP_RETURN_ON_ERROR(mcpwm_operator_connect_timer(mcpwm_operator,mcpwm_timer_handle),TAG,"Cannot connect new operator to MCPWM timer.");
    l298n_driver_handle->mcpwm_operator = mcpwm_operator;
    // acquire two new generators for dual channel 
    mcpwm_gen_handle_t pwm_generator_a = NULL;
    mcpwm_gen_handle_t pwm_generator_b = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = config->gpio_ena_num,
    };
    ESP_RETURN_ON_ERROR(mcpwm_new_generator(mcpwm_operator,&generator_config,&pwm_generator_a),TAG,"Cannot acquire MCPWM Generator A.");
    generator_config.gen_gpio_num = config->gpio_enb_num;
    ESP_RETURN_ON_ERROR(mcpwm_new_generator(mcpwm_operator,&generator_config,&pwm_generator_b),TAG,"Cannot acquire MCPWM Generator B.");
    l298n_driver_handle->mcpwm_gen_a = pwm_generator_a;
    l298n_driver_handle->mcpwm_gen_b = pwm_generator_b;
    // acquire the comparators
    mcpwm_cmpr_handle_t mcpwm_comparator_a = NULL;
    mcpwm_cmpr_handle_t mcpwm_comparator_b = NULL;
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_RETURN_ON_ERROR(mcpwm_new_comparator(mcpwm_operator,&comparator_config,&mcpwm_comparator_a),TAG,"Cannot acquire MCPWM comparator A");
    ESP_RETURN_ON_ERROR(mcpwm_new_comparator(mcpwm_operator,&comparator_config,&mcpwm_comparator_b),TAG,"Cannot acquire MCPWM comparator B");
    l298n_driver_handle->mcpwm_comparator_a = mcpwm_comparator_a;
    l298n_driver_handle->mcpwm_comparator_b = mcpwm_comparator_b;
    // set default PWM behaviours 
    ESP_RETURN_ON_ERROR(mcpwm_generator_set_actions_on_timer_event(pwm_generator_a,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
            MCPWM_GEN_TIMER_EVENT_ACTION_END()),
            TAG,"Cannot set behaviour of generator A.");
    ESP_RETURN_ON_ERROR(mcpwm_generator_set_actions_on_compare_event(pwm_generator_a,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, mcpwm_comparator_a, MCPWM_GEN_ACTION_LOW),
            MCPWM_GEN_COMPARE_EVENT_ACTION_END()),
            TAG,"Cannot set behaviour of generator A.");
    ESP_RETURN_ON_ERROR(mcpwm_generator_set_actions_on_timer_event(pwm_generator_b,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
            MCPWM_GEN_TIMER_EVENT_ACTION_END()),
            TAG,"Cannot set behaviour of generator B");
    ESP_RETURN_ON_ERROR(mcpwm_generator_set_actions_on_compare_event(pwm_generator_b,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, mcpwm_comparator_b, MCPWM_GEN_ACTION_LOW),
            MCPWM_GEN_COMPARE_EVENT_ACTION_END()),
            TAG,"Cannot set behavour of generator B.");

    *handle = l298n_driver_handle;
    return ESP_OK;
}

/**
 * Deallocates the internal resources of the L298N driver.
 * 
 * @param handle the driver that will be deleted
 * @returns esp_err_t the success of the operation
 */
esp_err_t l298n_driver_del(l298n_driver_handle_t handle){
    if(!handle) {return ESP_ERR_NOT_FOUND;}

    mcpwm_timer_disable(handle->mcpwm_timer);
    ESP_RETURN_ON_ERROR(mcpwm_del_comparator(handle->mcpwm_comparator_a),TAG,"Cannot delete comparator A.");
    ESP_RETURN_ON_ERROR(mcpwm_del_comparator(handle->mcpwm_comparator_a),TAG,"Cannot delete comparator B.");
    ESP_RETURN_ON_ERROR(mcpwm_del_generator(handle->mcpwm_gen_a),TAG,"Cannot delete generator A.");
    ESP_RETURN_ON_ERROR(mcpwm_del_generator(handle->mcpwm_gen_b),TAG,"Cannot delete generator B.");
    ESP_RETURN_ON_ERROR(mcpwm_del_operator(handle->mcpwm_operator),TAG,"Cannot delete MCPWM operator.");
    ESP_RETURN_ON_ERROR(mcpwm_del_timer(handle->mcpwm_timer),TAG,"Cannot delete MCPWM timer.");
    free(handle);
    return ESP_OK;
}




/**
 * Internal function to set speed of motor A.
 * 
 * @param handle operand
 * @returns esp_err_t whether the operation was successfull or not
 */
static esp_err_t l298n_driver_set_speed_a(l298n_driver_handle_t handle, int speed){
    if(handle != NULL){
        if(speed > handle->mcpwm_max_pwm_ticks){
            ESP_LOGW(TAG,"Speed command %d bigger than upper limit %d. Clamping at upper limit.",speed,handle->mcpwm_max_pwm_ticks);
            speed = handle->mcpwm_max_pwm_ticks;
        }
        else if( (0 < speed ) && (speed < handle->mcpwm_min_pwm_ticks) ){
            ESP_LOGW(TAG,"Speed command %d smaller than lower limit %d. Motor will not move, or calibrate.",speed,handle->mcpwm_min_pwm_ticks);

        }
        else if( speed < 0)
        {
            ESP_LOGE(TAG,"Invalid speed %d !",speed);            
            return ESP_ERR_INVALID_ARG;
        }

        handle->motor_speed_a = speed;
        ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(handle->mcpwm_comparator_a, speed),TAG,"Cannot set comparator on motor A.");
        return ESP_OK;
    }
    return ESP_ERR_NOT_FOUND;
    }


/**
 * Internal function to set speed of motor B.
 * 
 * @param handle operand
 * @returns esp_err_t
 */
static esp_err_t l298n_driver_set_speed_b(l298n_driver_handle_t handle, int speed){
    if(handle != NULL){
        if(speed > handle->mcpwm_max_pwm_ticks){
            ESP_LOGW(TAG,"Speed command %d bigger than upper limit %d. Clamping at upper limit.",speed,handle->mcpwm_max_pwm_ticks);
            speed = handle->mcpwm_max_pwm_ticks;
        }
        else if( (0 < speed) && (speed < handle->mcpwm_min_pwm_ticks) ){
            ESP_LOGW(TAG,"Speed command %d smaller than lower limit %d. Motor will not move, or calibrate.",speed,handle->mcpwm_min_pwm_ticks);

        }
        else if( speed < 0)
        {
            ESP_LOGE(TAG,"Invalid speed %d !",speed);            
            return ESP_ERR_INVALID_ARG;
        }

        handle->motor_speed_b = speed;
        ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(handle->mcpwm_comparator_b, speed),TAG,"Cannot set comparator on motor B.");
        return ESP_OK;
    }
    return ESP_ERR_NOT_FOUND;
}

/**
 * Internal function to set direction of motor A.
 * 
 * @param handle operand
 * @returns esp_err_t whether the op was successfull or not. 
 */
static esp_err_t l298n_driver_set_direction_a(l298n_driver_handle_t handle, l298n_driver_actions_t action){
    if(!handle) {return ESP_ERR_NOT_FOUND;}
    switch (action)
    {
    case L298N_STATE_MOTOR_FORWARD: // IN1 : 1, IN2: 0
        ESP_RETURN_ON_ERROR(gpio_set_level(handle->gpio_in1_num, 1),TAG,"Cannot set GPIO level on IN1.");
        ESP_RETURN_ON_ERROR(gpio_set_level(handle->gpio_in2_num, 0),TAG,"Cannot set GPIO level on IN2.");
        handle->motor_state_a = action;
        break;
    case L298N_STATE_MOTOR_BACKWARD: // IN1 : 0, IN2: 1
        ESP_RETURN_ON_ERROR(gpio_set_level(handle->gpio_in1_num, 0),TAG,"Cannot set GPIO level on IN1.");
        ESP_RETURN_ON_ERROR(gpio_set_level(handle->gpio_in2_num, 1),TAG,"Cannot set GPIO level on IN2.");
        handle->motor_state_a = action;
        break;
    case L298N_STATE_MOTOR_COAST:
        ESP_RETURN_ON_ERROR(gpio_set_level(handle->gpio_in1_num, 0),TAG,"Cannot set GPIO level on IN1.");
        ESP_RETURN_ON_ERROR(gpio_set_level(handle->gpio_in2_num, 0),TAG,"Cannot set GPIO level on IN2.");
        handle->motor_state_a = action;
        break;
    case L298N_STATE_MOTOR_BRAKE:
        ESP_RETURN_ON_ERROR(l298n_driver_set_speed_a(handle,0),TAG,"Cannot turn off motor A"); // Turn OFF A
        ESP_RETURN_ON_ERROR(gpio_set_level(handle->gpio_in1_num, 0),TAG,"Cannot set GPIO level on IN1.");
        ESP_RETURN_ON_ERROR(gpio_set_level(handle->gpio_in2_num, 0),TAG,"Cannot set GPIO level on IN2.");
        handle->motor_state_a = action;
        break;
    
    default:
        ESP_LOGE(TAG,"Unkown action for motor A");
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

/**
 * Internal function to set direction of motor B.
 * 
 * @param handle operand
 * @returns esp_err_t whether the op was successfull or not. 
 */
static esp_err_t l298n_driver_set_direction_b(l298n_driver_handle_t handle, l298n_driver_actions_t action){
    if(!handle) {return ESP_ERR_NOT_FOUND;}
    switch (action)
    {
    case L298N_STATE_MOTOR_FORWARD: // IN3 : 1, IN4: 0
        ESP_RETURN_ON_ERROR(gpio_set_level(handle->gpio_in3_num, 1),TAG,"Cannot set GPIO level on IN3.");
        ESP_RETURN_ON_ERROR(gpio_set_level(handle->gpio_in4_num, 0),TAG,"Cannot set GPIO level on IN4.");
        handle->motor_state_b = action;
        break;
    case L298N_STATE_MOTOR_BACKWARD: // IN3 : 0, IN4: 1
        ESP_RETURN_ON_ERROR(gpio_set_level(handle->gpio_in3_num, 0),TAG,"Cannot set GPIO level on IN3.");
        ESP_RETURN_ON_ERROR(gpio_set_level(handle->gpio_in4_num, 1),TAG,"Cannot set GPIO level on IN4.");
        handle->motor_state_b = action;
        break;
    case L298N_STATE_MOTOR_COAST:
        ESP_RETURN_ON_ERROR(gpio_set_level(handle->gpio_in3_num, 0),TAG,"Cannot set GPIO level on IN3.");
        ESP_RETURN_ON_ERROR(gpio_set_level(handle->gpio_in4_num, 0),TAG,"Cannot set GPIO level on IN4.");
        handle->motor_state_b = action;
        break;
    case L298N_STATE_MOTOR_BRAKE:
        ESP_RETURN_ON_ERROR(l298n_driver_set_speed_b(handle,0),TAG,"Cannot turn off motor B"); // Turn OFF B
        ESP_RETURN_ON_ERROR(gpio_set_level(handle->gpio_in3_num, 0),TAG,"Cannot set GPIO level on IN3.");
        ESP_RETURN_ON_ERROR(gpio_set_level(handle->gpio_in4_num, 0),TAG,"Cannot set GPIO level on IN4.");
        handle->motor_state_b = action;
        break;
    
    default:
        ESP_LOGE(TAG,"Unkown action for motor B");
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}
/**
 * Internal function to reset the internal motor states & speeds.
 * 
 * @param handle operand
 * @returns 
 */
static void l298n_driver_reset_all_internal_state(l298n_driver_handle_t handle){
    if(handle != NULL){
        handle->motor_state_a = L298N_STATE_MOTOR_COAST;
        handle->motor_state_b = L298N_STATE_MOTOR_COAST;
        l298n_driver_set_speed_a(handle,0);
        l298n_driver_set_speed_b(handle,0);
    }
}


/**
 * Enable the hardware associated with the L298N Driver.
 * This will not make the motors move, the motor needs to be also started.
 * 
 * @param handle The handle which will be enabled
 * @returns esp_err_t Whether the operation was successfull or not.
 */
esp_err_t l298n_driver_enable(l298n_driver_handle_t handle){
    if(!handle) {return ESP_ERR_NOT_FOUND;}
    ESP_RETURN_ON_ERROR(mcpwm_timer_enable(handle->mcpwm_timer),TAG,"Cannot enable hardware.");
    return ESP_OK;
}

/**
 * Disable the hardware associated with the L298N Driver.
 * This will cease all actions and will RESET all internal state.
 * 
 * @param handle The handle which will be enabled
 * @returns esp_err_t Whether the operation was successfull or not.
 */
esp_err_t l298n_driver_disable(l298n_driver_handle_t handle){
    if(!handle) {return ESP_ERR_NOT_FOUND;}
    ESP_RETURN_ON_ERROR(mcpwm_timer_disable(handle->mcpwm_timer),TAG,"Cannot disable hardware.");
    l298n_driver_reset_all_internal_state(handle);
    return ESP_OK; 
}

/**
 * Start the motors. After this command is executed, setting the speed and direction
 *  will take effect. Doing this once is enough. It can be though of as a software enable.
 * .
 * 
 * @param handle The handle to start the motors of 
 * @returns esp_err_t Whether the operation was successfull or not.
 */
esp_err_t l298n_driver_start_motors(l298n_driver_handle_t handle){
    if(!handle) {return ESP_ERR_NOT_FOUND;}
    ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(handle->mcpwm_timer,MCPWM_TIMER_START_NO_STOP),TAG,"Cannot access hardware to start.");
    return ESP_OK;
}

/**
 * Stop the motors, timer hardware is still enabled in this state. 
 *  The motors will immidieatly stop.  Direction and speed will NOT be  reset.
 *  You can continue driving by calling the start command after this.
 * 
 * 
 * @param handle The handle to start the motors of 
 * @returns esp_err_t Whether the operation was successfull or not.
 */
esp_err_t l298n_driver_stop_motors(l298n_driver_handle_t handle){
     if(!handle) {return ESP_ERR_NOT_FOUND;}
    ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(handle->mcpwm_timer,MCPWM_TIMER_STOP_FULL),TAG,"Cannot access hardware to stop.");
    return ESP_OK;   
}

/**
 * Get the minimum speed that moves the motor reported by the user.
 * This needs to be calibrated by the user. You can check your calibration with the
 * calibration checking utility command.
 * 
 * 
 * @param handle The handle to get the minmum speed from 
 * @param min_speed  int pointer where the min speed will be stored
 * @returns esp_err_t Whether the operation was successfull or not.
 */
esp_err_t l298n_driver_get_min_speed(l298n_driver_handle_t handle, int *min_speed){
    if(!handle || !min_speed) {return ESP_ERR_NOT_FOUND;}
    *min_speed = handle->mcpwm_min_pwm_ticks;
    return ESP_OK;
}

/**
 * Gets the maximum speed value that the PWM hardware can achieve. Setting this
 * as the speed will mean that the motors are working at %100 percent. This value is
 * influenced by timer resolution and pwm frequency given by the user in the config. 
 * 
 * 
 * @param handle The handle to get the maximum speed from 
 * @param max_speed  int pointer where the max speed will be stored
 * @returns esp_err_t Whether the operation was successfull or not.
 */
esp_err_t l298n_driver_get_max_speed(l298n_driver_handle_t handle, int *max_speed){
    if(!handle || !max_speed) {return ESP_ERR_NOT_FOUND;}
    *max_speed = handle->mcpwm_max_pwm_ticks;
    return ESP_OK;
}

/**
 *  This will get the speed from the supplied target motor. Supports a single motor to get from.
 *  
 * 
 * @param handle The handle of L298N driver
 * @param target The target motor to get the speed from
 * @param motor_speed  int pointer where the current motor speed will be stored
 * @returns esp_err_t Whether the operation was successfull or not.
 */
esp_err_t l298n_driver_get_speed(l298n_driver_handle_t handle,l298n_driver_motor_targets_t target, int * motor_speed){
    if(!handle || !motor_speed) {return ESP_ERR_NOT_FOUND;}
    switch (target)
    {
    case L298N_TARGET_MOTOR_A:
        *motor_speed = handle->motor_speed_a;
        break;
    case L298N_TARGET_MOTOR_B:
        *motor_speed = handle->motor_speed_b;
        break;
    case L298N_TARGET_BOTH_MOTORS:
        ESP_LOGE(TAG,"Both motors is not supported in this function.");
        break;
            
    default:
        ESP_LOGE(TAG,"Unkown motor target.");
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}
/**
 *  This will set the speed from the supplied target motor. 
 *  
 * 
 * @param handle The handle of L298N driver
 * @param target The target motor to set the speed 
 * @param motor_speed  the new motor speed 
 * @returns esp_err_t Whether the operation was successfull or not.
 */ 
esp_err_t l298n_driver_set_speed(l298n_driver_handle_t handle,l298n_driver_motor_targets_t target, int motor_speed){
    if(!handle) {return ESP_ERR_NOT_FOUND;}
    switch (target)
    {
    case L298N_TARGET_MOTOR_A:
        ESP_RETURN_ON_ERROR(l298n_driver_set_speed_a(handle,motor_speed),TAG,"Cannot set speed on A.");
        break;
    case L298N_TARGET_MOTOR_B:
        ESP_RETURN_ON_ERROR(l298n_driver_set_speed_b(handle,motor_speed),TAG,"Cannot set speed on B.");
        break;
    case L298N_TARGET_BOTH_MOTORS:
        ESP_RETURN_ON_ERROR(l298n_driver_set_speed_a(handle,motor_speed),TAG,"Cannot set speed on A.");
        ESP_RETURN_ON_ERROR(l298n_driver_set_speed_b(handle,motor_speed),TAG,"Cannot set speed on B.");
        break;
            
    default:
        ESP_LOGE(TAG,"Unkown motor target.");
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;  
}

/**
 *  This will get the current direction state from the supplied motor.
 *  
 * 
 * @param handle The handle of L298N driver
 * @param target The target motor to get the direction from 
 * @param current_dir  pointer to the current direction, will write here  
 * @returns esp_err_t Whether the operation was successfull or not.
 */
esp_err_t l298n_driver_get_direction(l298n_driver_handle_t handle,l298n_driver_motor_targets_t target, l298n_driver_motor_states_t *current_dir){
    if(!handle || !current_dir) {return ESP_ERR_NOT_FOUND;}
    switch (target)
    {
    case L298N_TARGET_MOTOR_A:
        *current_dir = handle->motor_state_a;
        break;
    case L298N_TARGET_MOTOR_B:
        *current_dir = handle->motor_state_b;
        break;
    case L298N_TARGET_BOTH_MOTORS:
        ESP_LOGE(TAG,"This action is not supported for get_direction.");
        return ESP_ERR_INVALID_ARG;    
    default:
        ESP_LOGE(TAG,"Unknown target.");
        return ESP_ERR_INVALID_ARG;    
    }
    return ESP_OK;
}

/**
* Sets the direction of the target motor. 
* Please note that the L298N cannot directly run a motor while stopping
* the other one. To achieve this, set the direction of both motors in the desired direction 
* and set the speed of one of them to 0. 
* @param handle the handle of the driver.
* @param target the target motor to set the direction.
* @param target_dir the actual target direction to set to. 
*
*/
esp_err_t l298n_driver_set_direction(l298n_driver_handle_t handle, l298n_driver_motor_targets_t target, l298n_driver_motor_states_t target_dir){
    if(!handle) {return ESP_ERR_NOT_FOUND;}
    switch (target)
    {
    case L298N_TARGET_MOTOR_A:
        ESP_RETURN_ON_ERROR(l298n_driver_set_direction_a(handle,target_dir),TAG,"Cannot set direction on A.");
        break;
    case L298N_TARGET_MOTOR_B:
        ESP_RETURN_ON_ERROR(l298n_driver_set_direction_b(handle,target_dir),TAG,"Cannot set direction on B.");
        break;
    case L298N_TARGET_BOTH_MOTORS:
        ESP_RETURN_ON_ERROR(l298n_driver_set_direction_a(handle,target_dir),TAG,"Cannot set direction on A.");
        ESP_RETURN_ON_ERROR(l298n_driver_set_direction_b(handle,target_dir),TAG,"Cannot set direction on B.");
        break;
    
    default:
        ESP_LOGE(TAG,"Unknown direction action.");
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}
// Resolves the supplied state enum into a string 
static const char* get_str_from_motor_state_enum(l298n_driver_motor_states_t state){
    switch (state)
    {
    case L298N_ACTION_FORWARD:
        return "FORWARD";
    case L298N_ACTION_BACKWARD:
        return "BACKWARD";
    case L298N_ACTION_COAST:
        return "COAST";
    case L298N_ACTION_BRAKE:
        return "BRAKE";
    
    default:
       return "UNKNOWN";
    }
}

// Resolves the supplied motor target enum into a string 
static const char* get_str_from_motor_target_enum(l298n_driver_motor_targets_t target){
    switch (target)
    {
    case L298N_TARGET_BOTH_MOTORS:
        return "BOTH MOTORS";
    case L298N_TARGET_MOTOR_A:
        return "MOTOR A";
    case L298N_TARGET_MOTOR_B:
        return "MOTOR B";
    default:
       return "UNKNOWN";
    }
}
esp_err_t l298n_driver_print_motor_info(l298n_driver_handle_t handle){
    ESP_LOGI(TAG, "\nMotorA:\n"
                  "Current State: %s\n"
                  "Speed: %d\n\n"
                  "MotorB:\n"
                  "Current State: %s\n"
                  "Speed:%d",
                get_str_from_motor_state_enum(handle->motor_state_a),handle->motor_speed_a,
                get_str_from_motor_state_enum(handle->motor_state_b),handle->motor_speed_b);

    return ESP_OK;
}

esp_err_t l298n_driver_calibration_check(l298n_driver_handle_t handle){
    int max_speed = handle->mcpwm_max_pwm_ticks;
    ESP_LOGI(TAG,"Entering calibration mode. Please prop your robot up as it will move. Enable HW interfaces and start motor before calling.");
    ESP_LOGI(TAG,"Commencing calibration in 3 seconds...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    ESP_LOGI(TAG,"Both motors will move forwards for 5 seconds.");
    l298n_driver_set_direction(handle,L298N_TARGET_BOTH_MOTORS,L298N_ACTION_FORWARD);
    l298n_driver_set_speed(handle,L298N_TARGET_BOTH_MOTORS,max_speed);
    l298n_driver_print_motor_info(handle);
    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_LOGI(TAG,"Both motors will move backwards for 5 seconds.");
    l298n_driver_set_direction(handle,L298N_TARGET_BOTH_MOTORS,L298N_ACTION_BACKWARD);
    l298n_driver_set_speed(handle,L298N_TARGET_BOTH_MOTORS,max_speed);
    l298n_driver_print_motor_info(handle);
    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_LOGI(TAG,"Only motor B will move forward for 5 seconds.");
    l298n_driver_set_direction(handle,L298N_TARGET_BOTH_MOTORS,L298N_ACTION_FORWARD);
    l298n_driver_set_speed(handle,L298N_TARGET_MOTOR_A,0);
    l298n_driver_set_speed(handle,L298N_TARGET_MOTOR_B,max_speed);
    l298n_driver_print_motor_info(handle);
    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_LOGI(TAG,"Only motor A will move forward for 5 seconds.");
    l298n_driver_set_direction(handle,L298N_TARGET_BOTH_MOTORS,L298N_ACTION_FORWARD);
    l298n_driver_set_speed(handle,L298N_TARGET_MOTOR_A,max_speed);
    l298n_driver_set_speed(handle,L298N_TARGET_MOTOR_B,0);
    l298n_driver_print_motor_info(handle);

    vTaskDelay(pdMS_TO_TICKS(5000));    
    ESP_LOGI(TAG,"Motor A will move forward, Motor B will move backward for 5 seconds.");
    l298n_driver_set_direction(handle,L298N_TARGET_MOTOR_A,L298N_ACTION_FORWARD);
    l298n_driver_set_direction(handle,L298N_TARGET_MOTOR_B,L298N_ACTION_BACKWARD);
    l298n_driver_set_speed(handle,L298N_TARGET_BOTH_MOTORS,max_speed);
    l298n_driver_print_motor_info(handle);

    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_LOGI(TAG,"Motor A will move backward, Motor B will move forward for 5 seconds.");
    l298n_driver_set_direction(handle,L298N_TARGET_MOTOR_A,L298N_ACTION_BACKWARD);
    l298n_driver_set_direction(handle,L298N_TARGET_MOTOR_B,L298N_ACTION_FORWARD);
    l298n_driver_set_speed(handle,L298N_TARGET_BOTH_MOTORS,max_speed);
    l298n_driver_print_motor_info(handle);

    vTaskDelay(pdMS_TO_TICKS(5000));

    ESP_LOGI(TAG,"Brake test, both will move forward for 5 seconds, then brake.");
    l298n_driver_set_direction(handle,L298N_TARGET_BOTH_MOTORS,L298N_ACTION_FORWARD);
    l298n_driver_set_speed(handle,L298N_TARGET_BOTH_MOTORS,max_speed);
    l298n_driver_set_speed(handle,L298N_TARGET_BOTH_MOTORS,max_speed);
    l298n_driver_print_motor_info(handle);

    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_LOGI(TAG,"BRAKE!");
    l298n_driver_set_direction(handle,L298N_TARGET_BOTH_MOTORS,L298N_ACTION_BRAKE);
    l298n_driver_print_motor_info(handle);
    vTaskDelay(pdMS_TO_TICKS(3000));
    ESP_LOGI(TAG,"Motor A at half, Motor B at full power.");
    l298n_driver_set_direction(handle,L298N_TARGET_BOTH_MOTORS,L298N_ACTION_FORWARD);
    l298n_driver_set_speed(handle,L298N_TARGET_MOTOR_A,(max_speed + handle->mcpwm_min_pwm_ticks) / 2);
    l298n_driver_set_speed(handle,L298N_TARGET_MOTOR_B,max_speed);
    l298n_driver_print_motor_info(handle);

 



    return ESP_OK;
}


