#include"task_drive.h"
#include"robot_configuration.h"
#include "esp_timer.h"
#include <math.h>

#define MCPWM_TICK_MAX 800
static float target_cps_motor_left;
static float target_cps_motor_right;
static l298n_driver_handle_t motor_driver_handle;
static sphase_encoder_handle_t sphase_encoder_left;
static sphase_encoder_handle_t sphase_encoder_right;
static pid_ctrl_block_handle_t pid_ctrl_motor_left;
static pid_ctrl_block_handle_t pid_ctrl_motor_right;

static const char* TAG = "DRIVE TASK";

void task_drive_set_left_motor_cps(float left)
{
    task_drive_set_both_motor_cps(left,target_cps_motor_right);
}

void task_drive_set_right_motor_cps(float right)
{
    task_drive_set_both_motor_cps(target_cps_motor_left,right);
}


void task_drive_set_both_motor_cps(float left, float right)
{

    if(left == 0 && right == 0){
        l298n_driver_set_direction(motor_driver_handle,L298N_TARGET_BOTH_MOTORS,L298N_ACTION_BRAKE);
        sphase_encoder_change_direction(sphase_encoder_left,SPHASE_ENCODER_INC_ON_RISING_EDGE_INC_ON_FALLING_EDGE);
        sphase_encoder_change_direction(sphase_encoder_right,SPHASE_ENCODER_INC_ON_RISING_EDGE_INC_ON_FALLING_EDGE);
    }
    // case : both forward
    else if( left > 0 && right > 0){
        l298n_driver_set_direction(motor_driver_handle,L298N_TARGET_BOTH_MOTORS,L298N_ACTION_FORWARD);
        sphase_encoder_change_direction(sphase_encoder_left,SPHASE_ENCODER_INC_ON_RISING_EDGE_INC_ON_FALLING_EDGE);
        sphase_encoder_change_direction(sphase_encoder_right,SPHASE_ENCODER_INC_ON_RISING_EDGE_INC_ON_FALLING_EDGE);
    }
    // case : both backwards
    else if(left < 0 &&  right < 0){
        l298n_driver_set_direction(motor_driver_handle,L298N_TARGET_BOTH_MOTORS,L298N_ACTION_BACKWARD);
        sphase_encoder_change_direction(sphase_encoder_left,SPHASE_ENCODER_DEC_ON_RISING_EDGE_DEC_ON_FALLING_EDGE);
        sphase_encoder_change_direction(sphase_encoder_right,SPHASE_ENCODER_DEC_ON_RISING_EDGE_DEC_ON_FALLING_EDGE); 
    }
    // left backwards right forwards
    else if(left < 0 && right > 0){
        l298n_driver_set_direction(motor_driver_handle,L298N_TARGET_MOTOR_A,L298N_ACTION_BACKWARD);
        l298n_driver_set_direction(motor_driver_handle,L298N_TARGET_MOTOR_B,L298N_ACTION_FORWARD);
        sphase_encoder_change_direction(sphase_encoder_left,SPHASE_ENCODER_DEC_ON_RISING_EDGE_DEC_ON_FALLING_EDGE);
        sphase_encoder_change_direction(sphase_encoder_right,SPHASE_ENCODER_INC_ON_RISING_EDGE_INC_ON_FALLING_EDGE);
    }
    // left forwards right backwards
    else{
        l298n_driver_set_direction(motor_driver_handle,L298N_TARGET_MOTOR_A,L298N_ACTION_FORWARD);
        l298n_driver_set_direction(motor_driver_handle,L298N_TARGET_MOTOR_B,L298N_ACTION_BACKWARD);
        sphase_encoder_change_direction(sphase_encoder_left,SPHASE_ENCODER_INC_ON_RISING_EDGE_INC_ON_FALLING_EDGE);
        sphase_encoder_change_direction(sphase_encoder_right,SPHASE_ENCODER_DEC_ON_RISING_EDGE_DEC_ON_FALLING_EDGE);
    }
    target_cps_motor_left = fabsf(left);
    target_cps_motor_right = fabsf(right);

}

void task_drive_set_pid_parameters(float p, float i, float d)
{
        pid_ctrl_parameter_t pid_runtime_param = {
        .kp = p,
        .ki = i,
        .kd = d,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output   = 1,
        .min_output   = 0,
        .max_integral = 100,
        .min_integral = -100,
    };

    pid_update_parameters(pid_ctrl_motor_left,&pid_runtime_param);
    pid_update_parameters(pid_ctrl_motor_right,&pid_runtime_param);
    
}

void task_drive_get_left_encoder_counts(int *left_enc)
{
    int64_t left_cnt;
    sphase_encoder_get_count_from_accumulator(sphase_encoder_left,&left_cnt);
    *left_enc = left_cnt;        
}

void task_drive_get_right_encoder_counts(int *right_enc)
{
    int64_t right_cnt;
    sphase_encoder_get_count_from_accumulator(sphase_encoder_right,&right_cnt);
    *right_enc = right_cnt;
}

void task_drive_get_both_encoder_counts(int *left_enc, int *right_enc)
{
    int64_t left_cnt;
    sphase_encoder_get_count_from_accumulator(sphase_encoder_left,&left_cnt);
    int64_t right_cnt;
    sphase_encoder_get_count_from_accumulator(sphase_encoder_right,&right_cnt);
    *left_enc = left_cnt;
    *right_enc = right_cnt;
}


void task_drive_motors_func(void *pvParameters)
{
    //esp_log_level_set(TAG,ESP_LOG_ERROR);
    
    // init static vars
    target_cps_motor_left = 0;
    target_cps_motor_right = 0;

    // prepare the barrier semaphore to release it later
    SemaphoreHandle_t barrier_sem = (SemaphoreHandle_t) pvParameters;

    // prepare the l298n driver configuration struct
    l298n_driver_config_t motor_drv_conf = {
        .mpcwm_timer_resolution_hz = 10000000, // 10MHz, 0.1us per tick 
        .mcpwm_pwm_freq_hz = 12500, // 12.5kHz,
        .mcpwm_group_id = 0,
        .mcpwm_min_pwm_ticks = 430,
        .gpio_ena_num = MOTOR_DRIVER_GPIO_ENA,
        .gpio_enb_num = MOTOR_DRIVER_GPIO_ENB,
        .gpio_in1_num = MOTOR_DRIVER_GPIO_IN1,
        .gpio_in2_num = MOTOR_DRIVER_GPIO_IN2,
        .gpio_in3_num = MOTOR_DRIVER_GPIO_IN3,
        .gpio_in4_num = MOTOR_DRIVER_GPIO_IN4,
    };
    motor_driver_handle = NULL;
    // create the driver object and start the hardware 
    ESP_ERROR_CHECK(l298n_driver_new(&motor_drv_conf,&motor_driver_handle));
    ESP_ERROR_CHECK(l298n_driver_enable(motor_driver_handle));


    // set up single phase encoders that will be used later 
    // set up left 
    sphase_encoder_left = NULL;
    sphase_encoder_config_t sphase_conf_left = {
        .edge_gpio_num = MOTOR_LEFT_ENCODER_GPIO,
        .high_limit = 1000,
        .low_limit = -1000,
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(sphase_encoder_new(&sphase_conf_left,&sphase_encoder_left));
    ESP_ERROR_CHECK(sphase_encoder_change_direction(sphase_encoder_left,SPHASE_ENCODER_INC_ON_RISING_EDGE_INC_ON_FALLING_EDGE));
    ESP_ERROR_CHECK(sphase_encoder_enable(sphase_encoder_left));
    // set up right 
    sphase_encoder_right = NULL;
    sphase_encoder_config_t sphase_conf_right = {
        .edge_gpio_num = MOTOR_RIGHT_ENCODER_GPIO,
        .high_limit = 1000,
        .low_limit = -1000,
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(sphase_encoder_new(&sphase_conf_right,&sphase_encoder_right));
    ESP_ERROR_CHECK(sphase_encoder_change_direction(sphase_encoder_right,SPHASE_ENCODER_INC_ON_RISING_EDGE_INC_ON_FALLING_EDGE));
    ESP_ERROR_CHECK(sphase_encoder_enable(sphase_encoder_right));    
    // set up the pid controller 

    pid_ctrl_parameter_t pid_runtime_param = {
        .kp = 0.3,
        .ki = 0.6,
        .kd = 0.05,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output   = 1,
        .min_output   = 0,
        .max_integral = 100,
        .min_integral = -100,
    };

    pid_ctrl_motor_left = NULL;
    pid_ctrl_motor_right = NULL;

    pid_ctrl_config_t pid_config = {
        .init_param = pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &pid_ctrl_motor_left));
    ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &pid_ctrl_motor_right));
    
    // actually start the hw 
    l298n_driver_start_motors(motor_driver_handle);
    sphase_encoder_start(sphase_encoder_left);
    sphase_encoder_start(sphase_encoder_right);
    // signal main thread to continue execution
    if(barrier_sem){
        xSemaphoreGive(barrier_sem);
    }
    

    int64_t last_observed_enc_count_left = 0;
    int64_t last_observed_enc_count_right = 0;   
    while (true)
    {
        // get raw encoder counts
        int64_t current_enc_count_left;
        int64_t current_enc_count_right;
        sphase_encoder_get_count_from_accumulator(sphase_encoder_left,&current_enc_count_left);
        sphase_encoder_get_count_from_accumulator(sphase_encoder_right,&current_enc_count_right);

        int64_t enc_count_left_diff =  current_enc_count_left - last_observed_enc_count_left;
        int64_t enc_count_right_diff =  current_enc_count_right - last_observed_enc_count_right;
        // update last observed variables
        last_observed_enc_count_left = current_enc_count_left;
        last_observed_enc_count_right = current_enc_count_right;

        float elapsed_time_s = 1;        
        // calculate the encoder count per second 
        float left_enc_cps = enc_count_left_diff / elapsed_time_s;
        float right_enc_cps = enc_count_right_diff / elapsed_time_s;
        // take absolute value as we may be moving backwards 
        left_enc_cps = fabsf(left_enc_cps);
        right_enc_cps = fabsf(right_enc_cps);
        // calculate the error, also normalize it
        float cps_motor_left_error = (target_cps_motor_left) - left_enc_cps;
        float cps_motor_right_error = (target_cps_motor_right) - right_enc_cps; 
        float cps_motor_left_error_normalized = cps_motor_left_error / 30;
        float cps_motor_right_error_normalized = cps_motor_right_error / 30;

        float new_left_throttle_normalized;
        float new_right_throttle_normalized;
        pid_compute(pid_ctrl_motor_left,cps_motor_left_error_normalized , &new_left_throttle_normalized);
        pid_compute(pid_ctrl_motor_right,cps_motor_right_error_normalized, &new_right_throttle_normalized);
        // un-normalize (map) the output
        float new_left_throttle= 430 + new_left_throttle_normalized * 370;
        float new_right_throttle = 430 + new_right_throttle_normalized * 370;

        // write this to the controllers
        l298n_driver_set_speed(motor_driver_handle,L298N_TARGET_MOTOR_A,new_left_throttle);
        l298n_driver_set_speed(motor_driver_handle,L298N_TARGET_MOTOR_B,new_right_throttle);

        ESP_LOGD(TAG,"\nCurrent left cps: %f , current right cps: %0.5f ",left_enc_cps,right_enc_cps);    
        ESP_LOGD(TAG," left enc diff: %lld, , right enc diff : %lld ",enc_count_left_diff,enc_count_right_diff);    
        ESP_LOGD(TAG,"Current left power: %0.f , current right power : %0.1f",new_left_throttle,new_right_throttle);
        ESP_LOGD(TAG,"current left error: %f , current right error: %f\n",cps_motor_left_error,cps_motor_right_error);

        vTaskDelay(pdMS_TO_TICKS(MOTOR_DRIVER_CONTROL_LOOP_DELAY_MS));
    
    }
    // exit
    sphase_encoder_del(sphase_encoder_left);
    sphase_encoder_del(sphase_encoder_right);
    l298n_driver_del(motor_driver_handle);
    pid_del_control_block(pid_ctrl_motor_left);
    pid_del_control_block(pid_ctrl_motor_right);
    vTaskDelete(NULL);
}
