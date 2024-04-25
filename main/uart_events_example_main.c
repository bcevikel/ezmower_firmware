/* UART Events Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"
#include"uart_packets.h"
#include"esp_packets_types.h"

static const char *TAG = "UART RXTX";


/*
*  UART Driver Configurations
*/
// The RX-TX BUFFER
#define UART_RX_TX_BUF_SIZE (1024)
#define USE_UART_NUM UART_NUM_0 // Which UART peripherhal to bind to 
// The buffer which will hold pattern detection events
#define UART_EVENT_BUFFER_LEN 20
#define UART_PATTERN_TIMEOUT_BAUDS 9

/*
* UART implementation specific configurations
*/
#define PACKET_HEADER_LEN 4
#define PACKET_HEADER_SIZE_BYTE_OFFSET 0
#define PACKET_HEADER_TYPE_BYTE_OFFSET 1
#define PACKET_TERM_CHAR '#'
#define PACKET_TERM_CHAR_COUNT 4 // How many consecutive characters shall terminate the packet ? 
#define PACKET_EXCPETED_PACKET_LEN 32
#define PACKET_EXCPECTED_BODY_LEN (PACKET_EXCPETED_PACKET_LEN - PACKET_HEADER_LEN - PACKET_TERM_CHAR_COUNT)

#define PACKET_PERIODIC_DISPATCH_DELAY_US 50000 // 50ms
/*
* Static Variables
*/
static QueueHandle_t uart0_event_queue;


static void send_state_packets_periodic(void* param){
    char packet_buf[PACKET_EXCPETED_PACKET_LEN];
    uart_packets_packet_conf_t packet_conf = {
        .header_len = PACKET_HEADER_LEN,
        .packet_size_offset = PACKET_HEADER_SIZE_BYTE_OFFSET,
        .termination_chr = PACKET_TERM_CHAR,
        .termination_len = PACKET_TERM_CHAR_COUNT
    };
    uart_packets_packet_wrapper_t wrapper = {0};
    uart_packets_static_create_empty_packet(packet_conf,&packet_buf,PACKET_EXCPETED_PACKET_LEN,&wrapper);

    // dispatch encoder counts
    uint8_t cur_type = ESP_PACKETS_FRAME_ENC_COUNTS;
    uart_packets_set_header_data_from_wrapper(&wrapper,PACKET_HEADER_TYPE_BYTE_OFFSET,&cur_type,sizeof(uint8_t));
    esp_packets_data_frame_encoder enc_frame = {
        .encoder_count_left = 313131,
        .encoder_count_right = 525252,
    };
    uart_packets_set_data_from_wrapper(&wrapper,0,&enc_frame,sizeof(esp_packets_data_frame_encoder));
    uart_write_bytes(USE_UART_NUM,packet_buf,PACKET_EXCPETED_PACKET_LEN);

    // dispatch imu rotational data
    cur_type = ESP_PACKETS_FRAME_IMU_ROT;
    uart_packets_set_header_data_from_wrapper(&wrapper,PACKET_HEADER_TYPE_BYTE_OFFSET,&cur_type,sizeof(uint8_t));
    esp_packets_data_frame_imu_rot imu_rot_frame = {
        .m_field_x = 1213124,
        .m_field_y = 5436346,
        .m_field_z = 19359359,
        .pitch = 150,
        .roll = 1.535,
        .yaw = 31.52,
    };
    uart_packets_set_data_from_wrapper(&wrapper,0,&imu_rot_frame,sizeof(esp_packets_data_frame_imu_rot));
    uart_write_bytes(USE_UART_NUM,packet_buf,PACKET_EXCPETED_PACKET_LEN);

    // dispatch imu trans data
    cur_type = ESP_PACKETS_FRAME_IMU_TRANS;
    uart_packets_set_header_data_from_wrapper(&wrapper,PACKET_HEADER_TYPE_BYTE_OFFSET,&cur_type,sizeof(uint8_t));
    esp_packets_data_frame_imu_trans imu_trans_frame = {
        .linear_acc_x = 567865,
        .linear_acc_y = 123456,
        .linear_acc_z = 16072001.31,
        .m_field_x = 55555,
        .m_field_y = 66666,
        .m_field_z = 7777,
    };
    uart_packets_set_data_from_wrapper(&wrapper,0,&imu_trans_frame,sizeof(esp_packets_data_frame_imu_trans));
    uart_write_bytes(USE_UART_NUM,packet_buf,PACKET_EXCPETED_PACKET_LEN);

    // dispatch imu temp data
    cur_type = ESP_PACKETS_FRAME_IMU_TEMP;
    uart_packets_set_header_data_from_wrapper(&wrapper,PACKET_HEADER_TYPE_BYTE_OFFSET,&cur_type,sizeof(uint8_t));
    esp_packets_data_frame_imu_temp imu_temp_frame = {
        .temperature = 27.3152
    };
    uart_packets_set_data_from_wrapper(&wrapper,0,&imu_temp_frame,sizeof(esp_packets_data_frame_imu_temp));
    uart_write_bytes(USE_UART_NUM,packet_buf,PACKET_EXCPETED_PACKET_LEN);

    // dispatch sonar distance data
    cur_type = ESP_PACKETS_FRAME_SONAR;
    uart_packets_set_header_data_from_wrapper(&wrapper,PACKET_HEADER_TYPE_BYTE_OFFSET,&cur_type,sizeof(uint8_t));
    esp_packets_data_frame_sonar_dist sonar_dist_frame = {
        .distance_center = 31,
        .distance_left = 15,
        .distance_right = 52, // these should be in meters, not cm 
    };
    uart_packets_set_data_from_wrapper(&wrapper,0,&sonar_dist_frame,sizeof(esp_packets_data_frame_sonar_dist));
    uart_write_bytes(USE_UART_NUM,packet_buf,PACKET_EXCPETED_PACKET_LEN);


}

static void uart_event_task(void *pvParameters)
{
    // set up the timer for periodic dispatch of states

    esp_timer_create_args_t periodic_timer_args = {
        .arg = NULL,
        .callback = send_state_packets_periodic,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "periodic_packet_dispatcher",
    };
    esp_timer_handle_t periodic_dispatch_timer;
    esp_timer_create(&periodic_timer_args,&periodic_dispatch_timer);
    esp_timer_start_periodic(periodic_dispatch_timer,PACKET_PERIODIC_DISPATCH_DELAY_US);

    esp_log_level_set(TAG, ESP_LOG_WARN);
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    //Install UART driver, and get the queue.
    // use double the size given because we are doing circular buffers
    uart_driver_install(USE_UART_NUM, UART_RX_TX_BUF_SIZE * 2, 0, UART_EVENT_BUFFER_LEN, &uart0_event_queue, 0);
    uart_param_config(USE_UART_NUM, &uart_config);

    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(USE_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    //Set uart pattern detect function.
    uart_enable_pattern_det_baud_intr(USE_UART_NUM, PACKET_TERM_CHAR, PACKET_TERM_CHAR_COUNT, UART_PATTERN_TIMEOUT_BAUDS, 0, 0);
    //Reset the pattern queue length to record at most 20 pattern positions.
    uart_pattern_queue_reset(USE_UART_NUM, 20);

    char packet_buffer[PACKET_EXCPETED_PACKET_LEN];
    uart_packets_packet_conf_t packet_conf = {
        .header_len = PACKET_HEADER_LEN,
        .packet_size_offset = PACKET_HEADER_SIZE_BYTE_OFFSET,
        .termination_chr = PACKET_TERM_CHAR,
        .termination_len = PACKET_TERM_CHAR_COUNT
    };
    uart_packets_packet_wrapper_t wrapper = {0};
    uart_packets_static_create_empty_packet(packet_conf,&packet_buffer,PACKET_EXCPETED_PACKET_LEN,&wrapper);

    uart_event_t event;
    size_t buffered_size;
    uint8_t* rx_buffer = (uint8_t*) malloc(UART_RX_TX_BUF_SIZE);
    while(true) {
        //Waiting for UART event.
        if(xQueueReceive(uart0_event_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
        // zero out the buffer
        bzero(rx_buffer, UART_RX_TX_BUF_SIZE);
        if(event.type == UART_PATTERN_DET) {
                uart_get_buffered_data_len(USE_UART_NUM, &buffered_size);
                int pos = uart_pattern_pop_pos(USE_UART_NUM);
                ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                if (pos == -1) {
                    // pattern queue full case 
                    uart_flush_input(USE_UART_NUM);
                    ESP_LOGE(TAG,"Packet recieve pattern queue overrun.");
                } else {
                    // found valid terminator, read packet + terminator 
                    uart_read_bytes(USE_UART_NUM, rx_buffer, pos + PACKET_TERM_CHAR_COUNT, 100 / portTICK_PERIOD_MS);
                    // skip the whole termination and get to last term + 1 
                    pos += PACKET_TERM_CHAR_COUNT;
                    // now go back packet len, which will place us at the first byte of the packet
                    pos -= PACKET_EXCPETED_PACKET_LEN;
                    // deserialize the packet 
                    esp_err_t err = uart_packets_deserialize_bytes_to_existing_wrapper(packet_conf,&rx_buffer[pos],&wrapper);
                    if(err == ESP_OK){
                        ESP_LOGI(TAG,"Valid packet recieved.");
                        uint8_t packet_type;
                        char packet_data[PACKET_EXCPECTED_BODY_LEN];
                        uart_packets_get_data_from_wrapper(&wrapper,0,&packet_data,PACKET_EXCPECTED_BODY_LEN);
                        uart_packets_get_header_data_from_wrapper(&wrapper,PACKET_HEADER_TYPE_BYTE_OFFSET,&packet_type,sizeof(uint8_t));
                        // dispatch the incoming packets
                        switch (packet_type)
                        {
                            case ESP_PACKETS_CMD_SET_PID:
                            {
                                break;
                            }
                            case ESP_PACKETS_CMD_SET_SPEED_BOTH:
                            {
                                break;
                            }
                            case ESP_PACKETS_CMD_SET_SPEED_LEFT:
                            {
                                break;
                            }
                            case ESP_PACKETS_CMD_SET_SPEED_RIGHT:
                            {
                                break;
                            }

                                                                                                                                                                                                                                            
                            default:
                            {   
                                break;
                            }
                        }

                    }
                
                }
            }
        }
    }
    free(rx_buffer);
    rx_buffer = NULL;
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGE("TEST","This is a test error !!");

    vTaskDelay(pdMS_TO_TICKS(500));
    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
    //vTaskDelay(portMAX_DELAY);

}


















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
