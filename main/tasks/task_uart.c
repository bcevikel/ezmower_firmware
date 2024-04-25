#include "task_uart.h"
#include"tasks/task_drive.h"
#include"tasks/task_sonar.h"
#include"tasks/task_imu.h"


static const char *TAG = "UART TASK";


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
    int enc_left;
    int enc_right;
    task_drive_get_both_encoder_counts(&enc_left,&enc_right);
    uint8_t cur_type = ESP_PACKETS_FRAME_ENC_COUNTS;
    uart_packets_set_header_data_from_wrapper(&wrapper,PACKET_HEADER_TYPE_BYTE_OFFSET,&cur_type,sizeof(uint8_t));
    esp_packets_data_frame_encoder enc_frame = {
        .encoder_count_left = enc_left,
        .encoder_count_right = enc_right,
    };
    uart_packets_set_data_from_wrapper(&wrapper,0,&enc_frame,sizeof(esp_packets_data_frame_encoder));
    uart_write_bytes(USE_UART_NUM,packet_buf,PACKET_EXCPETED_PACKET_LEN);

    esp_packets_data_frame_imu_rot imu_rot_frame = {0};
    esp_packets_data_frame_imu_trans imu_trans_frame = {0};
    esp_packets_data_frame_imu_temp imu_temp_frame = {0};
    float imu_tmp;
    task_imu_get_data(&imu_rot_frame,&imu_trans_frame,&imu_tmp);
    imu_temp_frame.temperature = imu_tmp;
    // dispatch imu rotational data
    
    cur_type = ESP_PACKETS_FRAME_IMU_ROT;
    uart_packets_set_header_data_from_wrapper(&wrapper,PACKET_HEADER_TYPE_BYTE_OFFSET,&cur_type,sizeof(uint8_t));
    uart_packets_set_data_from_wrapper(&wrapper,0,&imu_rot_frame,sizeof(esp_packets_data_frame_imu_rot));
    uart_write_bytes(USE_UART_NUM,packet_buf,PACKET_EXCPETED_PACKET_LEN);

    // dispatch imu trans data
    cur_type = ESP_PACKETS_FRAME_IMU_TRANS;
    uart_packets_set_header_data_from_wrapper(&wrapper,PACKET_HEADER_TYPE_BYTE_OFFSET,&cur_type,sizeof(uint8_t));
    uart_packets_set_data_from_wrapper(&wrapper,0,&imu_trans_frame,sizeof(esp_packets_data_frame_imu_trans));
    uart_write_bytes(USE_UART_NUM,packet_buf,PACKET_EXCPETED_PACKET_LEN);

    // dispatch imu temp data
    cur_type = ESP_PACKETS_FRAME_IMU_TEMP;
    uart_packets_set_header_data_from_wrapper(&wrapper,PACKET_HEADER_TYPE_BYTE_OFFSET,&cur_type,sizeof(uint8_t));

    uart_packets_set_data_from_wrapper(&wrapper,0,&imu_temp_frame,sizeof(esp_packets_data_frame_imu_temp));
    uart_write_bytes(USE_UART_NUM,packet_buf,PACKET_EXCPETED_PACKET_LEN);

    // dispatch sonar distance data
    float left,center,right;
    task_sonar_get_distances(&left,&center,&right);
    cur_type = ESP_PACKETS_FRAME_SONAR;
    uart_packets_set_header_data_from_wrapper(&wrapper,PACKET_HEADER_TYPE_BYTE_OFFSET,&cur_type,sizeof(uint8_t));
    esp_packets_data_frame_sonar_dist sonar_dist_frame = {
        .distance_center = center,
        .distance_left = left,
        .distance_right = right, // these should be in meters, not cm 
    };
    uart_packets_set_data_from_wrapper(&wrapper,0,&sonar_dist_frame,sizeof(esp_packets_data_frame_sonar_dist));
    uart_write_bytes(USE_UART_NUM,packet_buf,PACKET_EXCPETED_PACKET_LEN);


}
 
void task_uart_func(void *pvParameters)
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
                                esp_packets_data_frame_command_pid* pid_ptr;
                                pid_ptr = (esp_packets_data_frame_command_pid*) packet_data;
                                task_drive_set_pid_parameters(pid_ptr->kp,pid_ptr->ki,pid_ptr->kd);
                                break;
                            }
                            case ESP_PACKETS_CMD_SET_SPEED_BOTH:
                            {
                                esp_packets_data_frame_command_cps* cps_ptr;
                                cps_ptr = (esp_packets_data_frame_command_cps*) packet_data;
                                task_drive_set_both_motor_cps(cps_ptr->cps_left,cps_ptr->cps_right);
                                break;
                            }
                            case ESP_PACKETS_CMD_SET_SPEED_LEFT:
                            {
                                esp_packets_data_frame_command_cps* cps_ptr;
                                cps_ptr = (esp_packets_data_frame_command_cps*) packet_data;
                                task_drive_set_left_motor_cps(cps_ptr->cps_left);                               
                                break;
                            }
                            case ESP_PACKETS_CMD_SET_SPEED_RIGHT:
                            {
                                esp_packets_data_frame_command_cps* cps_ptr;
                                cps_ptr = (esp_packets_data_frame_command_cps*) packet_data;
                                task_drive_set_right_motor_cps(cps_ptr->cps_right);   
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
