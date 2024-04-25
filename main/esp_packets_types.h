#ifndef ESP_PACKETS_TYPE_H
#define ESP_PACKETS_TYPE_H
#include <stdio.h>


    enum esp_packets_message_types_t {
        ESP_PACKETS_CMD_ESP_CMD = 0,
        ESP_PACKETS_FRAME_ENC_COUNTS = 1,
        ESP_PACKETS_FRAME_IMU_TRANS = 2,
        ESP_PACKETS_FRAME_IMU_ROT = 3,
        ESP_PACKETS_FRAME_IMU_TEMP = 4,
        ESP_PACKETS_FRAME_SONAR = 5,
        ESP_PACKETS_CMD_SET_SPEED_LEFT = 6,
        ESP_PACKETS_CMD_SET_SPEED_RIGHT = 7,
        ESP_PACKETS_CMD_SET_SPEED_BOTH = 8,
       ESP_PACKETS_CMD_SET_PID = 9,
    };


    // Generic data packet that is sent over serial.
    typedef struct __attribute__((packed)) {
        uint8_t message_length;
        uint8_t message_type;
        uint8_t reserved1;
        uint8_t reserved2; // First 4 bytes are always reserved for configuration
        char data[24];
        char termination[4]; 
    } esp_packets_data_packet_generic;


    // Data frame that is used for transmitting encoder count information.
    typedef struct __attribute__((packed)) {
       int encoder_count_left;
       int encoder_count_right;
    } esp_packets_data_frame_encoder;

    // Data frame that is used for transmitting encoder PID values.
    typedef struct __attribute__((packed)) {
        float kp;
        float ki;
        float kd;
    } esp_packets_data_frame_command_pid;

    // Data frame that is used for set cps values.
    typedef struct __attribute__((packed)) {
        float cps_left;
        float cps_right;
    } esp_packets_data_frame_command_cps;

    // Data frame that is used for transmitting imu rotational information.
    typedef struct __attribute__((packed)) {
       float pitch;
       float yaw;
       float roll;
       float m_field_x;
       float m_field_y;
       float m_field_z;   
    } esp_packets_data_frame_imu_rot;

    // Data frame that is used for transmitting imu translational information.
    typedef struct __attribute__((packed)) {
       float linear_acc_x;
       float linear_acc_y;
       float linear_acc_z;
       float m_field_x;
       float m_field_y;
       float m_field_z;   
    } esp_packets_data_frame_imu_trans;

    // Data frame that is used for transmitting imu translational information.
    typedef struct __attribute__((packed)) {
       float temperature;
    } esp_packets_data_frame_imu_temp;

    // Data frame that is used for transmitting sonar distance information.
    typedef struct __attribute__((packed)) {
       float distance_left;
       float distance_center;
       float distance_right;
    } esp_packets_data_frame_sonar_dist;











#endif