#ifndef UART_PACKETS_H
#define UART_PACKETS_H
#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"

typedef struct {
    // The length of the header of the packet in bytes
    uint8_t header_len;
    // The offset of the packet size varialbe of type uint8_t from the start of the packet
    uint8_t packet_size_offset;
    // The termination character of the packet
    char termination_chr;
    // The termination length, ie how many of the term. chars will be at the the end of the packet
    uint8_t termination_len;
} uart_packets_packet_conf_t;




typedef struct {
    void* packet_ptr;
    uart_packets_packet_conf_t wrapper_config;
    uint8_t packet_len;  
    bool is_dynamic_allocated;
} uart_packets_packet_wrapper_t;

esp_err_t uart_packets_serialize_wrapper_to_bytes(uart_packets_packet_wrapper_t* wrapper_ptr,void* dest);
uart_packets_packet_wrapper_t uart_packets_deserialize_bytes_to_new_wrapper(uart_packets_packet_conf_t config, void* src, size_t len);
esp_err_t uart_packets_deserialize_bytes_to_existing_wrapper(uart_packets_packet_conf_t config, void* src, uart_packets_packet_wrapper_t* wrapper_ptr);
uart_packets_packet_wrapper_t uart_packets_create_empty_packet(uart_packets_packet_conf_t config, size_t packet_len);
esp_err_t uart_packets_static_create_empty_packet(uart_packets_packet_conf_t config, void* packet_buf, uint8_t packet_len, uart_packets_packet_wrapper_t* wrapper_ptr);
esp_err_t uart_packets_destroy_packet(uart_packets_packet_wrapper_t* wrapper_ptr);

esp_err_t uart_packets_get_header_data_from_wrapper(uart_packets_packet_wrapper_t* wrapper_ptr, size_t offset, void* dest, size_t len);
esp_err_t uart_packets_get_data_from_wrapper(uart_packets_packet_wrapper_t* wrapper_ptr, size_t offset, void* dest, size_t len);

esp_err_t uart_packets_set_header_data_from_wrapper(uart_packets_packet_wrapper_t* wrapper_ptr, size_t offset, void* src, size_t len);
esp_err_t uart_packets_set_data_from_wrapper(uart_packets_packet_wrapper_t* wrapper_ptr, size_t offset, void* src, size_t len);



#endif