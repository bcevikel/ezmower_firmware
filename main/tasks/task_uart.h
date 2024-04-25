#ifndef TASK_UART_H
#define TASK_UART_H
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include"uart_packets.h"
#include"esp_packets_types.h"


void task_uart_func(void* pvParams);



#endif