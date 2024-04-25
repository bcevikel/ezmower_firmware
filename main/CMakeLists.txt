idf_component_register(SRCS "sonar_sensor.c" "main.c" "encoders.c" "l298n_driver.c" "sonar_sensor.c" "uart_packets.c" "tasks/task_drive.c" "tasks/task_sonar.c"  "tasks/task_imu.c" "tasks/task_uart.c"
                    INCLUDE_DIRS "." "tasks/"
                    REQUIRES driver
                    REQUIRES esp_timer)