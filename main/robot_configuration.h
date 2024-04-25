#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H


#define MOTOR_DRIVER_GPIO_ENA 18
#define MOTOR_DRIVER_GPIO_ENB 19
#define MOTOR_DRIVER_GPIO_IN1 17
#define MOTOR_DRIVER_GPIO_IN2 5
#define MOTOR_DRIVER_GPIO_IN3 4
#define MOTOR_DRIVER_GPIO_IN4 16

#define MOTOR_DRIVER_PID_KP 1.3
#define MOTOR_DRIVER_PID_KI 1
#define MOTOR_DRIVER_PID_KD 0.4
#define MOTOR_DRIVER_CONTROL_LOOP_DELAY_MS 1000

#define HCSR_04_LEFT_ECHO_GPIO 36 
#define HCSR_04_LEFT_TRIG_GPIO 25 
#define HCSR_04_LEFT_ENUM 1

#define HCSR_04_CENTER_ECHO_GPIO 39
#define HCSR_04_CENTER_TRIG_GPIO 33 
#define HCSR_04_CENTER_ENUM 2

#define HCSR_04_RIGHT_ECHO_GPIO 34 
#define HCSR_04_RIGHT_TRIG_GPIO 32 
#define HCSR_04_RIGHT_ENUM 3

#define HCSR_04_MIN_MEAS_CM 0
#define HCSR_04_MAX_MEAS_CM 400

#define MOTOR_LEFT_ENCODER_GPIO 21
#define MOTOR_RIGHT_ENCODER_GPIO 22















#endif