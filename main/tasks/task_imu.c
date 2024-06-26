#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_task_wdt.h"

#include "driver/i2c.h"

#include "ahrs.h"
#include "mpu9250.h"
#include "calibrate.h"
#include "common.h"
#include"task_imu.h"


static float imu_sample_linear_acc_x;
static float imu_sample_linear_acc_y;
static float imu_sample_linear_acc_z;
static float imu_sample_mag_f_x;
static float imu_sample_mag_f_y;
static float imu_sample_mag_f_z;
static float imu_sample_pitch;
static float imu_sample_yaw;
static float imu_sample_roll;
static float imu_sample_temp;


void task_imu_get_data(esp_packets_data_frame_imu_rot *rot, esp_packets_data_frame_imu_trans *trans, float *temp)
{
    rot->m_field_x = imu_sample_mag_f_x;
    rot->m_field_y = imu_sample_mag_f_y;
    rot->m_field_z = imu_sample_mag_f_z;
    rot->pitch = imu_sample_pitch;
    rot->yaw = imu_sample_yaw;
    rot->roll = imu_sample_roll;

    trans->linear_acc_x = imu_sample_linear_acc_x;
    trans->linear_acc_y = imu_sample_linear_acc_y;
    trans->linear_acc_z = imu_sample_linear_acc_z;
    trans->m_field_x = imu_sample_mag_f_x;            
    trans->m_field_y = imu_sample_mag_f_y;            
    trans->m_field_z = imu_sample_mag_f_z;            
    *temp = imu_sample_temp;

}



static const char *TAG = "main";

#define I2C_MASTER_NUM I2C_NUM_0 /*!< I2C port number for master dev */

calibration_t cal = {
    .gyro_bias_offset = {.x = -3.542834, .y = -0.251411, .z = -0.036511},
    .accel_offset = {.x = 0.037232, .y = 0.028771, .z = -0.196840},
    .accel_scale_lo = {.x = 1.007649, .y = 1.015083, .z = 0.904143},
    .accel_scale_hi = {.x = -0.983117, .y = -0.991380, .z = -1.119431},
    .mag_offset = {.x = 64.968750, .y = -278.523438, .z = 109.421875},
    .mag_scale = {.x = 0.938980, .y = 0.974819, .z = 1.099888},
        };
/**
 * Transformation:
 *  - Rotate around Z axis 180 degrees
 *  - Rotate around X axis -90 degrees
 * @param  {object} s {x,y,z} sensor
 * @return {object}   {x,y,z} transformed
 */
static void transform_accel_gyro(vector_t *v)
{
  float x = v->x;
  float y = v->y;
  float z = v->z;

  v->x = -x;
  v->y = -z;
  v->z = -y;
}

/**
 * Transformation: to get magnetometer aligned
 * @param  {object} s {x,y,z} sensor
 * @return {object}   {x,y,z} transformed
 */
static void transform_mag(vector_t *v)
{
  float x = v->x;
  float y = v->y;
  float z = v->z;

  v->x = -y;
  v->y = z;
  v->z = -x;
}

void run_imu(void* pvParams)
{
  SemaphoreHandle_t barrier_sem = pvParams;

  i2c_mpu9250_init(&cal);
  ahrs_init(SAMPLE_FREQ_Hz, 0.8);
  if(barrier_sem){
    xSemaphoreGive(barrier_sem);
  }

  while (true)
  {
    vector_t va, vg, vm;

    // Get the Accelerometer, Gyroscope and Magnetometer values.
    ESP_ERROR_CHECK(get_accel_gyro_mag(&va, &vg, &vm));

    // Transform these values to the orientation of our device.
    transform_accel_gyro(&va);
    transform_accel_gyro(&vg);
    transform_mag(&vm);

    // Apply the AHRS algorithm
    ahrs_update(DEG2RAD(vg.x), DEG2RAD(vg.y), DEG2RAD(vg.z),
                va.x, va.y, va.z,
                vm.x, vm.y, vm.z);
    // write samples 
    imu_sample_linear_acc_x = va.x;
    imu_sample_linear_acc_y = va.y;
    imu_sample_linear_acc_z = va.z;
    imu_sample_mag_f_x = vg.x;
    imu_sample_mag_f_y = vg.y;
    imu_sample_mag_f_z = vg.z;
    // get temp,yaw,pitch,roll
    float temp,heading,pitch,roll;
    ESP_ERROR_CHECK(get_temperature_celsius(&temp));
    ahrs_get_euler_in_degrees(&heading, &pitch, &roll);
    imu_sample_pitch = pitch - 4.5;
    imu_sample_roll = roll - 90;
    imu_sample_yaw = heading -180;
    imu_sample_temp = temp;
    //   float temp;
    //   ESP_ERROR_CHECK(get_temperature_celsius(&temp));

    //   float heading, pitch, roll;
    //   ahrs_get_euler_in_degrees(&heading, &pitch, &roll);
    //   ESP_LOGI(TAG, "heading: %2.3f°, pitch: %2.3f°, roll: %2.3f°, Temp %2.3f°C", heading, pitch, roll, temp);
    vTaskDelay(1);
    pause();
  }
}

void task_imu_func(void *pvParams)
{
#ifdef CONFIG_CALIBRATION_MODE
  calibrate_gyro();
  alibrate_accel();
  calibrate_mag();
#else
  run_imu(pvParams);
#endif

  // Exit
  vTaskDelay(100 / portTICK_PERIOD_MS);
  i2c_driver_delete(I2C_MASTER_NUM);

  vTaskDelete(NULL);
}
