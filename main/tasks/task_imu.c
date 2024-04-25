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


void task_imu_func(void *pvParams)
{
    SemaphoreHandle_t barrier_sem = pvParams;
    // do stuff...
    xSemaphoreGive(barrier_sem);

    vTaskDelay(portMAX_DELAY);
}

