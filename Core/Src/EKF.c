/*
 * EKF.c
 *
 *  Created on: Mar 28, 2025
 *      Author: crist
 */

#include "EKF.h"



#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


/*---------------------------------------------------------------*/

float state[3] = {0};  // [roll, pitch, yaw]
float P[3][3] = {{0.1, 0, 0}, {0, 0.1, 0}, {0, 0, 0.1}};
float Q[3][3] = {{0.3, 0, 0}, {0, 0.3, 0}, {0, 0, 0.3}};
float R[2][2] = {{0.3, 0}, {0, 0.3}};
float gyro_bias[3] = {0};
uint32_t prev_time;




void CalculateGyroBias(BMI088* imu, int samples) {
    float bias[3] = {0};

    for (int i = 0; i < samples; i++) {
        bias[0] += imu->gyr_rps[0];
        bias[1] += imu->gyr_rps[1];
        bias[2] += imu->gyr_rps[2];
    }
    for (int i = 0; i < 3; i++) {
        gyro_bias[i] = bias[i] / samples;
    }

    /*char buffer[128];
    sprintf(buffer, "BIAS: X=%.2f°, Y=%.2f°, Z=%.2f°\r\n", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
    while(CDC_Transmit_FS((uint8_t *) buffer, strlen(buffer)) == HAL_BUSY);
	*/
}


void EKF_Predict(float dt, float gyro_data[3]) {
    float gx = gyro_data[0]; //- gyro_bias[0];
    float gy = gyro_data[1] - gyro_bias[1];
    float gz = gyro_data[2] - gyro_bias[2];

    state[0] += gx * dt;
    state[1] += gy * dt;
    state[2] += gz * dt;
}



void EKF_Update(float accel_data[3])
{
    float accel_roll = atan2f(accel_data[1], accel_data[2]);
    float accel_pitch = atan2f(-accel_data[0], sqrtf(accel_data[1] * accel_data[1] + accel_data[2] * accel_data[2]));

    float y[2] = {accel_roll - state[0], accel_pitch - state[1]};

    state[0] += y[0] * 0.1;
    state[1] += y[1] * 0.1;
}


void ProcessIMU(float accel_data[3], float gyro_data[3])
{
    uint32_t curr_time = HAL_GetTick();
    float dt = (curr_time - prev_time) / 1000.0f;
    prev_time = curr_time;

    EKF_Predict(dt, gyro_data);
    EKF_Update(accel_data);

    char buffer[128];

    sprintf(buffer, "E,%lu,%.4f,%.4f,%.4f\r\n", HAL_GetTick(), state[0] * (180.0 / M_PI), state[1] * (180.0 / M_PI), state[2] * (180.0 / M_PI));
    while(CDC_Transmit_FS((uint8_t *) buffer, strlen(buffer)) == HAL_BUSY);
    /*
    sprintf(buffer, "Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°\r\n", state[0] * (180.0 / M_PI), state[1] * (180.0 / M_PI), state[2] * (180.0 / M_PI));
    while(CDC_Transmit_FS((uint8_t *) buffer, strlen(buffer)) == HAL_BUSY);
	*/
}


















