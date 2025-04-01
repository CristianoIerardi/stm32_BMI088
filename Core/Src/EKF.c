
/**
 * @file EKF.c
 * @author CriIera
 * @brief EKF algorithm to combine gyroscope and accelerometer measurements
 * to find the orientation of the sensor.
 */

//#define USE_API


/// Includes
#include "EKF.h"
#include "main.h"
#include "EKF.h"
#include <stdint.h> 	// → It defines uint32_t e uint8_t
#include <math.h> 		// → Needed for atan2f and sqrtf
#include <stdio.h> 		// → Needed for sprintf and snprintf

/*----- Communication Libraries ------------------------*/
//#ifdef USE_SERIAL
	#include "Serial_Comm.h"
//#endif //USE_SERAIL
//#ifdef USE_API
	#include "API_Comm.h"
//#endif //USE_API

/*------------------------------------------------------*/
/*------------------------------------------------------*/
/*------------------------------------------------------*/
/// Constant variables
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


/*---------------------------------------------------------------*/
/// Global variables
float state[3] = {0};  // [roll, pitch, yaw]
float P[3][3] = {{0.1, 0, 0}, {0, 0.1, 0}, {0, 0, 0.1}};
float Q[3][3] = {{0.3, 0, 0}, {0, 0.3, 0}, {0, 0, 0.3}};
float R[2][2] = {{0.3, 0}, {0, 0.3}};
float gyro_bias[3] = {0};
uint32_t prev_time;


/**
 * @brief Function to calculate the bias of the measurements of the sensor BMI088
 * @param imu IMU algorithm structure
 * @param samples number of samples to calculate the mean average bias
 * @param ret_bias address (array) where to write the calculated bias values
 */
void EKF_CalculateGyroBias(BMI088* imu, int samples, float *ret_bias)
{
    float bias[3] = {0};

    for (int i = 0; i < samples; i++) {
        bias[0] += imu->gyr_rps[0];
        bias[1] += imu->gyr_rps[1];
        bias[2] += imu->gyr_rps[2];
    }
    for (int i = 0; i < 3; i++) {
        gyro_bias[i] = bias[i] / samples;
    }
}

/**
 * @brief Function to calculate the predict step of Extended Kalman Filter
 * @param dt delta time
 * @param gyro_data gyroscope measurements
 */
void EKF_Predict(float dt, float gyro_data[3])
{
    float gx = gyro_data[0]; //- gyro_bias[0];
    float gy = gyro_data[1] - gyro_bias[1];
    float gz = gyro_data[2] - gyro_bias[2];

    state[0] += gx * dt;
    state[1] += gy * dt;
    state[2] += gz * dt;
}


/**
 * @brief Function to calculate the update step of Extended Kalman Filter
 * @param accel_data accelerometer measurements
 */
void EKF_Update(float accel_data[3])
{
    float accel_roll = atan2f(accel_data[1], accel_data[2]);
    float accel_pitch = atan2f(-accel_data[0], sqrtf(accel_data[1] * accel_data[1] + accel_data[2] * accel_data[2]));

    float y[2] = {accel_roll - state[0], accel_pitch - state[1]};

    state[0] += y[0] * 0.1;
    state[1] += y[1] * 0.1;
}


/**
 * @brief Function to calculate the Euler angles of the sensor --> Roll, Pitch, Yaw
 * @param accel_data accelerometer measurements
 * @param gyro_data gyroscope measurements
 * @param ret_angles variable where to write the calculated angles values
 */
void EKF_FindAngles(float accel_data[3], float gyro_data[3], float* ret_angles)
{
    uint32_t curr_time = HAL_GetTick();
    float dt = (curr_time - prev_time) / 1000.0f;
    prev_time = curr_time;

    EKF_Predict(dt, gyro_data);
    EKF_Update(accel_data);

    ret_angles[0] = state[0] * (180.0 / M_PI);		// Roll
    ret_angles[1] = state[1] * (180.0 / M_PI);		// Pitch
    ret_angles[2] = state[2] * (180.0 / M_PI);		// Yaw

}






/// End of file











