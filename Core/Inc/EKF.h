/**
 * @file EKF.h
 * @author CriIera
 * @brief EKF algorithm to combine gyroscope and accelerometer measurements
 * to find the orientation of the sensor.
 */


#ifndef INC_EKF_H_
#define INC_EKF_H_

/// Includes
#include "BMI088.h"


/// Function declarations

void EKF_CalculateGyroBias(BMI088* imu, int samples, float *ret_bias);

void EKF_Predict(float dt, float gyro_data[3]);

void EKF_Update(float accel_data[3]);

void ProcessIMU(float accel_data[3], float gyro_data[3]);

void EKF_FindAngles(float accel_data[3], float gyro_data[3], float* ret_angles);




#endif /* INC_EKF_H_ */


/// End of file
