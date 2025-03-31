/*
 * EKF.h
 *
 *  Created on: Mar 28, 2025
 *      Author: crist
 */

#ifndef INC_EKF_H_
#define INC_EKF_H_

#include "BMI088.h"



void EKF_CalculateGyroBias(BMI088* imu, int samples);

void EKF_Predict(float dt, float gyro_data[3]);

void EKF_Update(float accel_data[3]);

void ProcessIMU(float accel_data[3], float gyro_data[3]);




#endif /* INC_EKF_H_ */
