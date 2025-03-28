/*
 * EKF.h
 *
 *  Created on: Mar 28, 2025
 *      Author: crist
 */

#ifndef INC_EKF_H_
#define INC_EKF_H_


#include "main.h"
#include "EKF.h"
#include <stdint.h> 	// → It defines uint32_t e uint8_t
#include <math.h> 		// → Needed for atan2f and sqrtf
#include <stdio.h> 		// → Needed for sprintf and snprintf
#include "BMI088.h"

void CalculateGyroBias(BMI088* imu, int samples);

void EKF_Predict(float dt, float gyro_data[3]);

void EKF_Update(float accel_data[3]);

void ProcessIMU(float accel_data[3], float gyro_data[3]);




#endif /* INC_EKF_H_ */
