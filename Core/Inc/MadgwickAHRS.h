/*
 * MadgwickAHRS.h
 *
 *  Created on: Apr 17, 2025
 *      Author: crist
 */

#ifndef INC_MADGWICKAHRS_H_
#define INC_MADGWICKAHRS_H_

//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================

#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

//----------------------------------------------------------------------------------------------------
// Variable declaration

/**
 * @brief Algorithm gain.
 *
 * This gain is used to control the convergence rate of the filter.
 */
extern volatile float beta;

/**
 * @brief Quaternion of sensor frame relative to auxiliary frame.
 *
 * These values represent the estimated orientation.
 */
extern volatile float q0, q1, q2, q3;

//----------------------------------------------------------------------------------------------------
// Function declarations

/**
 * @brief Madgwick's AHRS algorithm update using gyroscope, accelerometer, and magnetometer data.
 *
 * @param gx Gyroscope X axis (rad/s)
 * @param gy Gyroscope Y axis (rad/s)
 * @param gz Gyroscope Z axis (rad/s)
 * @param ax Accelerometer X axis (g)
 * @param ay Accelerometer Y axis (g)
 * @param az Accelerometer Z axis (g)
 * @param mx Magnetometer X axis (arbitrary units)
 * @param my Magnetometer Y axis (arbitrary units)
 * @param mz Magnetometer Z axis (arbitrary units)
 * @param sampleFreq Sampling frequency in Hz
 */
void MadgwickAHRSupdate(float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float mx, float my, float mz,
                        float sampleFreq);

/**
 * @brief Madgwick's AHRS algorithm update using only gyroscope and accelerometer data.
 *
 * @param gx Gyroscope X axis (rad/s)
 * @param gy Gyroscope Y axis (rad/s)
 * @param gz Gyroscope Z axis (rad/s)
 * @param ax Accelerometer X axis (g)
 * @param ay Accelerometer Y axis (g)
 * @param az Accelerometer Z axis (g)
 * @param sampleFreq Sampling frequency in Hz
 */
void MadgwickAHRSupdateIMU(float gx, float gy, float gz,
                           float ax, float ay, float az,
                           float sampleFreq);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================

#endif /* INC_MADGWICKAHRS_H_ */
