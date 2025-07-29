/*
 * LPF.h
 *
 *  Created on: Apr 15, 2025
 *      Author: crist
 */

#ifndef INC_FILTERS_H_
#define INC_FILTERS_H_

#include "ComputeOrientation.h"
#include "BMI088.h"

#define N_FILT_SAMPLES		2  ///< Number of samples used in the filtering history (current + previous)

/**
 * @struct LPF_FILTER
 * @brief Structure containing all data and coefficients needed for low-pass and high-pass filtering.
 *
 * This filter structure is used for:
 * - LPF on gyroscope, accelerometer and angle data
 * - HPF on gyroscope and accelerometer
 *
 * It maintains both filtered and unfiltered samples for proper high-pass filtering, as well as angle filtering history.
 */
typedef struct{
	// LPF alpha coefficient
	float alpha_gyr;     ///< LPF coefficient for gyroscope
	float alpha_acc;     ///< LPF coefficient for accelerometer
	// HPF beta coefficient
	float beta_gyr;      ///< HPF coefficient for gyroscope
	float beta_acc;      ///< HPF coefficient for accelerometer
	// LPF angle filter coefficient
	float coeff_filt_ang; ///< LPF coefficient for angles

	// Filtered gyroscope data history: [0] = current, [1] = previous
	float filt_gyr_x[N_FILT_SAMPLES];
	float filt_gyr_y[N_FILT_SAMPLES];
	float filt_gyr_z[N_FILT_SAMPLES];

	// Filtered accelerometer data history
	float filt_acc_x[N_FILT_SAMPLES];
	float filt_acc_y[N_FILT_SAMPLES];
	float filt_acc_z[N_FILT_SAMPLES];

	// Unfiltered gyroscope data history
	float not_filt_gyr_x[N_FILT_SAMPLES];
	float not_filt_gyr_y[N_FILT_SAMPLES];
	float not_filt_gyr_z[N_FILT_SAMPLES];

	// Unfiltered accelerometer data history
	float not_filt_acc_x[N_FILT_SAMPLES];
	float not_filt_acc_y[N_FILT_SAMPLES];
	float not_filt_acc_z[N_FILT_SAMPLES];

	// Filtered orientation angles history
	float filt_ang_x[N_FILT_SAMPLES];
	float filt_ang_y[N_FILT_SAMPLES];
	float filt_ang_z[N_FILT_SAMPLES];

} LPF_FILTER;


/// @brief Initialize the entire filter structure with LPF and HPF cutoff frequencies.
/// @param filt Pointer to the LPF_FILTER struct
/// @param f_LP_gyr LPF cutoff frequency for gyroscope
/// @param f_LP_acc LPF cutoff frequency for accelerometer
/// @param f_HP_gyr HPF cutoff frequency for gyroscope
/// @param f_HP_acc HPF cutoff frequency for accelerometer
/// @param f_LP_angles LPF cutoff frequency for angles
/// @param dt Sample period in seconds
void Filter_Init(LPF_FILTER *filt, float f_LP_gyr, float f_LP_acc, float f_HP_gyr, float f_HP_acc, float f_LP_angles, float dt);

/// @brief Compute LPF alpha coefficient from cutoff frequency and sample period.
/// @param f_cut Cutoff frequency in Hz
/// @param dt Sampling period in seconds
/// @return Alpha coefficient for LPF
float LPF_CalculateAlpha(float f_cut, float dt);

/// @brief Initialize LPF coefficients.
/// @param filt Pointer to LPF_FILTER
/// @param f_cut_gyr Gyroscope cutoff frequency
/// @param f_cut_acc Accelerometer cutoff frequency
/// @param f_cut_angles Angle cutoff frequency
/// @param dt Sample period
void LPF_Init(LPF_FILTER *filt, float f_cut_gyr, float f_cut_acc, float f_cut_angles, float dt);

/// @brief Manually set LPF alpha coefficients.
/// @param filt Pointer to LPF_FILTER
/// @param alpha_gyr LPF coefficient for gyroscope
/// @param alpha_acc LPF coefficient for accelerometer
/// @param coeff_filt_angl LPF coefficient for angle filtering
void LPF_SetAlpha(LPF_FILTER *filt, float alpha_gyr, float alpha_acc, float coeff_filt_angl);

/// @brief Perform LPF on a single value.
/// @param filt Pointer to LPF_FILTER
/// @param old_data Previous filtered value
/// @param data Current unfiltered value
/// @param alpha LPF coefficient
/// @return New filtered value
float LPF_Update_Single(LPF_FILTER *filt, float old_data, float data, float alpha);

/// @brief Update all gyroscope and accelerometer values using LPF.
/// @param filt Pointer to LPF_FILTER
/// @param gyr Pointer to raw gyroscope values [3]
/// @param acc Pointer to raw accelerometer values [3]
/// @return Updated filter structure
LPF_FILTER LPF_GyrAcc_Update_All(LPF_FILTER *filt, float *gyr, float *acc);

/// @brief Update all angle values using LPF.
/// @param filt Pointer to LPF_FILTER
/// @param angl Pointer to raw angles [3]
/// @return Updated filter structure
LPF_FILTER LPF_Angles_Update_All(LPF_FILTER *filt, float *angl);

/// @brief Compute HPF beta coefficient from cutoff frequency and sample period.
/// @param f_cut Cutoff frequency in Hz
/// @param dt Sampling period in seconds
/// @return Beta coefficient for HPF
float HPF_CalculateBeta(float f_cut, float dt);

/// @brief Initialize HPF coefficients.
/// @param filt Pointer to LPF_FILTER
/// @param f_cut_gyr Gyroscope HPF cutoff
/// @param f_cut_acc Accelerometer HPF cutoff
/// @param dt Sampling period
void HPF_Init(LPF_FILTER *filt, float f_cut_gyr, float f_cut_acc, float dt);

/// @brief Manually set HPF beta coefficients.
/// @param filt Pointer to LPF_FILTER
/// @param beta_gyr HPF coefficient for gyroscope
/// @param beta_acc HPF coefficient for accelerometer
void HPF_SetBeta(LPF_FILTER *filt, float beta_gyr, float beta_acc);

/// @brief Perform HPF on a single signal component.
/// @param filt Pointer to LPF_FILTER
/// @param filt_data Filtered signal history (2 samples)
/// @param not_filtered_data Raw signal history (2 samples)
/// @param beta HPF coefficient
/// @return New filtered value
float HPF_Update_Single(LPF_FILTER *filt, float *filt_data, float *not_filtered_data, float beta);

/// @brief Apply HPF to all gyroscope and accelerometer axes.
/// @param filt Pointer to LPF_FILTER
/// @param data_gyr Raw gyroscope vector
/// @param data_acc Raw accelerometer vector
/// @return Updated filter structure
LPF_FILTER HPF_Update_All(LPF_FILTER *filt, Vector3 data_gyr, Vector3 data_acc);

#endif /* INC_FILTERS_H_ */
