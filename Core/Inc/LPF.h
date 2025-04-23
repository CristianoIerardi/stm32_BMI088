/*
 * LPF.h
 *
 *  Created on: Apr 15, 2025
 *      Author: crist
 */

#ifndef INC_LPF_H_
#define INC_LPF_H_

#include "ComputeOrientation.h"

#define N_FILT_SAMPLES		2

typedef struct{
	// LPF alpha coefficient
	float alpha_gyr;
	float alpha_acc;
	// HPF beta coefficient
	float beta_gyr;
	float beta_acc;
	// LPF angle filter coefficient
	float coeff_filt_ang;

	float filt_gyr_x[N_FILT_SAMPLES];		// filt_gyr_x[1] -> past filtered sample   //	filt_gyr_x[0] -> curr filtered sample
	float filt_gyr_y[N_FILT_SAMPLES];
	float filt_gyr_z[N_FILT_SAMPLES];
	float filt_acc_x[N_FILT_SAMPLES];
	float filt_acc_y[N_FILT_SAMPLES];
	float filt_acc_z[N_FILT_SAMPLES];

	float not_filt_gyr_x[N_FILT_SAMPLES];		// not_filt_gyr_x[1] -> past not filtered sample   //	filt_gyr_x[0] -> curr not filtered sample
	float not_filt_gyr_y[N_FILT_SAMPLES];
	float not_filt_gyr_z[N_FILT_SAMPLES];
	float not_filt_acc_x[N_FILT_SAMPLES];
	float not_filt_acc_y[N_FILT_SAMPLES];
	float not_filt_acc_z[N_FILT_SAMPLES];

	float filt_ang_x[N_FILT_SAMPLES];
	float filt_ang_y[N_FILT_SAMPLES];
	float filt_ang_z[N_FILT_SAMPLES];

} LPF_FILTER;





/// General filter functions
void Filter_Init(LPF_FILTER *filt, float f_LP_gyr, float f_LP_acc, float f_HP_gyr, float f_HP_acc, float f_LP_angles, float dt);

/// LPF
float LPF_CalculateAlpha(float f_cut, float dt);
void LPF_Init(LPF_FILTER *filt, float f_cut_gyr, float f_cut_acc, float f_cut_angles, float dt);
void LPF_SetAlpha(LPF_FILTER *filt, float alpha_gyr, float alpha_acc, float coeff_filt_angl);
float LPF_Update_Single(LPF_FILTER *filt, float old_data, float data, float alpha);
LPF_FILTER LPF_GyrAcc_Update_All(LPF_FILTER *filt, Vector3 data_gyr, Vector3 data_acc);
LPF_FILTER LPF_Angles_Update_All(LPF_FILTER *filt, float *angl);

/// HPF
float HPF_CalculateBeta(float f_cut, float dt);
void HPF_Init(LPF_FILTER *filt, float f_cut_gyr, float f_cut_acc, float dt);
void HPF_SetBeta(LPF_FILTER *filt, float beta_gyr, float beta_acc);
float HPF_Update_Single(LPF_FILTER *filt, float *filt_data, float *not_filtered_data, float beta);
 LPF_FILTER HPF_Update_All(LPF_FILTER *filt, Vector3 data_gyr, Vector3 data_acc);








#endif /* INC_LPF_H_ */
