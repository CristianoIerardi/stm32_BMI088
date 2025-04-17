/*
 * LPF.c
 *
 *  Created on: Apr 15, 2025
 *      Author: crist
 */
#include "LPF.h"
#include <math.h>

/// ######################################################################################################
/// ##### FILTER SECTION #################################################################################
/// ######################################################################################################

void Filter_Init(LPF_FILTER *filt, float f_LP_gyr, float f_LP_acc, float f_HP_gyr, float f_HP_acc, float dt)
{

	LPF_Init(filt, f_LP_gyr, f_LP_acc, dt);
	HPF_Init(filt, f_HP_gyr, f_HP_acc, dt);

	for(int i=0; i<N_FILT_SAMPLES; i++)
	{
		filt->filt_gyr_x[i] = 0.0f;					// filt_gyr_x[0] -> past filtered sample   //	filt_gyr_x[1] -> curr filtered sample
		filt->filt_gyr_y[i] = 0.0f;					// filt_gyr_x[0] ==> y(n)
		filt->filt_gyr_z[i] = 0.0f;					// filt_gyr_x[1] ==> y(n-1)
		filt->filt_acc_x[i] = 0.0f;
		filt->filt_acc_y[i] = 0.0f;
		filt->filt_acc_z[i] = 0.0f;
	}

	for(int i=0; i<N_FILT_SAMPLES; i++)
	{
		filt->not_filt_gyr_x[i] = 0.0f;					// not_filt_gyr_x[0] -> past not filtered sample   //	not_filt_gyr_x[1] -> curr not filtered sample
		filt->not_filt_gyr_y[i] = 0.0f;					// not_filt_gyr_x[0] ==> x(n)
		filt->not_filt_gyr_z[i] = 0.0f;					// not_filt_gyr_x[1] ==> x(n-1)
		filt->not_filt_acc_x[i] = 0.0f;
		filt->not_filt_acc_y[i] = 0.0f;
		filt->not_filt_acc_z[i] = 0.0f;
	}
}

/// ######################################################################################################
/// ##### LOW PASS FILTER SECTION ########################################################################
/// ######################################################################################################
float LPF_CalculateAlpha(float f_cut, float dt)
{
	float tau = 1.0f / (2.0f * M_PI * f_cut);
	return dt / (tau + dt);
}



void LPF_Init(LPF_FILTER *filt, float f_cut_gyr, float f_cut_acc, float dt)
{
	// aplha for LPF settings
	float alpha_gyr = LPF_CalculateAlpha(f_cut_gyr, dt);
	float alpha_acc = LPF_CalculateAlpha(f_cut_acc, dt);
	LPF_SetAlpha(filt, alpha_gyr, alpha_acc);

}

void LPF_SetAlpha(LPF_FILTER *filt, float alpha_gyr, float alpha_acc)
{
	if(alpha_gyr > 1.0f)
	{
		alpha_gyr = 1.0f;
	}
	else if(alpha_gyr < 0.0f)
	{
		alpha_gyr = 0.0f;
	}

	if(alpha_acc > 1.0f)
	{
		alpha_acc = 1.0f;
	}
	else if(alpha_acc < 0.0f)
	{
		alpha_acc = 0.0f;
	}

	filt->alpha_gyr = alpha_gyr;
	filt->alpha_acc = alpha_acc;
}



/// ------ LOW PF ILTER, 1° ORDER ------------------------------------------------------------------------------------
float LPF_Update_Single(LPF_FILTER *filt, float old_data, float data, float alpha)
{
	 return ( (alpha * data) + ((1-alpha) * old_data) );
}


LPF_FILTER LPF_Update_All(LPF_FILTER *filt, Vector3 data_gyr, Vector3 data_acc)
{

	filt->filt_gyr_x[0] = LPF_Update_Single(filt, filt->filt_gyr_x[1], data_gyr.x, filt->alpha_gyr);
	filt->filt_gyr_y[0] = LPF_Update_Single(filt, filt->filt_gyr_y[1], data_gyr.y, filt->alpha_gyr);
	filt->filt_gyr_z[0] = LPF_Update_Single(filt, filt->filt_gyr_z[1], data_gyr.z, filt->alpha_gyr);
	filt->filt_acc_x[0] = LPF_Update_Single(filt, filt->filt_acc_x[1], data_acc.x, filt->alpha_acc);
	filt->filt_acc_y[0] = LPF_Update_Single(filt, filt->filt_acc_y[1], data_acc.y, filt->alpha_acc);
	filt->filt_acc_z[0] = LPF_Update_Single(filt, filt->filt_acc_z[1], data_acc.z, filt->alpha_acc);

	filt->filt_gyr_x[1] = filt->filt_gyr_x[0];
	filt->filt_gyr_y[1] = filt->filt_gyr_y[0];
	filt->filt_gyr_z[1] = filt->filt_gyr_z[0];
	filt->filt_acc_x[1] = filt->filt_acc_x[0];
	filt->filt_acc_y[1] = filt->filt_acc_y[0];
	filt->filt_acc_z[1] = filt->filt_acc_z[0];

	return *filt;
}


/// ######################################################################################################
/// ##### HIGH PASS FILTER SECTION #######################################################################
/// ######################################################################################################


void HPF_Init(LPF_FILTER *filt, float f_cut_gyr, float f_cut_acc, float dt)
{
	// beta for HPF settings
	float beta_gyr = HPF_CalculateBeta(f_cut_gyr, dt);
	float beta_acc = HPF_CalculateBeta(f_cut_acc, dt);
	HPF_SetBeta(filt, beta_gyr, beta_acc);

}


float HPF_CalculateBeta(float f_cut, float dt)
{
	return 1.0f - expf(-2.0f * (float)M_PI * f_cut * dt);
}


void HPF_SetBeta(LPF_FILTER *filt, float beta_gyr, float beta_acc)
{
	if(beta_gyr > 1.0f)
	{
		beta_gyr = 1.0f;
	}
	else if(beta_gyr < 0.0f)
	{
		beta_gyr = 0.0f;
	}

	if(beta_acc > 1.0f)
	{
		beta_acc = 1.0f;
	}
	else if(beta_acc < 0.0f)
	{
		beta_acc = 0.0f;
	}

	filt->beta_gyr = beta_gyr;
	filt->beta_acc = beta_acc;
}



/// ------ HIGH PF ILTER, 1° ORDER -----------------------------------------------------------------------------------
float HPF_Update_Single(LPF_FILTER *filt, float *filt_data, float *not_filtered_data, float beta)
{
	 return ( ((0.5) * (2-beta) * (not_filtered_data[0]-not_filtered_data[1])) + ((1-beta) * filt_data[1]) );	// it returns the current filtered data
}


LPF_FILTER HPF_Update_All(LPF_FILTER *filt, Vector3 data_gyr, Vector3 data_acc)
{
	filt->not_filt_gyr_x[1] = filt->not_filt_gyr_x[0];
	filt->not_filt_gyr_y[1] = filt->not_filt_gyr_y[0];
	filt->not_filt_gyr_z[1] = filt->not_filt_gyr_z[0];
	filt->not_filt_acc_x[1] = filt->not_filt_gyr_x[0];
	filt->not_filt_acc_y[1] = filt->not_filt_gyr_y[0];
	filt->not_filt_acc_z[1] = filt->not_filt_gyr_z[0];
	filt->not_filt_gyr_x[0] = data_gyr.x;
	filt->not_filt_gyr_y[0] = data_gyr.y;
	filt->not_filt_gyr_z[0] = data_gyr.z;
	filt->not_filt_acc_x[0] = data_acc.x;
	filt->not_filt_acc_y[0] = data_acc.y;
	filt->not_filt_acc_z[0] = data_acc.z;

	filt->filt_gyr_x[0] = HPF_Update_Single(filt, filt->filt_gyr_x, filt->not_filt_gyr_x, filt->beta_gyr);
	filt->filt_gyr_y[0] = HPF_Update_Single(filt, filt->filt_gyr_y, filt->not_filt_gyr_y, filt->beta_gyr);
	filt->filt_gyr_z[0] = HPF_Update_Single(filt, filt->filt_gyr_z, filt->not_filt_gyr_z, filt->beta_gyr);
	filt->filt_acc_x[0] = data_acc.x; //HPF_Update_Single(filt, filt->filt_acc_x, filt->not_filt_acc_x, filt->beta_acc);
	filt->filt_acc_y[0] = data_acc.y; //HPF_Update_Single(filt, filt->filt_acc_y, filt->not_filt_acc_y, filt->beta_acc);
	filt->filt_acc_z[0] = data_acc.z; //HPF_Update_Single(filt, filt->filt_acc_z, filt->not_filt_acc_z, filt->beta_acc);


	filt->filt_gyr_x[1] = filt->filt_gyr_x[0];
	filt->filt_gyr_y[1] = filt->filt_gyr_y[0];
	filt->filt_gyr_z[1] = filt->filt_gyr_z[0];
	filt->filt_acc_x[1] = filt->filt_acc_x[0];
	filt->filt_acc_y[1] = filt->filt_acc_y[0];
	filt->filt_acc_z[1] = filt->filt_acc_z[0];

	return *filt;
}
//------------------------------------------------------------------------------------------------------------------------






