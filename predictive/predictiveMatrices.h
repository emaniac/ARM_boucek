/*
 * predictive.c
 *
 *  Author: Jan Boucek
 */



#include "CMatrixLib.h"
#define T 12
#define T_MAX 200			// last value in pr_block

const float pr_block_arr[T];	//has to fit
const float pr_A_arr[6*6];
const float pr_Av_arr[(6*T)*(6)];				//extended A
const float pr_B_arr[6*2];
//const float pr_Bv_arr[(6*T)*(2*T)];				//extended B
const float pr_Hv_arr[(2*T)*(2*T)];
//const float pr_Qv_arr[6*T*6*T];
const float Qv_Bv_t_arr[2*T*6*T];

const float pr_Acx_arr[T*6];
const float pr_Acy_arr[T*6];
const float pr_Bcx_arr[T*T*2];
const float pr_Bcy_arr[T*T*2];
