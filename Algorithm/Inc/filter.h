//
// Created by Yuanbin on 22-10-3.
//

#ifndef FILTER_H
#define FILTER_H

#include "stdint.h"

typedef struct 
{
	float fliter_num[3];
	float fliter_1,fliter_2,fliter_3;
	float out;
	float in;
	
}lpf_data_t;

typedef struct 
{
	float in;
	float Last_P;	//�ϴι���Э����
	float Now_P;	//��ǰ����Э����
	float out;		//�������˲������
	float Kg;			//����������
	float Q;			//��������Э����
	float R;			//�۲�����Э����
	float ek;
	float rk;			//��������
}kf_data_t;

/* Exported functions --------------------------------------------------------*/

extern void LP_FilterInit(lpf_data_t *lp,float *fliter_num);
extern float LP_FilterCalc(lpf_data_t *lp,float input);
extern void Kalman_Init(kf_data_t *kf,float Q,float R);
extern float KalmanFilterCalc(kf_data_t *kf,float input);

#endif //FILTER_H
