//
// Created by Yuanbin on 22-10-3.
//

#ifndef DETECT_H
#define DETECT_H

#include "stdint.h"
#include "filter.h"

#define DETECT_NUM 50
#define CHECK_NUM 50

//���̼��ṹ��
typedef struct
{
	uint16_t cnt_LoseCtrl;			//ʧ�ؼ��
	
	float slip_rk[CHECK_NUM];		//�򻬼��
	float slip_warning_value;
	float slip_rk_sum;
	uint8_t slip_num_rk_now;
	uint8_t isSlipped;
	
}chassis_detect_t;

//�򻬼�����ṹ��
typedef struct
{
	lpf_data_t LPF_1,LPF_2;		//��ͨ�˲�
	kf_data_t KF;							//�������
	float rk_sen;							//���ж�
	uint16_t phase;						//��λ
	float KF_rk[DETECT_NUM];
	uint8_t num_rk_now;
	float rk_adj;
}detect_slip_t;


/* Exported functions --------------------------------------------------------*/
extern void detect_slip_init(detect_slip_t* data, uint16_t phase,	float sen);
extern void detect_slip_update(detect_slip_t* data, float new_data);
extern void check_slipped(chassis_detect_t *chassis_detect,detect_slip_t detect_slip[3]);
#endif //DETECT_H
