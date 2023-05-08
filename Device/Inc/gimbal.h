//
// Created by Yuanbin on 22-10-3.
//

#ifndef GIMBAL_H
#define GIMBAL_H

#include "stdint.h"
#include "bsp_rc.h"


//��̨ģʽö��
typedef enum
{
   Gimbal_WEEK = 0x00U,  //����
	 Gimbal_INS = 0x01U, //imuģʽ
	 Gimbal_Rcshoot = 0x02U, //ң��������
}Gimbal_Mode_e;

typedef struct _Gimbal_Info
{
	//ң����Ϣ
	RC_ctrl_t *rc_ctrl;
	
	Gimbal_Mode_e mode;
	
	//��̨״̬����ֵ
	struct{
		float pit_angle;
		float yaw_angle;
		float pit_gyro;
		float yaw_gyro;
	}Target;
	
	//��̨״̬�۲���
	struct{
		float *pit_angle;
		float *yaw_angle;
		float *pit_gyro;
		float *yaw_gyro;
	}Measure;
	
	//pitch���޷�
	struct{
		float min;
		float max;
	}Limit_pitch;
	
	//���Ƶ�������
	int16_t SendValue[2];

}Gimbal_Info_t;

/* Exported functions --------------------------------------------------------*/
extern void gimbal_mode_set(Gimbal_Info_t *Gimbal_Info);
extern void gimbal_Info_update(Gimbal_Info_t *Gimbal_Info);
extern void rc_communicate(_ctrl_t *rc_ctrl,CAN_TxFrameTypeDef *TxHeader);
#endif //GIMBAL_H
