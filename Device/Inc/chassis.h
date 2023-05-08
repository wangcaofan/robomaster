//
// Created by Yuanbin on 22-10-3.
//

#ifndef CHASSIS_H
#define CHASSIS_H

#include "stdint.h"
#include "stdbool.h"
#include "robot_ref.h"

#include "bsp_rc.h"
#include "motor.h"
#ifdef BalanceInfantry

#include "arm_math.h"

/* �����ְ뾶 */
#define Wheel_Radius  0.125f

/* 
 * ת��ת��(dps)ת���ɵ������ٶ�(m/s)�ı���
 * ���㹫ʽ:2*pi*m_r/(360.f*k) kΪ���ٱ�
 */
#if (defined(Wheel_Radius) && defined(PI))
	#define MOTOR_RPM_TO_LINE_SPEED   PI*Wheel_Radius/180.f
#endif

/* 
 * ת��ת��(rpm)ת�����������ٶ�(��/s)�ı���
 * ���㹫ʽ:360/(60*2*pi*k) kΪ���ٱ�
 */
#ifdef PI
	#define DJIGM6020_RPM_TO_OMEGA   3.f/PI
#endif

#endif
#define MOTOR_DISTANCE_TO_CENTER 0.2f


//����ģʽö��
typedef enum
{
   CHASSIS_WEEK = 0x00U,  //����
	 CHASSIS_FRONT= 0x01U,  //����Ե�
	 CHASSIS_FLY = 0x02U,  //����
	 CHASSIS_SPIN = 0x03U,  //С����
}Chassis_Mode_e;

typedef struct
{
//����ģʽ
	Chassis_Mode_e mode;
	
	bool IF_SPIN_ENABLE;
	
//�Ƿ�ƫ��������
	bool IF_YAW_ANGLE_OFFSET;
	bool IF_PIT_ANGLE_OFFSET;

//��������ֵ
	struct{
		float yaw_angle[2];
		float yaw_gyro;
		float pit_angle[2];
		float pit_gyro;
		float linespeed;
		float position;
	}Target;
	
//����״̬�۲���
	struct{
		float yaw_angle;
		float yaw_gyro;
		float yaw_err;
		float *pit_angle;
		float *pit_gyro;
		float *rol_gyro;
		float Wheelaccel[2];
		float linespeed;
		float position;
	}Measure;
	//�����޷�
	struct{
		float linespeed;
		float position;
	}Max;
	
	//ң����Ϣ
	
	
	int16_t SendValue[2];

}Chassis_Info_t;
typedef struct{
const RC_ctrl_t *chassis_rc;
const DJI_MOTOR *Chassis_motor[4];
float vx;
float vy;
float wz;
float vx_set;
float vy_set;
float wz_set;
float vx_max;
float vy_max;
float vx_min;
float vy_min;
}Chassis_move_t;
/* Exported functions --------------------------------------------------------*/
extern void chassis_mode_set(Chassis_Info_t *Chassis_Info);
extern void Chassis_Info_update(Chassis_Info_t *Chassis_Info);
extern float AcclerateCurve(float x , float k ,float x0);
//extern void get_rc_ctrl_data(uint32_t *canId, uint8_t *rxBuf,rc_ctrl_t *rc_ctrl);

#endif //CHASSIS_H
