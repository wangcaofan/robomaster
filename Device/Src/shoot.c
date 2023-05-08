//
// Created by Yuanbin on 22-10-3.
//
#include "robot_ref.h"

#if defined(GIMBAL_BOARD)

#include "shoot.h"

#include "pid.h"

#include "motor.h"


/**
 * @brief �������
 * @note  Ħ���ֱ��٣���̬����ת��
 */
float Temp_Fix_speed(uint16_t real_speed)
{
  float temp_scope = 50;//����仯��ΧΪ50���϶�
  float temp_low = 35;//��ʼ�¶��趨Ϊ35���϶�
  float res = 0;
  float temp_real= (float)(Gimbal_Motor[Left_Friction].Data.temperature + Gimbal_Motor[Right_Friction].Data.temperature)/2.f;
  float speed_bap = 0;
  
  if(13.5f<real_speed<=15.f)
  {
	speed_bap = -50.f;
  }else if(16.5f<real_speed<=18.f)
  {
	speed_bap = -70.f;
  }else if(27.5f<real_speed<=30.f)
  {
	speed_bap = -170.f;
  }
  
  if(temp_real >= temp_low)
    res = (temp_real - temp_low)/temp_scope * (-speed_bap);
  if(temp_real < temp_low)
    res = 0;
  if(temp_real > temp_low + temp_scope)
    res = -speed_bap;

	return res;
}

float SpeedAdapt(float real_S , float min_S, float max_S,float up_num , float down_num)
{
	float res=0;
	uint8_t SpeedErr_cnt=0;

  if(real_S < min_S && real_S > 8)SpeedErr_cnt++;
  else if(real_S >= min_S && real_S <= max_S )SpeedErr_cnt = 0;
	
  if(SpeedErr_cnt == 1)//����ƫ��
  {
    SpeedErr_cnt = 0;
    res += up_num;
  }
  if(real_S > max_S)//����ƫ��
    res -= down_num;
  return res;
}

//�����ж�
bool Judge_IF_SingeStuck(float angle_err)
{
	if(ABS(angle_err) >= 360.f/13.f)return true;
	
	return false;
}
bool Judge_IF_AutoBlock(float speed_err)
{
	if(ABS(speed_err) > 500 && ABS(Gimbal_Motor[Trigger].Data.velocity) < 300)return true;
	
	return false;
}

#endif
