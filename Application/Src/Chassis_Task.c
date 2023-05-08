//
// Created by YanYuanbin on 22-10-12.
//

#include "robot_ref.h"



#include "cmsis_os.h"

#include "Chassis_Task.h"

#include "referee_info.h"

#include "motor.h"

#include "pid.h"

#include "bsp_rc.h"
//PID
PID_TypeDef_t Chassis_pid1,Chassis_pid2,Chassis_pid3,Chassis_pid4;
//Ò£¿ØÆ÷
Chassis_move_t Chassis_ctrl;

float wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float current_set[4] = {0.0f, 0.0f, 0.0f, 0.0f};

float f_chassis_Pid_Para[4][PID_PARAMETER_CNT]={
		[0] = {0,2500,16000,13.f,0.5f,0.f,},
		[1] = {0,2500,16000,13.f,0.5f,0.f,},
		[2] = {0,2500,16000,13.f,0.5f,0.f,},
		[3] = {0,2500,16000,13.f,0.5f,0.f,},
};

;

static void chassis_init(void);
static void chassis_control(Chassis_move_t *chassis_control);
//static void chassis_posture_control(Chassis_Info_t *Chassis_Info);	
//static void chassis_send_current(Chassis_Info_t *Chassis_Info_send,CAN_TxFrameTypeDef *TXFrame);
//static float CHAS_Power_Control(float target_linespeed);

/* USER CODE BEGIN Header_Chassis_Task */
/**
* @brief Function implementing the StartChassisTas thread.
* @param argument: Not used
* @retval None

*/
/* USER CODE END Header_Chassis_Task */
void Chassis_Task(void const * argument)
{
  /* USER CODE BEGIN Chassis_Task */
  
	chassis_init();
  
  /* Infinite loop */
  for(;;)
  {
  chassis_control(&Chassis_ctrl);
	
    osDelay(1);
  }
  /* USER CODE END Chassis_Task */
}

/**
  * @brief          µ×ÅÌ³õÊ¼»¯
  * @param[out]     none
  * @retval         none
  */
static void chassis_init(void)
{		
	//PID init
	PID_Init(&Chassis_pid1, PID_VELOCITY,f_chassis_Pid_Para[0]);
	PID_Init(&Chassis_pid2,PID_VELOCITY,f_chassis_Pid_Para[1]);
	PID_Init(&Chassis_pid3, PID_VELOCITY,f_chassis_Pid_Para[2]);
	PID_Init(&Chassis_pid4, PID_VELOCITY,f_chassis_Pid_Para[3]);
   Chassis_ctrl.chassis_rc = get_remote_control_point();
}
void chassis_vector_to_mecanum_wheel_speed(const float vx_set, const float vy_set, const float wz_set, float wheel_speed[4]){
    wheel_speed[0] = -vx_set - vy_set + MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] = vx_set - vy_set + MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] = vx_set + vy_set + MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[3] = -vx_set + vy_set + MOTOR_DISTANCE_TO_CENTER * wz_set;
}
/**
  * @brief          µ×ÅÌ×ËÌ¬¿ØÖÆ
  * @param[out]     Chassis_Info:µ×ÅÌÐÅÏ¢±äÁ¿Ö¸Õë.
  * @retval         none
  */
static void chassis_control(Chassis_move_t *chassis_control)	
{
		 chassis_control->vx_set=chassis_control->chassis_rc->rc.ch[3]*10;
		 chassis_control->vy_set=chassis_control->chassis_rc->rc.ch[2]*10;
		 chassis_control->wz_set=chassis_control->chassis_rc->rc.ch[0]*10;
		 chassis_vector_to_mecanum_wheel_speed(chassis_control->vx_set,chassis_control->vy_set, chassis_control->wz_set,wheel_speed);
	  current_set[0]=f_PID_Calculate(&Chassis_pid1,wheel_speed[0],Chassis_Motor[0].Data.speed_rpm);
	  current_set[1]=f_PID_Calculate(&Chassis_pid2,wheel_speed[1],Chassis_Motor[1].Data.speed_rpm);
		current_set[2]=f_PID_Calculate(&Chassis_pid3,wheel_speed[2],Chassis_Motor[2].Data.speed_rpm);
	  current_set[3]=f_PID_Calculate(&Chassis_pid4,wheel_speed[3],Chassis_Motor[3].Data.speed_rpm);
	  CAN_cmd_chassis(current_set[0],current_set[1],current_set[2],current_set[3]);
}


