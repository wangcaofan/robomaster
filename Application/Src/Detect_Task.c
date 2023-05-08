//
// Created by Yuanbin on 22-10-3.
//

#include "robot_ref.h"
#include "cmsis_os.h"
#include "pid.h"

#if defined(CHASSIS_BOARD)

#include "Detect_Task.h"
#include "Chassis_Task.h"

#include "referee_info.h"
#include "motor.h"
#include "ui.h"

chassis_detect_t chassis_detect;

detect_slip_t detect_slip[3];

/* USER CODE BEGIN Header_DetectTask */
/**
* @brief Function implementing the StartDetectTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DetectTask */
void DetectTask(void const * argument)
{
  /* USER CODE BEGIN DetectTask */
	detect_slip_init(&detect_slip[0],0	,3.73f);
	detect_slip_init(&detect_slip[1],40	,-0.00629f);//-0.00629f
	detect_slip_init(&detect_slip[2],0	,-1.16f);
	chassis_detect.slip_warning_value = 15;

  /* Infinite loop */
	
  for(;;)
  {
		//·­µ¹¼ì²â
		if(ABS(*Chassis_Ctrl.Measure.pit_angle) > 25.f && Chassis_Ctrl.mode != CHASSIS_WEEK)
		{
			chassis_detect.cnt_LoseCtrl = chassis_detect.cnt_LoseCtrl < 500 ? (chassis_detect.cnt_LoseCtrl++) : (chassis_detect.cnt_LoseCtrl);
		}
		else if(chassis_detect.cnt_LoseCtrl > 0)chassis_detect.cnt_LoseCtrl--;
		
		detect_slip_update(&detect_slip[0],Chassis_Ctrl.Measure.linespeed/100.f);
		detect_slip_update(&detect_slip[1],0.5f*(MT9025[1].Data.current - MT9025[0].Data.current));
		detect_slip_update(&detect_slip[2],*Chassis_Ctrl.Measure.pit_gyro/180*PI);
		check_slipped(&chassis_detect,detect_slip);
		
		Report_Referee_Data(&REFEREE_TxFrame);
		
		Startjudge_task();

    osDelay(2);
  }
  /* USER CODE END DetectTask */
}

#endif

#if defined(GIMBAL_BOARD)

#include "gimbal.h"
#include "bsp_can.h"
#include "bsp_tim.h"
#include "tim.h"


uint16_t SetPWM = 0;

static void Cover_Ctrl(bool IF_Cover_ENABLE);

/* USER CODE BEGIN Header_DetectTask */
/**
* @brief Function implementing the StartDetectTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DetectTask */
void DetectTask(void const * argument)
{
  /* USER CODE BEGIN DetectTask */

  /* Infinite loop */
  for(;;)
  {
		rc_ctrl_monitor(&rc_ctrl);
		
		rc_communicate(&rc_ctrl,&RBCTxFrame);
		
		Cover_Ctrl(Key_R());
		

    osDelay(2);
  }
  /* USER CODE END DetectTask */
}

static void Cover_Ctrl(bool IF_Cover_ENABLE)
{
	static bool  IF_Cover_Open = false;
	
	if(IF_Cover_ENABLE)
	{
		IF_Cover_Open = ((IF_Cover_Open == false) ?  true : false);
	}
	
	if(IF_Cover_Open)
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 500);
	else
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 2200);
}

#endif
