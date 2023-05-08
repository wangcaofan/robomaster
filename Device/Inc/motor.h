/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2

/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,

} can_msg_id_e;

//rm motor data
typedef enum{
    Left_Font=0,
    Righ_Font,
    Left_Rear,
    Righ_Rear,
	  PITCH,
    YAW,
    MOTOR_USAGE_NUM,
}MOTOR_USAGE;

typedef enum{
    _6020=0,
    _3508,
    _2006,
    MOTOR_TYPE_NUM,
}MOTOR_TYPE;

typedef struct
{
 CAN_RxHeaderTypeDef hcan_header;
 uint8_t data[8];
}CAN_RxFrameTypeDef;

typedef struct
{
 CAN_TxHeaderTypeDef hcan_header;
 uint8_t data[8];
}CAN_TxFrameTypeDef;
typedef struct
{
    struct{
			   uint32_t StdId;
         uint16_t ecd;
         int16_t speed_rpm;
         int16_t given_current;
         uint8_t temperate;
         int16_t last_ecd;
    }Data;
    CAN_HandleTypeDef *hcanx;
    CAN_TxFrameTypeDef *TxMsg;
    MOTOR_TYPE type;
    MOTOR_USAGE usage;

}DJI_MOTOR;
extern DJI_MOTOR Chassis_Motor[7];
extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern const DJI_MOTOR *get_chassis_motor_measure_point(uint8_t i);
#endif

