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

#include "motor.h"
#include "main.h"
#include "can.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read

void get_Motor_Data(CAN_HandleTypeDef* hcanx,CAN_RxFrameTypeDef* RxMsg,DJI_MOTOR* DJI_Motor)                               \
    {                           
       if(hcanx!=DJI_Motor->hcanx)return ;	
//       if(RxMsg->hcan_header.StdId==DJI_Motor->Data.StdId){
        DJI_Motor->Data.last_ecd = DJI_Motor->Data.ecd;                                   
        DJI_Motor->Data.ecd = (int16_t)RxMsg->data[0]<<8 | (int16_t)RxMsg->data[1];             
        DJI_Motor->Data.speed_rpm = (int16_t)RxMsg->data[2] << 8 | (int16_t)RxMsg->data[3];      
        DJI_Motor->Data.given_current = (int16_t)RxMsg->data[4] << 8 | (int16_t)RxMsg->data[5];  
        DJI_Motor->Data.temperate = (int16_t)RxMsg->data[6];
//			}				
    }
/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
电机数据, 0:底盘电机1 3508电机,  1:底盘电机2 3508电机,2:底盘电机3 3508电机,3:底盘电机4 3508电机;
4:yaw云台电机 6020电机; 5:pitch云台电机 6020电机; 6:拨弹电机 2006电机*/
DJI_MOTOR Chassis_Motor[7]={
    [Left_Font] = {
                .type=_3508,
                .usage=Left_Font,
                .hcanx=&hcan1,
                .Data.StdId=0x201,
                .TxMsg=&CAN_TxMsg[_CAN1][_0x200],
            },

    [Righ_Font]={
                .type=_3508,
                .usage=Righ_Font,
                .hcanx=&hcan1,
                .Data.StdId=0x202,
                .TxMsg=&CAN_TxMsg[_CAN1][_0x200],
            },

    [Left_Rear]={
                .type=_3508,
                .usage=Left_Rear,
                .hcanx=&hcan1,
                .Data.StdId=0x203,
                .TxMsg=&CAN_TxMsg[_CAN1][_0x200],
            },

    [Righ_Rear]={
                .type=_3508,
                .usage=Righ_Rear,
                .hcanx=&hcan1,
                .Data.StdId=0x204,
                .TxMsg=&CAN_TxMsg[_CAN1][_0x200],
            },
		 [PITCH]={
                .type=_3508,
                .usage=Righ_Rear,
                .hcanx=&hcan1,
                .Data.StdId=0x206,
                .TxMsg=&CAN_TxMsg[_CAN2][_0x1ff],
            },
    [YAW]={
                .type=_6020,
                .usage=YAW,
                .hcanx=&hcan2,
                .Data.StdId=0x205,
                .TxMsg=&CAN_TxMsg[_CAN2][_0x1ff],
            },
};


static CAN_TxFrameTypeDef  chassis_TxMsg;
static CAN_TxFrameTypeDef  gibmal_TxMsg;



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxFrameTypeDef RxMsg;

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMsg.hcan_header, RxMsg.data);

    switch (RxMsg.hcan_header.StdId)
    {
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        case CAN_YAW_MOTOR_ID:
        case CAN_PIT_MOTOR_ID:
        case CAN_TRIGGER_MOTOR_ID:
        {
            static uint8_t i = 0;
            //get motor id
            i = RxMsg.hcan_header.StdId - CAN_3508_M1_ID;
            get_Motor_Data(hcan,&RxMsg,&Chassis_Motor[i]);
            break;
        }

        default:
        {
            break;
        }
    }
}
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_TxMsg.hcan_header.StdId=CAN_TxMsg[_CAN1][_0x200].hcan_header.StdId;
    chassis_TxMsg.hcan_header.IDE = CAN_TxMsg[_CAN1][_0x200].hcan_header.IDE;
    chassis_TxMsg.hcan_header.RTR = CAN_TxMsg[_CAN1][_0x200].hcan_header.RTR;
    chassis_TxMsg.hcan_header.DLC = CAN_TxMsg[_CAN1][_0x200].hcan_header.DLC;
    chassis_TxMsg.data[0] = motor1 >> 8;
    chassis_TxMsg.data[1] = motor1;
    chassis_TxMsg.data[2] = motor2 >> 8;
    chassis_TxMsg.data[3] = motor2;
    chassis_TxMsg.data[4] = motor3 >> 8;
    chassis_TxMsg.data[5] = motor3;
    chassis_TxMsg.data[6] = motor4 >> 8;
    chassis_TxMsg.data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_TxMsg.hcan_header, chassis_TxMsg.data, &send_mail_box);
}
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    uint32_t send_mail_box;
    gibmal_TxMsg.hcan_header.StdId = CAN_TxMsg[_CAN2][_0x1ff].hcan_header.StdId;;
    gibmal_TxMsg.hcan_header.IDE  =  CAN_TxMsg[_CAN2][_0x1ff].hcan_header.IDE;
    gibmal_TxMsg.hcan_header.RTR  =  CAN_TxMsg[_CAN2][_0x1ff].hcan_header.RTR;
    gibmal_TxMsg.hcan_header.DLC= 0x08;
    gibmal_TxMsg.data[0] = (yaw >> 8);
    gibmal_TxMsg.data[1] = yaw;
    gibmal_TxMsg.data[2] = (pitch >> 8);
    gibmal_TxMsg.data[3] = pitch;
    gibmal_TxMsg.data[4] = (shoot >> 8);
    gibmal_TxMsg.data[5] = shoot;
    gibmal_TxMsg.data[6] = (rev >> 8);
    gibmal_TxMsg.data[7] = rev;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gibmal_TxMsg.hcan_header, gibmal_TxMsg.data, &send_mail_box);
}
const DJI_MOTOR *get_chassis_motor_measure_point(uint8_t i)
{
	return &Chassis_Motor[(i & 0x03)];

}
