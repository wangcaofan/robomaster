//
// Created by Yuanbin on 22-10-3.
//
#include "crc.h"
#include "bsp_referee.h"
#include "referee_Info.h"
#include "bsp_uart.h"

#define REFEREE_DATALENGTH  200

uint8_t Referee_Rx_Buf[REFEREE_DATALENGTH];

void referee_receive_init(void)
{
		/* UART IDLE ENABLE */
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
		
		UART_Start_Receive_DMA(&huart1,Referee_Rx_Buf,REFEREE_DATALENGTH);
}

void Referee_Receive_RxEvent(UART_HandleTypeDef *huart)
{
	uint16_t index=0;	//��ǰ�������
	uint16_t data_length = 0;	//֡���ݳ���
	
	if(__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) != RESET)
	{
		
		__HAL_UART_CLEAR_IDLEFLAG(huart);
	
		__HAL_DMA_DISABLE(huart->hdmarx);

		//����ϵͳ��������ѡ����
		while(Referee_Rx_Buf[index]==0XA5) //֡ͷSOFУ��
		{
			if(Verify_CRC8_Check_Sum(&Referee_Rx_Buf[index],5) == 1)
			{
				data_length = Referee_Rx_Buf[index+2]<<8 | Referee_Rx_Buf[index+1] + 9;
				if(Verify_CRC16_Check_Sum(&Referee_Rx_Buf[index],data_length))	//CRC16У�飨CRC8������У�飩
				{
					//����ϵͳ���ݽ���
					RefereeInfo_Decode(&Referee_Rx_Buf[index]);	
				}
			}
			index += data_length;
		}
		//�������
		memset(Referee_Rx_Buf,0,REFEREE_DATALENGTH);
		index = 0;
		//�������DMA������
		__HAL_DMA_SET_COUNTER(huart->hdmarx,REFEREE_DATALENGTH);
	
		UART_Start_Receive_DMA(&huart1,Referee_Rx_Buf,REFEREE_DATALENGTH);
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
}

