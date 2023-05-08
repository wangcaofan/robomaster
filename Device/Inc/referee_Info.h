//
// Created by Yuanbin on 22-10-3.
//

#ifndef REFEREE_INFO_H
#define REFEREE_INFO_H

#include "stdint.h"
#include "string.h"
#include "robot_ref.h"


//֡ͷ��ʽ
typedef struct
{                                       		 //�ֽ�ƫ��     ��С     ˵��
	uint8_t SOF;                               //   0          1      ����֡��ʼ�ֽڣ��̶�ֵΪ0XA5
	uint16_t data_length;                      //   1          2      ����֡��Data���� 
	uint8_t seq;                               //   3          1      ����� 
	uint8_t CRC8;                              //   4          1      ֡ͷCRCУ��
}FrameHeader_t;

//ͨ��Э���ʽ
typedef struct
{                                       		 //�ֽ�ƫ��     ��С     ˵��
	FrameHeader_t FrameHeader;      					 //   0          5      ֡ͷЭ��
	uint16_t CmdID;                            //   5          2      ������ID
	uint16_t Date[50];                         //   7          n      ����
	uint16_t FrameTail;                        //   7+n        2      CRC16У��
}RefereeInfo_t;


#pragma pack(1)

//����״̬���ݣ�0X0001  1HZ
typedef struct          
{
	uint8_t game_type : 4;	//��������
	uint8_t game_progress : 4;	//��ǰ�����׶�
	uint16_t stage_remain_time;	//��ǰ�׶�ʣ��ʱ��
	uint64_t SyncTimeStamp;	//�����˽��յ���ָ��ľ�ȷ Unix ʱ�䣬�����ض��յ���Ч�� NTP ��������ʱ����Ч
} ext_game_status_t;

//����������ݣ�0x0002  ������������
typedef struct
{
	uint8_t winner;	//0 ƽ�� 1 �췽ʤ�� 2 ����ʤ��
} ext_game_result_t;

//������Ѫ������:0x0003  1Hz
typedef struct
{
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP;
	uint16_t red_3_robot_HP;
	uint16_t red_4_robot_HP;
	uint16_t red_5_robot_HP;
	uint16_t red_7_robot_HP;
	uint16_t red_outpost_HP;	//�췽ǰ��սѪ��
	uint16_t red_base_HP;			//�췽����Ѫ��
	uint16_t blue_1_robot_HP;
	uint16_t blue_2_robot_HP;
	uint16_t blue_3_robot_HP;
	uint16_t blue_4_robot_HP;
	uint16_t blue_5_robot_HP;
	uint16_t blue_7_robot_HP;
	uint16_t blue_outpost_HP;	//����ǰ��վѪ��
	uint16_t blue_base_HP;	//��������Ѫ��
}ext_game_robot_HP_t;

//�����¼����ݣ� 0x0101  �¼��ı����
typedef struct
{
	uint32_t event_type;
} ext_event_data_t;

//����վ������ʶ��0x0102   �����ı����
typedef struct
{
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
	uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

//����ϵͳ������Ϣ��0x0104 �������
typedef struct
{
 uint8_t level; // 1���� 2���� 3�и�
 uint8_t offending_robot_id;// Υ������� ID���и���˫������ʱ����ֵΪ 0
}referee_warning_t;

//���ڷ���ڵ���ʱ��cmd_id (0x0105)������Ƶ�ʣ�1Hz ���ڷ��ͣ����ͷ�Χ������������
typedef struct
{
	uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

//����������״̬��0x0201  10Hz
typedef struct
{
	uint8_t robot_id;	//������ID
	uint8_t robot_level;	//�����˵ȼ�
	uint16_t remain_HP;	//������ʣ��Ѫ��
	uint16_t max_HP;	//����������Ѫ��
	uint16_t shooter_id1_17mm_cooling_rate;	//������ 1 �� 17mm ǹ��ÿ����ȴֵ
	uint16_t shooter_id1_17mm_cooling_limit;	//������ 1 �� 17mm ǹ����������
	uint16_t shooter_id1_17mm_speed_limit;	//������ 1 �� 17mm ǹ�������ٶ� ��λ m/s
	uint16_t shooter_id2_17mm_cooling_rate;	//������ 2 �� 17mm ǹ��ÿ����ȴֵ
	uint16_t shooter_id2_17mm_cooling_limit;	//������ 2 �� 17mm ǹ����������
	uint16_t shooter_id2_17mm_speed_limit;	//������ 2 �� 17mm ǹ�������ٶ� ��λ m/s
	uint16_t shooter_id1_42mm_cooling_rate;	//������ 42mm ǹ��ÿ����ȴֵ
	uint16_t shooter_id1_42mm_cooling_limit;	//������ 42mm ǹ����������
	uint16_t shooter_id1_42mm_speed_limit;	//������ 42mm ǹ�������ٶ� ��λ m/s
	uint16_t chassis_power_limit;	//�����˵��̹�����������
	uint8_t mains_power_gimbal_output : 1;	//���ص�Դ��������1Ϊ24v��0Ϊ�����
	uint8_t mains_power_chassis_output : 1;
	uint8_t mains_power_shooter_output : 1;
} ext_game_robot_status_t;

//ʵʱ�����������ݣ�0x0202������Ƶ�ʣ�50Hz
typedef struct
{
	uint16_t chassis_volt;	//���������ѹ ��λ ����
	uint16_t chassis_current;	//����������� ��λ ����
	float chassis_power;	//����������� ��λ W ��
	uint16_t chassis_power_buffer;	//���̹��ʻ��� ��λ J ���� ��ע�����¸��ݹ��������� 250J
	uint16_t shooter_id1_17mm_cooling_heat;	//1 �� 17mm ǹ������
	uint16_t shooter_id2_17mm_cooling_heat;	//2 �� 17mm ǹ������
	uint16_t shooter_id1_42mm_cooling_heat;	//42mm ǹ������
} ext_power_heat_data_t;

// ������λ�ã�0x0203 10Hz
typedef struct
{
	float x;
	float y;
	float z;
	float yaw;
} ext_game_robot_pos_t;

//���������棺0x0204������Ƶ�ʣ�1Hz
/*
bit 0��������Ѫ����Ѫ״̬
bit 1��ǹ��������ȴ����
bit 2�������˷����ӳ�
bit 3�������˹����ӳ�
���� bit ����
*/
typedef struct
{
		uint8_t power_rune_buff;
}ext_buff_t;

//���л���������״̬��0x0205  10Hz
typedef struct
{
uint8_t attack_time;
} aerial_robot_energy_t;

//�˺�״̬��0x0206    �˺���������
typedef struct
{
uint8_t armor_id : 4;
uint8_t hurt_type : 4;
} ext_robot_hurt_t;

//�����Ϣ��0x0207  �������
typedef struct
{
uint8_t bullet_type;	//�ӵ�����: 1��17mm ���� 2��42mm ����
uint8_t shooter_id;	//������� ID��1��1 �� 17mm �������2��2 �� 17mm ������� 3��42mm �������
uint8_t bullet_freq;	//�ӵ���Ƶ ��λ Hz
float bullet_speed;	//�ӵ����� ��λ m/s
} ext_shoot_data_t;

//�ӵ�ʣ�෢����Ŀ��0x0208 1HZ
typedef struct
{
 uint16_t projectile_allowance_17mm;//17mm ������������
 uint16_t projectile_allowance_42mm;//42mm ������������
 uint16_t remaining_gold_coin;//ʣ��������
}projectile_allowance_t;

//������ RFID ״̬��0x0209������Ƶ�ʣ�1Hz�����ͷ�Χ����һ������
typedef struct
{
uint32_t rfid_status;
} ext_rfid_status_t;
// ���ڻ����˿ͻ���ָ�����ݣ�0x020A������Ƶ�ʣ�10Hz�����ͷ�Χ����һ������
typedef struct
{
uint8_t dart_launch_opening_status;
uint8_t dart_attack_target;
uint16_t target_change_time;
uint8_t first_dart_speed;
uint8_t second_dart_speed;
uint8_t third_dart_speed;
uint8_t fourth_dart_speed;
uint16_t last_dart_launch_time;
uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;


typedef enum{
    INITIAL,
    BURST,
    COOLING,
    RATE,
    ROBOT_MODE_NUM,
}Robot_Mode_e;

typedef enum{
    LV_1,
    LV_2,
    LV_3,
    ROBOT_LEVEL_NUM,
}Robot_Level_e;

typedef struct
{
	uint16_t	cooling_rate;		//ǹ��ÿ����ȴֵ
	uint16_t	cooling_limit;  //ǹ����������
	uint16_t	speed_limit;    //ǹ�������ٶ� ��λ m/s
}ext_shoot_t;

typedef struct 
{
		uint8_t id; //������Id 0��1��
	
		Robot_Level_e level; //�ȼ�
	
		Robot_Mode_e mode; //�����������
		uint8_t mains_power_shooter_output;//������������0 ����� 1 24V
		uint16_t cooling_heat;   //ǹ������
		float bullet_speed;//ʵʱ����
		ext_shoot_t shooter_id1_17mm;		//���������Ϣ
}ROBOT_Info_t;

#pragma pack()

extern ext_game_status_t                   game_state;					         //0x0001
extern ext_game_robot_HP_t                 game_robot_HP;                //0x0003
extern ext_game_robot_status_t             game_robot_state;             //0x0201
extern ext_power_heat_data_t 	        	   power_heat_data;	      	     //0x0202
extern ext_buff_t                          buff_data;                    //0x0204
extern ext_shoot_data_t                    real_shoot_data;              //0x0207
extern projectile_allowance_t              bullet_remaining;             //0x0208\

extern ROBOT_Info_t robot;

/* Exported functions --------------------------------------------------------*/
extern void RefereeInfo_Decode(uint8_t *UsartRx_Info);
//extern void Report_Referee_Data(CAN_TxFrameTypeDef *CAN_TxHandler);
extern void get_Referee_Data(uint32_t *StdId, uint8_t *rxBuf);
#endif //REFEREE_INFO_H



