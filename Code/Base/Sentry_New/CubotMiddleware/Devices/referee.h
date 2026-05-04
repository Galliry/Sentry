#ifndef __REFEREE_H
#define __REFEREE_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

#define data_addr 7
#define max_single_pack_len 50
#define packs 15
#define frame_header_len 5 
#define pack_len 7+referee2022.frame_info.head.data_len+2
#define BSP_USART3_DMA_RX_BUF_LEN	256 
#define sentry_id 0x0120

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
	
typedef struct
{
	struct
	{
		struct
		{
			uint8_t sof;
			uint16_t data_len;
			uint8_t seq;
			uint8_t crc8;
		}head;
		uint16_t cmd_id;
		uint8_t  frame_tail[2];
	}frame_info;

	/*1. 比赛状态数据： 0x0001。 发送频率： 1Hz*/
	struct  
	{
		uint8_t game_type : 4;
		uint8_t game_progress : 4;
		uint16_t stage_remain_time;
	}game_status;

	/*2. 比赛结果数据： 0x0002。 发送频率：比赛结束后发送*/
	 struct
	{
		uint8_t winner;
	}game_result;
	/*3. 机器人血量数据： 0x0003。发送频率： 1Hz*/
	 struct
	{
		uint16_t robot_HP[8];
		uint16_t outpost_HP;
		uint16_t base_HP;	
	}game_robot_hp;
	
	/*4. 飞镖发射状态：0x0004。发送频率：飞镖发射后发送，发送范围：所有机器人*/
	struct
	{
		uint8_t dart_belong;   //发射飞镖队伍 1：红方飞镖 2：蓝方飞镖
		uint16_t stage_remaining_time;//发射时的剩余比赛时间 单位 s
	}dart_state;

	/*5. 人工智能挑战赛加成与惩罚区状态：0x0005。发送频率：1Hz 周期发送，发送范围：所有机器人。*/
	 struct
	{
		uint8_t F1_zone_status:1;
		uint8_t F1_zone_buff_debuff_status:3;
		uint8_t F2_zone_status:1;
		uint8_t F2_zone_buff_debuff_status:3;
		uint8_t F3_zone_status:1;
		uint8_t F3_zone_buff_debuff_status:3;
		uint8_t F4_zone_status:1;
		uint8_t F4_zone_buff_debuff_status:3;
		uint8_t F5_zone_status:1;
		uint8_t F5_zone_buff_debuff_status:3;
		uint8_t F6_zone_status:1;
		uint8_t F6_zone_buff_debuff_status:3;
	}ext_ICRA_buff_debuff_zone_status_t;


	/*6.场地事件数据：0x0101。发送频率：1Hz 周期发送，发送范围：己方机器人。*/
	 struct
	{
		uint32_t event_type;
	} event_data;

	/*7. 补给站动作标识：0x0102。发送频率：动作触发后发送，发送范围：己方机器人。*/
	 struct
	{
		uint8_t supply_projectile_id;
		uint8_t supply_robot_id;
		uint8_t supply_projectile_step;
		uint8_t supply_projectile_num;
	} supply_projectile_action;

	/*8. 裁判警告信息： cmd_id (0x0104)。发送频率：警告发生后发送*/
	struct
	{
		uint8_t level;
		uint8_t foul_robot_id;
	} referee_warning;

	/*9. 飞镖发射口倒计时：cmd_id (0x0105)。发送频率：1Hz 周期发送，发送范围：己方机器人。*/
	 struct
	{
		uint8_t dart_remaining_time;
	}dart_remaining_time;

	/*10. 比赛机器人状态： 0x0201。 发送频率： 10Hz，发送范围：单一机器人。*/
	 struct
	{
		uint8_t robot_id;
		uint8_t robot_level;
		uint16_t remain_HP;
		uint16_t max_HP;
		uint16_t chassis_power_limit;
		uint8_t mains_power_gimbal_output : 1;
		uint8_t mains_power_chassis_output : 1;
		uint8_t mains_power_shooter_output : 1;
		uint16_t shooter_id1_17mm_cooling_rate;
		uint16_t shooter_id1_17mm_cooling_limit;
	} game_robot_status;

	/*11. 实时功率热量数据： 0x0202。 发送频率： 50Hz，发送范围：单一机器人。*/
	 struct
	{
		uint16_t chassis_power_buffer; 
		uint16_t shooter_id1_17mm_cooling_heat;
	} power_heat_data;

	/*12. 机器人位置： 0x0203。 发送频率： 10Hz，发送范围：单一机器人。*/
	struct
	{
		float x;
		float y;
		float angle;
	} game_robot_pos;

	/*13. 机器人增益： 0x0204。发送频率：1Hz 周期发送，发送范围：单一机器人。*/
	 struct
	{ 
		uint8_t recovery_buff;  
		uint8_t cooling_buff;  
		uint8_t defence_buff;  
		uint8_t vulnerability_buff; 
		uint16_t attack_buff; 
		uint8_t remaining_energy;
	}buff;

	/*14. 空中机器人能量状态： 0x0205。 发送频率： 10Hz */
	 struct
	{
		uint16_t energy_point;
		uint8_t attack_time;
	} ext_aerial_robot_energy_t;


	/*15. 伤害状态： 0x0206。 发送频率：伤害发生后发送 */
	 struct
	{
		uint8_t armor_id;
		uint8_t hurt_type;
	}robot_hurt;

	/*16. 实时射击信息： 0x0207。 发送频率：射击后发送*/
	 struct
	{
		uint8_t bullet_type;
		uint8_t shooter_id;
		uint8_t bullet_freq;
		float bullet_speed;
	} shoot_data;

	/*17. 子弹剩余发射数： 0x0208。发送频率： 1Hz 周期发送， 空中机器人以及哨兵机器人主控发送*/
	struct
	{
		uint16_t bullet_remaining_num;
		uint16_t money;
	} bullet_remaining;

	//18.机器人 RFID 状态：0x0209。发送频率：1Hz，发送范围：单一机器人。
	 struct
	{
		uint32_t rfid_status;
		uint8_t rfid_status_2;
	}  rfid_status;
	
	//哨兵自主决策信息
	struct
	{
		uint32_t sentry_info;
		uint16_t sentry_info_2;
		
		uint16_t exchange_bullet;
		uint8_t remote_bullet;
		uint8_t remote_HP;
		uint8_t free_revival;
		uint8_t out_of_combat;
		uint16_t remain_bullet;
		uint8_t posture;
		uint8_t energy_device;
	}sentry_info_t;

	//19. 飞镖机器人客户端指令数据：0x020A。发送频率：10Hz，发送范围：单一机器人。
	 struct
	{
		uint8_t dart_launch_opening_status; //飞镖发射口状态
		uint8_t dart_attack_target;
		uint16_t target_change_time;
		uint8_t first_dart_speed;
		uint8_t second_dart_speed;
		uint8_t third_dart_speed;
		uint8_t fourth_dart_speed;
		uint16_t last_dart_launch_time;
		uint16_t operate_launch_cmd_time;
	} dart_client_cmd;
	
	struct
	{
		float hero_x;
		float hero_y;
		float engineer_x;
		float engineer_y;
		float standard_3_x;
		float standard_3_y;
		float standard_4_x;
		float standard_4_y;
	}groud_robot_position_t;

	//20. 烧饼机器人状态数据: 0x020D。
	struct{
		/**
		 * |bit|func|
		 * |-|-|
		 * |0-10|哨兵已成功兑换的发弹量|
		 * |11-14|哨兵已成功兑换发弹量的次数|
		 * |15-18|哨兵已成功兑换血量的次数|
		 * |19|哨兵是否可以确认免费复活|
		 * |20|哨兵是否可以兑换立即复活|
		 * |21-30|哨兵如果兑换立即复活所需花费的金币数|
		 * |31|保留|
		 * |32|哨兵是否处于脱战|
		 * |33-43|队伍小弹丸剩余可兑换数|
		 * |44-45|哨兵当前姿态，1为进攻，2为防御，3为移动|
		 * |46|己方能力机关是否能够进入正在激活状态|
		 * |47|保留|
		 */
		uint8_t sentry_info[6];
	} sentry_info_rec;

	//机器人间交互数据
	//1. 交互数据接收信息：0x0301。
	 struct
	{
		struct
		{
			uint8_t SOF;	//0xA5
			uint8_t data_length[2];	//定义成uint16_t会莫名其妙多一个位，神奇BUG
			uint8_t seq;	
			uint8_t CRC8;
		}header;
		uint16_t cmd_id;
		uint16_t data_cmd_id;
		uint16_t sender_ID;
		uint16_t receiver_ID;
		uint8_t data[4];
		uint8_t CRC16[2];
	}robot_interactive_data;

	 struct //0x0301 客户端内容id 0xd180
	{
		uint8_t data[30];
		uint16_t data_cmd_id;
		uint16_t sender_ID;
		uint16_t receiver_ID;
		float lidar_station_x;
		float lidar_station_y;
		uint8_t lidar_id;
	}ext_student_interactive_header_data;
	//0x303 云台手小地图数据
	struct 
	{ 
		float target_position_x; 
		float target_position_y; 
		uint8_t cmd_keyboard; 
		uint8_t target_robot_id; 
		uint8_t cmd_source; 
	}map_command_t; 
	struct
	{
		uint32_t decision_data;
		uint8_t resurrection;
		uint16_t shoot_num;
		uint8_t target_posture;
		uint8_t open_energy_device;
	}sentry_decision;
}Referee2022;

		
extern Referee2022 referee2022;
extern uint16_t bullet_num_17mm;
extern uint8_t meta_data[BSP_USART3_DMA_RX_BUF_LEN];
extern uint8_t sentry_shooting_time;

unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Referee_Data_Diapcak(uint8_t *data,uint8_t this_time_len);
void sentry_send_meseage(void);
void sentry_decision_control(void);
void Judege_reverge(void);
void Sentry_multiMachineInteraction(void);
uint8_t Referee_callback(uint8_t * recBuffer, uint16_t len);
#endif
