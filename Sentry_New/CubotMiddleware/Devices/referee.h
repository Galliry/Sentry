#ifndef __REFEREE_H
#define __REFEREE_H

#include "stm32h7xx_hal.h"

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
		uint16_t red_robot_HP[8];
		uint16_t red_outpost_HP;
		uint16_t red_base_HP;
		
		uint16_t blue_robot_HP[8];
		uint16_t blue_outpost_HP;
		uint16_t blue_base_HP;
		
		uint16_t red_robot_HP_last[8];
		uint8_t red_robot_Hurt[8];
		uint16_t blue_robot_HP_last[8];
		uint8_t blue_robot_Hurt[8];
		
		uint8_t red_robot_revge[8];
		uint8_t blue_robot_revge[8];	
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

	/*7. 补给站动作标识：0x0102。发送频率：动作触发后发送，发送范围：己方机器人。
	*/
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
	}  dart_remaining_time;

	/*10. 比赛机器人状态： 0x0201。 发送频率： 10Hz，发送范围：单一机器人。*/
	 struct
	{
		uint8_t robot_id;
		uint8_t robot_level;
		uint16_t remain_HP;
		uint16_t max_HP;
		uint16_t shooter_heat0_cooling_rate;
		uint16_t shooter_heat0_cooling_limit;
		uint16_t shooter_heat1_cooling_rate;
		uint16_t shooter_heat1_cooling_limit;
		uint16_t chassis_power_limit;
		uint8_t shooter_heat0_speed_limit;
		uint8_t shooter_heat1_speed_limit;
		uint8_t max_chassis_power;
		uint8_t mains_power_gimbal_output : 1;
		uint8_t mains_power_chassis_output : 1;
		uint8_t mains_power_shooter_output : 1;
		uint16_t shooter_id1_17mm_cooling_rate;
		uint16_t shooter_id1_17mm_cooling_limit;
		uint16_t shooter_id1_17mm_speed_limit;
		uint16_t shooter_id2_17mm_cooling_rate;
		uint16_t shooter_id2_17mm_cooling_limit;
		uint16_t shooter_id2_17mm_speed_limit;
	} game_robot_status;

	/*11. 实时功率热量数据： 0x0202。 发送频率： 50Hz，发送范围：单一机器人。
	*/
	 struct
	{
		uint16_t chassis_volt;
		uint16_t chassis_current;
		float chassis_power;
		uint16_t chassis_power_buffer;
		uint16_t shooter_heat0; 
		uint16_t shooter_heat1; 
		uint16_t mobile_shooter_heat2;
		uint16_t shooter_id1_17mm_cooling_heat;
		uint8_t renewchassis_power;
		uint8_t renewchassis_power_buffer;
		uint8_t renewshooter_heat0;
	} power_heat_data;

	/*12. 机器人位置： 0x0203。 发送频率： 10Hz，发送范围：单一机器人。
	*/
	struct
	{
		float x;
		float y;
		float z;
		float yaw;
	} game_robot_pos;

	/*13. 机器人增益： 0x0204。发送频率：1Hz 周期发送，发送范围：单一机器人。
	*/
	 struct
	{
		 
	  uint8_t recovery_buff;  
	  uint8_t cooling_buff;  
	  uint8_t defence_buff;  
	  uint8_t vulnerability_buff; 
	  uint16_t attack_buff; 
		uint8_t remaining_energy;
	}buff;

	/*14. 空中机器人能量状态： 0x0205。 发送频率： 10Hz
	*/
	 struct
	{
		uint16_t energy_point;
		uint8_t attack_time;
	} ext_aerial_robot_energy_t;


	/*15. 伤害状态： 0x0206。 发送频率：伤害发生后发送
	*/
	 struct
	{
		uint8_t armor_id : 4;
		uint8_t hurt_type : 4;
	} robot_hurt;

	/*16. 实时射击信息： 0x0207。 发送频率：射击后发送*/
	 struct
	{
		uint8_t bullet_type;
		uint8_t shooter_id;
		uint8_t bullet_freq;
		float bullet_speed;
	} shoot_data;

	//17. 子弹剩余发射数： 0x0208。发送频率： 1Hz 周期发送， 空中机器人以及哨兵机器人主控发送

	struct
	{
		uint16_t bullet_remaining_num;
		uint16_t money;
	} bullet_remaining;


	//18.机器人 RFID 状态：0x0209。发送频率：1Hz，发送范围：单一机器人。

	 struct
	{
		uint32_t rfid_status;
	}  rfid_status;

	struct
	{
	  uint8_t sentry_respawn_flag ;
	  uint16_t sentry_shooting_num;
	  uint8_t  sentry_shooting_num_far;
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

	//机器人间交互数据

	//1. 交互数据接收信息：0x0301。
	 struct
	{
		struct
		{
		uint8_t SOF;			    //0xA5
		uint8_t data_length[2]; //data ???//定义成uint16_t会莫名其妙多一个位，神奇BUG
		uint8_t seq;				  //???
		uint8_t CRC8;
		}header;
		uint8_t cmd_id[2];
		uint8_t data_cmd_id[2];
		uint8_t sender_ID[2];
		uint8_t receiver_ID[2];
		uint8_t data[4];
		uint8_t CRC16[2];
	} robot_interactive_data;

	 struct //0x0301 客户端内容id 0xd180
	{
		uint8_t data[30];
		uint8_t data_cmd_id[2];
		uint8_t sender_ID[2];
		uint8_t receiver_ID[2];
	} ext_student_interactive_header_data;

	struct 
	{ 
		float target_position_x; 
		float target_position_y; 
		uint8_t cmd_keyboard; 
		uint8_t target_robot_id; 
		uint8_t cmd_source; 
	}map_command_t; 

}Referee2022;

		
extern Referee2022 referee2022;
extern uint16_t bullet_num_17mm;
extern uint8_t meta_data[BSP_USART3_DMA_RX_BUF_LEN];
extern uint8_t sentry_shooting_time;
extern uint8_t referee_cnt;

unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Referee_Data_Diapcak(uint8_t *data,uint8_t this_time_len);
void sentry_send_meseage(void);
void sentry_decision_control(void);
void Judege_reverge();
void Sentry_multiMachineInteraction(void);
uint8_t Referee_callback(uint8_t * recBuffer, uint16_t len);
#endif
