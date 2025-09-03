#include "referee.h"
#include "driver_usart.h"
#include "check.h"
#include "interaction.h"
#include "string.h"

Referee2022  referee2022;//已更新至2024赛季 版本V1.6.1
/************************************************************哨兵决策指令，还没封装，有空再说******************************************************************/
uint32_t sentry_decision;
uint16_t recieve_time;
uint8_t sentry_respawn_ok_flag=0;;
uint8_t sentry_respawn_need;
uint8_t sentry_respawn_ins_flag=0;
uint16_t sentry_shooting_num=0;
uint16_t sentry_shooting_num1;
uint16_t sentry_shooting_num_syn;//发弹量决策信息同步
uint8_t sentry_shooting_time=0;
uint8_t sentry_shooting_time1;
uint8_t sentry_shooting_time_syn;//兑换发弹次数决策信息同步
uint8_t sentry_recover_time=0;
uint8_t sentry_recover_time_syn;//兑换血量次数决策信息同步
uint16_t sentry_blood=400;
float lidar_station_x;
float lidar_station_y;
uint8_t lidar_station_id;
uint16_t bullet_num_17mm;
uint8_t temp[15];
//uint8_t referee_cnt;
/**
  * @brief   串口3裁判系统回调函数 
  * @param[in]  
  */
uint8_t Referee_callback(uint8_t * recBuffer, uint16_t len)
{
	check_robot_state.Check_Usart.Check_referee_cnt=0;
	tim14_FPS.Referee_cnt++;
	Referee_Data_Diapcak(recBuffer,len);
	return 0;
}

int blue_3=0;
/******************************************************************
函数名；_Data_Diapcak
功能：  裁判系统数据解算+映射到对应结构体成员
参数；  uint8_t *pdata
返回值：无
上级函数：Referee_Data_Diapcak
******************************************************************/
void _Data_Diapcak(uint8_t *pdata)
{
	uint16_t cmd_id;
	cmd_id=*(pdata+6)<<8|*(pdata+5); 

	/**************************************************************读取裁判系统比赛状态，比赛正式开始之后再巡航***********************/
			if(cmd_id==0x0001)
	{
		referee2022.game_status.game_type = (*(pdata + data_addr ))& 0x0F;
		referee2022.game_status.game_progress = (*(pdata+data_addr )) >>4;
		referee2022.game_status.stage_remain_time = *(pdata+data_addr+2)<<8|*(pdata+data_addr+1);
//		referee_cnt++;
	}
	
		if(cmd_id==0x0003)
	{
		for (int i=1;i<=4;i++)
		{BYTE0(referee2022.game_robot_hp.red_robot_HP[i]) = *(pdata+data_addr+2*i-2);
		BYTE1(referee2022.game_robot_hp.red_robot_HP[i]) = *(pdata+data_addr +2*i-1);}


		BYTE0(referee2022.game_robot_hp.red_robot_HP[7]) = *(pdata+data_addr + 10);
		BYTE1(referee2022.game_robot_hp.red_robot_HP[7]) = *(pdata+data_addr + 11);

		BYTE0(referee2022.game_robot_hp.red_outpost_HP) = *(pdata+data_addr + 12);
		BYTE1(referee2022.game_robot_hp.red_outpost_HP) = *(pdata+data_addr + 13);

		BYTE0(referee2022.game_robot_hp.red_base_HP) = *(pdata+data_addr + 14);
		BYTE1(referee2022.game_robot_hp.red_base_HP) = *(pdata+data_addr + 15);
		
		for (int i=1;i<=4;i++)
		{BYTE0(referee2022.game_robot_hp.blue_robot_HP[i]) = *(pdata+data_addr +2*i+14);
		BYTE1(referee2022.game_robot_hp.blue_robot_HP[i]) = *(pdata+data_addr +2*i+15);}

		BYTE0(referee2022.game_robot_hp.blue_robot_HP[7]) = *(pdata+data_addr + 26);
		BYTE1(referee2022.game_robot_hp.blue_robot_HP[7]) = *(pdata+data_addr + 27);
		
		BYTE0(referee2022.game_robot_hp.blue_outpost_HP) = *(pdata+data_addr + 28);
		BYTE1(referee2022.game_robot_hp.blue_outpost_HP) = *(pdata+data_addr + 29);

		BYTE0(referee2022.game_robot_hp.blue_base_HP) = *(pdata+data_addr + 30);
		BYTE1(referee2022.game_robot_hp.blue_base_HP) = *(pdata+data_addr + 31);
		
		for (int i=0;i<8;i++)
		
		{
		if (referee2022.game_robot_hp.blue_robot_HP_last[i]>0&&referee2022.game_robot_hp.blue_robot_HP[i]==0) referee2022.game_robot_hp.blue_robot_revge[i]=0;
		if (referee2022.game_robot_hp.blue_robot_HP_last[i]==0&&referee2022.game_robot_hp.blue_robot_HP[i]>0 &&  referee2022.game_robot_hp.blue_robot_revge[i]==0) referee2022.game_robot_hp.blue_robot_revge[i]=2;
		
		if (referee2022.game_robot_hp.red_robot_HP_last[i]>0&&referee2022.game_robot_hp.red_robot_HP[i]==0) referee2022.game_robot_hp.red_robot_revge[i]=0;
		if (referee2022.game_robot_hp.red_robot_HP_last[i]==0&&referee2022.game_robot_hp.red_robot_HP[i]>0 &&  referee2022.game_robot_hp.red_robot_revge[i]==0) referee2022.game_robot_hp.red_robot_revge[i]=2;
		
		referee2022.game_robot_hp.blue_robot_HP_last[i]=referee2022.game_robot_hp.blue_robot_HP[i];
		referee2022.game_robot_hp.red_robot_HP_last[i]=referee2022.game_robot_hp.red_robot_HP[i];
		}				
	}	
		
	if(cmd_id==0x0101)
	{
		referee2022.event_data.event_type = *(pdata + data_addr );
		//chassis_motor.game1=event_data.event_type >>10;
	}
	/********************************************************************************************************************************/
	if(cmd_id==0x201)
	{
		referee2022.game_robot_status.robot_id = *(pdata + data_addr);
		referee2022.game_robot_status.robot_level = *(pdata+data_addr + 1);

		BYTE0(referee2022.game_robot_status.remain_HP) = *(pdata+data_addr + 2);
		BYTE1(referee2022.game_robot_status.remain_HP) = *(pdata+data_addr + 3);

		BYTE0(referee2022.game_robot_status.max_HP) = *(pdata+data_addr + 4);
		BYTE1(referee2022.game_robot_status.max_HP) = *(pdata+data_addr + 5);

		BYTE0(referee2022.game_robot_status.shooter_id1_17mm_cooling_rate) = *(pdata+data_addr + 6);
		BYTE1(referee2022.game_robot_status.shooter_id1_17mm_cooling_rate) = *(pdata+data_addr + 7);

		BYTE0(referee2022.game_robot_status.shooter_id1_17mm_cooling_limit) = *(pdata+data_addr + 8);
		BYTE1(referee2022.game_robot_status.shooter_id1_17mm_cooling_limit) = *(pdata+data_addr + 9);

		BYTE0(referee2022.game_robot_status.chassis_power_limit) = *(pdata+data_addr + 10);
		BYTE1(referee2022.game_robot_status.chassis_power_limit) = *(pdata+data_addr + 11);
//		
		referee2022.game_robot_status.mains_power_gimbal_output = (*(pdata + data_addr + 12)) & 0x01 ;// 通过板间通讯发给下云台 云台上电情况  裁判系统的bit是从右往左数
		referee2022.game_robot_status.mains_power_chassis_output = (*(pdata + data_addr + 12))>>1; //& 0x02 ;
		referee2022.game_robot_status.mains_power_shooter_output = (*(pdata + data_addr + 12))>>2; //& 0x04 ;
	}
	
	if(cmd_id==0x202)
	{
		BYTE0(referee2022.power_heat_data.chassis_volt) = *(pdata + data_addr);
		BYTE1(referee2022.power_heat_data.chassis_volt) = *(pdata + data_addr + 1);

		BYTE0(referee2022.power_heat_data.chassis_current) = *(pdata + data_addr + 2);
		BYTE1(referee2022.power_heat_data.chassis_current) = *(pdata + data_addr + 3);

		BYTE0(referee2022.power_heat_data.chassis_power) = *(pdata + data_addr + 4);
		BYTE1(referee2022.power_heat_data.chassis_power) = *(pdata + data_addr + 5);
		BYTE2(referee2022.power_heat_data.chassis_power) = *(pdata + data_addr + 6);
		BYTE3(referee2022.power_heat_data.chassis_power) = *(pdata + data_addr + 7);

		BYTE0(referee2022.power_heat_data.chassis_power_buffer) = *(pdata + data_addr + 8);
		BYTE1(referee2022.power_heat_data.chassis_power_buffer) = *(pdata + data_addr + 9);

		BYTE0(referee2022.power_heat_data.shooter_id1_17mm_cooling_heat) = *(pdata + data_addr + 10);//17毫米枪口
		BYTE1(referee2022.power_heat_data.shooter_id1_17mm_cooling_heat) = *(pdata + data_addr + 11);
//		
		BYTE0(referee2022.power_heat_data.shooter_heat1) = *(pdata + data_addr + 12);//机动17毫米枪口
		BYTE1(referee2022.power_heat_data.shooter_heat1) = *(pdata + data_addr + 13);

//		BYTE0(power_heat_data.shooter_id1_42mm_cooling_heat) = *(pdata + data_addr + 14);//42毫米枪口
//		BYTE1(power_heat_data.shooter_id1_42mm_cooling_heat) = *(pdata + data_addr + 15);
		referee2022.power_heat_data.renewchassis_power=1;	
		referee2022.power_heat_data.renewchassis_power_buffer=1;
		referee2022.power_heat_data.renewshooter_heat0=1;
	}			
	if(cmd_id==0x204)
	{
		referee2022.buff.recovery_buff=  *(pdata + data_addr);
		referee2022.buff.cooling_buff =  *(pdata + data_addr + 1);
		referee2022.buff.defence_buff =  *(pdata + data_addr + 2);
		referee2022.buff.vulnerability_buff=*(pdata + data_addr + 3);
		BYTE0(referee2022.buff.attack_buff)=*(pdata + data_addr + 4);
		BYTE1(referee2022.buff.attack_buff)=*(pdata + data_addr + 5);
		referee2022.buff.remaining_energy=*(pdata + data_addr + 6);
		

	}
//	if(cmd_id==0x0205)
//	{	
//		aerial_robot_energy.attack_time=*(pdata+data_addr + 1);		 //剩余时间s		
//	}
//	
		if(cmd_id==0x0206)
	{	
		referee2022.robot_hurt.hurt_type=(*(pdata + data_addr ))>>4;
		referee2022.robot_hurt.armor_id=(*(pdata + data_addr ))& 0x0F;		 // 新增 不再由血量变化判断挨打 读取受击打装甲板ID 为0时为未被击打
		referee2022.robot_hurt.armor_id+=1;
	}
	
	if(cmd_id==0x0207)
	{
		
		referee2022.shoot_data.bullet_type=*(pdata+data_addr);
		referee2022.shoot_data.shooter_id =	*(pdata+data_addr + 1);		
		referee2022.shoot_data.bullet_freq=*(pdata+data_addr + 2);

		BYTE0(referee2022.shoot_data.bullet_speed) = *(pdata+data_addr + 3);
		BYTE1(referee2022.shoot_data.bullet_speed) = *(pdata+data_addr + 4);
		BYTE2(referee2022.shoot_data.bullet_speed) = *(pdata+data_addr + 5);
		BYTE3(referee2022.shoot_data.bullet_speed) = *(pdata+data_addr + 6);
//		if (Brain.Autoaim.Mode!=Outpost) {bullet_num_17mm++;Brain.Autoaim.Attack_state.shoot_num++;}
	}
	if(cmd_id==0x0208)
	{
		BYTE0(referee2022.bullet_remaining.bullet_remaining_num) = *(pdata+data_addr);
		BYTE1(referee2022.bullet_remaining.bullet_remaining_num) = *(pdata+data_addr + 1);
		
	//	BYTE0(referee2022.bullet_remaining_num_42mm) = *(pdata+data_addr + 2);
	//	BYTE1(referee2022.bullet_remaining_num_42mm) = *(pdata+data_addr + 3);

	}
	
	if(cmd_id==0x020D)
	{
//		BYTE0(referee2022.sentry_info_t.sentry_info)=*(pdata+data_addr);
//		BYTE1(referee2022.sentry_info_t.sentry_info)=*(pdata+data_addr+1);
//		BYTE2(referee2022.sentry_info_t.sentry_info)=*(pdata+data_addr+2);
//		BYTE3(referee2022.sentry_info_t.sentry_info)=*(pdata+data_addr+3);
//		
//		//对同步信息进行处理
//		sentry_shooting_num_syn=referee2022.sentry_info_t.sentry_info&0x3FF;
//		sentry_shooting_time_syn=(referee2022.sentry_info_t.sentry_info>>10)&0x0F;
//		sentry_recover_time_syn=(referee2022.sentry_info_t.sentry_info>>14)&0x0F;vg
	}
	
	if(cmd_id == 0x0301)//雷达站通讯
	{
		BYTE0(referee2022.ext_student_interactive_header_data.data_cmd_id) = *(pdata+data_addr);
		BYTE1(referee2022.ext_student_interactive_header_data.data_cmd_id) = *(pdata+data_addr + 1);

		BYTE0(referee2022.ext_student_interactive_header_data.sender_ID) = *(pdata+data_addr + 2);
		BYTE1(referee2022.ext_student_interactive_header_data.sender_ID) = *(pdata+data_addr + 3);

		BYTE0(referee2022.ext_student_interactive_header_data.receiver_ID) = *(pdata+data_addr + 4);
		BYTE1(referee2022.ext_student_interactive_header_data.receiver_ID) = *(pdata+data_addr + 5);
		// 颠倒 为什么？  问学长   和雷达通信本质是通过裁判系统
		
		referee2022.ext_student_interactive_header_data.data[0] = *(pdata+data_addr + 6);
		referee2022.ext_student_interactive_header_data.data[1] = *(pdata+data_addr + 7);
		referee2022.ext_student_interactive_header_data.data[2] = *(pdata+data_addr + 8);
		referee2022.ext_student_interactive_header_data.data[3] = *(pdata+data_addr + 9);
		referee2022.ext_student_interactive_header_data.data[4] = *(pdata+data_addr + 10);
		
		lidar_station_id=referee2022.ext_student_interactive_header_data.data[0];
		lidar_station_x=(referee2022.ext_student_interactive_header_data.data[1]+(float)(referee2022.ext_student_interactive_header_data.data[2])/100);
		lidar_station_y=(referee2022.ext_student_interactive_header_data.data[3]+(float)(referee2022.ext_student_interactive_header_data.data[4])/100);
		
//		referee2022.ext_student_interactive_header_data.data[5] = *(pdata+data_addr + 11);
//		referee2022.ext_student_interactive_header_data.data[6] = *(pdata+data_addr + 12);
//		
//		if(ext_student_interactive_header_data.receiver_ID == 9 && ext_student_interactive_header_data.sender_ID == 7)
//		RadarPackage(pdata+data_addr);
		
	}
	   if(cmd_id==0x0303)
		{

			BYTE0(referee2022.map_command_t.target_position_x)=*(pdata+data_addr);
			BYTE1(referee2022.map_command_t.target_position_x)=*(pdata+data_addr+1);
			BYTE2(referee2022.map_command_t.target_position_x)=*(pdata+data_addr+2);
			BYTE3(referee2022.map_command_t.target_position_x)=*(pdata+data_addr+3);
			
			BYTE0(referee2022.map_command_t.target_position_y)=*(pdata+data_addr+4);
			BYTE1(referee2022.map_command_t.target_position_y)=*(pdata+data_addr+5);
			BYTE2(referee2022.map_command_t.target_position_y)=*(pdata+data_addr+6);
			BYTE3(referee2022.map_command_t.target_position_y)=*(pdata+data_addr+7);
			
			referee2022.map_command_t.cmd_keyboard=*(pdata+data_addr+8);
			

			
		}
}

//uint8_t ref_packge[packs][max_single_pack_len];  //最多一次收10个包
uint8_t ref_packge[packs][max_single_pack_len];//__attribute__((at(0x24008000)));  //最多一次收10个包

//只处理了连续帧，没处理断帧
/******************************************************************
函数名；Referee_Data_Diapcak
功能：  裁判系统数据验证＋解算
参数；  uint8_t *data,uint8_t this_time_len
返回值：无
下级函数：_Data_Diapcak
上级函数：USART3_IDLE_CALLBACK
******************************************************************/
uint8_t index_i = 0 ;
uint8_t max_i=0;//用于在debug中观察
void Referee_Data_Diapcak(uint8_t *data,uint8_t this_time_len)
{
	uint32_t Verify_CRC8_OK;
	uint32_t Verify_CRC16_OK;
	uint8_t i = 0;
	uint8_t pack_size=0;
	do {
		if( *data==0xA5)//判断帧头
		{
			referee2022.frame_info.head.sof=*data;
			referee2022.frame_info.head.data_len=*(data+2)| *(data+1);
			referee2022.frame_info.head.seq=*(data+3);//包序号
			referee2022.frame_info.head.crc8=*(data+4); 				
			referee2022.frame_info.cmd_id=*(data+6)<<8|*(data+5); 
			Verify_CRC8_OK=Verify_CRC8_Check_Sum(data, frame_header_len);
			Verify_CRC16_OK= Verify_CRC16_Check_Sum(data,pack_len);	

			if((Verify_CRC8_OK==1)&&(Verify_CRC16_OK==1))  //校验通过
			{
				memcpy(ref_packge[i],  data, 7+referee2022.frame_info.head.data_len+2);	
				_Data_Diapcak(ref_packge[i]);					
				i++;
			}
			
		//	check_robot_state.usart_state.Check_referee = 0;
		}
		data += (7+referee2022.frame_info.head.data_len+2);
		pack_size += (7+referee2022.frame_info.head.data_len+2);
		
	}while(pack_size < this_time_len);
	index_i = i;
	if (index_i>max_i) max_i=index_i;//用于在debug中观察
}



//crc8 generator polynomial:G(x)=x8+x5+x4+x0  //1 0011000 1(一共9个bit)  0x31 
const unsigned char CRC8_INIT = 0xff;
const unsigned char CRC8_TAB[256] =//8位最多256个
{
	0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
	0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
	0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
	0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
	0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
	0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
	0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
	0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
	0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
	0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
	0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
	0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
	0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
	0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
	0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
	0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};


/**
  * @brief 取得大疆官方crc8检验码
  */
unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8)
{
	unsigned char ucIndex;                     //与0异或保持不变，与1异或反转
	while (dwLength--)   //ucCRC8是什么??
	{
		ucIndex = ucCRC8^(*pchMessage++);//第一次是取反?? 
		ucCRC8 = CRC8_TAB[ucIndex];//余式表             
	}                                                   
	return(ucCRC8);
}

/**
  * @brief 验证大疆官方crc8检验码
  */
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)  //Verify 验证
{
	unsigned char ucExpected = 0;
	if ((pchMessage == 0) || (dwLength <= 2)) return 0;
	ucExpected = Get_CRC8_Check_Sum (pchMessage, dwLength-1, CRC8_INIT);
	return ( ucExpected == pchMessage[dwLength-1] );
}

/**
  * @brief 附加大疆官方crc8检验码
  */
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength) //Append 附加
{
	unsigned char ucCRC = 0;
	if ((pchMessage == 0) || (dwLength <= 2)) 
	{
		return;
	}
	ucCRC = Get_CRC8_Check_Sum ( (unsigned char *)pchMessage, dwLength-1, CRC8_INIT);
	pchMessage[dwLength-1] = ucCRC;
}

uint16_t CRC_INIT = 0xffff;
const uint16_t wCRC_Table[256] =
{
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
	0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
	0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
	0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
	0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/**
  * @brief 取得大疆官方crc16检验码
  */
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC)
{
	uint8_t chData;
	if (pchMessage == NULL)//无效地址
	{
		return 0xFFFF;
	}
	while(dwLength--)
	{
		chData = *pchMessage++;
		(wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) &0x00ff];
	}
	return wCRC;
}

/*
** Descriptions: CRC16 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
/**
  * @brief 验证大疆官方crc16检验码
  */
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
	uint16_t wExpected = 0;
	if ((pchMessage == NULL) || (dwLength <= 2))
	{
	return 0;//FALSE;
	}
	wExpected = Get_CRC16_Check_Sum ( pchMessage, dwLength - 2, CRC_INIT);
	return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) ==
	pchMessage[dwLength - 1]);
}

/**
  * @brief 附加大疆官方crc16检验码
  */
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength)
{
	uint16_t wCRC = 0;
	if ((pchMessage == NULL) || (dwLength <= 2))
	{
		return;
	}
	wCRC = Get_CRC16_Check_Sum ( (uint8_t *)pchMessage, dwLength-2, CRC_INIT );
	pchMessage[dwLength-2] = (uint8_t)(wCRC & 0x00ff);
	pchMessage[dwLength-1] = (uint8_t)((wCRC >> 8)& 0x00ff);
}

void sentry_decision_control()//复活，买弹逻辑
{
	referee2022.sentry_info_t.sentry_respawn_flag = (referee2022.game_robot_status.remain_HP == 0) ? 1 : 0;
//	if (referee2022.bullet_remaining.bullet_remaining_num<=50)
//	referee2022.sentry_info_t.sentry_shooting_num+=100;

	sentry_decision=referee2022.sentry_info_t.sentry_respawn_flag+referee2022.sentry_info_t.sentry_shooting_num*4+referee2022.sentry_info_t.sentry_shooting_num_far*4096;
	sentry_send_meseage();
}
	
uint8_t Data_Pack[50];
void sentry_send_meseage()//上发哨兵决策信息
{
	
	uint16_t crc16_temp;
	
	referee2022.robot_interactive_data.header.SOF=0xA5;	
	referee2022.robot_interactive_data.header.data_length[0]=0x0A;
	referee2022.robot_interactive_data.header.data_length[1]=0x00;
	referee2022.robot_interactive_data.header.seq=0;//包序号？
	referee2022.robot_interactive_data.header.CRC8=Get_CRC8_Check_Sum((unsigned char*)&referee2022.robot_interactive_data.header,4,0xFF);
	
	referee2022.robot_interactive_data.cmd_id[1]=0x03;
	referee2022.robot_interactive_data.cmd_id[0]=0x01;
	
	referee2022.robot_interactive_data.data_cmd_id[1]=0x01;
	referee2022.robot_interactive_data.data_cmd_id[0]=0x20;
	if(referee2022.game_robot_status.robot_id==0x07){
	referee2022.robot_interactive_data.sender_ID[0]=0x07;
	referee2022.robot_interactive_data.sender_ID[1]=0x00;}
	else if(referee2022.game_robot_status.robot_id==0x6b){
	referee2022.robot_interactive_data.sender_ID[0]=0x6b;
	referee2022.robot_interactive_data.sender_ID[1]=0x00;}
	referee2022.robot_interactive_data.receiver_ID[0]=0x80;
	referee2022.robot_interactive_data.receiver_ID[1]=0x80;//裁判系统服务器ID
	
	referee2022.robot_interactive_data.data[0]=sentry_decision&0xff;
	referee2022.robot_interactive_data.data[1]=(sentry_decision>>8)&0xff;
	referee2022.robot_interactive_data.data[2]=(sentry_decision>>16)&0xff;
	referee2022.robot_interactive_data.data[3]=(sentry_decision>>24)&0xff;
	
	crc16_temp=Get_CRC16_Check_Sum((unsigned char*)&referee2022.robot_interactive_data, 17, 0xFFFF);
	
	referee2022.robot_interactive_data.CRC16[0]=crc16_temp&0xff;
	referee2022.robot_interactive_data.CRC16[1]=crc16_temp>>8;
	//a=sizeof(referee2022.robot_interactive_data);
	memcpy(Data_Pack,(unsigned char*)&referee2022.robot_interactive_data,sizeof(referee2022.robot_interactive_data));
	
	HAL_UART_Transmit_DMA(&huart3,Data_Pack,19);
	
}

void Judege_reverge()
{
	static uint16_t cnt[16];
	for (int i=0;i<8;i++)
	{
		if  (referee2022.game_robot_hp.blue_robot_revge[i]==2) cnt[i]++;if (cnt[i]>10000) {referee2022.game_robot_hp.blue_robot_revge[i]=1;cnt[i]=0;}
		if  (referee2022.game_robot_hp.red_robot_revge[i]==2) cnt[i+8]++;if (cnt[i+8]>10000) {referee2022.game_robot_hp.red_robot_revge[i]=1;cnt[i+8]=0;}
	}
	
}
void Sentry_multiMachineInteraction(void)
{
	uint16_t crc16_temp;
	
	referee2022.robot_interactive_data.header.SOF=0xA5;	
	referee2022.robot_interactive_data.header.data_length[0]=0x0A;
	referee2022.robot_interactive_data.header.data_length[1]=0x00;
	referee2022.robot_interactive_data.header.seq=0;//包序号?
	referee2022.robot_interactive_data.header.CRC8=Get_CRC8_Check_Sum((unsigned char*)&referee2022.robot_interactive_data.header,4,0xFF);
	
	referee2022.robot_interactive_data.cmd_id[1]=0x03;
	referee2022.robot_interactive_data.cmd_id[0]=0x01;
	
	referee2022.robot_interactive_data.data_cmd_id[1]=0x02;
	referee2022.robot_interactive_data.data_cmd_id[0]=0x00;
	
	if(referee2022.game_robot_status.robot_id==0x07){
	referee2022.robot_interactive_data.sender_ID[0]=0x07;
	referee2022.robot_interactive_data.sender_ID[1]=0x00;}
	else if(referee2022.game_robot_status.robot_id==0x6b){
	referee2022.robot_interactive_data.sender_ID[0]=0x6b;
	referee2022.robot_interactive_data.sender_ID[1]=0x00;}
	
	referee2022.robot_interactive_data.receiver_ID[0]=0x65;
	referee2022.robot_interactive_data.receiver_ID[1]=0x00;
	
	referee2022.robot_interactive_data.data[0]=0x01;
	referee2022.robot_interactive_data.data[1]=0x01;
	referee2022.robot_interactive_data.data[2]=0x01;
	referee2022.robot_interactive_data.data[3]=0x01;
	
	crc16_temp=Get_CRC16_Check_Sum((unsigned char*)&referee2022.robot_interactive_data, 17, 0xFFFF);
	referee2022.robot_interactive_data.CRC16[0]=crc16_temp&0xff;
	referee2022.robot_interactive_data.CRC16[1]=crc16_temp>>8;
	
	//a=sizeof(referee2022.robot_interactive_data);
	
	memcpy(Data_Pack,(unsigned char*)&referee2022.robot_interactive_data,sizeof(referee2022.robot_interactive_data));
	
	HAL_UART_Transmit_DMA(&huart3,Data_Pack,19);

}
