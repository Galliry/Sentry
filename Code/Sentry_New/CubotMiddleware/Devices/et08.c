	/**@file  et08.c
	* @brief    设备层
	* @details  主要包括构建串口管理器，提供串口初始化和用户回调重定义
	* @author      RyanJiao  any question please send mail to 1095981200@qq.com
	* @date        2024-9-6
	* @version     V1.0
	* @copyright    Copyright (c) 2021-2121  中国矿业大学CUBOT战队
	**********************************************************************************
	* @attention
	* 硬件平台: STM32H750VBT \n
	* SDK版本：-++++
	* @par 修改日志:
	* <table>
	* <tr><th>Date        <th>Version  <th>Author    <th>Description
	* <tr><td>2021-8-12  <td>1.0      <td>RyanJiao  <td>创建初始版本
	* </table>
	*
	**********************************************************************************
	 ==============================================================================
														How to use this driver  
	 ==============================================================================
	 

		********************************************************************************
		* @attention
		* 硬件平台: STM32H750VBT \n
		* SDK版本：-++++
		* if you had modified this file, please make sure your code does not have many 
		* bugs, update the version NO., write dowm your name and the date, the most
		* important is make sure the users will have clear and definite understanding 
		* through your new brief.
		********************************************************************************
	*/

	#include "et08.h"
	#include "hardware_config.h"
	#include "check.h"
	/**
		* @brief 对全局变量rc_Ctrl赋值，以供其他函数调用
		*/
	RC_Ctrl_ET rc_Ctrl_et={
		.isUnpackaging = 0,
		.isOnline = 0
	};


	/**
		* @brief  初始化接收机数据类型的数据，将杆量和按键信息归零
		*/
	void ET08Init(RC_Ctrl_ET* RC_Ctl)
	{
		RC_Ctl->rc.ch0=1024;
		RC_Ctl->rc.ch1=1024;
		RC_Ctl->rc.ch2=1024;
		RC_Ctl->rc.ch3=1024;
		RC_Ctl->rc.sA=0;
		RC_Ctl->rc.s1=3;
		RC_Ctl->rc.s2=3;
		RC_Ctl->rc.sD=0;
	}


	/**
		* @brief  创建et08的回调函数
		*/
	uint8_t ET08_callback(uint8_t * recBuffer, uint16_t len)
	{
		
		ET08_DataUnpack(&rc_Ctrl_et, recBuffer, len);  //< callback函数由格式限制

		return 0;
	}
/**
		* @brief  遥控保护
		*/
void ET08_online_protection()
	{
		if(rc_Ctrl_et.rc.sA==0)
			rc_Ctrl_et.isOnline=0;
		else
			rc_Ctrl_et.isOnline=1;
	}

	
	
	void ET08_DataUnpack(RC_Ctrl_ET* rc_ctrl, uint8_t * recBuffer, uint16_t len )
	{ 
		uint8_t SA,SB,SC,SD;
		if(recBuffer[0]==0x0f)																																												//< 数据完整性验证 
		{
		check_robot_state.Check_Usart.Check_receiver_cnt=0;
		tim14_FPS.Receiver_cnt++;
			rc_ctrl->rc.ch0 = (recBuffer[1]>>0| (recBuffer[2] << 8)) & 0x07ff; 																					//< Channel 0   高8位与低3位
			rc_ctrl->rc.ch1 = ((recBuffer[2] >> 3) | (recBuffer[3] << 5)) & 0x07ff; 																	//< Channel 1   高5位与低6位
			rc_ctrl->rc.ch2 = ((recBuffer[3] >> 6) | (recBuffer[4] << 2) |(recBuffer[5] << 10)) & 0x07ff; 						//< Channel 2
			rc_ctrl->rc.ch3 = ((recBuffer[5] >> 1) | (recBuffer[6] << 7)) & 0x07ff; 																	//< Channel 3
			SD = ((recBuffer[6] >> 4)|(recBuffer[7] << 4)) & 0x7ff; 																											//!< Switch left
			SA = ((recBuffer[7] >> 7)|(recBuffer[8] << 1)|(recBuffer[9] << 9)) & 0x7ff;																											//!< Switch right
			SB=  ((recBuffer[9] >> 2)|(recBuffer[10] << 6)) & 0x7ff;
			SC=  ((recBuffer[10] >> 5)|(recBuffer[11] << 3)) & 0x7ff;
			
			if((rc_ctrl->rc.ch0>990)&&(rc_ctrl->rc.ch0<1050))          //遥控器零飘
				rc_ctrl->rc.ch0=1024;
			if((rc_ctrl->rc.ch1>990)&&(rc_ctrl->rc.ch1<1050))
				rc_ctrl->rc.ch1=1024;
			if((rc_ctrl->rc.ch2>990)&&(rc_ctrl->rc.ch2<1050))
				rc_ctrl->rc.ch2=1024;
			if((rc_ctrl->rc.ch3>990)&&(rc_ctrl->rc.ch3<1050))
				rc_ctrl->rc.ch3=1024;
            if(SB==0)
			rc_ctrl->rc.s1 = 3;	
			else if(SB==158)
			rc_ctrl->rc.s1 = 2;	
			else if(SB==97)
			rc_ctrl->rc.s1 = 1;	
			if(SC==0)
			rc_ctrl->rc.s2 = 3;	
			else if(SC==158)
			rc_ctrl->rc.s2 = 2;	
			else if(SC==97)
			rc_ctrl->rc.s2 = 1;	
			
			if(SA==158)
			rc_ctrl->rc.sA = 0;	
			else if(SA==97)
			rc_ctrl->rc.sA = 1;
			}
		}

