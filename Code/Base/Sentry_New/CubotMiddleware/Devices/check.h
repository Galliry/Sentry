#ifndef _CHECK_H_
#define _CHECK_H_
#include "stdint.h"
#include "et08.h"
#include "dr16.h"
typedef struct
{
	struct
	{
		uint8_t Check_receiver;//接收机
		uint8_t Check_referee;//裁判系统
		uint8_t Check_cap;//超电
		uint8_t Check_board;

		uint16_t Check_receiver_cnt;//接收机
		uint16_t Check_referee_cnt;//裁判系统
		uint16_t Check_cap_cnt;//超电
		uint16_t Check_board_cnt;
	}Check_Usart;
}Check_Robot_State;

typedef struct
{
	uint16_t Vision_FPS;
	uint16_t Lidar_FPS;
	uint16_t Receiver_FPS;//遥控器
	uint16_t Referee_FPS;
	uint16_t Gyro_FPS;
	uint16_t Camera_FPS;

	uint16_t Receiver_cnt;
	uint16_t Referee_cnt;
	uint16_t Vision_cnt;
	uint16_t Lidar_cnt;
	uint16_t Gyro_cnt;
	uint16_t Camera_cnt;
}FPS;

extern Check_Robot_State check_robot_state;
extern FPS tim14_FPS;
void RobotOnlineState(Check_Robot_State* CheckRobotState,RC_Ctrl_ET* rc_ctrl,RC_Ctrl* rc_ctrl_dr16);
void FPS_Check(FPS * fps);
void Motor_CheckFPS(void);
#endif
