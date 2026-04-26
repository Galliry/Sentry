#include "brain.h"
Brain_t Brain;
uint8_t RobotToBrainTimeBuffer[50];
uint8_t RobotToBrainChassisTimeBuffer[22];

uint8_t Brain_Autoaim_Callback(uint8_t * recBuffer, uint16_t len)
{
	check_robot_state.Check_Usart.Check_vision_cnt = 0;
	Brain_Autoaim_DataUnpack(&Brain,recBuffer);
	return 0;
}

uint8_t Brain_Lidar_Callback(uint8_t * recBuffer, uint16_t len)
{
	check_robot_state.Check_Usart.Check_lidar_cnt = 0;
	Brain_Lidar_DataUnpack(&Brain,recBuffer);
	return 0;
}

void Brain_Autoaim_DataUnpack(Brain_t* brain ,uint8_t * recBuffer)
{
	if (recBuffer[0] == 0xAA)
	{
		brain->Autoaim.Brain_Data.FrameType = recBuffer[1];
		brain->Autoaim.Brain_Data.FrameCoreID = recBuffer[2];

		if ((brain->Autoaim.Brain_Data.FrameType == 1) && recBuffer[11] == 0xDD ) //< 解算偏转角
		{
			brain->Autoaim.mode_cnt = 0;
			brain->Autoaim.mode = Lock;
			
			brain->Autoaim.Yaw_add = ((recBuffer[3] >> 6) == 0 ? 1 : -1) * ((float)((recBuffer[3] & 0x3f) * 100 + recBuffer[4]) / 100);
			brain->Autoaim.Pitch_add = ((recBuffer[5] >> 6) == 0 ? 1 : -1) * ((float)((recBuffer[5] & 0x3f) * 100 + recBuffer[6]) / 100);
			brain->Autoaim.Distance = (float)(recBuffer[7]) / 10;
			
			if(rc_Ctrl_et.rc.s2 == 2)
			{
				Holder.Yaw_S.Target_Angle = -Brain.Autoaim.Yaw_add + Holder.Yaw_S.Can_Angle;
				Holder.Pitch.Target_Angle = Brain.Autoaim.Pitch_add * 1.15f + Holder.Pitch.GYRO_Angle;
			}
			
			if(ABS(Holder.Yaw_S.Target_Angle -Holder.Yaw_S.Can_Angle) < 0.8f && ABS(Holder.Pitch.Target_Angle - Holder.Pitch.GYRO_Angle) < 0.8f)
				brain->Autoaim.IsFire = 1;
			else brain->Autoaim.IsFire = 0;
		} else
		{
			if (brain->Autoaim.mode_cnt >= 4)
			{
				
				brain->Autoaim.mode = Cruise;
				brain->Autoaim.IsFire = 0;
			}else 
			{
				brain->Autoaim.mode_cnt ++;
			}
		}

	}
}


void Brain_Lidar_DataUnpack(Brain_t* brain ,uint8_t * recBuffer)
{
	if (recBuffer[0] == 0xAA)
	{
		brain->Lidar.Brain_Data.FrameType = recBuffer[1];

		brain->Lidar.movemode = recBuffer[2];

		if (brain->Lidar.Brain_Data.FrameType == BRAIN_TO_ROBOT_CMD) //< 解算偏转角
		{
			brain->Lidar.vx = (((recBuffer[3] & 0x40) == 0) ? 1.0f : -1.0f) * (((float)((recBuffer[3] & 0x3F) * 100 + recBuffer[4]) / 100.0f)) ;
			brain->Lidar.vy = ((recBuffer[5] & 0x40) ? -1.0f : 1.0f) * ((float)((recBuffer[5] & 0x3f) * 100 + recBuffer[6]) / 100.0f) ;
			memcpy(&brain->Lidar.MyPosition_x,&recBuffer[7],sizeof(float));
			memcpy(&brain->Lidar.MyPosition_y,&recBuffer[11],sizeof(float));
		}
	}
}

void RobotToBrain_Autoaim(float yaw,Brain_t* brain)//发给自瞄
{
	int16_t tmp0, tmp1, tmp2, tmp3;

	tmp0 = (int16_t)(INS_attitude->q[0] * 30000);
	tmp1 = -(int16_t)(INS_attitude->q[1] * 30000);
	tmp2 = -(int16_t)(INS_attitude->q[2] * 30000);
	tmp3 = (int16_t)(INS_attitude->q[3] * 30000);

	RobotToBrainTimeBuffer[0] = 0xAA;
	RobotToBrainTimeBuffer[1] = 0x07;	// Type ;  //固定为0x07
	RobotToBrainTimeBuffer[2] = 0x01;	// coreID;  //目前固定为0x01
	RobotToBrainTimeBuffer[3] = 0x01;	// 索引，int16_t型
	RobotToBrainTimeBuffer[4] = 0x01;

	RobotToBrainTimeBuffer[5] = (tim14.ClockTime >> 24);	// 定时器时间，int32_t型
	RobotToBrainTimeBuffer[6] = ((tim14.ClockTime >> 16) & 0xff);
	RobotToBrainTimeBuffer[7] = ((tim14.ClockTime >> 8) & 0xff);
	RobotToBrainTimeBuffer[8] = ((tim14.ClockTime & 0xff));

	RobotToBrainTimeBuffer[9] = (Receive.Top.Referee.robot_id > 10) ? 1 : 0;

	RobotToBrainTimeBuffer[10] = tmp0 & 0xFF;	// 四元数q0，float型
	RobotToBrainTimeBuffer[11] = tmp0 >> 8;
	RobotToBrainTimeBuffer[12] = tmp1 & 0xFF;
	RobotToBrainTimeBuffer[13] = tmp1 >> 8;
	RobotToBrainTimeBuffer[14] = tmp2 & 0xFF;	// 四元数q1，float型
	RobotToBrainTimeBuffer[15] = tmp2 >> 8;
	RobotToBrainTimeBuffer[16] = tmp3 & 0xFF;
	RobotToBrainTimeBuffer[17] = tmp3 >> 8;

	RobotToBrainTimeBuffer[18] = 0x01; // 0是预测 1是跟随 4 ceres 静止或低速
	RobotToBrainTimeBuffer[19] = 0x01;

	RobotToBrainTimeBuffer[20] = 0x04; // 1 是ekf 0是shou 23 fu 4 ceres
	RobotToBrainTimeBuffer[21] = 0xDD; // 忽略装甲板

	RobotToBrainTimeBuffer[22] = 0xDD;

	HAL_UART_Transmit_DMA(&huart2, RobotToBrainTimeBuffer, 23);
}

void RobotToBrain_Lidar(Brain_t* Brain)
{
	//  x = referee2022.map_command_t.target_position_x * 100;
	//	y = referee2022.map_command_t.target_position_y * 100;
	RobotToBrainChassisTimeBuffer[0] = 0xBB;
	RobotToBrainChassisTimeBuffer[1] = Receive.Top.Referee.game_prograss;
	if ( Receive.Top.Referee.game_prograss == 3){
		RobotToBrainChassisTimeBuffer[1] = 4;
	}
	if (referee2022.game_status.game_progress == 4)
	{
		RobotToBrainChassisTimeBuffer[2] = Receive.Top.Referee.game_time & 0xff; // referee2022.game_status.stage_remain_time
		RobotToBrainChassisTimeBuffer[3] = Receive.Top.Referee.game_time >> 8;
	}
	else
	{
		RobotToBrainChassisTimeBuffer[2] = 0;
		RobotToBrainChassisTimeBuffer[3] = 0;
	}
	RobotToBrainChassisTimeBuffer[4] = Receive.Top.Referee.robot_HP & 0xff;
	RobotToBrainChassisTimeBuffer[5] = Receive.Top.Referee.robot_HP >> 8;
	RobotToBrainChassisTimeBuffer[6] = Brain->Autoaim.Rune_Flag;
	RobotToBrainChassisTimeBuffer[7] = 0x00;
	RobotToBrainChassisTimeBuffer[8] = 0xDD;
	HAL_UART_Transmit_DMA(&huart4, RobotToBrainChassisTimeBuffer, 9);

}

void RobotToBrain(Brain_t *brain)
{
	RobotToBrain_Autoaim(0.0f,brain);
	if(tim14.ClockTime % 2 == 0)
	{
		RobotToBrain_Lidar(brain);	
	}

}

