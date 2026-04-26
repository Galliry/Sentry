#include "interboard.h"

CAN_TxBuffer RefereeData;
Base_t Base;

void RefereeDataTrans(Referee2022* referee)
{
	static uint8_t i = 0;
	if(i == 0)
	{
		RefereeData.Identifier = 0x104;
		RefereeData.Data[0] = referee->game_status.game_progress;
		RefereeData.Data[1] = (uint8_t)(referee->game_robot_status.shooter_id1_17mm_cooling_limit & 0xff);
		RefereeData.Data[2] = (uint8_t)((referee->game_robot_status.shooter_id1_17mm_cooling_limit >> 8) & 0xff);
		RefereeData.Data[3] = (uint8_t)(referee->power_heat_data.shooter_id1_17mm_cooling_heat & 0xff);
		RefereeData.Data[4] = (uint8_t)((referee->power_heat_data.shooter_id1_17mm_cooling_heat >> 8) & 0xff);
		RefereeData.Data[5] = referee->game_robot_status.mains_power_shooter_output;
		RefereeData.Data[6] = (uint8_t)(referee->game_robot_status.remain_HP & 0xff);
		RefereeData.Data[7] =  (uint8_t)((referee->game_robot_status.remain_HP >> 8) & 0xff);
		CAN_Send(&can1,&RefereeData);
		i++;
	}else if(i == 1)
	{
		RefereeData.Identifier = 0x105;
		RefereeData.Data[0] = (uint8_t)(referee->game_status.stage_remain_time & 0xff);
		RefereeData.Data[1] =  (uint8_t)((referee->game_status.stage_remain_time >> 8) & 0xff);
		RefereeData.Data[2] = referee2022.game_robot_status.mains_power_gimbal_output;
		RefereeData.Data[3] = referee2022.game_robot_status.robot_id;
		RefereeData.Data[4] = (uint8_t)(referee->bullet_remaining.bullet_remaining_num & 0xff);
		RefereeData.Data[5] = (uint8_t)((referee->bullet_remaining.bullet_remaining_num >> 8) & 0xff);
		CAN_Send(&can1,&RefereeData);
		i = 0;
	}
}

void TopBoard_Callback(CAN_RxBuffer* rxBuffer)
{
	if(rxBuffer->Header.Identifier == 0x101)
	{
		check_robot_state.Check_Usart.Check_board_cnt = 0;
		Base.Rc.rc_Ctrl_ch0 = rxBuffer->Data[0] | (rxBuffer->Data[1] << 8);
		Base.Rc.rc_Ctrl_ch0 &= 0x07FF;		
		Base.Rc.rc_Ctrl_ch1 = ((rxBuffer->Data[1] >> 3) | (rxBuffer->Data[2] << 5)) & 0x07FF;
		Base.Rc.rc_Ctrl_ch1 &= 0x07FF;  
		Base.Rc.rc_Ctrl_ch2 = ((rxBuffer->Data[2] >> 6) | (rxBuffer->Data[3] << 2) | (rxBuffer->Data[4] << 10)) & 0x07FF;
		Base.Rc.rc_Ctrl_ch2 &= 0x07FF;
		Base.Rc.rc_Ctrl_ch3 = ((rxBuffer->Data[4] >> 1) | (rxBuffer->Data[5] << 7)) & 0x07FF; 
		Base.Rc.rc_Ctrl_ch3 &= 0x07FF;
		Base.Rc.rc_Ctrl_s1 = (rxBuffer->Data[5] >> 4) & 0x03;  
		Base.Rc.rc_Ctrl_s2 = (rxBuffer->Data[5] >> 6) & 0x03;
		Base.Rc.isOnline = (rxBuffer->Data[6] & 0x01);
		Base.Lidar.isOnline = ((rxBuffer->Data[6] >> 1) & 0x01);
		Base.Lidar.Movemode = ((rxBuffer->Data[6] >> 2) & 0x03);
	}
	if(rxBuffer->Header.Identifier == 0x102)
	{
		check_robot_state.Check_Usart.Check_board_cnt = 0;
		memcpy(&Base.Gyro.Gyro_Angle,&rxBuffer->Data[0],sizeof(float));
		memcpy(&Base.Gyro.Gyro_Data,&rxBuffer->Data[4],sizeof(float));
	}
	if(rxBuffer->Header.Identifier == 0x103)
	{
		check_robot_state.Check_Usart.Check_board_cnt = 0;
		memcpy(&Base.Lidar.Vx,&rxBuffer->Data[0],sizeof(float));
		memcpy(&Base.Lidar.Vy,&rxBuffer->Data[4],sizeof(float));
	}
}