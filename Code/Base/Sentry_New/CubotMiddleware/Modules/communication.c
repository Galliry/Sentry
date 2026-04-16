#include "communication.h"
#include "driver_timer.h"
Receive_t Receive={
    .Base.Online_check.StatusCnt = 50,
    .Base.rc.rc_Ctrl_ch0 = 1024,
    .Base.rc.rc_Ctrl_ch1 = 1024,
    .Base.rc.rc_Ctrl_ch2 = 1024,
	.Base.rc.rc_Ctrl_ch3 = 1024,
	.Base.rc.rc_Ctrl_s1 = 3,
    .Base.rc.rc_Ctrl_s2 = 3,
};

Transmit_t Transmit;

/**
	* @brief  
	*/
uint8_t SolutionData_FromTop(uint8_t *rxBuffer, uint16_t len)
{
	Receive.Base.Online_check.StatusCnt = 0;
	Receive.Base.rc.rc_Ctrl_ch0 = rxBuffer[0] | (rxBuffer[1] << 8);
	Receive.Base.rc.rc_Ctrl_ch0 &= 0x07FF;		
	Receive.Base.rc.rc_Ctrl_ch1 = ((rxBuffer[1] >> 3) | (rxBuffer[2] << 5)) & 0x07FF;
	Receive.Base.rc.rc_Ctrl_ch1 &= 0x07FF;  
	Receive.Base.rc.rc_Ctrl_ch2 = ((rxBuffer[2] >> 6) | (rxBuffer[3] << 2) | (rxBuffer[4] << 10)) & 0x07FF;
	Receive.Base.rc.rc_Ctrl_ch2 &= 0x07FF;
	Receive.Base.rc.rc_Ctrl_ch3 = ((rxBuffer[4] >> 1) | (rxBuffer[5] << 7)) & 0x07FF; 
	Receive.Base.rc.rc_Ctrl_ch3 &= 0x07FF;
	Receive.Base.rc.rc_Ctrl_s1 = (rxBuffer[5] >> 4) & 0x03;  
	Receive.Base.rc.rc_Ctrl_s2 = (rxBuffer[5] >> 6) & 0x03;
	memcpy(&Receive.Base.Gyro.Gyro_Angle,&rxBuffer[6],sizeof(float));
	memcpy(&Receive.Base.Gyro.Gyro_Data,&rxBuffer[10],sizeof(float));
	Receive.Base.Lidar.Online = rxBuffer[14];
	memcpy(&Receive.Base.Lidar.Vx,&rxBuffer[15],sizeof(float));
	memcpy(&Receive.Base.Lidar.Vy,&rxBuffer[19],sizeof(float));
	Receive.Base.Lidar.Movemode =rxBuffer[23];
	Receive.Base.AutoAim.mode = rxBuffer[24];
	return 0;
}

void Trans_forBasetoTop(Referee2022* referee)
{
	Transmit.TransData[0] = referee->game_status.game_progress;
	Transmit.TransData[1] = (uint8_t)(referee->game_robot_status.shooter_id1_17mm_cooling_limit & 0xff);
	Transmit.TransData[2] = (uint8_t)((referee->game_robot_status.shooter_id1_17mm_cooling_limit >> 8) & 0xff);
	Transmit.TransData[3] = (uint8_t)(referee->power_heat_data.shooter_id1_17mm_cooling_heat & 0xff);
	Transmit.TransData[4] = (uint8_t)((referee->power_heat_data.shooter_id1_17mm_cooling_heat >> 8) & 0xff);
	Transmit.TransData[5] = referee->game_robot_status.mains_power_shooter_output;
	Transmit.TransData[6] = (uint8_t)(referee->game_robot_status.remain_HP & 0xff);
	Transmit.TransData[7] =  (uint8_t)((referee->game_robot_status.remain_HP >> 8) & 0xff);
	
	Transmit.TransData[8] = (uint8_t)(referee->game_status.stage_remain_time & 0xff);
	Transmit.TransData[9] =  (uint8_t)((referee->game_status.stage_remain_time >> 8) & 0xff);
	Transmit.TransData[10] = referee2022.game_robot_status.mains_power_gimbal_output;
	Transmit.TransData[11] = referee2022.game_robot_status.robot_id;
	Transmit.TransData[12] = (uint8_t)(referee->bullet_remaining.bullet_remaining_num & 0xff);
	Transmit.TransData[13] = (uint8_t)((referee->bullet_remaining.bullet_remaining_num >> 8) & 0xff);
	HAL_UART_Transmit_DMA(&huart2,Transmit.TransData,14);
}