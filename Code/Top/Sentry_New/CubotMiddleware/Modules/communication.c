#include "communication.h"

Transmit_t Transmit;
Receive_t Receive={.Top.Online_check.StatusCnt = 50};
int flag = 0;
/**
	* @brief  
	*/
void Trans_forToptoBase(RC_Ctrl* rc_ctrl)
{
    Transmit.TransData[0] =  rc_ctrl->rc.ch0 & 0xFF;                   
    Transmit.TransData[1] = (rc_ctrl->rc.ch0 >> 8) | ((rc_ctrl->rc.ch1 & 0x1F) << 3);  
    Transmit.TransData[2] = (rc_ctrl->rc.ch1 >> 5) | ((rc_ctrl->rc.ch2 & 0x03) << 6);  
    Transmit.TransData[3] = (rc_ctrl->rc.ch2 >> 2) & 0xFF;             
    Transmit.TransData[4] = (rc_ctrl->rc.ch2 >> 10) | ((rc_ctrl->rc.ch3 & 0x7F) << 1);  
    Transmit.TransData[5] = (rc_ctrl->rc.ch3 >> 7) | ((rc_ctrl->rc.s1 & 0x03) << 4) | ((rc_ctrl->rc.s2 & 0x03) << 6);
	memcpy(&Transmit.TransData[6],&mpu6050.Yaw_total_angle,sizeof(float));
	memcpy(&Transmit.TransData[6+sizeof(float)],&mpu6050.mpu6050_Data.gyro[2],sizeof(float));
	Transmit.TransData[14] = check_robot_state.Check_Usart.Check_lidar;
	memcpy(&Transmit.TransData[15],&Brain.Lidar.vx,sizeof(float));
	memcpy(&Transmit.TransData[19],&Brain.Lidar.vy,sizeof(float));
	Transmit.TransData[23] = Brain.Lidar.movemode;
    
	HAL_UART_Transmit_DMA(&huart5,Transmit.TransData,24);
}
uint8_t BaseData_Callback(uint8_t * recBuffer, uint16_t len)
{
	Receive.Top.Online_check.StatusCnt = 0;
	Receive.Top.Referee.game_prograss = recBuffer[0];
	Receive.Top.Referee.cooling_limit = ((uint16_t)recBuffer[1] | (uint16_t)(recBuffer[2] << 8));
	Receive.Top.Referee.cooling_heat = ((uint16_t)recBuffer[3] | (uint16_t)(recBuffer[4] << 8));
	Receive.Top.Referee.shooter_output = recBuffer[5];
	Receive.Top.Referee.robot_HP = ((uint16_t)recBuffer[6] | (uint16_t)(recBuffer[7] << 8));
	Receive.Top.Referee.game_time = ((uint16_t)recBuffer[8] | (uint16_t)(recBuffer[9] << 8));
	Receive.Top.Referee.RFID_zx =recBuffer[10];
	Receive.Top.Referee.RFID_bj = recBuffer[11];
	return 0;
}
