#include "interboard.h"
#include "check.h"
#include "brain.h"
#include "DM_imu.h"
#include "holder.h"
CAN_TxBuffer RemoteData = {.Identifier = 0x61};
CAN_TxBuffer GyroData = {.Identifier = 0x62};
CAN_TxBuffer LidarData = {.Identifier = 0x63};
CAN_TxBuffer AutoaimData = {.Identifier = 0x64};
Top_t Top;
uint8_t yaw_turn = 0;
void RemoteDataTrans(RC_Ctrl_ET* rc_ctrl)
{ 
	if(Holder.Yaw_S.Target_Angle > 30)
	{
		yaw_turn = 1;
	}else if(Holder.Yaw_S.Target_Angle < -30)
	{
		yaw_turn = 2;
	}else
	{
		yaw_turn = 0;
	}
	RemoteData.Data[0] = rc_ctrl->rc.ch0 & 0xFF;                   
    RemoteData.Data[1] = (rc_ctrl->rc.ch0 >> 8) | ((rc_ctrl->rc.ch1 & 0x1F) << 3);  
    RemoteData.Data[2] = (rc_ctrl->rc.ch1 >> 5) | ((rc_ctrl->rc.ch2 & 0x03) << 6);  
    RemoteData.Data[3] = (rc_ctrl->rc.ch2 >> 2) & 0xFF;             
    RemoteData.Data[4] = (rc_ctrl->rc.ch2 >> 10) | ((rc_ctrl->rc.ch3 & 0x7F) << 1);  
    RemoteData.Data[5] = (rc_ctrl->rc.ch3 >> 7) | ((rc_ctrl->rc.s1 & 0x03) << 4) | ((rc_ctrl->rc.s2 & 0x03) << 6);
	RemoteData.Data[6] = ((rc_ctrl->isOnline & 0x01) | ((check_robot_state.Check_Usart.Check_lidar & 0x01) << 1) | ((Brain.Lidar.movemode & 0x03) << 2) | ((Brain.Autoaim.All_Sense & 0x07) << 4) | ((Brain.Autoaim.mode & 0x01) << 7));
	RemoteData.Data[7] = ((yaw_turn & 0x03));
	CAN_Send(&can2,&RemoteData);
}

void GyroDataTrans(void)
{
	memcpy(&GyroData.Data[0],&mpu6050.Yaw_total_angle,sizeof(float));
	memcpy(&GyroData.Data[4],&mpu6050.mpu6050_Data.gyro[2],sizeof(float));
	
	CAN_Send(&can2,&GyroData);
}

void LidarDataTrans(void)
{
	memcpy(&LidarData.Data[0],&Brain.Lidar.vx,sizeof(float));
	memcpy(&LidarData.Data[4],&Brain.Lidar.vy,sizeof(float));

	// LidarData.Data[9] = Brain.Autoaim.All_Sense;
	
	CAN_Send(&can2,&LidarData);
}

void AutoaimDataTrans(void)
{
	memcpy(&AutoaimData.Data[0],&Brain.Autoaim.Yaw,sizeof(float));
	memcpy(&AutoaimData.Data[4],&Holder.Motors.Yaw_S.Data.Angle, sizeof(float));

	CAN_Send(&can2,&AutoaimData);
}

void TopBoardDataTrans(RC_Ctrl_ET* rc_ctrl)
{
	static uint8_t i = 0;
	if(i == 0){RemoteDataTrans(rc_ctrl); i++;}
	else if(i == 1){GyroDataTrans(); i++;}
	else if(i == 2){LidarDataTrans(); i++;}
	else if(i == 3){AutoaimDataTrans(); i = 0;}

}
void BaseBoard_Callback(CAN_RxBuffer* rxBuffer)
{
	if(rxBuffer->Header.Identifier == 0x104)
	{
		Top.Referee.game_prograss = rxBuffer->Data[0];
		Top.Referee.cooling_limit = ((uint16_t)rxBuffer->Data[1] | (uint16_t)(rxBuffer->Data[2] << 8));
		Top.Referee.cooling_heat = ((uint16_t)rxBuffer->Data[3] | (uint16_t)(rxBuffer->Data[4] << 8));
		Top.Referee.shooter_output = rxBuffer->Data[5];
		Top.Referee.robot_HP = ((uint16_t)rxBuffer->Data[6] | (uint16_t)(rxBuffer->Data[7] << 8));
	}else if(rxBuffer->Header.Identifier == 0x105)
	{
		Top.Referee.game_time = ((uint16_t)rxBuffer->Data[0] | (uint16_t)(rxBuffer->Data[1] << 8));
		Top.Referee.gimbal_output = rxBuffer->Data[2];
		Top.Referee.robot_id = rxBuffer->Data[3];
		Top.Referee.shoot_num = ((uint16_t)rxBuffer->Data[4] | (uint16_t)(rxBuffer->Data[5] << 8));
		Top.Referee.lidar_target_pos = rxBuffer->Data[6] & 0x03;
		Top.Referee.small_buff = (rxBuffer->Data[6] >> 2) & 0x03;
		Top.Referee.big_buff = (rxBuffer->Data[6] >> 4) & 0x03;
		Top.Referee.posture = (rxBuffer->Data[7] & 0x03);
		Top.Referee.base_flag = ((rxBuffer->Data[7] >> 2) & 0x01);
	}
//	else if(rxBuffer->Header.Identifier == 0x106)
//	{
//		memcpy(&Top.Referee.lidar_pos_x,&rxBuffer->Data[0],4);
//		memcpy(&Top.Referee.lidar_pos_y,&rxBuffer->Data[4],4);
//	}

}