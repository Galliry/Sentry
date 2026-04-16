#include "interboard.h"
CAN_TxBuffer RemoteData = {.Identifier = 0x101};
CAN_TxBuffer GyroData = {.Identifier = 0x102};
CAN_TxBuffer LidarData = {.Identifier = 0x103};
Top_t Top;
void RemoteDataTrans(RC_Ctrl_ET* rc_ctrl)
{ 
	RemoteData.Data[0] =  rc_ctrl->rc.ch0 & 0xFF;                   
    RemoteData.Data[1] = (rc_ctrl->rc.ch0 >> 8) | ((rc_ctrl->rc.ch1 & 0x1F) << 3);  
    RemoteData.Data[2] = (rc_ctrl->rc.ch1 >> 5) | ((rc_ctrl->rc.ch2 & 0x03) << 6);  
    RemoteData.Data[3] = (rc_ctrl->rc.ch2 >> 2) & 0xFF;             
    RemoteData.Data[4] = (rc_ctrl->rc.ch2 >> 10) | ((rc_ctrl->rc.ch3 & 0x7F) << 1);  
    RemoteData.Data[5] = (rc_ctrl->rc.ch3 >> 7) | ((rc_ctrl->rc.s1 & 0x03) << 4) | ((rc_ctrl->rc.s2 & 0x03) << 6);
	RemoteData.Data[6] = rc_ctrl->isOnline;
	
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
	
	CAN_Send(&can2,&LidarData);
}

void TopBoardDataTrans(RC_Ctrl_ET* rc_ctrl)
{
	static uint8_t i = 0;
	if(i == 0){RemoteDataTrans(rc_ctrl); i++;}
	else if(i == 1){GyroDataTrans(); i++;}
	else if(i == 2){LidarDataTrans(); i = 0;}

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
	}

}