#include "interboard.h"
#include "check.h"
#include "brain.h"
#include "DM_imu.h"
#include "holder.h"
#include "driver_usart.h"
CAN_TxBuffer RemoteData = {.Identifier = 0x101};
CAN_TxBuffer GyroData = {.Identifier = 0x102};
CAN_TxBuffer LidarData = {.Identifier = 0x103};
CAN_TxBuffer AutoaimData = {.Identifier = 0x104};
uint8_t RemoteDataU[9];
uint8_t GyroDataU[9];
uint8_t LidarDataU[9];
uint8_t AutoaimDataU[9];
Top_t Top;
uint8_t yaw_turn = 0;
void RemoteDataTrans(RC_Ctrl_ET* rc_ctrl)
{ 
	if(Holder.Yaw_S.Target_Angle > 27)
	{
		yaw_turn = 1;
	}else if(Holder.Yaw_S.Target_Angle < -27)
	{
		yaw_turn = 2;
	}else
	{
		yaw_turn = 0;
	}
	// RemoteDataU[0] = 0x01;
	// RemoteDataU[1] = rc_ctrl->rc.ch0 & 0xFF;                   
    // RemoteDataU[2] = (rc_ctrl->rc.ch0 >> 8) | ((rc_ctrl->rc.ch1 & 0x1F) << 3);  
    // RemoteDataU[3] = (rc_ctrl->rc.ch1 >> 5) | ((rc_ctrl->rc.ch2 & 0x03) << 6);  
    // RemoteDataU[4] = (rc_ctrl->rc.ch2 >> 2) & 0xFF;             
    // RemoteDataU[5] = (rc_ctrl->rc.ch2 >> 10) | ((rc_ctrl->rc.ch3 & 0x7F) << 1);  
    // RemoteDataU[6] = (rc_ctrl->rc.ch3 >> 7) | ((rc_ctrl->rc.s1 & 0x03) << 4) | ((rc_ctrl->rc.s2 & 0x03) << 6);
	// RemoteDataU[7] = ((rc_ctrl->isOnline & 0x01) | ((check_robot_state.Check_Usart.Check_lidar & 0x01) << 1) | ((Brain.Lidar.movemode & 0x03) << 2) | ((Brain.Autoaim.All_Sense & 0x07) << 4) | ((Brain.Autoaim.mode & 0x01) << 7));
	// RemoteDataU[8] = ((yaw_turn & 0x03));
	// HAL_UART_Transmit_DMA(&huart5,RemoteDataU,9);
	
	RemoteData.Data[0] = rc_ctrl->rc.ch0 & 0xFF;                   
	RemoteData.Data[1] = (rc_ctrl->rc.ch0 >> 8) | ((rc_ctrl->rc.ch1 & 0x1F) << 3);  
	RemoteData.Data[2] = (rc_ctrl->rc.ch1 >> 5) | ((rc_ctrl->rc.ch2 & 0x03) << 6);  
	RemoteData.Data[3] = (rc_ctrl->rc.ch2 >> 2) & 0xFF;             
	RemoteData.Data[4] = (rc_ctrl->rc.ch2 >> 10) | ((rc_ctrl->rc.ch3 & 0x7F) << 1);  
	RemoteData.Data[5] = (rc_ctrl->rc.ch3 >> 7) | ((rc_ctrl->rc.s1 & 0x03) << 4) | ((rc_ctrl->rc.s2 & 0x03) << 6);
	RemoteData.Data[6] = ((rc_ctrl->isOnline & 0x01) | ((check_robot_state.Check_Usart.Check_lidar & 0x01) << 1) | ((Brain.Lidar.movemode & 0x03) << 2) | ((Brain.Autoaim.All_Sense & 0x07) << 4) | ((Brain.Autoaim.mode & 0x01) << 7));
	RemoteData.Data[7] = ((yaw_turn & 0x03) | ((Brain.Lidar.stance & 0x07) << 2)); 
	CAN_Send(&can2,&RemoteData);
}

void GyroDataTrans(void)
{
	memcpy(&GyroData.Data[0],&mpu6050.Yaw_total_angle,sizeof(float));
	memcpy(&GyroData.Data[4],&mpu6050.mpu6050_Data.gyro[2],sizeof(float));
	CAN_Send(&can2,&GyroData);
	
	// GyroDataU[0] = 0x02;
	// memcpy(&GyroDataU[1],&mpu6050.Yaw_total_angle,sizeof(float));
	// memcpy(&GyroDataU[5],&mpu6050.mpu6050_Data.gyro[2],sizeof(float));
	// HAL_UART_Transmit_DMA(&huart5,GyroDataU,9);
}

void LidarDataTrans(void)
{
	pack_float_to_2bytes(LidarData.Data,0,Brain.Lidar.vx);
	pack_float_to_2bytes(LidarData.Data,2,Brain.Lidar.vy);
	memcpy(&LidarData.Data[4],&Brain.Autoaim.Yaw,sizeof(float));
	CAN_Send(&can2,&LidarData);
	// LidarDataU[0] = 0x03;
	// memcpy(&LidarDataU[1],&Brain.Lidar.vx,sizeof(float));
	// memcpy(&LidarDataU[5],&Brain.Lidar.vy,sizeof(float));
	// HAL_UART_Transmit_DMA(&huart5,LidarDataU,9);
}

void TopBoardDataTrans(RC_Ctrl_ET* rc_ctrl)
{
	static uint8_t i = 0;
	if(i == 0){RemoteDataTrans(rc_ctrl); i++;}
	else if(i == 1){GyroDataTrans(); i++;}
	else if(i == 2){LidarDataTrans(); i=0;}

}

void BaseBoard_Callback(CAN_RxBuffer* rxBuffer)
{
	if(rxBuffer->Header.Identifier == 0x105)
	{
		Top.Referee.game_prograss = rxBuffer->Data[0];
		Top.Referee.cooling_limit = ((uint16_t)rxBuffer->Data[1] | (uint16_t)(rxBuffer->Data[2] << 8));
		Top.Referee.cooling_heat = ((uint16_t)rxBuffer->Data[3] | (uint16_t)(rxBuffer->Data[4] << 8));
		Top.Referee.shooter_output = rxBuffer->Data[5];
		Top.Referee.robot_HP = ((uint16_t)rxBuffer->Data[6] | (uint16_t)(rxBuffer->Data[7] << 8));
	}else if(rxBuffer->Header.Identifier == 0x106)
	{
		Top.Referee.game_time = ((uint16_t)rxBuffer->Data[0] | (uint16_t)(rxBuffer->Data[1] << 8));
		Top.Referee.gimbal_output = rxBuffer->Data[2];
		Top.Referee.robot_id = rxBuffer->Data[3];
		Top.Referee.shoot_num = ((uint16_t)rxBuffer->Data[4] | (uint16_t)(rxBuffer->Data[5] << 8));
		Top.Referee.lidar_target_state = rxBuffer->Data[6] & 0x03;
		Top.Referee.small_buff = (rxBuffer->Data[6] >> 2) & 0x03;
		Top.Referee.big_buff = (rxBuffer->Data[6] >> 4) & 0x03;
		Top.Referee.posture = (rxBuffer->Data[7] & 0x03);
		Top.Referee.base_flag = ((rxBuffer->Data[7] >> 2) & 0x01);
		Top.Referee.outpost_flag = ((rxBuffer->Data[7] >> 3) & 0x01);
	}
}

static void pack_float_to_2bytes(uint8_t *buffer, int index, float val) 
{
    int scaled = (int)roundf(val * 100.0f);
    if (scaled > 6555)  scaled = 6555;
    if (scaled < -6555) scaled = -6555;
    uint8_t sign_bit = (scaled < 0) ? 0x40 : 0x00;
    int abs_scaled = abs(scaled);
    int high = abs_scaled / 100;
    int low  = abs_scaled % 100;
    if (high > 63) {
        low += (high - 63) * 100;
        high = 63;
    }
    buffer[index]     = (uint8_t)((high & 0x3F) | sign_bit);
    buffer[index + 1] = (uint8_t)(low & 0xFF);
}
