#include "communication.h"

Receive_t Receive={
    .Base.Online_check.StatusCnt = 50,
    .Base.rc.rc_Ctrl_ch0 = 1024,
    .Base.rc.rc_Ctrl_ch1 = 1024,
    .Base.rc.rc_Ctrl_ch2 = 1024,
	.Base.rc.rc_Ctrl_ch3 = 1024,
	.Base.rc.rc_Ctrl_s1 = 3,
    .Base.rc.rc_Ctrl_s2 = 3,
};

/**
	* @brief  底板处理上板数据
	*/
uint8_t SolutionData_FromTop(uint8_t *rxBuffer, uint16_t len)
{
	Receive.Base.Online_check.StatusCnt = 0;
	Receive.Base.rc.rc_Ctrl_ch0 = rxBuffer[0] | (rxBuffer[1] << 8);
	Receive.Base.rc.rc_Ctrl_ch0 &= 0x07FF;		// 保证只取 11 位
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
	return 0;
}

