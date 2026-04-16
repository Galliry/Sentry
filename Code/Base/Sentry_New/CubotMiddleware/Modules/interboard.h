#ifndef _BOARD_H_
#define _BOARD_H_
#include "driver_can.h"
#include "dr16.h"
#include "et08.h"
#include "user_lib.h"
#include "referee.h"

typedef struct
{
	struct
	{
		uint16_t rc_Ctrl_ch0;
		uint16_t rc_Ctrl_ch1;
		uint16_t rc_Ctrl_ch2;
		uint16_t rc_Ctrl_ch3;
		uint8_t rc_Ctrl_s1;
		uint8_t rc_Ctrl_s2;
		uint8_t isOnline;
	}Rc;
	struct
	{
		float Gyro_Angle;
		float Gyro_Data;
	}Gyro;
	struct
	{
		float Vx;
		float Vy;
		uint8_t isOnline;
		uint8_t Movemode;
	}Lidar;
	struct 
	{
		uint8_t Status;
		uint8_t StatusCnt;
	}Online_check;

}Base_t;

extern Base_t Base;
void RefereeDataTrans(Referee2022* referee);
void TopBoard_Callback(CAN_RxBuffer* rxBuffer);
#endif 
