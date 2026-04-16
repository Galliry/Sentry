#ifndef _BOARD_H_
#define _BOARD_H_
#include "driver_can.h"
#include "dr16.h"
#include "et08.h"
#include "mpu6050.h"
#include "brain.h"
typedef struct
{
	struct
	{
		uint8_t game_prograss;
		uint16_t cooling_limit;
		uint16_t cooling_heat;
		uint8_t shooter_output;
		uint16_t robot_HP;
		uint16_t game_time;
		uint8_t gimbal_output;
		uint8_t robot_id;
		uint16_t shoot_num;
	}Referee;

}Top_t;

extern Top_t Top;
void RemoteDataTrans(RC_Ctrl_ET* rc_ctrl);
void GyroDataTrans(void);
void LidarDataTrans(void);
void TopBoardDataTrans(RC_Ctrl_ET* rc_ctrl);
void BaseBoard_Callback(CAN_RxBuffer* rxBuffer);
#endif

