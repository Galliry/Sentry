#ifndef _COMMUNICATION_H
#define _COMMUNICATION_H
#include "driver_can.h"
#include "dr16.h"
#include "et08.h"
#include "mpu6050.h"
#include "check.h"
#include "brain.h"
#include "usart.h"
typedef struct 
{
    uint8_t TransData[50];
}Transmit_t;

typedef struct 
{
    struct 
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
		int ClockTime;
		struct 
        {
            uint8_t Status;
            uint8_t StatusCnt;
        }Online_check;
    }Top;
    
}Receive_t;

extern Transmit_t Transmit;
extern Receive_t Receive;
void Trans_forToptoBase(RC_Ctrl* rc_ctrl);
uint8_t BaseData_Callback(uint8_t * recBuffer, uint16_t len);
#endif
