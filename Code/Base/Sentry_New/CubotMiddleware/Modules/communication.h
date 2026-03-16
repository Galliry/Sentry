#ifndef _COMMUNICATION_H
#define _COMMUNICATION_H
#include "driver_can.h"
#include "dr16.h"
#include "et08.h"
#include "user_lib.h"
typedef struct 
{
    struct 
    {
        struct
		{
			uint16_t rc_Ctrl_ch0;
			uint16_t rc_Ctrl_ch1;
			uint16_t rc_Ctrl_ch2;
			uint16_t rc_Ctrl_ch3;
			uint8_t rc_Ctrl_s1;
			uint8_t rc_Ctrl_s2;
		}rc;
		
        struct
		{
			float Gyro_Angle;
			float Gyro_Data;
		}Gyro;
		struct
		{
			float Vx;
			float Vy;
			uint8_t Movemode;
			uint8_t Online;
		}Lidar;
		
        struct 
        {
            uint8_t Status;
            uint8_t StatusCnt;
        }Online_check;
    }Base;
    
}Receive_t;

extern Receive_t Receive;
uint8_t SolutionData_FromTop(uint8_t *rxBuffer, uint16_t len);
#endif
