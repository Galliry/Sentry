#ifndef _HOLDER_H_
#define _HOLDER_H_
#include "stm32h7xx_hal.h"
#include "pid.h"
#include "motor.h"
#include "et08.h"
struct Holder_Motor_Info
{
	float Target_Angle;
	float GYRO_Angle;
	float GYRO_AngleSpeed;
	float Can_Angle;
	int16_t Can_AngleSpeed;//RPM
	float Sensitivity;
	DualPID_Object PID;
};

typedef struct 
{
	struct Holder_Motor_Info Pitch;
	struct Holder_Motor_Info Yaw_M;//大Yaw轴
	struct Holder_Motor_Info Yaw_S;//小Yaw轴
	
	struct 
	{
		Motor Pitch;
		Motor Yaw_M;
		Motor Yaw_S;
	}Motors6020;
	
	struct
	{
	uint32_t Pitch_time;	
	uint32_t Yaw_S_time;	
	float Pitch_sense;	
	float Yaw_S_sense;		
	}Cruise_Mode;//巡航
	
	struct
	{
	uint16_t Lock_cnt;	
	uint8_t Flag_Fllow;	
	}Yaw_Fllow_Mode;//大云台跟随
	//限幅
	float up_litmit;
	float down_litmit;
	float right_litmit;
	float left_litmit;
}Holder_t;

extern Holder_t Holder;;
void HolderInit(Holder_t* holder,DualPID_Object* pitch,DualPID_Object* yaw_m,DualPID_Object* yaw_s);
void Holder_Control(Holder_t* holder,RC_Ctrl_ET* rc_ctrl);
#endif
