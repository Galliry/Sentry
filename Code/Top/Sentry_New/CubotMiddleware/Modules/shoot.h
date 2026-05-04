#ifndef _SHOOT_H
#define _SHOOT_H
#include "pid.h"
#include "motor.h"
#include "interboard.h"
typedef struct
{
	//拨弹盘
	struct
	{
		Motor motor2006;
		SinglePID_t RunPID_angle;
		SinglePID_t RunPID_speed;
		float Delta_Angle;//积分角度
		float Plate_Angle;//拨弹盘角度
		float Target_Angle;//拨弹盘目标角度
		float Angle_Sense;
		float Plate_Out;//输出
		uint32_t Shoot_Cut;
		uint16_t Fire_Rate;
		uint16_t  Fire_Divider;//分频
		uint8_t  Fire_Margin;//热量余量
		int16_t ShootNum;
		uint8_t  Shoot_rest_flag;
		uint8_t heat_status;
		int16_t Jam;
	}Shoot_Plate;
	//摩擦轮
	struct
	{
		Motor motor3508[2];		
		SinglePID_t Friction_PID[2];				
		float Friction_Target_Speed[2];
		float Friction_Out[2];
		int16_t Friction_Speed[2];	
		int16_t Friction_Start;
	}Friction_Wheel;
}Ammo_Booster;

extern Ammo_Booster AmmoBooster;
void AmmoBoosterInit(Ammo_Booster *ammo_booster, SinglePID_t *friction_pid0, SinglePID_t *friction_pid1, SinglePID_t *load_pid_angle, SinglePID_t *load_pid_speed);
void ShootPlantControl(Ammo_Booster* ammo_booster);
void FrictionWheelControl(Ammo_Booster *ammo_booster);
#endif
