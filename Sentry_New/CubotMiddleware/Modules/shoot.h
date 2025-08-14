#ifndef _SHOOT_H
#define _SHOOT_H
#include "pid.h"
#include "motor.h"
typedef struct
{
	//������
	struct
	{
		Motor motor2006;
		SinglePID_t RunPID;
		float Delta_Angle;//���ֽǶ�
		float Plate_Angle;//�����̽Ƕ�
		float Target_Angle;//������Ŀ��Ƕ�
		float Angle_Sense;
		float Plate_Out;//���
		uint32_t Shoot_Cut;
		uint16_t Fire_Rate;
		uint8_t  Fire_Divider;//��Ƶ
		uint8_t  Fire_Margin;//��������
		int16_t ShootNum;
		uint8_t  Shoot_rest_flag;
		uint8_t heat_status;
		int16_t Jam;
	}Shoot_Plate;
	//Ħ����
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
void AmmoBoosterInit(Ammo_Booster *ammo_booster,SinglePID_t* friction_pid0, SinglePID_t* friction_pid1,SinglePID_t* load_pid);
void ShootPlantControl(Ammo_Booster* ammo_booster);
void FrictionWheelControl(Ammo_Booster *ammo_booster);
#endif
