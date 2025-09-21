#include "shoot.h"
#include "et08.h"
#include "driver_timer.h"
#include "user_lib.h"

Ammo_Booster AmmoBooster;

void AmmoBoosterInit(Ammo_Booster *ammo_booster,SinglePID_t* friction_pid0, SinglePID_t* friction_pid1,SinglePID_t* load_pid)
{
	MotorInit(&ammo_booster->Friction_Wheel.motor3508[0], 0 , Motor3508, CAN1, 0x201);
	MotorInit(&ammo_booster->Friction_Wheel.motor3508[1], 0 , Motor3508, CAN1, 0x202);
	MotorInit(&ammo_booster->Shoot_Plate.motor2006, 0 , Motor2006, CAN1, 0x204);

	BasePID_Init(&ammo_booster->Shoot_Plate.RunPID, load_pid->Kp, load_pid->Ki, load_pid->Kd, load_pid->KiPartDetachment);
	BasePID_Init(&ammo_booster->Friction_Wheel.Friction_PID[0], friction_pid0->Kp, friction_pid0->Ki, friction_pid0->Kd, friction_pid0->KiPartDetachment);
	BasePID_Init(&ammo_booster->Friction_Wheel.Friction_PID[1], friction_pid1->Kp, friction_pid1->Ki, friction_pid1->Kd, friction_pid1->KiPartDetachment);

	ammo_booster->Shoot_Plate.Fire_Rate = 8000;//5250//8000
	ammo_booster->Shoot_Plate.Fire_Margin = 40;
	ammo_booster->Shoot_Plate.Angle_Sense = 0.1689f;

	ammo_booster->Friction_Wheel.Friction_Start = 0;
	ammo_booster->Friction_Wheel.Friction_Speed[0] = -5600;
	ammo_booster->Friction_Wheel.Friction_Speed[1] =-5590;
}

void ShootPlantControl(Ammo_Booster* ammo_booster)
{
	ammo_booster->Shoot_Plate.Delta_Angle = ammo_booster->Shoot_Plate.motor2006.Data.SpeedRPM*0.001*ammo_booster->Shoot_Plate.Angle_Sense;
	if(ammo_booster->Shoot_Plate.Delta_Angle > 0.005f)
	{
		ammo_booster->Shoot_Plate.Plate_Angle += ammo_booster->Shoot_Plate.Delta_Angle;
	}
	
	if(rc_Ctrl_et.isOnline == 1)
	{
		ammo_booster->Shoot_Plate.Fire_Divider=50;
		if(ammo_booster->Shoot_Plate.Shoot_rest_flag) ammo_booster->Shoot_Plate.Shoot_Cut++;
		if(ammo_booster->Shoot_Plate.Shoot_Cut%ammo_booster->Shoot_Plate.Fire_Divider == 0) ammo_booster->Shoot_Plate.Shoot_rest_flag = 0;
		if(rc_Ctrl_et.rc.s1 == 1 && rc_Ctrl_et.rc.s2 != 2 && ammo_booster->Shoot_Plate.Shoot_rest_flag == 0)
		{
			ammo_booster->Shoot_Plate.Target_Angle += 45;
			ammo_booster->Shoot_Plate.ShootNum++;
			ammo_booster->Shoot_Plate.Shoot_rest_flag=1;
			ammo_booster->Shoot_Plate.Shoot_Cut=0;
		}
	}
	
	if(ammo_booster->Shoot_Plate.Target_Angle-ammo_booster->Shoot_Plate.Plate_Angle > 5)
		ammo_booster->Shoot_Plate.Plate_Out = BasePID_SpeedControl(&ammo_booster->Shoot_Plate.RunPID,ammo_booster->Shoot_Plate.Fire_Rate,ammo_booster->Shoot_Plate.motor2006.Data.SpeedRPM);
	else
		ammo_booster->Shoot_Plate.Plate_Out = BasePID_SpeedControl(&ammo_booster->Shoot_Plate.RunPID,0,ammo_booster->Shoot_Plate.motor2006.Data.SpeedRPM);
	
	if(ammo_booster->Shoot_Plate.Target_Angle-ammo_booster->Shoot_Plate.Plate_Angle > 5 && ammo_booster->Shoot_Plate.motor2006.Data.SpeedRPM < 400)
	{
		ammo_booster->Shoot_Plate.Jam++;
		if(ammo_booster->Shoot_Plate.Jam >= 900)
		{
			ammo_booster->Shoot_Plate.Target_Angle = ammo_booster->Shoot_Plate.Plate_Angle+6;
			ammo_booster->Shoot_Plate.Jam++;
			ammo_booster->Shoot_Plate.Plate_Out = BasePID_SpeedControl(&ammo_booster->Shoot_Plate.RunPID, -2000, ammo_booster->Shoot_Plate.motor2006.Data.SpeedRPM);
			if(ammo_booster->Shoot_Plate.Jam > 1600) ammo_booster->Shoot_Plate.Jam = 0;	
		}
	}
	else ammo_booster->Shoot_Plate.Jam = 0;
	
	MotorFillData(&ammo_booster->Shoot_Plate.motor2006, ammo_booster->Shoot_Plate.Plate_Out);
}

void FrictionWheelControl(Ammo_Booster *ammo_booster)
{
	if(tim14.ClockTime % 2 == 0)
	{
		if(rc_Ctrl_et.isOnline == 1) ammo_booster->Friction_Wheel.Friction_Start++;
		else ammo_booster->Friction_Wheel.Friction_Start--;
	}
	ammo_booster->Friction_Wheel.Friction_Start = int16_constrain(ammo_booster->Friction_Wheel.Friction_Start,0,1000);
	ammo_booster->Friction_Wheel.Friction_Target_Speed[0] = ammo_booster->Friction_Wheel.Friction_Speed[0] * ammo_booster->Friction_Wheel.Friction_Start * 0.001;
	ammo_booster->Friction_Wheel.Friction_Target_Speed[1] = ammo_booster->Friction_Wheel.Friction_Speed[1] * ammo_booster->Friction_Wheel.Friction_Start * 0.001*(-1);
	
	ammo_booster->Friction_Wheel.Friction_Out[0] = BasePID_SpeedControl(&ammo_booster->Friction_Wheel.Friction_PID[0],ammo_booster->Friction_Wheel.Friction_Target_Speed[0],ammo_booster->Friction_Wheel.motor3508[0].Data.SpeedRPM);
	ammo_booster->Friction_Wheel.Friction_Out[1] = BasePID_SpeedControl(&ammo_booster->Friction_Wheel.Friction_PID[1],ammo_booster->Friction_Wheel.Friction_Target_Speed[1],ammo_booster->Friction_Wheel.motor3508[1].Data.SpeedRPM);

	MotorFillData(&ammo_booster->Friction_Wheel.motor3508[0],ammo_booster->Friction_Wheel.Friction_Out[0]);
	MotorFillData(&ammo_booster->Friction_Wheel.motor3508[1],ammo_booster->Friction_Wheel.Friction_Out[1]);
}
