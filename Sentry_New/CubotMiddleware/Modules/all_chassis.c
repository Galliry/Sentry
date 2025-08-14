#include "all_chassis.h"

AllChassis allchassis;

/**
  * @brief  麦轮底盘初始化函数，创建四个底盘电机，并且拷贝相同的PID参数，
  */
void AllChassisInit(AllChassis* chassis,SinglePID_t* run_pid,SinglePID_t* follow_pid)
{
	MotorInit(&chassis->Motors.motor[0],0,Motor3508,CAN2,0x201);
	MotorInit(&chassis->Motors.motor[1],0,Motor3508,CAN2,0x202);
	MotorInit(&chassis->Motors.motor[2],0,Motor3508,CAN2,0x203);
	MotorInit(&chassis->Motors.motor[3],0,Motor3508,CAN2,0x204);
	BasePID_Init(&chassis->Motors.RunPID[0],run_pid->Kp,run_pid->Ki,run_pid->Kd,run_pid->KiPartDetachment);
	BasePID_Init(&chassis->Motors.RunPID[1],run_pid->Kp,run_pid->Ki,run_pid->Kd,run_pid->KiPartDetachment);
	BasePID_Init(&chassis->Motors.RunPID[2],run_pid->Kp,run_pid->Ki,run_pid->Kd,run_pid->KiPartDetachment);
	BasePID_Init(&chassis->Motors.RunPID[3],run_pid->Kp,run_pid->Ki,run_pid->Kd,run_pid->KiPartDetachment);
	BasePID_Init(&chassis->Motors.FollowPID,follow_pid->Kp, follow_pid->Ki, follow_pid->Kd, follow_pid->KiPartDetachment);
	chassis->Movement.Vomega_Sensitivity = 1;
	chassis->Movement.Vx_Sensitivity     = 1;
	chassis->Movement.Vy_Sensitivity     = 1;
}

void AllChassis_SpeedControl(AllChassis* chassis,float canAngle)
{
//	float angle = -canAngle * AtR-Holder.Motors6020.motor[0].Data.SpeedRPM*0.0026;	
}




















