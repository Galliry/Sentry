#include "pid.h"

SinglePID_t pid_load;
SinglePID_t pid_friction0;
SinglePID_t pid_friction1;
SinglePID_t pid_yaw_m_angle;
SinglePID_t pid_yaw_m_speed;
SinglePID_t pid_yaw_s_angle;
SinglePID_t pid_yaw_s_speed;
SinglePID_t pid_pitch_angle;
SinglePID_t pid_pitch_speed;
DualPID_Object pid_yaw_m = {.ShellPID=&pid_yaw_m_angle,.CorePID=&pid_yaw_m_speed};
DualPID_Object pid_yaw_s = {.ShellPID=&pid_yaw_s_angle,.CorePID=&pid_yaw_s_speed};
DualPID_Object pid_pitch = {.ShellPID=&pid_pitch_angle,.CorePID=&pid_pitch_speed};
SinglePID_t pid_run;
SinglePID_t pid_follow;
/**
  * @brief 单环PID初始化
  */
void BasePID_Init(SinglePID_t* base_pid, float kp, float ki, float kd, float detach)
{
	base_pid->KiPartDetachment = detach;
	
	base_pid->Kp = kp;
	base_pid->Ki = ki;
	base_pid->Kd = kd;

	base_pid->KpPart = 0;
	base_pid->KiPart = 0;
	base_pid->KdPart = 0;
	
}

/**
  * @brief 双环PID初始化
  */
void DualPID_Init(DualPID_Object* dual_pid, SinglePID_t* ShellPID,SinglePID_t* CorePID)
{
	dual_pid->ShellPID = ShellPID;
	dual_pid->CorePID=CorePID;
}


/**
  * @brief 单环比例积分速度控制
  */
int16_t BasePID_SpeedControl(SinglePID_t* base_pid, float target_speed, float feedback_speed)
{
	base_pid->Error = target_speed - feedback_speed;
	
	base_pid->KpPart = base_pid->Error * base_pid->Kp;
	base_pid->KiPart += base_pid->Error * base_pid->Ki;
	
if (fabs(base_pid->Error) > base_pid->KiPartDetachment) base_pid->KiPart = 0;
 
	base_pid->KdPart = (-1) * base_pid->Kd * (base_pid->Error - base_pid->LastError);
	base_pid->Out = base_pid->KpPart + base_pid->KiPart+base_pid->KdPart;
	base_pid->LastError=base_pid->Error;
	return base_pid->Out;
}


/**
  * @brief 单环比例积分角度控制   
  */
float  BasePID_AngleControl(SinglePID_t* base_pid, float target_angle, float feedback_angle)
{
	base_pid->Error = target_angle - feedback_angle;
	base_pid->KpPart = base_pid->Error * base_pid->Kp;
	base_pid->KiPart += base_pid->Error * base_pid->Ki;

  if (fabs(base_pid->Error) > base_pid->KiPartDetachment) base_pid->KiPart = 0;
	
	base_pid->KdPart = (-1) * base_pid->Kd * (base_pid->Error - base_pid->LastError);
	base_pid->Out = base_pid->KpPart + base_pid->KiPart + base_pid->KdPart;
	base_pid->LastError=base_pid->Error;
	return base_pid->Out;
}

void PID_Init(void)
{
	BasePID_Init(&pid_load,4,0 ,0, 0);
	BasePID_Init(&pid_friction0,10,1.5 , 2, 0);
	BasePID_Init(&pid_friction1,10,1.5 , 2, 0);
	BasePID_Init(&pid_pitch_angle,0,0,0,0);
	BasePID_Init(&pid_pitch_speed,0,0,0,0);
	BasePID_Init(&pid_yaw_m_angle,0,0,0,0);
	BasePID_Init(&pid_yaw_m_speed,0,0,0,0);
	BasePID_Init(&pid_yaw_s_angle,0,0,0,0);
	BasePID_Init(&pid_yaw_s_speed,0,0,0,0);
	BasePID_Init(&pid_run,0,0,0,0);
	BasePID_Init(&pid_follow,0,0,0,0);
}