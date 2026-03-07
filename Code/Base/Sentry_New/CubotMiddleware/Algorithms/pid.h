#ifndef __PID_H_
#define __PID_H_
#include "stm32h7xx_hal.h"
//#include "referee.h"

/**
  * @brief  单环PID 
  */
typedef struct 
{	
	float Kp, Ki, Kd;
	float Error;
	float iError;//累积误差
	float KpPart, KiPart, KdPart;
	
	float KiPartDetachment;
	float LastError;

	float Out;
}SinglePID_t;


/**
  * @brief  双环PID
  */
typedef struct 
{
	SinglePID_t* ShellPID;
	SinglePID_t* CorePID;
}DualPID_Object;
 

/**
  * @brief 单环PID初始化
  */
void BasePID_Init(SinglePID_t* base_pid, float kp, float ki, float kd, float detach);


/**
  * @brief 双环PID初始化
  */
void DualPID_Init(DualPID_Object* dual_pid, SinglePID_t* ShellPID,SinglePID_t* CorePID);


/**
  * @brief 单环比例积分速度控制
  */
int16_t BasePID_SpeedControl(SinglePID_t* base_pid, float target_speed, float feedback_speed);


/**
  * @brief 单环比例微分角度控制, 角速度不依靠IMU数据，仅靠编码器进行角度控制
  */
float  BasePID_AngleControl(SinglePID_t* base_pid, float target_angle, float feedback_angle);

float  BasePID_AngleControl_Swerve(SinglePID_t* base_pid, float target_angle, float feedback_angle);

void PID_Init(void);
extern SinglePID_t pid_load;
extern SinglePID_t pid_friction0;
extern SinglePID_t pid_friction1;
extern DualPID_Object pid_yaw_m;
extern DualPID_Object pid_yaw_s;
extern DualPID_Object pid_pitch;
extern SinglePID_t pid_run;
extern SinglePID_t pid_follow;
extern DualPID_Object pid_turn[4];
#endif

