#ifndef __PID_H_
#define __PID_H_
#include "stm32h7xx_hal.h"
//#include "referee.h"

/**
  * @brief  ����PID 
  */
typedef struct 
{	
	float Kp, Ki, Kd;
	float Error;
	float iError;//�ۻ����
	float KpPart, KiPart, KdPart;
	
	float KiPartDetachment;
	float LastError;

	float Out;
}SinglePID_t;


/**
  * @brief  ˫��PID
  */
typedef struct 
{
	SinglePID_t* ShellPID;
	SinglePID_t* CorePID;
}DualPID_Object;
 

/**
  * @brief ����PID��ʼ��
  */
void BasePID_Init(SinglePID_t* base_pid, float kp, float ki, float kd, float detach);


/**
  * @brief ˫��PID��ʼ��
  */
void DualPID_Init(DualPID_Object* dual_pid, SinglePID_t* ShellPID,SinglePID_t* CorePID);


/**
  * @brief �������������ٶȿ���
  */
int16_t BasePID_SpeedControl(SinglePID_t* base_pid, float target_speed, float feedback_speed);


/**
  * @brief ��������΢�ֽǶȿ���, ���ٶȲ�����IMU���ݣ��������������нǶȿ���
  */
float  BasePID_AngleControl(SinglePID_t* base_pid, float target_angle, float feedback_angle);

void PID_Init(void);
extern SinglePID_t pid_load;
extern SinglePID_t pid_friction0;
extern SinglePID_t pid_friction1;
extern DualPID_Object pid_yaw_m;
extern DualPID_Object pid_yaw_s;
extern DualPID_Object pid_pitch;
#endif

