#ifndef SWERVE_CHASSIS_H
#define SWERVE_CHASSIS_H
#include "motor.h"
#include "pid.h"
#include "et08.h"
#include "dr16.h"
#include "communication.h"
#include "holder.h"
#define Pi 3.1415926
// #define AtR 0.01745328  // pi/180 角度制转化为弧度制
#define COS_45_DEG 0.7071067812f
#define SIN_45_DEG 0.7071067812f

typedef struct
{
    struct
    {
        Motor motor[4];
        SinglePID_t RunPID[4];          //运动PID
    }Motors3508;
    
    struct 
    {
       Motor motor[4];
       DualPID_Object TurnPID[4];
		SinglePID_t FollowPID;          //底盘跟随PID
    }Motors6020;
    
    struct
    {
        float Vx;
        float Vy;
        float Omega;
        float Vx_Sensitivity;       //灵敏度
		float Vy_Sensitivity;
    }Movement;

    struct 
	{ 
        float Vx[4];
		float Vy[4];
        float Velocity[4];
        float Theta[4];
        float Target_Angle[4];
        float UnTarget_Angle[4];
	}Vectors;
}SwerveChassis;

void SwerveChassisInit(SwerveChassis* chassis,DualPID_Object* turn_pid,SinglePID_t* run_pid,SinglePID_t* follow_pid);
void SwerveChassis_Control(SwerveChassis* chassis,Receive_t* rec);
extern SwerveChassis swervechassis;
#endif
