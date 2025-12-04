#ifndef SWERVE_CHASSIS_H
#define SWERVE_CHASSIS_H
#include "motor.h"
#include "pid.h"
#include "et08.h"
#include "dr16.h"

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
        SinglePID_t FollowPID;          //底盘跟随PID
    }Motors3508;
    
    struct 
    {
       Motor motor[4];
       DualPID_Object TurnPID[4];
    }Motors6020;
    
    struct
    {
        float Vx;
        float Vy;
        float Omega;
        float Vx_Sensitivity;       //灵敏度
		float Vy_Sensitivity;
        float ModuleOfSpeed;        //速度向量的模值
		float AngleOfSpeed;	        //速度向量的角度
		float K_OmeToSpeed;
		float Now_Omega;
		float Now_Vy;
		float Now_Vx;
    }Movement;

    struct 
	{ 
        float Vx[4];
		float Vy[4];
        float Velocity[4];
        float Theta[4];
        float Target_Angle[4];
        float UnTarget_Angle[4];
        // float Turn_To_Angle[4];
		// float Rc_Ctrl_ModuleOfSpeed;
		// float Rc_Ctrl_AngleOfSpeed;		
		// int16_t TargetEcd[4];
		// float FeedbackAngle[4];
		// float SpeedNo[4];
		// uint8_t SpeedChangeFlag[4];
	}Vectors;
}SwerveChassis;

void SwerveChassisInit(SwerveChassis* chassis,DualPID_Object* turn_pid,SinglePID_t* run_pid);
void SwerveChassis_Control(SwerveChassis* chassis,RC_Ctrl*  rc_ctrl);
extern SwerveChassis swervechassis;
#endif