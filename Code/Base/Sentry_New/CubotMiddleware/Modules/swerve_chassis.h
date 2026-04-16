#ifndef SWERVE_CHASSIS_H
#define SWERVE_CHASSIS_H
#include "motor.h"
#include "pid.h"
#include "et08.h"
#include "dr16.h"
#include "communication.h"
#include "holder.h"
#include "referee.h"
#include "interboard.h"
#include "Supercap.h"

#define Pi 3.1415926
#define COS_45_DEG 0.7071067812f
#define SIN_45_DEG 0.7071067812f
#define TORQUE_COEFFICIENT_6020 1.421e-05
#define speed_term_k2_6020 4.0e-08
#define torque_term_k1_6020 2.0e-7
#define CONSTANT_COEFFICIENT_6020 0.73
#define torque_term_k1_3508 1.9e-07
#define speed_term_k2_3508  4.0e-07
#define TORQUE_COEFFICIENT_3508   1.99688994e-6f
#define CONSTANT_COEFFICIENT_3508 0.71f
typedef struct
{
    struct
    {
        Motor motor[4];
        SinglePID_t RunPID[4];          //运动PID
		float initial_give_power[4];
		float scaled_give_power[4];
    }Motors3508;
    
    struct 
    {
		Motor motor[4];
		DualPID_Object TurnPID[4];
		SinglePID_t FollowPID;          //底盘跟随PID
		float initial_give_power[4];
		float scaled_give_power[4];
    }Motors6020;
    
    struct
    {
        float Vx;
        float Vy;
		float Vx_Move;
		float Vy_Move;
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
	
	struct
	{
		float super_power;
		float max_power;
		float target_require_power_sum;
		float turn_power;
		double scaling_ratio;
	}Power;
}SwerveChassis;

void SwerveChassisInit(SwerveChassis* chassis,DualPID_Object* turn_pid,SinglePID_t* run_pid,SinglePID_t* follow_pid);
void SwerveChassis_Control(SwerveChassis* chassis,Base_t* rec);
static void SwerveChassisPowerCtrl(SwerveChassis *chassis);
void SwerveChassisSetSpeed(SwerveChassis* chassis);
extern SwerveChassis swervechassis;
#endif
