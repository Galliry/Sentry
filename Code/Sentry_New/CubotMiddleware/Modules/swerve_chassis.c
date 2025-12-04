#include "swerve_chassis.h"
#include "user_lib.h"

float kkk[4];
SwerveChassis swervechassis;
void SwerveChassisSetSpeed(SwerveChassis* chassis);
void SwerveChassisInit(SwerveChassis* chassis,DualPID_Object* turn_pid,SinglePID_t* run_pid)
{
    uint8_t i;
    MotorInit(&chassis->Motors6020.motor[0],1045,Motor6020,CAN2,0x205);
    MotorInit(&chassis->Motors6020.motor[1],2991,Motor6020,CAN1,0x206);
    MotorInit(&chassis->Motors6020.motor[2],12590,Motor6020,CAN2,0x207);
    MotorInit(&chassis->Motors6020.motor[3],5981,Motor6020,CAN2,0x208);
    MotorInit(&chassis->Motors3508.motor[0],0,Motor3508,CAN2,0x201);
    MotorInit(&chassis->Motors3508.motor[1],0,Motor3508,CAN2,0x202);
    MotorInit(&chassis->Motors3508.motor[2],0,Motor3508,CAN2,0x203);
    MotorInit(&chassis->Motors3508.motor[3],0,Motor3508,CAN2,0x204);
    for(i=0;i<4;i++)
    {
        DualPID_Init(&chassis->Motors6020.TurnPID[i],turn_pid[i].ShellPID,turn_pid[i].CorePID);
    }
	BasePID_Init(&chassis->Motors3508.RunPID[0],run_pid->Kp,run_pid->Kd,run_pid->Kd,run_pid->KiPartDetachment);
	BasePID_Init(&chassis->Motors3508.RunPID[1],-run_pid->Kp,run_pid->Kd,run_pid->Kd,run_pid->KiPartDetachment);
	BasePID_Init(&chassis->Motors3508.RunPID[2],-run_pid->Kp,run_pid->Kd,run_pid->Kd,run_pid->KiPartDetachment);
	BasePID_Init(&chassis->Motors3508.RunPID[3],run_pid->Kp,run_pid->Kd,run_pid->Kd,run_pid->KiPartDetachment);;
	chassis->Movement.Vx_Sensitivity = 5;
	chassis->Movement.Vy_Sensitivity = 5;
}

void SwerveChassis_Control(SwerveChassis* chassis,RC_Ctrl*  rc_ctrl)
{
    chassis->Movement.Vx = (rc_ctrl->rc.ch1 - 1024) * chassis->Movement.Vx_Sensitivity;
    chassis->Movement.Vy = (-1) * (rc_ctrl->rc.ch0 - 1024) * chassis->Movement.Vy_Sensitivity;
    chassis->Movement.Omega = (rc_ctrl->rc.ch2 - 1024) * chassis->Movement.Vx_Sensitivity;
	SwerveChassisSetSpeed(chassis);
}

void SwerveChassisSetSpeed(SwerveChassis* chassis)
{
    float error;
    chassis->Vectors.Vx[0] = chassis->Movement.Vx + chassis->Movement.Omega * COS_45_DEG;
    chassis->Vectors.Vy[0] = chassis->Movement.Vy - chassis->Movement.Omega * SIN_45_DEG;
    chassis->Vectors.Vx[1] = chassis->Movement.Vx - chassis->Movement.Omega * COS_45_DEG;
    chassis->Vectors.Vy[1] = chassis->Movement.Vy - chassis->Movement.Omega * SIN_45_DEG;
    chassis->Vectors.Vx[2] = chassis->Movement.Vx - chassis->Movement.Omega * COS_45_DEG;
    chassis->Vectors.Vy[2] = chassis->Movement.Vy + chassis->Movement.Omega * SIN_45_DEG;
    chassis->Vectors.Vx[3] = chassis->Movement.Vx + chassis->Movement.Omega * COS_45_DEG;
    chassis->Vectors.Vy[3] = chassis->Movement.Vy + chassis->Movement.Omega * SIN_45_DEG;
    for(int i=0;i<4;i++)
    {
        chassis->Vectors.Velocity[i] = sqrt(chassis->Vectors.Vx[i]*chassis->Vectors.Vx[i]+chassis->Vectors.Vy[i]*chassis->Vectors.Vy[i]);
    }
    for(int i=0;i<4;i++)
    {
        chassis->Vectors.Theta[i] = atan2(chassis->Vectors.Vy[i],chassis->Vectors.Vx[i]) * 180 / Pi;
		chassis->Vectors.Target_Angle[i] = chassis->Vectors.Theta[i];
        error = chassis->Vectors.Target_Angle[i] + chassis->Motors6020.motor[i].Data.Angle;
        chassis->Vectors.UnTarget_Angle[i] = chassis->Vectors.Target_Angle[i] + 180;
        if(chassis->Vectors.UnTarget_Angle[i] > 180)
        {
            chassis->Vectors.UnTarget_Angle[i] = chassis->Vectors.Target_Angle[i] - 180;
        }
        if(error > 180) 
        {
            error = 360 - error;
        }
        else if(error < -180)
        {
            error += 360;
        }
		error = fabsf(error);
        if(error > 91.0f)
        {
            chassis->Vectors.Target_Angle[i] = chassis->Vectors.UnTarget_Angle[i];
            chassis->Vectors.Velocity[i] = -chassis->Vectors.Velocity[i];
        }
        
    }
	chassis->Motors3508.motor[0].Data.Output = BasePID_SpeedControl(&chassis->Motors3508.RunPID[0],chassis->Vectors.Velocity[0],chassis->Motors3508.motor[0].Data.SpeedRPM);
	chassis->Motors3508.motor[1].Data.Output = BasePID_SpeedControl(&chassis->Motors3508.RunPID[1],chassis->Vectors.Velocity[1],-chassis->Motors3508.motor[1].Data.SpeedRPM);
	chassis->Motors3508.motor[2].Data.Output = BasePID_SpeedControl(&chassis->Motors3508.RunPID[2],chassis->Vectors.Velocity[2],-chassis->Motors3508.motor[2].Data.SpeedRPM);
	chassis->Motors3508.motor[3].Data.Output = BasePID_SpeedControl(&chassis->Motors3508.RunPID[3],chassis->Vectors.Velocity[3],chassis->Motors3508.motor[3].Data.SpeedRPM);
	for(int i=0;i<4;i++)
    {
        chassis->Motors6020.motor[i].Data.Output = BasePID_SpeedControl(chassis->Motors6020.TurnPID[i].CorePID,
		    BasePID_AngleControl_Swerve(chassis->Motors6020.TurnPID[i].ShellPID,chassis->Vectors.Target_Angle[i],-(chassis->Motors6020.motor[i].Data.Angle)),chassis->Motors6020.motor[i].Data.SpeedRPM);    
        
		//оч╥Ы
        chassis->Motors6020.motor[i].Data.Output = float_constrain(chassis->Motors6020.motor[i].Data.Output,-16000,16000);
		chassis->Motors3508.motor[i].Data.Output = float_constrain(chassis->Motors3508.motor[i].Data.Output,-16000,16000);
        MotorFillData(&chassis->Motors6020.motor[i],chassis->Motors6020.motor[i].Data.Output);
        MotorFillData(&chassis->Motors3508.motor[i],chassis->Motors3508.motor[i].Data.Output);
    }
    
}