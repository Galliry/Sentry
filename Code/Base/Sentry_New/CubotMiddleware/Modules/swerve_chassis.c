#include "swerve_chassis.h"
#include "user_lib.h"
SwerveChassis swervechassis;
float Vx_c;
float Vy_c;
void SwerveChassisSetSpeed(SwerveChassis* chassis);
void SwerveChassisInit(SwerveChassis* chassis,DualPID_Object* turn_pid,SinglePID_t* run_pid,SinglePID_t* follow_pid)
{
    uint8_t i;
    MotorInit(&chassis->Motors6020.motor[0],6859,Motor6020,CAN2,0x205);
    MotorInit(&chassis->Motors6020.motor[1],6191,Motor6020,CAN2,0x206);     
    MotorInit(&chassis->Motors6020.motor[2],7700,Motor6020,CAN2,0x207);
    MotorInit(&chassis->Motors6020.motor[3],4829,Motor6020,CAN1,0x208);
    MotorInit(&chassis->Motors3508.motor[0],0,Motor3508,CAN2,0x201);
    MotorInit(&chassis->Motors3508.motor[1],0,Motor3508,CAN2,0x202);
    MotorInit(&chassis->Motors3508.motor[2],0,Motor3508,CAN2,0x203);
    MotorInit(&chassis->Motors3508.motor[3],0,Motor3508,CAN1,0x204);
    for(i=0;i<4;i++)
    {
        DualPID_Init(&chassis->Motors6020.TurnPID[i],turn_pid[i].ShellPID,turn_pid[i].CorePID);
    }
	BasePID_Init(&chassis->Motors3508.RunPID[0],run_pid->Kp,run_pid->Ki,run_pid->Kd,run_pid->KiPartDetachment);
	BasePID_Init(&chassis->Motors3508.RunPID[1],run_pid->Kp,run_pid->Ki,run_pid->Kd,run_pid->KiPartDetachment);
	BasePID_Init(&chassis->Motors3508.RunPID[2],run_pid->Kp,run_pid->Ki,run_pid->Kd,run_pid->KiPartDetachment);
	BasePID_Init(&chassis->Motors3508.RunPID[3],run_pid->Kp,run_pid->Ki,run_pid->Kd,run_pid->KiPartDetachment);
	
	BasePID_Init(&chassis->Motors6020.FollowPID,follow_pid->Kp,follow_pid->Ki,follow_pid->Kd,follow_pid->KiPartDetachment);
	chassis->Movement.Vx_Sensitivity = 5;
	chassis->Movement.Vy_Sensitivity = 5;
}

void SwerveChassis_Control(SwerveChassis* chassis,Receive_t* rec)
{
    if(rec->Base.rc.rc_Ctrl_s1 == 2)
	{
		if(rec->Base.Lidar.Online == 0)
		{
			chassis->Movement.Vx_Move = 0;
			chassis->Movement.Vy_Move = 0;
			chassis->Movement.Omega = 0;
		}
		else
		{
			if(rec->Base.Lidar.Movemode == 0)
			{
			
			}else if(rec->Base.Lidar.Movemode == 1)
			{
				chassis->Movement.Vx_Move = rec->Base.Lidar.Vx * 2000;
				chassis->Movement.Vy_Move = rec->Base.Lidar.Vy * 2000;
				chassis->Movement.Omega = BasePID_SpeedControl(&chassis->Motors6020.FollowPID,103.0f,Holder.Motors.Yaw_M.angle);
			}
			
		}
		SwerveChassisSetSpeed(chassis);
	}
	else
	{
		chassis->Movement.Vx_Move = (rec->Base.rc.rc_Ctrl_ch1 - 1024) * chassis->Movement.Vx_Sensitivity;
		chassis->Movement.Vy_Move = (-1) * (rec->Base.rc.rc_Ctrl_ch0 - 1024) * chassis->Movement.Vy_Sensitivity;
		chassis->Movement.Omega = BasePID_SpeedControl(&chassis->Motors6020.FollowPID,103.0f,Holder.Motors.Yaw_M.angle);
		SwerveChassisSetSpeed(chassis);
	}
	
}

void SwerveChassisSetSpeed(SwerveChassis* chassis)
{
    float error;
	float angle = Holder.Motors.Yaw_M.angle_raw-1.80f;
	chassis->Movement.Vx = chassis->Movement.Vx_Move * cos(angle) - chassis->Movement.Vy_Move * sin(angle);
	chassis->Movement.Vy = chassis->Movement.Vy_Move * cos(angle) + chassis->Movement.Vx_Move * sin(angle);
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
        if(error > 180.0f) 
        {
            error = 360 - error;
        }
        else if(error < -180.0f)
        {
            error += 360;
        }
		error = fabsf(error);
        if(error > 90.0f)
        {
            chassis->Vectors.Target_Angle[i] = chassis->Vectors.UnTarget_Angle[i];
            chassis->Vectors.Velocity[i] = -chassis->Vectors.Velocity[i];
        }  
    }
	chassis->Motors3508.motor[0].Data.Output = BasePID_SpeedControl(&chassis->Motors3508.RunPID[0],-chassis->Vectors.Velocity[0],chassis->Motors3508.motor[0].Data.SpeedRPM);
	chassis->Motors3508.motor[1].Data.Output = BasePID_SpeedControl(&chassis->Motors3508.RunPID[1],chassis->Vectors.Velocity[1],chassis->Motors3508.motor[1].Data.SpeedRPM);
	chassis->Motors3508.motor[2].Data.Output = BasePID_SpeedControl(&chassis->Motors3508.RunPID[2],chassis->Vectors.Velocity[2],chassis->Motors3508.motor[2].Data.SpeedRPM);
	chassis->Motors3508.motor[3].Data.Output = BasePID_SpeedControl(&chassis->Motors3508.RunPID[3],-chassis->Vectors.Velocity[3],chassis->Motors3508.motor[3].Data.SpeedRPM);
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
    
}/*


static void ChassisMotionCtrl(HeroChassis_t *chassis){
    int i;

   if(chassis->chassisFlag.follow == 1 ) {
        float yaw_mech_angle = heroYaw.yawMotor.angle * 57.2957795f; 
        float yaw_center_deg = 0.58f * 57.2957795f;
        float current_error_deg = Angle_Limit(yaw_mech_angle - yaw_center_deg);
        if (ABS(current_error_deg) < 3.0f) current_error_deg = 0;

        chassis->movement.omega = One_Pid_Ctrl(0, current_error_deg, &chassis->rudderMotor.spinpid[0]);
    }

    if(ABS(chassis->movement.omega) < 10.0f) 
        chassis->movement.omega = 0;
        
    if (ABS(chassis->movement.v_x) < 5.0f) chassis->movement.v_x = 0;
    if (ABS(chassis->movement.raw_vy) < 5.0f) chassis->movement.raw_vy = 0;

    chassis->movement.v_y[0] = chassis->movement.raw_vy - chassis->movement.omega;
    chassis->movement.v_y[1] = chassis->movement.raw_vy + chassis->movement.omega;

    for (i = 0; i < 2; i++) {
       
        chassis->wheelMotor.raw_targetspeed = Sqrt(chassis->movement.v_x * chassis->movement.v_x + chassis->movement.v_y[i] * chassis->movement.v_y[i]);
        
        if ((chassis->wheelMotor.raw_targetspeed > 5.0f) || chassis->movement.omega != 0 ) {
            chassis->rudderMotor.raw_targetangle = atan2f(chassis->movement.v_x, chassis->movement.v_y[i]) * RtA;
        } else {
            chassis->wheelMotor.raw_targetspeed = 0;
        }
        
        chassis->rudderMotor.raw_deltaangle = Angle_Limit(chassis->rudderMotor.raw_targetangle - chassis->rudderMotor.m6020[i].treatedData.angle);
        
        if (fabs(chassis->rudderMotor.raw_deltaangle) > 95.0f) {
            if (chassis->rudderMotor.raw_deltaangle > 0) chassis->rudderMotor.raw_deltaangle -= 180.0f;
            else chassis->rudderMotor.raw_deltaangle += 180.0f;
            
            chassis->wheelMotor.raw_targetspeed = -chassis->wheelMotor.raw_targetspeed; 
        }
        
        chassis->rudderMotor.delta_angle[i] = chassis->rudderMotor.raw_deltaangle;
   
        if (fabs(chassis->rudderMotor.raw_deltaangle) > 45.0f) {
            chassis->wheelMotor.raw_targetspeed = 0;
        }
 
        if (i == 1 )
            chassis->wheelMotor.raw_targetspeed = -chassis->wheelMotor.raw_targetspeed;

        chassis->wheelMotor.target_speed[i] = chassis->wheelMotor.raw_targetspeed;

        #if(POWERCTRL == 0)
            chassis->wheelMotor.m3508[i].treatedData.motor_output = One_Pid_Ctrl(
                chassis->wheelMotor.target_speed[i],  
                chassis->wheelMotor.m3508[i].rawData.speed_rpm, 
                &chassis->wheelMotor.chassisSpeedPID[i]);
            
            chassis->rudderMotor.m6020[i].treatedData.motor_output = Double_Pid_Ctrl(
                0, 
                -chassis->rudderMotor.delta_angle[i], 
                chassis->rudderMotor.m6020[i].rawData.speed_rpm,
                &chassis->rudderMotor.chassisAnglePID[i]);
        #endif
    }
}
*/

