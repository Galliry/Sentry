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
				chassis->Movement.Vx_Move = rec->Base.Lidar.Vx * 2500;
				chassis->Movement.Vy_Move = rec->Base.Lidar.Vy * 2500;
			}else if(rec->Base.Lidar.Movemode == 1)
			{
				chassis->Movement.Vx_Move = rec->Base.Lidar.Vx * 2500;
				chassis->Movement.Vy_Move = rec->Base.Lidar.Vy * 2500;
			}
			chassis->Movement.Omega = BasePID_SpeedControl(&chassis->Motors6020.FollowPID,103.0f,Holder.Motors.Yaw_M.angle);
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
		//限幅
        chassis->Motors6020.motor[i].Data.Output = float_constrain(chassis->Motors6020.motor[i].Data.Output,-16000,16000);
		chassis->Motors3508.motor[i].Data.Output = float_constrain(chassis->Motors3508.motor[i].Data.Output,-16000,16000);
    }
	SwerveChassisPowerCtrl(chassis);
}


static void SwerveChassisPowerCtrl(SwerveChassis *chassis)
{
	chassis->Power.now_power = referee2022.power_heat_data.chassis_power; // 实时功率			
//	chassis->Power.max_power = referee2024.robot_status_t.chassis_power_limit + (referee2024.power_heat_data_t.buffer_energy - 15) * 5 + sup_power;       // 功率上限			
	if(Receive.Base.Lidar.Movemode == 0)
	chassis->Power.max_power = 150;      // 功率上限			
	else if(Receive.Base.Lidar.Movemode == 1)
		chassis->Power.max_power = 50;
	else if(Receive.Base.Lidar.Movemode == 2)
		chassis->Power.max_power = 50;
	if(chassis->Power.max_power < 0) chassis->Power.max_power = 0;
			
//	if(referee2022.power_heat_data.chassis_power_buffer > 40  )//加速起步且在不开超电时保证功率利用
//	{
//		chassis->Power.max_power += 100;
//	}
		

	chassis->Power.target_require_power_sum = 0;
	chassis->Power.turn_power=0;
	
    for (int8_t i = 0; i < 4; i++) 
	{
		chassis->Motors6020.initial_give_power[i] = chassis->Motors6020.motor[i].Data.Output * TORQUE_COEFFICIENT_6020 * chassis->Motors6020.motor[i].Data.SpeedRPM +
														speed_term_k2_6020 * chassis->Motors6020.motor[i].Data.SpeedRPM * chassis->Motors6020.motor[i].Data.SpeedRPM +
														torque_term_k1_6020 * chassis->Motors6020.motor[i].Data.Output * chassis->Motors6020.motor[i].Data.Output + CONSTANT_COEFFICIENT_6020;
		if (chassis->Motors6020.initial_give_power[i] < 0) continue;	// negative power not included (transitory)          
		chassis->Power.turn_power += chassis->Motors6020.initial_give_power[i];
    }
	for (int8_t i = 0; i < 4; i++) 
	{
		chassis->Motors3508.initial_give_power[i] = chassis->Motors3508.motor[i].Data.Output * TORQUE_COEFFICIENT_3508 * chassis->Motors3508.motor[i].Data.SpeedRPM +
														speed_term_k2_3508 * chassis->Motors3508.motor[i].Data.SpeedRPM * chassis->Motors3508.motor[i].Data.SpeedRPM +
														torque_term_k1_3508 * chassis->Motors3508.motor[i].Data.Output * chassis->Motors3508.motor[i].Data.Output + CONSTANT_COEFFICIENT_3508;
        if (chassis->Motors3508.initial_give_power[i] < 0) continue;// negative power not included (transitory)
        chassis->Power.target_require_power_sum += chassis->Motors3508.initial_give_power[i];
	}

    chassis->Power.scaling_ratio = (chassis->Power.max_power - chassis->Power.turn_power) / chassis->Power.target_require_power_sum;
    if(chassis->Power.scaling_ratio > 1) chassis->Power.scaling_ratio = 1;
			
    for (uint8_t i = 0; i < 4; i++) 
	{
		if (chassis->Power.scaling_ratio == 1) 
		{
			chassis->Motors3508.scaled_give_power[i] = chassis->Motors3508.initial_give_power[i];
			continue;		
		}
		else
		{
			chassis->Motors3508.scaled_give_power[i] = chassis->Motors3508.initial_give_power[i] * chassis->Power.scaling_ratio; // get scaled power

			if (chassis->Motors3508.scaled_give_power[i] < 0)
			{
				chassis->Motors3508.motor[i].Data.Output = 0;
				continue;
			}
			float b = TORQUE_COEFFICIENT_3508 * chassis->Motors3508.motor[i].Data.SpeedRPM;
			float c = speed_term_k2_3508 * chassis->Motors3508.motor[i].Data.SpeedRPM * chassis->Motors3508.motor[i].Data.SpeedRPM - chassis->Motors3508.scaled_give_power[i] + CONSTANT_COEFFICIENT_3508;

			if (chassis->Motors3508.motor[i].Data.Output > 0) // Selection of the calculation formula according to the direction of the original moment
			{
				double temp = (-b + sqrt(b * b - 4 * torque_term_k1_3508 * c)) / (2 * torque_term_k1_3508);
				if (temp > 16000) 
				{
					chassis->Motors3508.motor[i].Data.Output  = 16000;
				} 
				else chassis->Motors3508.motor[i].Data.Output  = temp;				
			}
			else 
			{
				double temp = (-b - sqrt(b * b - 4 * torque_term_k1_3508 * c)) / (2 * torque_term_k1_3508);
				if (temp < -16000) 
				{
					chassis->Motors3508.motor[i].Data.Output  = -16000;
				} 
				else chassis->Motors3508.motor[i].Data.Output  = temp;
			}
		}
	}

	for(int j =0;j < 4;j++)
	{
		chassis->Motors6020.motor[j].Data.Output = float_constrain(chassis->Motors6020.motor[j].Data.Output,-16000,16000);
		chassis->Motors3508.motor[j].Data.Output = float_constrain(chassis->Motors3508.motor[j].Data.Output,-16000,16000);
		MotorFillData(&chassis->Motors6020.motor[j],chassis->Motors6020.motor[j].Data.Output);
        MotorFillData(&chassis->Motors3508.motor[j],chassis->Motors3508.motor[j].Data.Output);
	}//最后上一层保险
	
	
}
