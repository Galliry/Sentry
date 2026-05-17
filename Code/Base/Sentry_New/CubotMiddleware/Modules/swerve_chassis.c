#include "swerve_chassis.h"
#include "Supercap.h"
#include "control_logic.h"
#include "referee.h"
#include "user_lib.h"
#include "arm_math.h"
#ifndef PI
#define PI 3.1415926535f
#endif

float Chassis_Slew_Rate_Limiter(float target, float current, float accel_step, float decel_step);
float Calculate_Variable_Omega(float amplitude, float offset, float period_s, float dt_s);
void SwerveChassisSetSpeed(SwerveChassis *chassis);
SwerveChassis swervechassis;

void SwerveChassisInit(SwerveChassis *chassis, DualPID_Object *turn_pid, SinglePID_t *run_pid, SinglePID_t *follow_pid)
{
    uint8_t i;
    MotorInit(&chassis->Motors6020.motor[0], 6859, Motor6020, CAN2, 0x205);
    MotorInit(&chassis->Motors6020.motor[1], 6191, Motor6020, CAN2, 0x206);
    MotorInit(&chassis->Motors6020.motor[2], 7700, Motor6020, CAN2, 0x207);
    MotorInit(&chassis->Motors6020.motor[3], 4829, Motor6020, CAN1, 0x208);
    MotorInit(&chassis->Motors3508.motor[0], 0, Motor3508, CAN2, 0x201);
    MotorInit(&chassis->Motors3508.motor[1], 0, Motor3508, CAN2, 0x202);
    MotorInit(&chassis->Motors3508.motor[2], 0, Motor3508, CAN2, 0x203);
    MotorInit(&chassis->Motors3508.motor[3], 0, Motor3508, CAN1, 0x204);
    for (i = 0; i < 4; i++)
    {
        DualPID_Init(&chassis->Motors6020.TurnPID[i], turn_pid[i].ShellPID, turn_pid[i].CorePID);
    }
    BasePID_Init(&chassis->Motors3508.RunPID[0], run_pid->Kp, run_pid->Ki, run_pid->Kd, run_pid->KiPartDetachment);
    BasePID_Init(&chassis->Motors3508.RunPID[1], run_pid->Kp, run_pid->Ki, run_pid->Kd, run_pid->KiPartDetachment);
    BasePID_Init(&chassis->Motors3508.RunPID[2], run_pid->Kp, run_pid->Ki, run_pid->Kd, run_pid->KiPartDetachment);
    BasePID_Init(&chassis->Motors3508.RunPID[3], run_pid->Kp, run_pid->Ki, run_pid->Kd, run_pid->KiPartDetachment);

    BasePID_Init(&chassis->Motors6020.FollowPID, follow_pid->Kp, follow_pid->Ki, follow_pid->Kd, follow_pid->KiPartDetachment);
    chassis->Movement.Move_Sensitivity = 5;
    chassis->Movement.Lidar_Sensitivity = 2732.852;
	chassis->Movement.Posture = 3;
}

void SwerveChassis_Control(SwerveChassis *chassis, Base_t *rec)
{
    if (rec->Rc.rc_Ctrl_s1 == 2 || referee2022.game_status.game_progress == 4)
    {
        if(ABS(rec->Rc.rc_Ctrl_ch1 - 1024) > 300)
		{
			chassis->Movement.Vx_Tar = 0;
            chassis->Movement.Vy_Tar = 0;
			chassis->Movement.Omega = (rec->Rc.rc_Ctrl_ch1 - 1024) * 15;
			super_cap.cap_state.Supercap_Mode = 1;
			SwerveChassisSetSpeed(chassis);
		}
		else
		{
			super_cap.cap_state.Supercap_Mode = 0;
			if (rec->Lidar.isOnline == 0 && referee2022.game_status.game_progress == 4)
			{
				chassis->Movement.Vx_Tar = 0;
				chassis->Movement.Vy_Tar = 0;
				chassis->Movement.Omega = -8000;
				super_cap.cap_state.Supercap_Mode = 0;
				SwerveChassisSetSpeed(chassis);
			}
			else
			{
				if (rec->Lidar.Movemode == 1)
				{
					chassis->Movement.Vx_Tar = 0;
					chassis->Movement.Vy_Tar = 0;
					chassis->Movement.Posture = 1;	//进攻姿态
				}
				else if (rec->Lidar.Movemode == 0)
				{
					chassis->Movement.Vx_Tar = rec->Lidar.Vx * chassis->Movement.Lidar_Sensitivity;
					chassis->Movement.Vy_Tar = rec->Lidar.Vy * chassis->Movement.Lidar_Sensitivity;
					chassis->Movement.Posture = 3;	//移动姿态
				}
				else if (rec->Lidar.Movemode == 2)
				{
					chassis->Movement.Vx_Tar = rec->Lidar.Vx * chassis->Movement.Lidar_Sensitivity;
					chassis->Movement.Vy_Tar = rec->Lidar.Vy * chassis->Movement.Lidar_Sensitivity;
					chassis->Movement.Posture = 3;
				}
				
				chassis->Movement.Vx_Move = Chassis_Slew_Rate_Limiter(chassis->Movement.Vx_Tar,chassis->Movement.Vx_Move,15.0f,7.0f);
				chassis->Movement.Vy_Move = Chassis_Slew_Rate_Limiter(chassis->Movement.Vy_Tar,chassis->Movement.Vy_Move,15.0f,7.0f);
				if(referee2022.robot_hurt.hurt_type == 1 && referee2022.robot_hurt.armor_id != 0)
				{
					chassis->Movement.Omega = Calculate_Variable_Omega(2000,6000,2.0f,0.001f);
					if(referee2022.game_robot_status.remain_HP < 400)
						super_cap.cap_state.Supercap_Mode = 1;
				}else if(referee2022.game_status.stage_remain_time > 390 && referee2022.game_status.game_progress == 4)
				{
					super_cap.cap_state.Supercap_Mode = 1;
					chassis->Movement.Omega = BasePID_SpeedControl(&chassis->Motors6020.FollowPID, 103.28, Holder.Motors.Yaw_M.angle);
				}else if(Base.Lidar.Movemode == 2)
				{
					super_cap.cap_state.Supercap_Mode = 1;
					chassis->Movement.Omega = BasePID_SpeedControl(&chassis->Motors6020.FollowPID, 103.28, Holder.Motors.Yaw_M.angle);
				}
				else
				{
					chassis->Movement.Omega = BasePID_SpeedControl(&chassis->Motors6020.FollowPID, 103.28, Holder.Motors.Yaw_M.angle);
					super_cap.cap_state.Supercap_Mode = 0;
				}
				
				if (fabs(chassis->Movement.Vx_Move) <= 5 && fabs(chassis->Movement.Vy_Move) <= 5 && chassis->Movement.Omega == 0 && (rec->Lidar.Movemode == 0 || rec->Lidar.Movemode == 2))
				{
					;
				}
				else
					SwerveChassisSetSpeed(chassis);
			}
		}          
    }
    else
    {
        chassis->Movement.Vx_Tar = (rec->Rc.rc_Ctrl_ch1 - 1024) * chassis->Movement.Move_Sensitivity;
        chassis->Movement.Vy_Tar = (-1) * (rec->Rc.rc_Ctrl_ch0 - 1024) * chassis->Movement.Move_Sensitivity;
		chassis->Movement.Vx_Move = Chassis_Slew_Rate_Limiter(chassis->Movement.Vx_Tar,chassis->Movement.Vx_Move,15.0f,7.0f);
        chassis->Movement.Vy_Move = Chassis_Slew_Rate_Limiter(chassis->Movement.Vy_Tar,chassis->Movement.Vy_Move,15.0f,7.0f);
		chassis->Movement.Omega = BasePID_SpeedControl(&chassis->Motors6020.FollowPID, 103.28, Holder.Motors.Yaw_M.angle);
        super_cap.cap_state.Supercap_Mode = 0;
        SwerveChassisSetSpeed(chassis);
    }
}

void SwerveChassisSetSpeed(SwerveChassis *chassis)
{
    float error;
    float angle = (Holder.Motors.Yaw_M.angle_raw - 1.803f) + Holder.Motors.Yaw_M.speed_rpm * 0.0026f; // 前馈
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
    for (int i = 0; i < 4; i++)
    {
        chassis->Vectors.Velocity[i] = sqrt(chassis->Vectors.Vx[i] * chassis->Vectors.Vx[i] + chassis->Vectors.Vy[i] * chassis->Vectors.Vy[i]);
    }
    for (int i = 0; i < 4; i++)
    {
        chassis->Vectors.Theta[i] = atan2(chassis->Vectors.Vy[i], chassis->Vectors.Vx[i]) * 180 / Pi;
        chassis->Vectors.Target_Angle[i] = chassis->Vectors.Theta[i];
        error = chassis->Vectors.Target_Angle[i] + chassis->Motors6020.motor[i].Data.Angle;
        chassis->Vectors.UnTarget_Angle[i] = chassis->Vectors.Target_Angle[i] + 180;
        if (chassis->Vectors.UnTarget_Angle[i] > 180)
        {
            chassis->Vectors.UnTarget_Angle[i] = chassis->Vectors.Target_Angle[i] - 180;
        }
        if (error > 180.0f)
        {
            error = 360 - error;
        }
        else if (error < -180.0f)
        {
            error += 360;
        }
        error = fabsf(error);
        if (error > 90.0f)
        {
            chassis->Vectors.Target_Angle[i] = chassis->Vectors.UnTarget_Angle[i];
            chassis->Vectors.Velocity[i] = -chassis->Vectors.Velocity[i];
        }
    }
    chassis->Motors3508.motor[0].Data.Output = BasePID_SpeedControl(&chassis->Motors3508.RunPID[0], -chassis->Vectors.Velocity[0], chassis->Motors3508.motor[0].Data.SpeedRPM);
    chassis->Motors3508.motor[1].Data.Output = BasePID_SpeedControl(&chassis->Motors3508.RunPID[1], chassis->Vectors.Velocity[1], chassis->Motors3508.motor[1].Data.SpeedRPM);
    chassis->Motors3508.motor[2].Data.Output = BasePID_SpeedControl(&chassis->Motors3508.RunPID[2], chassis->Vectors.Velocity[2], chassis->Motors3508.motor[2].Data.SpeedRPM);
    chassis->Motors3508.motor[3].Data.Output = BasePID_SpeedControl(&chassis->Motors3508.RunPID[3], -chassis->Vectors.Velocity[3], chassis->Motors3508.motor[3].Data.SpeedRPM);
    for (int i = 0; i < 4; i++)
    {
        chassis->Motors6020.motor[i].Data.Output = BasePID_SpeedControl(chassis->Motors6020.TurnPID[i].CorePID,
                                                                        BasePID_AngleControl_Swerve(chassis->Motors6020.TurnPID[i].ShellPID, chassis->Vectors.Target_Angle[i], -(chassis->Motors6020.motor[i].Data.Angle)), chassis->Motors6020.motor[i].Data.SpeedRPM);
        // 限幅
        chassis->Motors6020.motor[i].Data.Output = float_constrain(chassis->Motors6020.motor[i].Data.Output, -16000, 16000);
        chassis->Motors3508.motor[i].Data.Output = float_constrain(chassis->Motors3508.motor[i].Data.Output, -16000, 16000);
    }
    SwerveChassisPowerCtrl(chassis);
}

static void SwerveChassisPowerCtrl(SwerveChassis *chassis)
{

    if (super_cap.cap_state.Supercap_Flag == 1)
    {
        if (chassis->Power.super_power < 150)
            chassis->Power.super_power += 0.03f;
        else
            chassis->Power.super_power = 150;
    }
    else
        chassis->Power.super_power = 0;

    chassis->Power.max_power = referee2022.game_robot_status.chassis_power_limit + (referee2022.power_heat_data.chassis_power_buffer - 15) * 5 + chassis->Power.super_power;

    if (chassis->Power.max_power < 0)
        chassis->Power.max_power = 0;

    chassis->Power.target_require_power_sum = 0;
    chassis->Power.turn_power = 0;

    for (int8_t i = 0; i < 4; i++)
    {
        chassis->Motors6020.initial_give_power[i] = chassis->Motors6020.motor[i].Data.Output * TORQUE_COEFFICIENT_6020 * chassis->Motors6020.motor[i].Data.SpeedRPM +
                                                    speed_term_k2_6020 * chassis->Motors6020.motor[i].Data.SpeedRPM * chassis->Motors6020.motor[i].Data.SpeedRPM +
                                                    torque_term_k1_6020 * chassis->Motors6020.motor[i].Data.Output * chassis->Motors6020.motor[i].Data.Output + CONSTANT_COEFFICIENT_6020;
        if (chassis->Motors6020.initial_give_power[i] < 0)
            continue; // negative power not included (transitory)
        chassis->Power.turn_power += chassis->Motors6020.initial_give_power[i];
    }
    for (int8_t i = 0; i < 4; i++)
    {
        chassis->Motors3508.initial_give_power[i] = chassis->Motors3508.motor[i].Data.Output * TORQUE_COEFFICIENT_3508 * chassis->Motors3508.motor[i].Data.SpeedRPM +
                                                    speed_term_k2_3508 * chassis->Motors3508.motor[i].Data.SpeedRPM * chassis->Motors3508.motor[i].Data.SpeedRPM +
                                                    torque_term_k1_3508 * chassis->Motors3508.motor[i].Data.Output * chassis->Motors3508.motor[i].Data.Output + CONSTANT_COEFFICIENT_3508;
        if (chassis->Motors3508.initial_give_power[i] < 0)
            continue; // negative power not included (transitory)
        chassis->Power.target_require_power_sum += chassis->Motors3508.initial_give_power[i];
    }

    chassis->Power.scaling_ratio = (chassis->Power.max_power - chassis->Power.turn_power) / chassis->Power.target_require_power_sum;
    if (chassis->Power.scaling_ratio > 1)
        chassis->Power.scaling_ratio = 1;

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
                    chassis->Motors3508.motor[i].Data.Output = 16000;
                }
                else
                    chassis->Motors3508.motor[i].Data.Output = temp;
            }
            else
            {
                double temp = (-b - sqrt(b * b - 4 * torque_term_k1_3508 * c)) / (2 * torque_term_k1_3508);
                if (temp < -16000)
                {
                    chassis->Motors3508.motor[i].Data.Output = -16000;
                }
                else
                    chassis->Motors3508.motor[i].Data.Output = temp;
            }
        }
    }

    for (int j = 0; j < 4; j++)
    {
        chassis->Motors6020.motor[j].Data.Output = float_constrain(chassis->Motors6020.motor[j].Data.Output, -16000, 16000);
        chassis->Motors3508.motor[j].Data.Output = float_constrain(chassis->Motors3508.motor[j].Data.Output, -16000, 16000);
        MotorFillData(&chassis->Motors6020.motor[j], chassis->Motors6020.motor[j].Data.Output);
        MotorFillData(&chassis->Motors3508.motor[j], chassis->Motors3508.motor[j].Data.Output);
    } // 最后上一层保险
}
// 防止急停翘头
float Chassis_Slew_Rate_Limiter(float target, float current, float accel_step, float decel_step) 
{
    if (target == current) return current;
    
    int is_accelerating = (fabsf(target) > fabsf(current)) && ((target * current) >= 0);
    
    float step = is_accelerating ? accel_step : decel_step;
    
    if (target > current) 
    {
        current += step;
        if (current > target) current = target;
    } 
    else 
    {
        current -= step;
        if (current < target) current = target;
    }
    
    return current;
}
/**
 * @brief  生成正弦波动的角速度 (或任意随时间正弦变化的变量)
 * @param  amplitude: 波动振幅 
 * @param  offset:    中心偏置 
 * @param  period_s:  完成一次完整波动的时间，单位：秒
 * @param  dt_s:      该函数被调用的周期，单位：秒
 * @retval 当前时刻应该输出的目标值
 */
float Calculate_Variable_Omega(float amplitude, float offset, float period_s, float dt_s)
{
    static float phase_angle = 0.0f;	//相位角
    if (period_s <= 0.0f){return offset;}

    float angle_step = (2.0f * PI) * (dt_s / period_s);	//步长
    phase_angle += angle_step;	// 累加相位
    if (phase_angle >= 2.0f * PI)
    {
        phase_angle -= 2.0f * PI;
    }
    return (amplitude * arm_sin_f32(phase_angle)) + offset;
}
