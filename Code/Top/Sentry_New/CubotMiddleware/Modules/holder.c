#include "holder.h"
#include "DM_motor.h"
// #include "communication.h"
#include "driver_timer.h"
#include "et08.h"
#include "fast_math_functions.h"
#include "filter.h"
#include "ins.h"
#include "mpu6050.h"
#include "stm32h7xx_hal.h"
#include "user_lib.h"
#include <stdint.h>
#include "check.h"
#include "interboard.h"
#include "brain.h"
#include "DM_imu.h"

Holder_t Holder;
float Yaw_TD = 250;
float Pitch_TD = 400;
int k = 0;
float fliter = 0.9;
// volatile float DEBUG_tar = 0.0f;
const float Yaw_FFk = 18.0f;
const float Pitch_FFk = 12.0f;

float x = 0.45;
float y = 0.012;
float z = -0.2;
#define DEBUG_HOLDER 0
#if DEBUG_HOLDER == 2
float pitch_speed = 0.7f;
#endif
#if DEBUG_HOLDER == 3
float yaw_speed = 1.2f;
#endif

//#ifndef arm_cos_f32
//#define arm_cos_f32(AngleSpeed) cosf(AngleSpeed)
//#endif

float PitchFF_Gravity(float target) // 重力补偿前馈
{
    // 测试源数据https://www.desmos.com/calculator/ikerwae6cv
//    float ff1, ff2;
    target = -target;

//    const float A = 0.94273f;   // 水平时重力力矩
//    const float B = -0.063365f; // 重心偏角 (rad)
    const float pi = 3.1415926f;
    const float w = pi/180.0f;
//    const float a = 0.670588f;
//    const float b = -0.31886f;
//    ff1 = A * arm_cos_f32(float_constrain(target, -40, 36) * 2 * pi / 360 + B); // R^2 = 0.9921
//    ff2 = a * arm_cos_f32(float_constrain(target, -40, 36) * 2 * pi / 360 + b); // R^2 = 0.9836

    // float ff3;
    // const float a3 = 0.865;
    // const float b3 = -0.222;
	// const float c3 = -0.035;
    // ff3 = a3 * arm_cos_f32(float_constrain(target, -35, 38) * 2 * pi / 360 + b3)+c3;

//    return ((target + 45) * ff1 + (65 - target) * ff2) / 115;

//    const float n_below = -42.5;
//    const float n_top = 62;

//    return ((target - n_below) * ff1 + (n_top - target) * ff2) / (n_top - n_below);

    // return ff3;

    // return target < -18 ? float_constrain(ff2, 0.35, 0.55) :
    // target < 0 ? float_constrain((0.738015f - 0.540642f) / 18.0f * target + 0.738015f, 0.54, 0.74) :
    // float_constrain((ff1 + 2 * ff2) / 3, 0.66, 0.76);

    return -0.83f * arm_cos_f32(float_constrain(target,-37,37) * w - 3.04811f) - 0.045f;
}

float YawFF_Speed(float target)
{
    static float targets[6];
    for (int i = 5; i > 0; i --)
    {
        targets[i] = targets[i-1];
    }
    targets[0] = target;

    return Yaw_FFk * ( targets[0] + targets[1] + targets[2] - targets[3] -targets[4] - targets[5] ) / (6-1) / (6/2);
}

float PitchFF_Speed(float target)
{
    static float targets[6];
    for (int i = 5; i > 0; i --)
    {
        targets[i] = targets[i-1];
    }
    targets[0] = target;

    return Pitch_FFk * ( targets[0] + targets[1] + targets[2] - targets[3] -targets[4] - targets[5] ) / (6-1) / (6/2);
}

/**
 * @brief 云台初始化上板
 */
void HolderInit_Top(Holder_t *holder, DualPID_Object *pitch, DualPID_Object *yaw_s)
{
    DMiaoInit(&holder->Motors.Pitch, 0x06, 0x05, MIT);
    MotorInit(&holder->Motors.Yaw_S, 4085, Motor6020, CAN1, 0x205);
    DualPID_Init(&holder->Yaw_S.PID, yaw_s->ShellPID, yaw_s->CorePID);
    DualPID_Init(&holder->Pitch.PID, pitch->ShellPID, pitch->CorePID);
    holder->Pitch.Sensitivity = 0.00085f; //-0.0015f
    holder->Yaw_S.Sensitivity = 0.00085f;
}
/**
 * @brief 云台控制
 */
uint8_t yawFlag = 0;
void HolderControl_Top(Holder_t *holder, RC_Ctrl_ET *rc_ctrl)
{
#if DEBUG_HOLDER == 0
    if (rc_ctrl->rc.s2 == 1)
    {
        holder->Yaw_S.Target_Angle -= ((rc_ctrl->rc.ch2 - 1024) * holder->Yaw_S.Sensitivity);
    }
    if (rc_ctrl->rc.s2 == 1)
    {
        holder->Pitch.Target_Angle += ((rc_ctrl->rc.ch3 - 1024) * holder->Pitch.Sensitivity);
    }
    if (rc_ctrl->rc.s2 == 2 || Top.Referee.game_prograss == 4)
    {
        if (check_robot_state.Check_Usart.Check_vision == 1 || rc_ctrl->rc.s2 == 2)
        {
            if (Brain.Autoaim.mode == Cruise)
            {
				if(Brain.Autoaim.Mode == Outpost)
				{
					holder->Yaw_S.Target_Angle = 20 * sin(HAL_GetTick() / 300.0f);
					holder->Pitch.Target_Angle = 5 * sin(HAL_GetTick() / 150.0f) + 10;//20;
				}else
				{
					holder->Yaw_S.Target_Angle = 30 * sin(HAL_GetTick () / 200.0f);
					holder->Pitch.Target_Angle = 20 * sin(HAL_GetTick() / 100.0f);
				} 
            }
            else if (Brain.Autoaim.mode == Lock)
            {
                Holder_TD(&holder->Pitch, holder->Pitch.Target_Angle, Pitch_TD, 0.001);
                Holder_TD(&holder->Yaw_S, holder->Yaw_S.Target_Angle, Yaw_TD, 0.001);
            }
        }
    }
#endif

    holder->Yaw_S.Can_Angle = holder->Motors.Yaw_S.Data.Angle;
    holder->Yaw_S.Can_AngleSpeed = holder->Motors.Yaw_S.Data.AngleSpeed;
    holder->Yaw_S.GYRO_Angle = IMU_S.Attitude.yaw;
    holder->Yaw_S.GYRO_AngleSpeed = IMU_S.Attitude.gyro[2] - mpu6050.mpu6050_Data.gyro[2];

    holder->Pitch.Can_Angle = holder->Motors.Pitch.angle;
    holder->Pitch.Can_AngleSpeed = holder->Motors.Pitch.speed_rpm;
    holder->Pitch.GYRO_Angle = IMU_S.Attitude.roll;
    holder->Pitch.GYRO_AngleSpeed = IMU_S.Attitude.gyro[0];

    holder->Yaw_S.Target_Angle = float_constrain(holder->Yaw_S.Target_Angle, -35, 35);
    holder->Pitch.Target_Angle = float_constrain(holder->Pitch.Target_Angle, -34, 25.5f);

    // if (holder->Pitch.GYRO_Angle > 20)
    // {
    //     holder->Pitch.PID.ShellPID->Kp = 0.1f;//0.45
    //     holder->Pitch.PID.ShellPID->Ki = 0.006f;
    //     holder->Pitch.PID.ShellPID->Kd = -0.2f;
    //     holder->Pitch.PID.CorePID->Kp = 0.25f;//0.45
    //     holder->Pitch.PID.CorePID->Ki = 0;
    //     holder->Pitch.PID.CorePID->Kd = -3;//5
    // }
    // else
    // {
    //     holder->Pitch.PID.ShellPID->Kp = 0.4f;//0.5
    //     holder->Pitch.PID.ShellPID->Ki = 0.02f;
    //     holder->Pitch.PID.ShellPID->Kd = -0.005f;
    //     holder->Pitch.PID.CorePID->Kp = 0.45;//0.55
    //     holder->Pitch.PID.CorePID->Ki = 0;
    //     holder->Pitch.PID.CorePID->Kd = -6;//3
    // }

#if DEBUG_HOLDER == 0
    if ( 0 && rc_ctrl->rc.s2 == 2 && Brain.Autoaim.mode == Lock)
    {
        holder->Motors.Yaw_S.Data.Output = k * holder->Yaw_S.v2 + BasePID_SpeedControl(holder->Yaw_S.PID.CorePID,
                                                                                       BasePID_AngleControl(holder->Yaw_S.PID.ShellPID,
                                                                                                            holder->Yaw_S.v1,
                                                                                                            holder->Yaw_S.Can_Angle),
                                                                                       holder->Yaw_S.GYRO_AngleSpeed);
        holder->Motors.Pitch.motor_output =PitchFF_Gravity(holder->Pitch.GYRO_Angle); + 
                                            BasePID_SpeedControl(holder->Pitch.PID.CorePID,
                                                                BasePID_AngleControl(holder->Pitch.PID.ShellPID,
                                                                                    holder->Pitch.v1,
                                                                                    holder->Pitch.GYRO_Angle),
                                                                holder->Pitch.GYRO_AngleSpeed);
    }
    else
    {
        holder->Motors.Yaw_S.Data.Output = BasePID_SpeedControl(holder->Yaw_S.PID.CorePID,
                                                                BasePID_AngleControl(holder->Yaw_S.PID.ShellPID,
                                                                                    holder->Yaw_S.Target_Angle,
                                                                                    holder->Yaw_S.Can_Angle)
                                                                + YawFF_Speed(holder->Yaw_S.Target_Angle),
                                                                holder->Yaw_S.GYRO_AngleSpeed);
        holder->Motors.Pitch.motor_output =  PitchFF_Gravity(holder->Pitch.GYRO_Angle) +
                                            BasePID_SpeedControl(holder->Pitch.PID.CorePID,
                                                                BasePID_AngleControl(holder->Pitch.PID.ShellPID,
                                                                                    holder->Pitch.Target_Angle,
                                                                                    holder->Pitch.GYRO_Angle)
                                                                + PitchFF_Speed(holder->Pitch.Target_Angle),
                                                                holder->Pitch.GYRO_AngleSpeed);
    }

    holder->Motors.Pitch.motor_output = float_constrain(holder->Motors.Pitch.motor_output, -8, 8);
    DMiaoMitControl(&holder->Motors.Pitch, 0, 0, 0, 0, holder->Motors.Pitch.motor_output);
    MotorFillData(&holder->Motors.Yaw_S, holder->Motors.Yaw_S.Data.Output);
	//  MotorFillData(&holder->Motors.Yaw_S, 0);

#endif
#if DEBUG_HOLDER == 1
    // DMiaoMitControl(&holder->Motors.Pitch, 0, 0, 0, 0, DEBUG_tar);

    // DEBUG_tar = -INS_attitude->roll;
    holder->Pitch.Target_Angle = DEBUG_tar;
    holder->Motors.Pitch.motor_output = BasePID_SpeedControl(holder->Pitch.PID.CorePID,
                                                             BasePID_AngleControl(holder->Pitch.PID.ShellPID,
                                                                                  holder->Pitch.Target_Angle,
                                                                                  holder->Pitch.GYRO_Angle),
                                                             holder->Pitch.GYRO_AngleSpeed) +
                                        PitchFF_Gravity(holder->Pitch.GYRO_Angle);
    holder->Motors.Pitch.motor_output = float_constrain(holder->Motors.Pitch.motor_output, -10, 10);
    DMiaoMitControl(&holder->Motors.Pitch, 0, 0, 0, 0, holder->Motors.Pitch.motor_output);

#endif
#if DEBUG_HOLDER == 2
    holder->Motors.Pitch.motor_output = BasePID_SpeedControl(holder->Pitch.PID.CorePID,
                                                             pitch_speed,
                                                             holder->Pitch.GYRO_AngleSpeed) +
                                        PitchFF_Gravity(holder->Pitch.GYRO_Angle);
    if (holder->Pitch.GYRO_Angle > 20 && pitch_speed > 0)
    {
        pitch_speed = -pitch_speed;
    }
    if (holder->Pitch.GYRO_Angle < -30 && pitch_speed < 0)
    {
        pitch_speed = -pitch_speed;
        pitch_speed += 0.1f;
    }
    DMiaoMitControl(&holder->Motors.Pitch, 0, 0, 0, 0, holder->Motors.Pitch.motor_output);
#endif
#if DEBUG_HOLDER == 3
    // holder->Motors.Yaw_S.Data.Output = yaw_out;
    // if (holder->Yaw_S.Can_Angle > 30 && yaw_out > 0)
    // {
    //     yaw_out = -yaw_out;
    // }
    // if (holder->Yaw_S.Can_Angle < -30 && yaw_out < 0)
    // {
    //     yaw_out = -yaw_out;
    //     yaw_out += 100.0f;
    // }

    holder->Motors.Yaw_S.Data.Output = BasePID_SpeedControl(holder->Yaw_S.PID.CorePID,
                                                            yaw_speed,
                                                            holder->Yaw_S.GYRO_AngleSpeed);

    if (holder->Yaw_S.Can_Angle > 30 && yaw_speed > 0)
    {
        yaw_speed = -yaw_speed;
    }
    if (holder->Yaw_S.Can_Angle < -30 && yaw_speed < 0)
    {
        yaw_speed = -yaw_speed;
        yaw_speed += 0.1f;
    }

    holder->Pitch.Target_Angle = 0;

    holder->Motors.Pitch.motor_output = BasePID_SpeedControl(holder->Pitch.PID.CorePID,
                                                             BasePID_AngleControl(holder->Pitch.PID.ShellPID,
                                                                                  holder->Pitch.Target_Angle,
                                                                                  holder->Pitch.GYRO_Angle),
                                                             holder->Pitch.GYRO_AngleSpeed) +
                                        PitchFF_Gravity(holder->Pitch.GYRO_Angle);

    holder->Motors.Pitch.motor_output = float_constrain(holder->Motors.Pitch.motor_output, -10, 10);

    MotorFillData(&holder->Motors.Yaw_S, holder->Motors.Yaw_S.Data.Output);
    DMiaoMitControl(&holder->Motors.Pitch, 0, 0, 0, 0, holder->Motors.Pitch.motor_output);
#endif
#if DEBUG_HOLDER == 4
    holder->Pitch.Target_Angle = 0;
    holder->Yaw_S.Target_Angle = 5 * sin(HAL_GetTick() / 180.0f) - 30;

    holder->Motors.Yaw_S.Data.Output = BasePID_SpeedControl(holder->Yaw_S.PID.CorePID,
                                                            BasePID_AngleControl(holder->Yaw_S.PID.ShellPID,
                                                                                 holder->Yaw_S.Target_Angle,
                                                                                 holder->Yaw_S.Can_Angle),
                                                            holder->Yaw_S.GYRO_AngleSpeed);
    holder->Motors.Pitch.motor_output = BasePID_SpeedControl(holder->Pitch.PID.CorePID,
                                                             BasePID_AngleControl(holder->Pitch.PID.ShellPID,
                                                                                  holder->Pitch.Target_Angle,
                                                                                  holder->Pitch.GYRO_Angle),
                                                             holder->Pitch.GYRO_AngleSpeed) +
                                        PitchFF_Gravity(holder->Pitch.GYRO_Angle);
    holder->Motors.Pitch.motor_output = float_constrain(holder->Motors.Pitch.motor_output, -10, 10);
    // holder->Motors.Pitch.motor_output = float_constrain(holder->Motors.Pitch.motor_output, -50, 50);
    DMiaoMitControl(&holder->Motors.Pitch, 0, 0, 0, 0, holder->Motors.Pitch.motor_output);
    MotorFillData(&holder->Motors.Yaw_S, holder->Motors.Yaw_S.Data.Output);
#endif
}

float Holder_TD(struct Holder_Motor_Info *holder_info, float Expect, float r, float h)
{
    double fh = -r * r * (holder_info->v1 - Expect) - 2 * r * holder_info->v2;
    holder_info->v1 += holder_info->v2 * h;
    holder_info->v2 += fh * h;
    return holder_info->v1;
}
