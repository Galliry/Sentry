#include "holder.h"
#include "DM_motor.h"
#include "communication.h"
#include "driver_timer.h"
#include "et08.h"
#include "fast_math_functions.h"
#include "filter.h"
#include "ins.h"
#include "mpu6050.h"
#include "user_lib.h"

Holder_t Holder;
float Yaw_TD = 200;
float Pitch_TD = 90;
int k = 0;
float fliter = 0.9;
volatile float DEBUG_tar = 0.0f;

#define DEBUG_HOLDER 0

#ifndef arm_cos_f32
#define arm_cos_f32(x) cosf(x)
#endif

float PitchFF_Gravity(float target) // 重力补偿前馈
{
    // 测试源数据https://www.desmos.com/calculator/ikerwae6cv
    float ff1, ff2;
    target = -target;

    const float A = 0.94273f;   // 水平时重力力矩
    const float B = -0.063365f; // 重心偏角 (rad)
    const float pi = 3.1415926f;
    const float a = 0.670588f;
    const float b = -0.31886f;
    ff1 = A * arm_cos_f32(float_constrain(target, -35, 38) * 2 * pi / 360 + B); // R^2 = 0.9921
    ff2 = a * arm_cos_f32(float_constrain(target, -35, 38) * 2 * pi / 360 + b); // R^2 = 0.9836

    return ((target + 45) * ff1 + (65 - target) * ff2) / 110;
    // return target < -18 ? float_constrain(ff2, 0.35, 0.55) :
    // target < 0 ? float_constrain((0.738015f - 0.540642f) / 18.0f * target + 0.738015f, 0.54, 0.74) :
    // float_constrain((ff1 + 2 * ff2) / 3, 0.66, 0.76);
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
void HolderControl_Top(Holder_t *holder, RC_Ctrl_ET *rc_ctrl)
{
#if DEBUG_HOLDER == 0
    if (rc_ctrl->rc.s2 == 1)
    {
        holder->Yaw_S.Target_Angle -= ((rc_ctrl->rc.ch2 - 1024) * holder->Yaw_S.Sensitivity);
    }
    holder->Pitch.Target_Angle += ((rc_ctrl->rc.ch3 - 1024) * holder->Pitch.Sensitivity);
    if (rc_ctrl->rc.s2 == 2 || Top.Referee.game_prograss == 4)
    {
        if (check_robot_state.Check_Usart.Check_vision == 1 || rc_ctrl->rc.s2 == 2)
        {
            if (Brain.Autoaim.mode == Cruise)
            {
                //				holder->Yaw_S.Target_Angle = 30 * sin(HAL_GetTick () / 200.0f);
                holder->Yaw_S.Target_Angle = 0;
                holder->Pitch.Target_Angle = 8 * sin(HAL_GetTick() / 100.0f) - 7;
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
    holder->Yaw_S.GYRO_Angle = INS_attitude->yaw;
    holder->Yaw_S.GYRO_AngleSpeed = INS_attitude->gyro[2] - mpu6050.mpu6050_Data.gyro[2];

    holder->Pitch.Can_Angle = holder->Motors.Pitch.angle;
    holder->Pitch.Can_AngleSpeed = holder->Motors.Pitch.speed_rpm;
    holder->Pitch.GYRO_Angle = -INS_attitude->roll;
    holder->Pitch.GYRO_AngleSpeed = -INS_attitude->gyro[1];

    holder->Yaw_S.Target_Angle = float_constrain(holder->Yaw_S.Target_Angle, -38, 38);
    holder->Pitch.Target_Angle = float_constrain(holder->Pitch.Target_Angle, -30, 30);

    holder->Yaw_S.Target_Angle = LPFilter(holder->Yaw_S.Target_Angle, &LPF_yaw_mpu);
    holder->Pitch.Target_Angle = LPFilter(holder->Pitch.Target_Angle, &LPF_pitch_mpu);

    holder->Yaw_S.Target_Angle = float_constrain(holder->Yaw_S.Target_Angle, -38, 38);
    holder->Pitch.Target_Angle = float_constrain(holder->Pitch.Target_Angle, -30, 30);

#if DEBUG_HOLDER == 0
    if (rc_ctrl->rc.s2 == 2 && Brain.Autoaim.mode == Lock)
    {
        holder->Motors.Yaw_S.Data.Output = k * holder->Yaw_S.v2 + BasePID_SpeedControl(holder->Yaw_S.PID.CorePID,
                                                                                       BasePID_AngleControl(holder->Yaw_S.PID.ShellPID, holder->Yaw_S.v1, holder->Yaw_S.Can_Angle), holder->Yaw_S.GYRO_AngleSpeed);
        holder->Motors.Pitch.motor_output = BasePID_SpeedControl(holder->Pitch.PID.CorePID,
                                                                 BasePID_AngleControl(holder->Pitch.PID.ShellPID,
                                                                                      holder->Pitch.v1,
                                                                                      holder->Pitch.GYRO_Angle),
                                                                 holder->Pitch.GYRO_AngleSpeed) +
                                            PitchFF_Gravity(holder->Pitch.GYRO_Angle);
    }
    else
    {
        holder->Motors.Yaw_S.Data.Output = BasePID_SpeedControl(holder->Yaw_S.PID.CorePID,
                                                                BasePID_AngleControl(holder->Yaw_S.PID.ShellPID, holder->Yaw_S.Target_Angle, holder->Yaw_S.Can_Angle), holder->Yaw_S.GYRO_AngleSpeed);
        holder->Motors.Pitch.motor_output = BasePID_SpeedControl(holder->Pitch.PID.CorePID,
                                                                 BasePID_AngleControl(holder->Pitch.PID.ShellPID,
                                                                                      holder->Pitch.Target_Angle,
                                                                                      holder->Pitch.GYRO_Angle),
                                                                 holder->Pitch.GYRO_AngleSpeed) +
                                            PitchFF_Gravity(holder->Pitch.GYRO_Angle);
    }

    holder->Motors.Pitch.motor_output = float_constrain(holder->Motors.Pitch.motor_output, -10, 10);
    DMiaoMitControl(&holder->Motors.Pitch, 0, 0, 0, 0, holder->Motors.Pitch.motor_output);
    MotorFillData(&holder->Motors.Yaw_S, holder->Motors.Yaw_S.Data.Output);

#else
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
}

float Holder_TD(struct Holder_Motor_Info *holder_info, float Expect, float r, float h)
{
    double fh = -r * r * (holder_info->v1 - Expect) - 2 * r * holder_info->v2;
    holder_info->v1 += holder_info->v2 * h;
    holder_info->v2 += fh * h;
    return holder_info->v1;
}