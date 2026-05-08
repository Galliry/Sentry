#include "holder.h"
#include "DM_motor.h"
#include "communication.h"
#include "driver_timer.h"
#include "et08.h"
#include "fast_math_functions.h"
#include "filter.h"
#include "ins.h"
#include "mpu6050.h"
#include "stm32h7xx_hal.h"
#include "user_lib.h"
#include <stdint.h>

Holder_t Holder;
float Yaw_TD = 240;
float Pitch_TD = 400;
int k = 0;
float fliter = 0.9;
volatile float DEBUG_tar = 0.0f;

#define DEBUG_HOLDER 0
#if DEBUG_HOLDER == 2
float pitch_speed = 0.7f;
#endif
#if DEBUG_HOLDER == 3
float yaw_speed = 1.2f;
#endif

#ifndef arm_cos_f32
#define arm_cos_f32(AngleSpeed) cosf(AngleSpeed)
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
    ff1 = A * arm_cos_f32(float_constrain(target, -40, 36) * 2 * pi / 360 + B); // R^2 = 0.9921
    ff2 = a * arm_cos_f32(float_constrain(target, -40, 36) * 2 * pi / 360 + b); // R^2 = 0.9836

    // float ff3;
    // const float a3 = (A+a)/2;
    // const float b3 = (B+b)/2;
    // ff3 = a3 * arm_cos_f32(float_constrain(target, -35, 38) * 2 * pi / 360 + b3);

    // return ((target + 45) * ff1 + (65 - target) * ff2) / 115;

    const float n_below = -46.5;
    const float n_top = 62;

    return ((target - n_below) * ff1 + (n_top - target) * ff2) / (n_top - n_below);

    // return ff3;

    // return target < -18 ? float_constrain(ff2, 0.35, 0.55) :
    // target < 0 ? float_constrain((0.738015f - 0.540642f) / 18.0f * target + 0.738015f, 0.54, 0.74) :
    // float_constrain((ff1 + 2 * ff2) / 3, 0.66, 0.76);
}

const float pitch_k = 0.0f;//6
const float yaw_k = 0.0f;//8

#define TarCache 6
float yawTar[TarCache];
float pitchTar[TarCache];

float SG_dYaw(float Target)
{
    for (int i = TarCache - 1; i > 0; i--)
    {
        yawTar[i] = yawTar[i - 1];
    }
    yawTar[0] = Target;
    float res = 0;
    for (int i = 0; i < TarCache; i++)
    {
        if (i < TarCache / 2)
        {
            res += yawTar[i];
        }
        else
        {
            res -= yawTar[i];
        }
    }
    return res / TarCache * 2;
}

float SG_dPitch(float Target)
{
    for (int i = TarCache - 1; i > 0; i--)
    {
        pitchTar[i] = pitchTar[i - 1];
    }
    pitchTar[0] = Target;
    float res = 0;
    for (int i = 0; i < TarCache; i++)
    {
        if (i < TarCache / 2)
        {
            res += pitchTar[i];
        }
        else
        {
            res -= pitchTar[i];
        }
    }
    return res / TarCache * 2;
}

float YawFF_Speed(float Target)
{
    return yaw_k * SG_dYaw(Target);
}

float PitchFF_Speed(float Target)
{
    return pitch_k * SG_dPitch(Target);
}

// float YawFF_Friction(float AngleSpeed, float Angle)
// {
//     // const float a = 5962.9f;
//     // const float b = 1.7597f;
//     // const float c = -67.2418f;
//     // const float d = -984.3845f;
//     // const float e = 811.0453f;
//     // const float f = e * 0.0136f;
//     // const float h = e * -0.0014f;
//     // const float g = e * -8.5133e-5;

//     // return a*atanf(b*AngleSpeed)+c*AngleSpeed*AngleSpeed+d*AngleSpeed
//     // + e + f*Angle + h*Angle*Angle + g*Angle*Angle*Angle;

//     const float a = 1978.5f;
//     const float b = 14.7484f;
//     const float c = 97.0689f;
//     const float d = 1028.0f;
//     // const float e = 0.0f;
//     const float f = -13.1279f;
//     const float h = 0.0f;
//     const float g = 24.3479f;
//     const float A = -16.1188f;
//     const float B = -4.7396f;
//     const float C = 1.7413f;
//     const float D = 0.0328f;

//     return a*atan(b*AngleSpeed)+
//     d*AngleSpeed+f*Angle+
//     c*AngleSpeed*AngleSpeed+h*Angle*Angle+g*AngleSpeed*Angle
//     +A*AngleSpeed*AngleSpeed*AngleSpeed+B*AngleSpeed*AngleSpeed*Angle+C*AngleSpeed*Angle*Angle+D*Angle*Angle*Angle;
//     // R^2: 0.88483
// }

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
        holder->Pitch.Target_Angle += ((rc_ctrl->rc.ch3 - 1024) * holder->Pitch.Sensitivity);
        // holder->Yaw_S.Target_Angle = 30 * sin(HAL_GetTick () / 200.0f);
        // holder->Pitch.Target_Angle = 20 * sin(HAL_GetTick() / 100.0f);

        // 模拟自瞄跟踪装甲板
        // const float w = 100.0f;
        // holder->Yaw_S.Target_Angle = 30 * (sin(HAL_GetTick()/w)+sin(HAL_GetTick()/w*2)/6.0f);

        // holder->Yaw_S.Target_Angle -= 0.02f;
        // if (holder->Yaw_S.Target_Angle < -5)
        // {
        //     holder->Yaw_S.Target_Angle = 5;
        // }

        // 模拟跟踪大符
        // const float w = 150.0f;
        // holder->Yaw_S.Target_Angle = 30 * sin(HAL_GetTick () / w);
        // holder->Pitch.Target_Angle = 20 * cos(HAL_GetTick() / w);
    }
    if (rc_ctrl->rc.s2 == 2 || Top.Referee.game_prograss == 4)
    {
        if (check_robot_state.Check_Usart.Check_vision == 1 || rc_ctrl->rc.s2 == 2)
        {
            if (Brain.Autoaim.mode == Cruise)
            {
                // holder->Yaw_S.Target_Angle = 30 * sin(HAL_GetTick () / 200.0f);
                holder->Yaw_S.Target_Angle = 0;
                // holder->Pitch.Target_Angle = 20 * sin(HAL_GetTick() / 100.0f);
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
    holder->Pitch.Target_Angle = float_constrain(holder->Pitch.Target_Angle, -34, 34);

    holder->Yaw_S.Target_Angle = LPFilter(holder->Yaw_S.Target_Angle, &LPF_yaw_mpu);
    holder->Pitch.Target_Angle = LPFilter(holder->Pitch.Target_Angle, &LPF_pitch_mpu);

    holder->Yaw_S.Target_Angle = float_constrain(holder->Yaw_S.Target_Angle, -38, 38);
    holder->Pitch.Target_Angle = float_constrain(holder->Pitch.Target_Angle, -34, 34);

    if (holder->Pitch.GYRO_Angle > 20)
    {
        holder->Pitch.PID.ShellPID->Kp = 0.35f;
        holder->Pitch.PID.ShellPID->Ki = 0.005f;
        holder->Pitch.PID.ShellPID->Kd = -0.2f;
        holder->Pitch.PID.CorePID->Kp = 0.32f;
        holder->Pitch.PID.CorePID->Ki = 0;
        holder->Pitch.PID.CorePID->Kd = 2;
    }
    else
    {
        // BasePID_Init(&pid_pitch_angle, 0.48, 0.005f, -0.005, 1.5);
        holder->Pitch.PID.ShellPID->Kp = 0.40f;
        holder->Pitch.PID.ShellPID->Ki = 0.005f;
        holder->Pitch.PID.ShellPID->Kd = -0.005f;
        // BasePID_Init(&pid_pitch_speed, 0.60, 0, 3, 0)
        holder->Pitch.PID.CorePID->Kp = 0.50;
        holder->Pitch.PID.CorePID->Ki = 0;
        holder->Pitch.PID.CorePID->Kd = 3;
    }

#if DEBUG_HOLDER == 0
    if (rc_ctrl->rc.s2 == 2 && Brain.Autoaim.mode == Lock)
    {
        holder->Motors.Yaw_S.Data.Output = k * holder->Yaw_S.v2 + BasePID_SpeedControl(holder->Yaw_S.PID.CorePID,
                                                                                       BasePID_AngleControl(holder->Yaw_S.PID.ShellPID,
                                                                                                            holder->Yaw_S.v1,
                                                                                                            holder->Yaw_S.Can_Angle) +
                                                                                           YawFF_Speed(holder->Yaw_S.v1),
                                                                                       holder->Yaw_S.GYRO_AngleSpeed);
        holder->Motors.Pitch.motor_output = BasePID_SpeedControl(holder->Pitch.PID.CorePID,
                                                                 BasePID_AngleControl(holder->Pitch.PID.ShellPID,
                                                                                      holder->Pitch.v1,
                                                                                      holder->Pitch.GYRO_Angle) +
                                                                     PitchFF_Speed(holder->Pitch.v1),
                                                                 holder->Pitch.GYRO_AngleSpeed) +
                                            PitchFF_Gravity(holder->Pitch.GYRO_Angle);
    }
    else
    {
        holder->Motors.Yaw_S.Data.Output = BasePID_SpeedControl(holder->Yaw_S.PID.CorePID,
                                                                BasePID_AngleControl(holder->Yaw_S.PID.ShellPID,
                                                                                     holder->Yaw_S.Target_Angle,
                                                                                     holder->Yaw_S.Can_Angle) +
                                                                    YawFF_Speed(holder->Yaw_S.Target_Angle),
                                                                holder->Yaw_S.GYRO_AngleSpeed);
        holder->Motors.Pitch.motor_output = BasePID_SpeedControl(holder->Pitch.PID.CorePID,
                                                                 BasePID_AngleControl(holder->Pitch.PID.ShellPID,
                                                                                      holder->Pitch.Target_Angle,
                                                                                      holder->Pitch.GYRO_Angle) +
                                                                     PitchFF_Speed(holder->Pitch.Target_Angle),
                                                                 holder->Pitch.GYRO_AngleSpeed) +
                                            PitchFF_Gravity(holder->Pitch.GYRO_Angle);
    }

    holder->Motors.Pitch.motor_output = float_constrain(holder->Motors.Pitch.motor_output, -10, 10);
    DMiaoMitControl(&holder->Motors.Pitch, 0, 0, 0, 0, holder->Motors.Pitch.motor_output);
    MotorFillData(&holder->Motors.Yaw_S, holder->Motors.Yaw_S.Data.Output);

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
