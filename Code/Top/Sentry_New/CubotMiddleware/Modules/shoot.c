#include "shoot.h"
#include "brain.h"
#include "communication.h"
#include "dr16.h"
#include "driver_timer.h"
#include "et08.h"
#include "interboard.h"
#include "pid.h"
#include "user_lib.h"

extern int flag_fire;
Ammo_Booster AmmoBooster;

void AmmoBoosterInit(Ammo_Booster *ammo_booster, SinglePID_t *friction_pid0, SinglePID_t *friction_pid1, SinglePID_t *load_pid_angle, SinglePID_t *load_pid_speed)
{
    MotorInit(&ammo_booster->Friction_Wheel.motor3508[0], 0, Motor3508, CAN1, 0x201);
    MotorInit(&ammo_booster->Friction_Wheel.motor3508[1], 0, Motor3508, CAN1, 0x202);
    MotorInit(&ammo_booster->Shoot_Plate.motor2006, 0, Motor2006, CAN1, 0x203);

    BasePID_Init(&ammo_booster->Shoot_Plate.RunPID_angle, load_pid_angle->Kp, load_pid_angle->Ki, load_pid_angle->Kd, load_pid_angle->KiPartDetachment);
    BasePID_Init(&ammo_booster->Shoot_Plate.RunPID_speed, load_pid_speed->Kp, load_pid_speed->Ki, load_pid_speed->Kd, load_pid_speed->KiPartDetachment);
    BasePID_Init(&ammo_booster->Friction_Wheel.Friction_PID[0], friction_pid0->Kp, friction_pid0->Ki, friction_pid0->Kd, friction_pid0->KiPartDetachment);
    BasePID_Init(&ammo_booster->Friction_Wheel.Friction_PID[1], friction_pid1->Kp, friction_pid1->Ki, friction_pid1->Kd, friction_pid1->KiPartDetachment);

    ammo_booster->Shoot_Plate.Fire_Rate = 8000; // 5250//8000
    ammo_booster->Shoot_Plate.Fire_Margin = 60;
    ammo_booster->Shoot_Plate.Angle_Sense = 0.1689f;

    ammo_booster->Friction_Wheel.Friction_Start = 0;
    ammo_booster->Friction_Wheel.Friction_Speed[0] = -5600;
    ammo_booster->Friction_Wheel.Friction_Speed[1] = -5590;
}

#define SHOOT_PLANT_VERSON 1

void ShootPlantControl(Ammo_Booster *ammo_booster)
{
    ammo_booster->Shoot_Plate.heat_status = (Top.Referee.cooling_heat >=
                                             (Top.Referee.cooling_limit - ammo_booster->Shoot_Plate.Fire_Margin))
                                                ? 0
                                                : 1;

    ammo_booster->Shoot_Plate.Delta_Angle = ammo_booster->Shoot_Plate.motor2006.Data.SpeedRPM * 0.001 * ammo_booster->Shoot_Plate.Angle_Sense;
    if (fabsf(ammo_booster->Shoot_Plate.Delta_Angle) > 0.005f)
    {
        ammo_booster->Shoot_Plate.Plate_Angle += ammo_booster->Shoot_Plate.Delta_Angle;
    }
#if SHOOT_PLANT_VERSON == 1
    if (rc_Ctrl_et.isOnline == 1 && Top.Referee.shooter_output == 1 && ammo_booster->Shoot_Plate.heat_status == 1)
    {
        if (Top.Referee.cooling_heat >= (Top.Referee.cooling_limit - ammo_booster->Shoot_Plate.Fire_Margin - 70))
            ammo_booster->Shoot_Plate.Fire_Divider = 1000; // 125
        else
            ammo_booster->Shoot_Plate.Fire_Divider = 1000; // 50
        if (ammo_booster->Shoot_Plate.Shoot_rest_flag)
            ammo_booster->Shoot_Plate.Shoot_Cut++;
        if (ammo_booster->Shoot_Plate.Shoot_Cut % ammo_booster->Shoot_Plate.Fire_Divider == 0)
            ammo_booster->Shoot_Plate.Shoot_rest_flag = 0;
        if ((rc_Ctrl_et.rc.s1 == 1 || Top.Referee.game_prograss == 4) && ((rc_Ctrl_et.rc.s2 != 2) || (rc_Ctrl_et.rc.s2 == 2 && Brain.Autoaim.IsFire == 1)) && ammo_booster->Shoot_Plate.Shoot_rest_flag == 0)
        {
            ammo_booster->Shoot_Plate.Target_Angle += 45;
            ammo_booster->Shoot_Plate.ShootNum++;
            ammo_booster->Shoot_Plate.Shoot_rest_flag = 1;
            ammo_booster->Shoot_Plate.Shoot_Cut = 0;
        }
    }

    if (ammo_booster->Shoot_Plate.Target_Angle - ammo_booster->Shoot_Plate.Plate_Angle > 5)
        ammo_booster->Shoot_Plate.Plate_Out = BasePID_SpeedControl(&ammo_booster->Shoot_Plate.RunPID_angle, ammo_booster->Shoot_Plate.Fire_Rate, ammo_booster->Shoot_Plate.motor2006.Data.SpeedRPM);
    else
        ammo_booster->Shoot_Plate.Plate_Out = BasePID_SpeedControl(&ammo_booster->Shoot_Plate.RunPID_angle, 0, ammo_booster->Shoot_Plate.motor2006.Data.SpeedRPM);

    if (ammo_booster->Shoot_Plate.Target_Angle - ammo_booster->Shoot_Plate.Plate_Angle > 5 && ammo_booster->Shoot_Plate.motor2006.Data.SpeedRPM < 400)
    {
        ammo_booster->Shoot_Plate.Jam++;
        if (ammo_booster->Shoot_Plate.Jam >= 900)
        {
            ammo_booster->Shoot_Plate.Target_Angle = ammo_booster->Shoot_Plate.Plate_Angle + 6;
            ammo_booster->Shoot_Plate.Jam++;
            ammo_booster->Shoot_Plate.Plate_Out = BasePID_SpeedControl(&ammo_booster->Shoot_Plate.RunPID_angle, -2000, ammo_booster->Shoot_Plate.motor2006.Data.SpeedRPM);
            if (ammo_booster->Shoot_Plate.Jam > 1600)
                ammo_booster->Shoot_Plate.Jam = 0;
        }
    }
    else
        ammo_booster->Shoot_Plate.Jam = 0;
#endif
#if SHOOT_PLANT_VERSON == 2
    if (rc_Ctrl_et.isOnline == 1 &&                 // 遥控器在线
        Top.Referee.shooter_output == 1 &&          // 电源管理模块有给发射机构上电
        ammo_booster->Shoot_Plate.heat_status == 1) // 热量条件满足
    {
        // 为了做最少改动，将开火分频变量Fire_Divider的含义重新定义为开火频率。在下面的代码中应该注意这一点
        if (Top.Referee.cooling_heat >= (Top.Referee.cooling_limit - ammo_booster->Shoot_Plate.Fire_Margin - 70)) // 热量较少
            ammo_booster->Shoot_Plate.Fire_Divider = 1;                                                           // 8
        else
            ammo_booster->Shoot_Plate.Fire_Divider = 10; // 20

        if ((rc_Ctrl_et.rc.s1 == 1 || Top.Referee.game_prograss == 4) && // 比赛状态中
            (rc_Ctrl_et.rc.s2 != 2 ||                                    // 非自瞄模式
             (rc_Ctrl_et.rc.s2 == 2 && Brain.Autoaim.IsFire == 1)) &&    // 自瞄模式下目标偏差可接收允许发弹
            ammo_booster->Shoot_Plate.Shoot_rest_flag == 0)              // 热量允许发弹
        {
            if (ammo_booster->Shoot_Plate.motor2006.Data.SpeedRPM < 400) // 转速过低认为可能有卡弹
            {
                ammo_booster->Shoot_Plate.Jam += 1;
            }
            else
            {
                ammo_booster->Shoot_Plate.Jam -= 2;
            }

            // 同样是为了避免改动，下面拨弹盘目标角度Target_Angle的意义重新定义为目标转速
            if (ammo_booster->Shoot_Plate.Jam >= 900) // 判断卡弹
            {
                if (ammo_booster->Shoot_Plate.Jam >= 1600) // 可能已解除卡弹，尝试正转
                {
                    ammo_booster->Shoot_Plate.Target_Angle = ammo_booster->Shoot_Plate.Fire_Divider * 60 / 8.0f;
                    if (ammo_booster->Shoot_Plate.motor2006.Data.SpeedRPM >= 0)
                    {
                        ammo_booster->Shoot_Plate.Jam = -200;
                    }
                }
                else // 尝试解除
                {
                    ammo_booster->Shoot_Plate.Target_Angle = -2000;
                }
            }
            else // 没卡弹
            {
                ammo_booster->Shoot_Plate.Target_Angle = ammo_booster->Shoot_Plate.Fire_Divider * 60 / 8.0f;
            }

            ammo_booster->Shoot_Plate.Plate_Out = BasePID_SpeedControl(&ammo_booster->Shoot_Plate.RunPID,
                                                                       ammo_booster->Shoot_Plate.Target_Angle,
                                                                       ammo_booster->Shoot_Plate.motor2006.Data.SpeedRPM);
        }
    }
#endif

#if SHOOT_PLANT_VERSON == 3
    if (rc_Ctrl_et.isOnline == 1 &&                 // 遥控器在线
        Top.Referee.shooter_output == 1 &&          // 电源管理模块有给发射机构上电
        ammo_booster->Shoot_Plate.heat_status == 1) // 热量条件满足
    {
        if (Top.Referee.cooling_heat >= (Top.Referee.cooling_limit - ammo_booster->Shoot_Plate.Fire_Margin - 70)) // 热量较少
            ammo_booster->Shoot_Plate.Fire_Divider = (1000/1);                                                        // 8hz
        else
            ammo_booster->Shoot_Plate.Fire_Divider = (1000/1); // 20hz
        if (ammo_booster->Shoot_Plate.Shoot_rest_flag)
            ammo_booster->Shoot_Plate.Shoot_Cut++;
        if (ammo_booster->Shoot_Plate.Shoot_Cut >= ammo_booster->Shoot_Plate.Fire_Divider)
            ammo_booster->Shoot_Plate.Shoot_rest_flag = 0;
        if ((rc_Ctrl_et.rc.s1 == 1 || Top.Referee.game_prograss == 4) && // 比赛状态中
            ((rc_Ctrl_et.rc.s2 != 2) ||                                  // 非自瞄模式
             (rc_Ctrl_et.rc.s2 == 2 && Brain.Autoaim.IsFire == 1)) &&    // 自瞄模式下目标偏差可接收允许发弹
            ammo_booster->Shoot_Plate.Shoot_rest_flag == 0)              // 热量允许发弹
        {
            ammo_booster->Shoot_Plate.Target_Angle += 45;
            ammo_booster->Shoot_Plate.ShootNum++;
            ammo_booster->Shoot_Plate.Shoot_rest_flag = 1;
            ammo_booster->Shoot_Plate.Shoot_Cut = 0;
        }
    }

    if (ammo_booster->Shoot_Plate.Target_Angle - ammo_booster->Shoot_Plate.Plate_Angle > 5)
        ammo_booster->Shoot_Plate.Plate_Out = BasePID_SpeedControl(&ammo_booster->Shoot_Plate.RunPID_speed,
                                                                   BasePID_AngleControl(&ammo_booster->Shoot_Plate.RunPID_angle,
                                                                                        ammo_booster->Shoot_Plate.Target_Angle,
                                                                                        ammo_booster->Shoot_Plate.Plate_Angle),
                                                                   ammo_booster->Shoot_Plate.motor2006.Data.SpeedRPM);
    else
        ammo_booster->Shoot_Plate.Plate_Out = BasePID_SpeedControl(&ammo_booster->Shoot_Plate.RunPID_speed,
                                                                   0,
                                                                   ammo_booster->Shoot_Plate.motor2006.Data.SpeedRPM);

    if (ammo_booster->Shoot_Plate.Target_Angle - ammo_booster->Shoot_Plate.Plate_Angle > 5 && ammo_booster->Shoot_Plate.motor2006.Data.SpeedRPM < 400)
    {
        ammo_booster->Shoot_Plate.Jam++;
        if (ammo_booster->Shoot_Plate.Jam >= 900)
        {
            ammo_booster->Shoot_Plate.Target_Angle = ammo_booster->Shoot_Plate.Plate_Angle + 6;
            ammo_booster->Shoot_Plate.Jam++;
            ammo_booster->Shoot_Plate.Plate_Out = BasePID_SpeedControl(&ammo_booster->Shoot_Plate.RunPID_speed,
                                                                       -2000,
                                                                       ammo_booster->Shoot_Plate.motor2006.Data.SpeedRPM);
            if (ammo_booster->Shoot_Plate.Jam > 1600)
                ammo_booster->Shoot_Plate.Jam = 0;
        }
    }
    else
        ammo_booster->Shoot_Plate.Jam = 0;
#endif

    MotorFillData(&ammo_booster->Shoot_Plate.motor2006, ammo_booster->Shoot_Plate.Plate_Out);
}

void FrictionWheelControl(Ammo_Booster *ammo_booster)
{
    if (tim14.ClockTime % 2 == 0)
    {
        if (rc_Ctrl_et.isOnline == 1)
            ammo_booster->Friction_Wheel.Friction_Start++;
        else
            ammo_booster->Friction_Wheel.Friction_Start--;
    }
    ammo_booster->Friction_Wheel.Friction_Start = int16_constrain(ammo_booster->Friction_Wheel.Friction_Start, 0, 1000);
    ammo_booster->Friction_Wheel.Friction_Target_Speed[0] = ammo_booster->Friction_Wheel.Friction_Speed[0] * ammo_booster->Friction_Wheel.Friction_Start * 0.001;
    ammo_booster->Friction_Wheel.Friction_Target_Speed[1] = ammo_booster->Friction_Wheel.Friction_Speed[1] * ammo_booster->Friction_Wheel.Friction_Start * 0.001 * (-1);

    ammo_booster->Friction_Wheel.Friction_Out[0] = BasePID_SpeedControl(&ammo_booster->Friction_Wheel.Friction_PID[0], ammo_booster->Friction_Wheel.Friction_Target_Speed[0], ammo_booster->Friction_Wheel.motor3508[0].Data.SpeedRPM);
    ammo_booster->Friction_Wheel.Friction_Out[1] = BasePID_SpeedControl(&ammo_booster->Friction_Wheel.Friction_PID[1], ammo_booster->Friction_Wheel.Friction_Target_Speed[1], ammo_booster->Friction_Wheel.motor3508[1].Data.SpeedRPM);

    MotorFillData(&ammo_booster->Friction_Wheel.motor3508[0], ammo_booster->Friction_Wheel.Friction_Out[0]);
    MotorFillData(&ammo_booster->Friction_Wheel.motor3508[1], ammo_booster->Friction_Wheel.Friction_Out[1]);
}
