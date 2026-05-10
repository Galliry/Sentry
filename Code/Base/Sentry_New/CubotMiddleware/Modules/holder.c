#include "holder.h"
#include "DM_motor.h"
#include "driver_timer.h"
#include "et08.h"
#include "ins.h"
#include "mpu6050.h"
#include "user_lib.h"
#include <stdint.h>

Holder_t Holder;

#define DEBUG_YAW 0
volatile float Debug_tar = 0;

/**
 * @brief ÔÆÌ¨³õÊ¼»¯µ×°å
 */
void HolderInit_Base(Holder_t *holder, DualPID_Object *yaw_m)
{
    DMiaoInit(&holder->Motors.Yaw_M, 0x01, 0x02, MIT);
    DualPID_Init(&holder->Yaw_M.PID, yaw_m->ShellPID, yaw_m->CorePID);
    holder->Yaw_M.Sensitivity = 0.00085f; // 0.003f 0.0015
    holder->Yaw_M.Target_Angle = holder->Yaw_M.GYRO_Angle;
}

float AngleMinus(float a, float b)
{
    float r = a - b;
    if (r > 180)
        r -= 360;
    if (r < -180)
        r += 360;
    return r;
}

void AngleLim(float *a)
{
    if (*a > 360)
    {
        *a -= 360;
    }
    if (*a < 0)
    {
        *a += 360;
    }
}

uint16_t AllSenseDelayCount = 0;

/**
 * @brief ÔÆÌ¨¿ØÖÆ
 */
void HolderControl_Base(Holder_t *holder, Base_t *rec)
{
    holder->Yaw_M.GYRO_Angle = rec->Gyro.Gyro_Angle;
    holder->Yaw_M.GYRO_AngleSpeed = rec->Gyro.Gyro_Data;
    holder->Yaw_M.Can_Angle = holder->Motors.Yaw_M.angle;
    holder->Yaw_M.Can_AngleSpeed = holder->Motors.Yaw_M.speed_rpm;

    AngleLim(&holder->Yaw_M.GYRO_Angle);

#if DEBUG_YAW == 0
    if (rec->Rc.rc_Ctrl_s2 != 1)
    {
        holder->Yaw_M.Target_Angle += ((rec->Rc.rc_Ctrl_ch2 - 1024) * holder->Yaw_M.Sensitivity);
    }
    if (AllSenseDelayCount == 0)
    {
        if (rec->Autoaim.All_Sense == 1)
        {
            holder->Yaw_M.Target_Angle = holder->Yaw_M.GYRO_Angle - 60;
        }
        if (rec->Autoaim.All_Sense == 2)
        {
            holder->Yaw_M.Target_Angle = holder->Yaw_M.GYRO_Angle - 120;
        }
        if (rec->Autoaim.All_Sense == 3)
        {
            holder->Yaw_M.Target_Angle = holder->Yaw_M.GYRO_Angle - 180;
        }
        if (rec->Autoaim.All_Sense == 4)
        {
            holder->Yaw_M.Target_Angle = holder->Yaw_M.GYRO_Angle + 120;
        }
        if (rec->Autoaim.All_Sense == 5)
        {
            holder->Yaw_M.Target_Angle = holder->Yaw_M.GYRO_Angle + 60;
        }
        AllSenseDelayCount = 125;
    }
    else
    {
        AllSenseDelayCount--;
    }

    AngleLim(&holder->Yaw_M.Target_Angle);
#else
    holder->Yaw_M.Target_Angle = Debug_tar;
#endif

    //	else if (rec->Base.AutoAim.mode == 0x00 /*Cruise*/ && (referee2022.game_status.game_progress == 4 || rec->Base.rc.rc_Ctrl_s2 == 2) && Receive.Base.Lidar.Movemode == 0 && Receive.Base.Lidar.Online == 1)
    //	{
    //		holder->Yaw_M.Target_Angle -= 0.08f;
    //	}

#if DEBUG_YAW == 0
    holder->Motors.Yaw_M.motor_output = BasePID_SpeedControl(holder->Yaw_M.PID.CorePID,
                                                             BasePID_AngleControl(holder->Yaw_M.PID.ShellPID,
                                                                                  0,
                                                                                  AngleMinus(holder->Yaw_M.GYRO_Angle, holder->Yaw_M.Target_Angle)),
                                                             holder->Yaw_M.GYRO_AngleSpeed);
    holder->Motors.Yaw_M.motor_output = float_constrain(holder->Motors.Yaw_M.motor_output, -10, 10);
    DMiaoMitControl(&holder->Motors.Yaw_M, 0, 0, 0, 0, holder->Motors.Yaw_M.motor_output);

#else

    DMiaoMitControl(&holder->Motors.Yaw_M, 0, 0, 0, 0, Debug_tar);
#endif
}
