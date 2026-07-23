#include "holder.h"
#include "DM_motor.h"
#include "driver_timer.h"
#include "et08.h"
#include "ins.h"
#include "interboard.h"
#include "mpu6050.h"
#include "user_lib.h"
#include <stdint.h>
#include "filter.h"

Holder_t Holder;
uint16_t AllSenseDelayCount;
uint8_t Follow_Flag = 0;
uint16_t Follow_Flag_cnt = 0;
#define DEBUG_YAW 0

/**
 * @brief 云台初�?�化
 */
void HolderInit_Base(Holder_t *holder, DualPID_Object *yaw_m)
{
    DMiaoInit(&holder->Motors.Yaw_M, 0x01, 0x02, MIT);
    DualPID_Init(&holder->Yaw_M.PID, yaw_m->ShellPID, yaw_m->CorePID);
    holder->Yaw_M.Sensitivity = 0.00085f; // 0.003f 0.0015
    holder->Yaw_M.Target_Angle = holder->Yaw_M.GYRO_Angle;
}

/**
 * @brief  下板云台控制
 */
void HolderControl_Base(Holder_t *holder, Base_t *rec)
{
    holder->Yaw_M.GYRO_Angle = rec->Gyro.Gyro_Angle;
    holder->Yaw_M.GYRO_AngleSpeed = rec->Gyro.Gyro_Data;
    holder->Yaw_M.Can_Angle = holder->Motors.Yaw_M.angle;
    holder->Yaw_M.Can_AngleSpeed = holder->Motors.Yaw_M.speed_rpm;

//    AngleLim(&holder->Yaw_M.GYRO_Angle);

#if DEBUG_YAW == 0
    if (rec->Rc.rc_Ctrl_s2 != 1)
    {
        holder->Yaw_M.Target_Angle += ((rec->Rc.rc_Ctrl_ch2 - 1024) * holder->Yaw_M.Sensitivity);
    }
	if(rec->Rc.rc_Ctrl_s2 == 2 && referee2022.game_status.stage_remain_time <= 360)
	{
		if (Base.All_sense.All_Sense_cnt != 0)
		{
			if (AllSenseDelayCount == 0)
			{
				holder->Yaw_M.Target_Angle += Base.All_sense.All_Sense_Angle[Base.All_sense.All_Sense_cnt];
				AllSenseDelayCount = 2000;
			}
			else
			{
				AllSenseDelayCount--;
			}
		}
		else
		{
			holder->Yaw_M.Target_Angle = LPFilter(Base.Autoaim.Target_Yaw, &LPF_Yaw_M);
		}
	}
	if(Follow_Flag == 1 && tim14.ClockTime % 100 == 0 && Base.Autoaim.Mode != 0)
	{
		if(Base.Autoaim.is_Follow == 1)
		{
			holder->Yaw_M.Target_Angle -= 25;
			Follow_Flag = 0;
		}
		else if(Base.Autoaim.is_Follow == 2)
		{
			holder->Yaw_M.Target_Angle += 25;
			Follow_Flag = 0;
		}
	}
	if(Follow_Flag == 0)
		Follow_Flag_cnt++;
	if(Follow_Flag_cnt > 500)
	{
		Follow_Flag_cnt = 0;
		Follow_Flag = 1;
	}
#else
    holder->Yaw_M.Target_Angle = Debug_tar;
#endif

#if DEBUG_YAW == 0
    holder->Motors.Yaw_M.motor_output = BasePID_SpeedControl(holder->Yaw_M.PID.CorePID,
                                                             BasePID_AngleControl(holder->Yaw_M.PID.ShellPID,
                                                                                  holder->Yaw_M.Target_Angle,
                                                                                  holder->Yaw_M.GYRO_Angle),
                                                             holder->Yaw_M.GYRO_AngleSpeed);
    holder->Motors.Yaw_M.motor_output = float_constrain(holder->Motors.Yaw_M.motor_output, -10, 10);
    DMiaoMitControl(&holder->Motors.Yaw_M, 0, 0, 0, 0, holder->Motors.Yaw_M.motor_output);

#else
    DMiaoMitControl(&holder->Motors.Yaw_M, 0, 0, 0, 0, Debug_tar);
#endif
}
