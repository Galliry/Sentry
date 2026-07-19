#include "control_logic.h"
#include "DM_motor.h"
#include "LK_motor.h"
#include "bmi088.h"
#include "brain.h"
#include "check.h"
#include "communication.h"
#include "dr16.h"
#include "driver_timer.h"
#include "driver_usart.h"
#include "et08.h"
#include "hardware_config.h"
#include "holder.h"
#include "ins.h"
#include "motor.h"
#include "mpu6050.h"
#include "shoot.h"
#include "swerve_chassis.h"
#include "usart.h"
#include "DM_imu.h"
#include "dm_imu_rs485.h"
int i = 0;
extern int error_flag;
extern uint16_t ignore_outpost;
uint8_t state_flag = 0;
int state_cnt = 0;
//< TIM14�Ĵ���Ƶ����CubeMX�б�����Ϊ1000Hz
void TIM14_Task(void)
{
    tim14.ClockTime++;
	

    // 离线尝试重启
    if ( IMU_isOnline(&IMU_S) == 0 ) 
    {
        DM_IMU_Run(&IMU_S);
    }
    RobotOnlineState(&check_robot_state, &rc_Ctrl_et, &rc_Ctrl);
    FPS_Check(&tim14_FPS);
    RobotToBrain(&Brain);
    if (tim14.ClockTime % 4 == 0)
        TopBoardDataTrans(&rc_Ctrl_et);
//	if(Top.Referee.game_prograss == 3 && state_flag == 0)
//	{
//		Brain.Lidar.Outpost_Flag = 0;
//		Brain.Autoaim.Mode = Small_Buff;
//		state_flag = 1;
//	}else if(Top.Referee.small_buff == 2 && state_flag == 1)
//	{
//		state_cnt++;
//		if(state_cnt > 20000)
//		{
//			state_flag = 2;
//		}
//	}else if(state_flag == 2 && Top.Referee.small_buff != 2 && ignore_outpost == 0)
//	{
//		Brain.Lidar.Outpost_Flag = 1;
//		Brain.Autoaim.Mode = Outpost;
//		if(Top.Referee.game_time <= 340)
//		{
//			state_cnt = 0;
//			state_flag = 3;
//		}
//	}else if(state_flag == 3)
//	{
//		Brain.Lidar.Outpost_Flag = 0;
//		Brain.Autoaim.Mode = Small_Buff;
//		state_flag = 4;
//	}else if(state_flag == 4 && Top.Referee.small_buff == 2)
//	{
//		state_cnt++;
//		if(state_cnt > 20000)
//		{
//			state_flag = 5;
//		}
//	}else
//	{
//		Brain.Autoaim.Mode = EKF;
//	}
	if(Top.Referee.game_prograss == 4 && Top.Referee.game_time >= 340 && Top.Referee.shoot_num >= 180)
	{
		Brain.Lidar.Outpost_Flag = 1;
		Brain.Autoaim.Mode = EKF;
	}else
	{
		Brain.Lidar.Outpost_Flag = 0;
		Brain.Autoaim.Mode = EKF;
	}
	
	// ET08 Contorl
//    if (tim14.ClockTime > 500)
//        FrictionWheelControl(&AmmoBooster);
	
    if (rc_Ctrl_et.isOnline == 1)
    {
        i++;
        if (i % 10 == 0)
            DMiao_Enable(can1, &Holder.Motors.Pitch);
        else
        {
            HolderControl_Top(&Holder, &rc_Ctrl_et);
        }
        ShootPlantControl(&AmmoBooster);
    }

    if (rc_Ctrl_et.isOnline == 1)
    {
        ;
    }
    else
    {
        i = 0;
        DR16Init(&rc_Ctrl);
        MotorFillData(&Holder.Motors.Yaw_S, 0);
        MotorFillData(&AmmoBooster.Shoot_Plate.motor2006, 0);
        DMiao_Disable(can1, &Holder.Motors.Pitch);
    }
    MotorCanOutput(can1, 0x1FF);
    MotorCanOutput(can1, 0x200);
    DMiao_CanOutput(can1, &Holder.Motors.Pitch);

    //	if (tim14.ClockTime%4==0)
    //	MotorCanOutput(can2, 0x1FE);
    //	MotorCanOutput(can2, 0x200);

    //  UsartDmaPrintf("%f,%f,%d,%f,%f,%f,%f\r\n",Holder.Pitch.Target_Angle,Holder.Yaw_S.Target_Angle,Brain.Autoaim.IsFire,
		// Holder.Yaw_S.Can_Angle,Holder.Pitch.GYRO_Angle,Holder.Yaw_S.v1,Holder.Pitch.v1);
    // UsartDmaPrintf("%d,%d,%d,%d,%d\r\n",error_flag,huart5.ErrorCode,Receive.Top.Referee.robot_HP,Receive.Top.Referee.robot_id,Receive.Top.Referee.game_prograss);
    // UsartDmaPrintf("%d,%d,%d\r\n",Brain.Lidar.movemode,Brain.Autoaim.mode,Brain.Autoaim.IsFire);

    // Gravity FeedForward Test
    // UsartDmaPrintf("%.3f, %.3f, %.3f\r\n", Holder.Motors.Pitch.torque, IMU_S.Attitude.roll, Holder.Pitch.PID.ShellPID->Out);
    // Pitch
    UsartDmaPrintf("%.2f, %.2f, %.2f, %.2f\r\n", Holder.Pitch.Target_Angle, Holder.Pitch.GYRO_Angle, Holder.Pitch.PID.ShellPID->Out, Holder.Pitch.GYRO_AngleSpeed);

    // Yaw
    // UsartDmaPrintf("%.2f, %.2f, %.2f, %.2f\r\n", Holder.Yaw_S.Target_Angle, Holder.Yaw_S.GYRO_Angle, Holder.Yaw_S.PID.ShellPID->Out, Holder.Yaw_S.GYRO_AngleSpeed);
    // UsartDmaPrintf("%.2f\r\n",Holder.Yaw_S.PID.CorePID->Out);

    // Pitch Friction Test
    // if (fabs(Holder.Pitch.GYRO_Angle) < 12)
    // UsartDmaPrintf("%.2f, %.2f\r\n", Holder.Pitch.GYRO_AngleSpeed, Holder.Pitch.PID.CorePID->Out - PitchFF_Gravity(Holder.Pitch.GYRO_Angle));

    // Yaw Friction Test
    //    if (fabs(Holder.Yaw_S.Can_Angle) < 28)
    // UsartDmaPrintf("%.2f, %.2f, %.2f, %.2f\r\n",
    //     Holder.Yaw_S.GYRO_AngleSpeed,
    //     Holder.Yaw_S.PID.CorePID->Out,
    //     // Holder.Motors.Yaw_S.Data.TorqueCurrent,
    //     INS_attitude->accel[2] - mpu6050.mpu6050_Data.accel[2],
    //     Holder.Yaw_S.Can_Angle);

    // UsartDmaPrintf("%.2f\r\n", INS_attitude->pitch);

    // Autoaim
    // UsartDmaPrintf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %d\r\n",
    //                 Holder.Pitch.PID.ShellPID->Error, Holder.Yaw_S.PID.ShellPID->Error,
    //                 Holder.Pitch.Target_Angle, Holder.Yaw_S.Target_Angle,
    //                 Holder.Pitch.GYRO_Angle, Holder.Yaw_S.Can_Angle,
    //                 Brain.Autoaim.Pitch_add, Brain.Autoaim.Yaw_add,
    //                 Brain.Autoaim.IsFire);

    // ShootPlate
    // UsartDmaPrintf("%.2f, %.2f, %.2f, %d\r\n",
    //                AmmoBooster.Shoot_Plate.Target_Angle, AmmoBooster.Shoot_Plate.Plate_Angle,
    //                AmmoBooster.Shoot_Plate.RunPID_angle.Out, AmmoBooster.Shoot_Plate.motor2006.Data.SpeedRPM);
}

void TIM13_Task(void)
{
    tim14_FPS.Gyro_cnt++;
    MPU6050_Read(&mpu6050.mpu6050_Data);
    IMUupdate(&mpu6050.mpu6050_Data);
    // INS_attitude = INS_GetAttitude(IMU_data);
}

/**
 * @brief  CAN1 CallBack
 */
uint8_t CAN1_rxCallBack(CAN_RxBuffer *rxBuffer)
{
    MotorRxCallback(&can1, rxBuffer);
    DMiao_CanUpdata(&Holder.Motors.Pitch, (*rxBuffer));
    return 0;
}

/**
 * @brief  CAN2 CallBack
 */
uint8_t CAN2_rxCallBack(CAN_RxBuffer *rxBuffer)
{
    MotorRxCallback(&can2, rxBuffer);
    BaseBoard_Callback(rxBuffer);
	IMU_UpdateData(&IMU_S,rxBuffer);
	
    return 0;
}
