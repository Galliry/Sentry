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

int i = 0;
extern int error_flag;
//< TIM14的触发频率在CubeMX中被配置为1000Hz
void TIM14_Task(void)
{
    tim14.ClockTime++;
    RobotOnlineState(&check_robot_state, &rc_Ctrl_et, &rc_Ctrl);
    FPS_Check(&tim14_FPS);
    RobotToBrain(&Brain);
    if (tim14.ClockTime % 4 == 0)
        TopBoardDataTrans(&rc_Ctrl_et);
    // ET08控制

    // if(tim14.ClockTime > 500) FrictionWheelControl(&AmmoBooster);

    if (rc_Ctrl_et.isOnline == 1)
    {
        i++;
        if (i % 10 == 0)
            DMiao_Enable(can1, &Holder.Motors.Pitch);
        else
            HolderControl_Top(&Holder, &rc_Ctrl_et);
        if (tim14.ClockTime % 10 == 0)
            Trans_forToptoBase(&rc_Ctrl);
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

    // UsartDmaPrintf("%f,%f,%d,%f,%f,%d,%f\r\n",Holder.Pitch.Target_Angle,Holder.Yaw_S.Target_Angle,Brain.Autoaim.IsFire,
    // Holder.Yaw_S.Can_Angle,Holder.Pitch.GYRO_Angle,Holder.Yaw_S.v1,Holder.Pitch.v1);
    // UsartDmaPrintf("%d,%d,%d,%d,%d\r\n",error_flag,huart5.ErrorCode,Receive.Top.Referee.robot_HP,Receive.Top.Referee.robot_id,Receive.Top.Referee.game_prograss);
    // UsartDmaPrintf("%d,%d,%d\r\n",Brain.Lidar.movemode,Brain.Autoaim.mode,Brain.Autoaim.IsFire);

	// Gravity FeedForward Test
	// UsartDmaPrintf("%.3f, %.3f\r\n", Holder.Motors.Pitch.torque, INS_attitude->roll);
    // Pitch
    UsartDmaPrintf("%.2f, %.2f, %.2f, %.2f\r\n", Holder.Pitch.Target_Angle, Holder.Pitch.GYRO_Angle, Holder.Pitch.PID.ShellPID->Out, Holder.Pitch.GYRO_AngleSpeed);
}

void TIM13_Task(void)
{
    tim14_FPS.Gyro_cnt++;
    MPU6050_Read(&mpu6050.mpu6050_Data);
    IMUupdate(&mpu6050.mpu6050_Data);
    INS_attitude = INS_GetAttitude(IMU_data);
}

/**
 * @brief  CAN1接收中断回调
 */
uint8_t CAN1_rxCallBack(CAN_RxBuffer *rxBuffer)
{
    MotorRxCallback(&can1, rxBuffer);
    DMiao_CanUpdata(&Holder.Motors.Pitch, (*rxBuffer));
    return 0;
}

/**
 * @brief  CAN2接收中断回调
 */
uint8_t CAN2_rxCallBack(CAN_RxBuffer *rxBuffer)
{
    MotorRxCallback(&can2, rxBuffer);
    BaseBoard_Callback(rxBuffer);
    return 0;
}
