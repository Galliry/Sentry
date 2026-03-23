#include "hardware_config.h"
#include "control_logic.h"
#include "motor.h"
#include "dr16.h"
#include "usart.h"
#include "driver_timer.h"
#include "et08.h"
#include "check.h"
#include "shoot.h"
#include "ins.h"
#include "mpu6050.h"
#include "bmi088.h"
#include "holder.h"
#include "swerve_chassis.h"
#include "dr16.h"
#include "DM_motor.h"
#include "LK_motor.h"
#include "communication.h"
#include "brain.h"
int i = 0;
extern int flag_fire;
//< TIM14的触发频率在CubeMX中被配置为1000Hz
void TIM14_Task(void)
{
	tim14.ClockTime++;
	RobotOnlineState(&check_robot_state,&rc_Ctrl_et,&rc_Ctrl);
	FPS_Check(&tim14_FPS);
	RobotToBrain(&Brain);
	//ET08控制
//	if(rc_Ctrl_et.isOnline == 1)
//	{
//		;
//	}
//	
	
//	
//	if(rc_Ctrl_et.isOnline == 1){;}
//	else
//	{
//		ET08Init(&rc_Ctrl_et);
//		MotorFillData(&AmmoBooster.Shoot_Plate.motor2006, 0);
//		// MotorFillData(&Holder.Motors6020.Yaw_M,0);
//		MotorFillData(&Holder.Motors.Yaw_S,0);
//		// MotorFillData(&Holder.Motors6020.Pitch,0);
//	}
	//DR16控制
	if(tim14.ClockTime > 500) FrictionWheelControl(&AmmoBooster);
	
	if(rc_Ctrl.is_online == 1)
	{
		i++;
		if(i <= 10) DMiao_Enable(can2,&Holder.Motors.Pitch);
		else HolderControl_Top(&Holder,&rc_Ctrl);
		if(tim14.ClockTime % 10 == 0)
		Trans_forToptoBase(&rc_Ctrl);
		
		ShootPlantControl(&AmmoBooster);
		
	}
	
	if(rc_Ctrl.is_online == 1){;}
	else
	{
		i = 0;
		DR16Init(&rc_Ctrl);
		MotorFillData(&Holder.Motors.Yaw_S,0);
		MotorFillData(&AmmoBooster.Shoot_Plate.motor2006, 0);
		DMiao_Disable(can2,&Holder.Motors.Pitch);
	} 
		
	MotorCanOutput(can1, 0x1FF);
	MotorCanOutput(can1, 0x200);
	DMiao_CanOutput(can2,&Holder.Motors.Pitch);
//	if (tim14.ClockTime%4==0)
//	MotorCanOutput(can2, 0x1FE);		
//	MotorCanOutput(can2, 0x200);
	
	UsartDmaPrintf("%f,%f,%f,%f,%d,%f,%f,%d\r\n",Brain.Autoaim.Pitch_add,-Brain.Autoaim.Yaw_add,Holder.Pitch.Target_Angle,Holder.Yaw_S.Target_Angle,Brain.Autoaim.IsFire,Holder.Yaw_S.Can_Angle,Holder.Pitch.GYRO_Angle,Holder.Motors.Yaw_S.Data.Output);
	
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
uint8_t CAN1_rxCallBack(CAN_RxBuffer* rxBuffer)
{
	MotorRxCallback(&can1, rxBuffer);
	return 0;
}

/**
  * @brief  CAN2接收中断回调
  */
uint8_t CAN2_rxCallBack(CAN_RxBuffer* rxBuffer)
{
	MotorRxCallback(&can2, rxBuffer);
	DMiao_CanUpdata(&Holder.Motors.Pitch,(*rxBuffer));
	return 0;
}


