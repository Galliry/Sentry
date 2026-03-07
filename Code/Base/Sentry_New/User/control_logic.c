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
uint8_t i = 0;
//< TIM14的触发频率在CubeMX中被配置为1000Hz
void TIM14_Task(void)
{
	tim14.ClockTime++;
	RobotOnlineState(&check_robot_state,&rc_Ctrl_et,&rc_Ctrl);
	FPS_Check(&tim14_FPS);

//	if(rc_Ctrl_et.isOnline == 1)
//	{
//		ShootPlantControl(&AmmoBooster);
//		Holder_Control(&Holder,&rc_Ctrl_et);
//	}
//	
//	if(tim14.ClockTime > 500) FrictionWheelControl(&AmmoBooster);
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
	if(Receive.Base.Online_check.Status == 1)
	{
		i++;
		if( i <= 10)DMiao_Enable(can1,&Holder.Motors.Yaw_M); 
		else HolderControl_Base(&Holder,&Receive);
		if(i > 100) i = 100;
		SwerveChassis_Control(&swervechassis,&Receive);
	}
	
	if(Receive.Base.Online_check.Status == 1){;}
	else
	{
		// DR16Init(&rc_Ctrl);
		i = 0;
		DMiao_Disable(can1,&Holder.Motors.Yaw_M);
		for(uint8_t i=0;i<4;i++)
		{
			MotorFillData(&swervechassis.Motors6020.motor[i],0);
        	MotorFillData(&swervechassis.Motors3508.motor[i],0);
		}
	}
	if (tim14.ClockTime%2==0)
	{
		MotorCanOutput(can1, 0x1FE);
		MotorCanOutput(can1, 0x200);
	}	
	MotorCanOutput(can2, 0x1FE);		//电流信号是FE
	MotorCanOutput(can2, 0x200);
	DMiao_CanOutput(can1,&Holder.Motors.Yaw_M);
	UsartDmaPrintf("%f,%f\r\n",Holder.Yaw_M.GYRO_Angle,Holder.Yaw_M.Target_Angle);
	
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
	DMiao_CanUpdata(&Holder.Motors.Yaw_M,*rxBuffer);
	return 0;
}

/**
  * @brief  CAN2接收中断回调
  */
uint8_t CAN2_rxCallBack(CAN_RxBuffer* rxBuffer)
{
	MotorRxCallback(&can2, rxBuffer);
	return 0;
}


