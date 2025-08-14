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
//< TIM14的触发频率在CubeMX中被配置为1000Hz

void TIM14_Task(void)
{
	tim14.ClockTime++;
//	RobotOnlineState(&check_robot_state,&rc_Ctrl_et);
	rc_Ctrl_et.isOnline=1;rc_Ctrl_et.rc.s1=1;
	if(rc_Ctrl_et.isOnline == 1)
	{
		ShootPlantControl(&AmmoBooster);
		Holder_Control(&Holder,&rc_Ctrl_et);
	}
	
	if(tim14.ClockTime > 500) FrictionWheelControl(&AmmoBooster);
	
	if(rc_Ctrl_et.isOnline == 1){;}
	else
	{
		ET08Init(&rc_Ctrl_et);
		MotorFillData(&AmmoBooster.Shoot_Plate.motor2006, 0);
		MotorFillData(&Holder.Motors6020.Yaw_M,0);
		MotorFillData(&Holder.Motors6020.Yaw_S,0);
		MotorFillData(&Holder.Motors6020.Pitch,0);
	}
		
	MotorCanOutput(can1, 0x1ff);
	MotorCanOutput(can1, 0x200);
	if (tim14.ClockTime%4==0)
	MotorCanOutput(can2, 0x1ff);
	MotorCanOutput(can2, 0x200);
	
//	UsartDmaPrintf("");
}



void TIM13_Task(void)
{
	tim14_FPS.Gyro_cnt++;
	MPU6050_Read(&mpu6050.mpu6050_Data);
	IMUupdate(&mpu6050.mpu6050_Data);
//	INS_attitude = INS_GetAttitude(IMU_data);
}

/**
  * @brief  CAN1接收中断回调
  */
uint8_t CAN1_rxCallBack(CAN_RxBuffer* rxBuffer)
{
	MotorRxCallback(can1, (*rxBuffer)); 
	return 0;
}

/**
  * @brief  CAN2接收中断回调
  */
uint8_t CAN2_rxCallBack(CAN_RxBuffer* rxBuffer)
{
	MotorRxCallback(can2, (*rxBuffer)); 	
	return 0;
}
