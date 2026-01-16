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
int16_t cur = 0;
int32_t speed = 0;
int32_t angle_add;
uint8_t flag = 10;
LKMotor_t LKMotor;
//< TIM14的触发频率在CubeMX中被配置为1000Hz
void TIM14_Task(void)
{
	tim14.ClockTime++;
	if(flag == 0)LKMotor_ReadPIDParam(ANGLE_PID,can2,&LKMotor);
	if(flag == 1)LKMotor_ReadPIDParam(SPEED_PID,can2,&LKMotor);
	if(flag == 2)LKMotor_ReadPIDParam(CURRENT_PID,can2,&LKMotor);
//	LKMotor_CurrentMode(cur,&LKMotor);
	LKMotor_SpeedMode(speed,cur,&LKMotor);
//	LKMotor_PositionMode(angle_add,&LKMotor);
	LKMotor_CANOutPut(can2,&LKMotor,SPEED_MODE);
	RobotOnlineState(&check_robot_state,&rc_Ctrl_et,&rc_Ctrl);
	//ET08控制
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
	//DR16控制
	if(rc_Ctrl.is_online == 1)
	{
		SwerveChassis_Control(&swervechassis,&rc_Ctrl);
	}
	
	if(rc_Ctrl.is_online == 1){;}
	else
	{
		DR16Init(&rc_Ctrl);
		for(uint8_t i=0;i<4;i++)
		{
			MotorFillData(&swervechassis.Motors6020.motor[i],0);
        	MotorFillData(&swervechassis.Motors3508.motor[i],0);
		}
	}
		
	MotorCanOutput(can1, 0x1FF);
	MotorCanOutput(can1, 0x200);
	if (tim14.ClockTime%4==0)
	MotorCanOutput(can2, 0x1FF);		//电流信号是FE
	MotorCanOutput(can2, 0x200);
	
	UsartDmaPrintf("%f,%f\r\n",swervechassis.Vectors.Target_Angle[0],-swervechassis.Motors6020.motor[0].Data.Angle);
}



void TIM13_Task(void)
{
	tim14_FPS.Gyro_cnt++;
//	MPU6050_Read(&mpu6050.mpu6050_Data);
//	IMUupdate(&mpu6050.mpu6050_Data);
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
	LK_CanUpdata(0x141,(*rxBuffer),&LKMotor);
	return 0;
}


