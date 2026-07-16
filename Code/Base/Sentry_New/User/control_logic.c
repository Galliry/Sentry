#include "driver_usart.h"
#include "hardware_config.h"
#include "control_logic.h"
#include "motor.h"
#include "dr16.h"
#include "stm32h7xx_hal.h"
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
#include "referee.h"
#include <stdint.h>
#include <stdlib.h>
int i = 0;
int flag = 0;
extern Motor a1308;

uint8_t beHit = 0;
uint32_t lastBeHitTick = 0;
uint16_t lastRobotHP = 400;

//< TIM14的触发频率在CubeMX中被配置为1000Hz
void TIM14_Task(void)
{
	tim14.ClockTime++;
	RobotOnlineState(&check_robot_state,&rc_Ctrl_et,&rc_Ctrl);
	FPS_Check(&tim14_FPS);
	if(tim14.ClockTime % 50 == 0)
		Sentry_To_Referee();
	
	if(HAL_GetTick() - referee2022.robot_hurt.last_hurt_time > 4000)
	{
		referee2022.robot_hurt.armor_id = 0;
		referee2022.robot_hurt.hurt_type = 0;
	}
	
	if(tim14.ClockTime % 10 == 0) 
		RefereeDataTrans(&referee2022);
	
	if(tim14.ClockTime % 9 == 0 && tim14.ClockTime % 2 != 0 && tim14.ClockTime % 2 != 0)
		SupercapControl(can1,&super_cap);
	
	if(Base.Rc.isOnline == 1)
	{
		flag++;
		if(flag > 1000)
		{
			i++;
			if(i % 10 == 0) DMiao_Enable(can1,&Holder.Motors.Yaw_M); 
			else HolderControl_Base(&Holder,&Base);
			SwerveChassis_Control(&swervechassis,&Base);
		}
	}
	if(Base.Rc.isOnline == 1) ;
	else
	{
		i = 0;
		DMiao_Disable(can1,&Holder.Motors.Yaw_M);
		for(uint8_t i=0;i<4;i++)
		{
			MotorFillData(&swervechassis.Motors6020.motor[i],0);
        	MotorFillData(&swervechassis.Motors3508.motor[i],0);
		}
		flag = 0;
	}
	if (tim14.ClockTime % 2 == 0 && tim14.ClockTime % 10 != 0)
	{
		// MotorCanOutput(can1, 0x1FE);
		// MotorCanOutput(can1, 0x200);
		DMiao_CanOutput(can1,&Holder.Motors.Yaw_M);
	}	
	// MotorCanOutput(can2, 0x1FE);		//电流信号是FE
	// MotorCanOutput(can2, 0x200);
	
//	UsartDmaPrintf("%d,%d,%f,%f\r\n",referee2022.power_heat_data.chassis_power_buffer,referee2022.game_robot_status.chassis_power_limit,Receive.Base.Lidar.Vx,Receive.Base.Lidar.Vy);
//	UsartDmaPrintf("%.2f, %.2f, %d\r\n",swervechassis.Movement.Vx_Move,swervechassis.Movement.Vy_Move,Receive.Base.Lidar.Movemode);
//	UsartDmaPrintf("%d,%d,%d\r\n",huart2.ErrorCode,referee2022.game_robot_status.remain_HP,referee2022.game_status.game_progress);
//	UsartDmaPrintf("%d,%d,%d\r\n",Receive.Base.AutoAim.mode,Receive.Base.Lidar.Movemode,Receive.Base.Lidar.Online);
//	UsartDmaPrintf("%d,%d,%d,%d\r\n",swervechassis.Motors3508.motor[0].Data.SpeedRPM,swervechassis.Motors3508.motor[1].Data.SpeedRPM,-swervechassis.Motors3508.motor[2].Data.SpeedRPM,-swervechassis.Motors3508.motor[3].Data.SpeedRPM);

	// Holder Yaw	
	// UsartDmaPrintf("%f,%f,%f,%f\r\n",Holder.Yaw_M.Target_Angle,Holder.Yaw_M.GYRO_Angle,Holder.Yaw_M.PID.ShellPID->Out,Holder.Yaw_M.GYRO_AngleSpeed);

	// FeedForward Friction
	// UsartDmaPrintf("\r\n");

	// Chassis Livewatch
	UsartDmaPrintf("%d, %d, %d, %d\r\n",
		abs(swervechassis.Motors3508.motor[0].Data.SpeedRPM),
		abs(swervechassis.Motors3508.motor[1].Data.SpeedRPM),
		abs(swervechassis.Motors3508.motor[2].Data.SpeedRPM),
		abs(swervechassis.Motors3508.motor[3].Data.SpeedRPM));
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
	// MotorRxCallback(&can1, rxBuffer);
	DMiao_CanUpdata(&Holder.Motors.Yaw_M,*rxBuffer);
	TopBoard_Callback(rxBuffer);
	Supercap_rxCallBack((*rxBuffer),&super_cap);
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


