#include "hardware_config.h"
#include "control_logic.h"
#include "motor.h"
#include "dr16.h"
#include "driver_timer.h"
#include "et08.h"
#include "driver.h"
#include "shoot.h"
#include "driver_counter.h"
#include "ins.h"
#include "mpu6050.h"
#include "bmi088.h"
#include "holder.h"
#include "swerve_chassis.h"
#include "DM_motor.h"
#include "brain.h"
#include "communication.h"
void MPU_Init_(void);
void MPU_Init_(void)
{
	//MUP structure variable define
	MPU_Region_InitTypeDef MPU_Config;
	
	/*-----------Open FPU--------*///High speed FLOAT calculate
	SCB->CPACR |= ((3UL << (10*2))|(3UL << (11*2)));  /* set CP10 and CP11 Full Access */
	/*-----------Open Cache------------*/
	SCB_EnableICache();
  SCB_EnableDCache();  
	SCB->CACR|=1<<2;   
	/*-----------Open MPU------------*/
	HAL_MPU_Disable();
	
	MPU_Config.Enable=MPU_REGION_ENABLE;
	MPU_Config.Number=MPU_REGION_NUMBER1;
	MPU_Config.BaseAddress= 0x24000000;
	MPU_Config.Size=MPU_REGION_SIZE_512KB;
	MPU_Config.SubRegionDisable=0x00;
	MPU_Config.TypeExtField=MPU_TEX_LEVEL0;
	MPU_Config.AccessPermission=MPU_REGION_FULL_ACCESS;
	MPU_Config.DisableExec=MPU_INSTRUCTION_ACCESS_ENABLE;
	MPU_Config.IsShareable=MPU_ACCESS_SHAREABLE;
	MPU_Config.IsCacheable=MPU_ACCESS_CACHEABLE;
	MPU_Config.IsBufferable=MPU_ACCESS_NOT_BUFFERABLE;
	HAL_MPU_ConfigRegion(&MPU_Config);
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}
/**
  * @brief  初始化指令合集
  */
void HardwareConfig(void)
{
	MPU_Init_();
	DWT_Init(480);
	
//	DR16Init(&rc_Ctrl);
 	ET08Init(&rc_Ctrl_et);
	
	PID_Init(); 	
	AmmoBoosterInit(&AmmoBooster,&pid_friction0,&pid_friction1,&pid_load_angle, &pid_load_speed);
	HolderInit_Top(&Holder,&pid_pitch,&pid_yaw_s);
	
 	UARTx_Init(&huart1,ET08_callback);
//	UARTx_Init(&huart1,DR16_Callback);
	UARTx_Init(&huart2,Brain_Autoaim_Callback); 	//视觉自瞄回调函数
	UARTx_Init(&huart4,Brain_Lidar_Callback); 	//导航雷达回调函数
	UARTx_Init(&huart5,BaseData_Callback); //Callback From Base
	UARTx_Init(&huart7,NULL);//  Vofa+

	INS_Init(&bmi088.bmi088_Data);
	MPU6050_Init(&mpu6050.mpu6050_Data);
		
	CANx_Init(&hfdcan1, CAN1_rxCallBack);
    CAN_Open (&can1);
    CANx_Init(&hfdcan2, CAN2_rxCallBack);
    CAN_Open (&can2 );	

	TIMx_Init(&htim14, TIM14_Task);//链接定时器回调
	TIM_Open(&tim14);
	TIMx_Init(&htim13, TIM13_Task);//链接定时器回调
	TIM_Open(&tim13);
}

