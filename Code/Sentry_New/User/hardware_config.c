#include "hardware_config.h"
#include "control_logic.h"
#include "motor.h"
#include "dr16.h"
#include "driver_timer.h"
#include "et08.h"
#include "driver.h"
#include "referee.h"
#include "shoot.h"
#include "driver_counter.h"
#include "ins.h"
#include "mpu6050.h"
#include "bmi088.h"
#include "holder.h"
#include "all_chassis.h"                                  
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
	 
	UARTx_Init(&huart1,ET08_callback);
	UARTx_Init(&huart7,NULL);//  Vofa+
	UARTx_Init(&huart3, Referee_callback);
	
	DR16Init(&rc_Ctrl);
	ET08Init(&rc_Ctrl_et);

	PID_Init();
	AmmoBoosterInit(&AmmoBooster,&pid_friction0,&pid_friction1,&pid_load);
	HolderInit(&Holder,&pid_pitch,&pid_yaw_m,&pid_yaw_s);
	INS_Init(&bmi088.bmi088_Data);
	MPU6050_Init(&mpu6050.mpu6050_Data);
	AllChassisInit(&allchassis,&pid_run,&pid_follow);
	
	CANx_Init(&hfdcan1, CAN1_rxCallBack);
    CAN_Open (&can1);
    CANx_Init(&hfdcan2, CAN2_rxCallBack);
    CAN_Open (&can2 );
	
	TIMx_Init(&htim14, TIM14_Task);//链接定时器回调
	TIM_Open(&tim14);
	TIMx_Init(&htim13, TIM13_Task);//链接定时器回调
	TIM_Open(&tim13);
}








