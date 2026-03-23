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
#include "swerve_chassis.h"
#include "DM_motor.h"
#include "LK_motor.h"
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
  * @brief  ��ʼ��ָ��ϼ�
  */
void HardwareConfig(void)
{
	MPU_Init_();
	DWT_Init(480);
	
	DR16Init(&rc_Ctrl);
//	ET08Init(&rc_Ctrl_et);
	
	PID_Init(); 	
//	SwerveChassisInit(&swervechassis,pid_turn,&pid_run,&pid_follow);
//	HolderInit_Base(&Holder,&pid_yaw_m);
	
//	UARTx_Init(&huart1,ET08_callback);
	UARTx_Init(&huart1,DR16_Callback);
	UARTx_Init(&huart2,SolutionData_FromTop);
	UARTx_Init(&huart4, Referee_callback);
	UARTx_Init(&huart7,NULL);//  Vofa+
		
//	INS_Init(&bmi088.bmi088_Data);
//	MPU6050_Init(&mpu6050.mpu6050_Data);
		
	CANx_Init(&hfdcan1, CAN1_rxCallBack);
    CAN_Open (&can1);
    CANx_Init(&hfdcan2, CAN2_rxCallBack);
    CAN_Open (&can2 );	

	TIMx_Init(&htim14, TIM14_Task);//���Ӷ�ʱ���ص�
	TIM_Open(&tim14);
//	TIMx_Init(&htim13, TIM13_Task);//���Ӷ�ʱ���ص�
//	TIM_Open(&tim13);
}

