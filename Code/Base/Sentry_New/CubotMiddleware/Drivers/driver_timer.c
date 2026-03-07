#include "driver_timer.h"


TIM_Object tim14={
	.ClockTime = 0
};
TIM_Object tim13={
	.ClockTime = 0
};
/**
  * @brief 初始化定时器的用户回调
  */
void TIMx_Init(TIM_HandleTypeDef* handle, TIM_ElapsedCallback callback)
{
	if(handle->Instance == TIM14)
	{
		tim14.Handle = handle;
		tim14.ElapCallback = callback;
	}
	if(handle->Instance == TIM13)
	{
		tim13.Handle = handle;
		tim13.ElapCallback = callback;
	}
}


/**
  * @brief     开启定时器溢出中断
  */
void TIM_Open(TIM_Object* tim)
{
	HAL_TIM_Base_Start_IT(tim->Handle);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)	
{
	
	if(htim==(&htim14))
	{
		tim14.ElapCallback();
	}
	if(htim==(&htim13))
	{
		tim13.ElapCallback();
	}
}
