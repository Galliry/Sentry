#ifndef DM_MOTOR_H
#define DM_MOTOR_H
#include "stm32h7xx_hal.h"
#include "fdcan.h"
#include "driver_can.h"
/**
 * @brief  达妙电机模式
 */
typedef enum 
{
	MIT,
	POSITIONSPEED,
	SPEED,
	CURRENT
}DMode_t;
	
/**
 * @brief  达妙电机结构体
 */
typedef struct
{
	uint16_t id;
	uint16_t online_cnt;
	int16_t torque_current;
    uint8_t temperature;
	uint8_t error;
	float angle;
	float torque;
	float speed_rpm;
	float motor_output;
	float angle_raw;
	CAN_TxBuffer txBufferforEnable;
	CAN_TxBuffer txBufferforDisable;
	CAN_TxBuffer txBufferforMitMode;
	CAN_TxBuffer txBufferforPositionSpeed;
	CAN_TxBuffer txBufferforSpeed;
	CAN_TxBuffer txBufferforCurrent;
	CAN_RxBuffer rxBufferforCurrent;
	DMode_t mode;
	struct
	{
		float kp;
		float kd;
	}MitMode;
}DMiao_t;

void DMiaoInit(DMiao_t *damiao, uint16_t canid, uint16_t masterid, DMode_t mode);
void DMiaoMitControl(DMiao_t *damiao, float position, float speed, float kp, float kd, float torque);
void DMiaoPositionSpeedControl(DMiao_t *damiao, float position, float speed);
void DMiaoSpeedControl(DMiao_t *damiao, float speed);
void DMiaoCurrentControl(DMiao_t *damiao, int16_t current);
void DMiao_Enable(CAN_Object can, DMiao_t *damiao);
void DMiao_Disable(CAN_Object can, DMiao_t *damiao);
void DMiao_CanOutput(CAN_Object can, DMiao_t *damiao);
void DMiao_CanUpdata(DMiao_t *damiao,CAN_RxBuffer rxBuffer);
#endif
