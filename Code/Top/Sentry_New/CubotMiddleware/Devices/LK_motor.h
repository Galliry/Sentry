#ifndef _LK_MOTOR_H_
#define _LK_MOTOR_H_
#include "driver_can.h"

typedef enum 
{	
	CURRENT_MODE,
	SPEED_MODE,
	POSITION_MODE
}LKMode_t;

typedef enum 
{
	ANGLE_PID,
	SPEED_PID,
	CURRENT_PID
}LKPidMode_t;

typedef struct
{
	uint16_t KP;
	uint16_t KI;
	uint16_t KD;
}PID_Param_t;

typedef struct
{
	struct
	{
		int8_t temperature;
		int16_t current_raw;
		int16_t speed; 			// 单位是DPS（度每秒）
		uint16_t encoder;
		float current;
	}Data;

	struct
	{
		PID_Param_t Angle;
		PID_Param_t Speed;
		PID_Param_t Current;
	}Param;

	struct
	{
		CAN_TxBuffer txBufferforData;
		CAN_TxBuffer txBufferforDisable;
		CAN_TxBuffer txBufferforEnable;
		CAN_TxBuffer txBufferforCurrent;
		CAN_TxBuffer txBufferforSpeed;
		CAN_TxBuffer txBufferforPosition;
		CAN_TxBuffer txBufferforReadPIDParam;
		CAN_TxBuffer txBufferforWritePIDParam;
	}Buffer;
}LKMotor_t;

void LK_CanUpdata(uint16_t id,CAN_RxBuffer rxBuffer,LKMotor_t* motor);
void LKMotor_CurrentMode(int16_t iqControl,LKMotor_t* motor);
void LKMotor_Init(uint16_t id,LKMotor_t* motor);
void LKMotor_CANOutPut(CAN_Object can,LKMotor_t* motor,uint8_t mode);
void LKMotor_SpeedMode(int32_t speedControl,int16_t iqControl,LKMotor_t* motor);
void LKMotor_PositionMode(int32_t angleIncrement,LKMotor_t* motor);
void LKMotor_ReadData(CAN_Object can,LKMotor_t* motor);
void LKMotor_Enable(CAN_Object can,LKMotor_t* motor);
void LKMotor_Disable(CAN_Object can,LKMotor_t* motor);
void LKMotor_ReadPIDParam(uint8_t Type,CAN_Object can, LKMotor_t* motor);
#endif
