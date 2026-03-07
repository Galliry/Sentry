#include "LK_motor.h"

void LKMotor_Init(uint16_t id,LKMotor_t* motor)
{
	motor->Buffer.txBufferforData.Identifier = id;
	motor->Buffer.txBufferforEnable.Identifier = id;
	motor->Buffer.txBufferforDisable.Identifier = id;
	motor->Buffer.txBufferforCurrent.Identifier = id;
	motor->Buffer.txBufferforSpeed.Identifier = id;
	motor->Buffer.txBufferforPosition.Identifier = id;
	motor->Buffer.txBufferforReadPIDParam.Identifier = id;
	motor->Buffer.txBufferforWritePIDParam.Identifier = id;
	
	motor->Buffer.txBufferforData.Data[0] = 0x9A;
	motor->Buffer.txBufferforEnable.Data[0] = 0x88;
	motor->Buffer.txBufferforDisable.Data[0] = 0x80;
	motor->Buffer.txBufferforCurrent.Data[0] = 0xA1;
	motor->Buffer.txBufferforSpeed.Data[0] = 0xA2;
	motor->Buffer.txBufferforPosition.Data[0] = 0xA7;
	motor->Buffer.txBufferforReadPIDParam.Data[0] = 0xC0;
	motor->Buffer.txBufferforWritePIDParam.Data[0] = 0xC1;
}

void LKMotor_ReadData(CAN_Object can,LKMotor_t* motor)
{
	CAN_Send(&can,&motor->Buffer.txBufferforData);
}

void LKMotor_Enable(CAN_Object can,LKMotor_t* motor)
{
	CAN_Send(&can,&motor->Buffer.txBufferforDisable);
}

void LKMotor_Disable(CAN_Object can,LKMotor_t* motor)
{
	CAN_Send(&can,&motor->Buffer.txBufferforDisable);
}

void LKMotor_CurrentMode(int16_t iqControl,LKMotor_t* motor)
{
	//оч╥Ы
	if(iqControl > 2048)
	{
		iqControl = 2048;
	}else if(iqControl < -2048)
	{
		iqControl = -2048;
	}
	motor->Buffer.txBufferforCurrent.Data[1] = 0x00;
	motor->Buffer.txBufferforCurrent.Data[2] = 0x00;
	motor->Buffer.txBufferforCurrent.Data[3] = 0x00;
	motor->Buffer.txBufferforCurrent.Data[4] = (uint8_t)(iqControl & 0xFF);
	motor->Buffer.txBufferforCurrent.Data[5] = (uint8_t)((iqControl >> 8) & 0xFF);
	motor->Buffer.txBufferforCurrent.Data[6] = 0x00;
	motor->Buffer.txBufferforCurrent.Data[7] = 0x00;

}

void LKMotor_SpeedMode(int32_t speedControl,int16_t iqControl,LKMotor_t* motor)
{
	//оч╥Ы
	if(iqControl > 2048)
	{
		iqControl = 2048;
	}else if(iqControl < -2048)
	{
		iqControl = -2048;
	}
	motor->Buffer.txBufferforSpeed.Data[1] = 0x00;
	motor->Buffer.txBufferforSpeed.Data[2] = (uint8_t)(iqControl );
	motor->Buffer.txBufferforSpeed.Data[3] = (uint8_t)((iqControl >> 8) & 0xFF);
	motor->Buffer.txBufferforSpeed.Data[4] = (uint8_t)(speedControl & 0xFF);
    motor->Buffer.txBufferforSpeed.Data[5] = (uint8_t)((speedControl >> 8) & 0xFF);
    motor->Buffer.txBufferforSpeed.Data[6] = (uint8_t)((speedControl >> 16) & 0xFF);
    motor->Buffer.txBufferforSpeed.Data[7] = (uint8_t)((speedControl >> 24) & 0xFF);
}

void LKMotor_PositionMode(int32_t angleIncrement,LKMotor_t* motor)
{
	motor->Buffer.txBufferforPosition.Data[1] = 0x00;
	motor->Buffer.txBufferforPosition.Data[2] = 0x00;
	motor->Buffer.txBufferforPosition.Data[3] = 0x00;
	motor->Buffer.txBufferforPosition.Data[4] = (uint8_t)(angleIncrement & 0xFF);
	motor->Buffer.txBufferforPosition.Data[5] = (uint8_t)((angleIncrement >> 8) & 0xFF);
	motor->Buffer.txBufferforPosition.Data[6] = (uint8_t)((angleIncrement >> 16) & 0xFF);
	motor->Buffer.txBufferforPosition.Data[7] = (uint8_t)((angleIncrement >> 24) & 0xFF);
}

void LKMotor_ReadPIDParam(uint8_t Type,CAN_Object can, LKMotor_t* motor)
{
	if(Type == ANGLE_PID)
		motor->Buffer.txBufferforReadPIDParam.Data[1] = 0x0A;
	if(Type == SPEED_PID)
		motor->Buffer.txBufferforReadPIDParam.Data[1] = 0x0B;
	if(Type == CURRENT_PID)
		motor->Buffer.txBufferforReadPIDParam.Data[1] = 0x0C;
	CAN_Send(&can,&motor->Buffer.txBufferforReadPIDParam);
}

void LK_CanUpdata(uint16_t id,CAN_RxBuffer rxBuffer,LKMotor_t* motor)
{
	if(rxBuffer.Header.Identifier == id)
	{
		if(rxBuffer.Data[0] == 0x9A || rxBuffer.Data[0] == 0xA1 || rxBuffer.Data[0] == 0xA2 || rxBuffer.Data[0] == 0xA7)
		{
			motor->Data.temperature = (int8_t)rxBuffer.Data[1];
			motor->Data.current_raw = (int16_t)((rxBuffer.Data[3]<<8) | rxBuffer.Data[2]);
			motor->Data.speed = (int16_t)((rxBuffer.Data[5]<<8) | rxBuffer.Data[4]);
			motor->Data.encoder = (uint16_t)((rxBuffer.Data[7]<<8) | rxBuffer.Data[6]);
			motor->Data.current = motor->Data.current_raw * (66.0f/4096.0f);
		}else if(rxBuffer.Data[0] == 0xC0 || rxBuffer.Data[0] == 0xC1)
		{
			if(rxBuffer.Data[1] == 0x0A)
			{
				motor->Param.Angle.KP = (uint16_t)((rxBuffer.Data[3]<<8) | rxBuffer.Data[2]);
				motor->Param.Angle.KI = (uint16_t)((rxBuffer.Data[5]<<8) | rxBuffer.Data[4]);
				motor->Param.Angle.KD = (uint16_t)((rxBuffer.Data[7]<<8) | rxBuffer.Data[6]);
			}else if(rxBuffer.Data[1] == 0x0B)
			{
				motor->Param.Speed.KP = (uint16_t)((rxBuffer.Data[3]<<8) | rxBuffer.Data[2]);
				motor->Param.Speed.KI = (uint16_t)((rxBuffer.Data[5]<<8) | rxBuffer.Data[4]);
				motor->Param.Speed.KD = (uint16_t)((rxBuffer.Data[7]<<8) | rxBuffer.Data[6]);
			}else if(rxBuffer.Data[1] == 0x0C)
			{
				motor->Param.Current.KP = (uint16_t)((rxBuffer.Data[3]<<8) | rxBuffer.Data[2]);
				motor->Param.Current.KI = (uint16_t)((rxBuffer.Data[5]<<8) | rxBuffer.Data[4]);
				motor->Param.Current.KD = (uint16_t)((rxBuffer.Data[7]<<8) | rxBuffer.Data[6]);
			}
		}
	}

}

void LKMotor_CANOutPut(CAN_Object can,LKMotor_t* motor,uint8_t mode)
{
	if(mode == CURRENT_MODE)
		CAN_Send(&can,&motor->Buffer.txBufferforCurrent);
	if(mode == SPEED_MODE)
		CAN_Send(&can,&motor->Buffer.txBufferforSpeed);
	if(mode == POSITION_MODE)
		CAN_Send(&can,&motor->Buffer.txBufferforPosition);
}

