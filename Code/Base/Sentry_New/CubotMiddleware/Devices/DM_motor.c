#include "DM_motor.h"
#include "user_lib.h"
float a;

/**
 * @brief  达妙电机初始化
 */
void DMiaoInit(DMiao_t *damiao, uint16_t canid, uint16_t masterid, DMode_t mode)
{
	damiao->id   = masterid;
	damiao->mode = mode;
	
	damiao->Buffer.txBufferforEnable.Identifier        = canid;
	damiao->Buffer.txBufferforDisable.Identifier       = canid;
	damiao->Buffer.txBufferforMitMode.Identifier       = canid;
	damiao->Buffer.txBufferforPositionSpeed.Identifier = 0x100+canid;
	damiao->Buffer.txBufferforSpeed.Identifier         = 0x200+canid;
	damiao->Buffer.txBufferforCurrent.Identifier       = 0x3FE;
	
		damiao->Buffer.txBufferforEnable.Data[0] = 0xFF;
		damiao->Buffer.txBufferforEnable.Data[1] = 0xFF;
		damiao->Buffer.txBufferforEnable.Data[2] = 0xFF;
		damiao->Buffer.txBufferforEnable.Data[3] = 0xFF;
		damiao->Buffer.txBufferforEnable.Data[4] = 0xFF;
		damiao->Buffer.txBufferforEnable.Data[5] = 0xFF;
		damiao->Buffer.txBufferforEnable.Data[6] = 0xFF;
		damiao->Buffer.txBufferforEnable.Data[7] = 0xFC;
	
		damiao->Buffer.txBufferforDisable.Data[0] = 0xFF;
		damiao->Buffer.txBufferforDisable.Data[1] = 0xFF;
		damiao->Buffer.txBufferforDisable.Data[2] = 0xFF;
		damiao->Buffer.txBufferforDisable.Data[3] = 0xFF;
		damiao->Buffer.txBufferforDisable.Data[4] = 0xFF;
		damiao->Buffer.txBufferforDisable.Data[5] = 0xFF;
		damiao->Buffer.txBufferforDisable.Data[6] = 0xFF;
		damiao->Buffer.txBufferforDisable.Data[7] = 0xFD;
}

/**
 * @brief  达妙电机MIT模式
 */
void DMiaoMitControl(DMiao_t *damiao, float position, float speed, float kp, float kd, float torque)
{
	static uint16_t i_position, i_speed, i_kp, i_kd, i_torque;
	
	i_position = float_to_uint(position, -12.5f, 12.5f, 16);
	i_speed    = float_to_uint(speed, -30.0f, 30.0f, 12);
	i_kp       = float_to_uint(kp, 0, 500.0f, 12);
	i_kd       = float_to_uint(kd, 0, 5.0f, 12);
	i_torque   = float_to_uint(torque, -10.0f, 10.0f, 12);
	
	damiao->Buffer.txBufferforMitMode.Data[0] = (i_position >> 8);
	damiao->Buffer.txBufferforMitMode.Data[1] = i_position;
	damiao->Buffer.txBufferforMitMode.Data[2] = (i_speed >> 4);
	damiao->Buffer.txBufferforMitMode.Data[3] = ((i_speed&0xF)<<4)|(i_kp>>8);
	damiao->Buffer.txBufferforMitMode.Data[4] = i_kp;
	damiao->Buffer.txBufferforMitMode.Data[5] = (i_kd >> 4);
	damiao->Buffer.txBufferforMitMode.Data[6] = ((i_kd&0xF)<<4)|(i_torque>>8);
	damiao->Buffer.txBufferforMitMode.Data[7] = i_torque;
}

/**
 * @brief  达妙电机位置速度模式
 */
void DMiaoPositionSpeedControl(DMiao_t *damiao, float position, float speed)
{
	static uint8_t *i_position, *i_speed;
	
	i_position=(uint8_t*)&position;
	i_speed=(uint8_t*)&speed;
	
	damiao->Buffer.txBufferforPositionSpeed.Data[0] = *i_position;
	damiao->Buffer.txBufferforPositionSpeed.Data[1] = *(i_position+1);
	damiao->Buffer.txBufferforPositionSpeed.Data[2] = *(i_position+2);
	damiao->Buffer.txBufferforPositionSpeed.Data[3] = *(i_position+3);
	damiao->Buffer.txBufferforPositionSpeed.Data[4] = *i_speed;
	damiao->Buffer.txBufferforPositionSpeed.Data[5] = *(i_speed+1);
	damiao->Buffer.txBufferforPositionSpeed.Data[6] = *(i_speed+2);
	damiao->Buffer.txBufferforPositionSpeed.Data[7] = *(i_speed+3);
}

/**
 * @brief  达妙电机速度模式
 */
void DMiaoSpeedControl(DMiao_t *damiao, float speed)
{
	static uint8_t *i_speed;
	
	i_speed=(uint8_t*)&speed;
	
	damiao->Buffer.txBufferforSpeed.Data[0] = *i_speed;
	damiao->Buffer.txBufferforSpeed.Data[1] = *(i_speed+1);
	damiao->Buffer.txBufferforSpeed.Data[2] = *(i_speed+2);
	damiao->Buffer.txBufferforSpeed.Data[3] = *(i_speed+3);
}

/**
 * @brief  达妙电机电流模式
 */
void DMiaoCurrentControl(DMiao_t *damiao, int16_t current)
{
	damiao->Buffer.txBufferforCurrent.Data[0] =  current;
	damiao->Buffer.txBufferforCurrent.Data[1] = (current >> 8);
}
/**
 * @brief  达妙电机输出函数
 */
void DMiao_CanOutput(CAN_Object can, DMiao_t *damiao)
{
	switch(damiao->mode)
	{
		case MIT: 
		{
			CAN_Send(&can, &damiao->Buffer.txBufferforMitMode);
			break;
		}
		case POSITIONSPEED: 
		{
			CAN_Send(&can, &damiao->Buffer.txBufferforPositionSpeed);
			break;
		}
		case SPEED: 
		{
			CAN_Send(&can, &damiao->Buffer.txBufferforSpeed);
			break;
		}
		case CURRENT:
		{
			CAN_Send(&can,&damiao->Buffer.txBufferforCurrent);
			break;
		}
		default:;
	}
}	

/**
 * @brief  达妙电机使能
 */
void DMiao_Enable(CAN_Object can, DMiao_t *damiao)
{
	CAN_Send(&can, &damiao->Buffer.txBufferforEnable);
}
	
/**
 * @brief  达妙电机失能
 */
void DMiao_Disable(CAN_Object can, DMiao_t *damiao)
{
	CAN_Send(&can, &damiao->Buffer.txBufferforDisable);
}

/**
 * @brief  达妙电机反馈函数
 */
void DMiao_CanUpdata(DMiao_t *damiao,CAN_RxBuffer rxBuffer)
{
	switch(damiao->mode)
	{
		static uint16_t i_position, i_speed, i_torque;
		case MIT:
		case POSITIONSPEED: 
		case SPEED:
		{	
			if(rxBuffer.Header.Identifier == damiao->id)
			{
				damiao->error   	= (rxBuffer.Data[0]>>4);
				i_position 			= ((rxBuffer.Data[2])|(rxBuffer.Data[1]<<8));
				i_speed   			= ((rxBuffer.Data[4]|(rxBuffer.Data[3]<<8))>>4);
				i_torque   			= (rxBuffer.Data[5])|((rxBuffer.Data[4]&0x0f)<<8);
					
				damiao->angle_raw       = uint_to_float(i_position, -12.5f, 12.5f, 16); // (-12.5,12.5)
				damiao->speed_rpm   = uint_to_float(i_speed, -45.0f, 45.0f, 12);// (-45.0,45.0)
				damiao->torque      = uint_to_float(i_torque, -18.0f, 18.0f, 12); // (-18.0,18.0)
				damiao->total_angle = damiao->angle_raw * 57.2957f;
				damiao->angle = -wrap_to_180((damiao->total_angle - 103.72f));
			}
			break;
		}
		default:;
	}
}

float wrap_to_180(float angle)
{
	angle = fmod(angle, 360.0);
    if (angle > 180.0) {
        angle -= 360.0;
    } else if (angle <= -180.0) {
        angle += 360.0;
    }
    return angle;	
}