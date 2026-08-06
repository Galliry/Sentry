#include "brain.h"
#include "holder.h"
#include "ins.h"
#include "interboard.h"
#include "shoot.h"
#include "DM_imu.h"
#include <cstring>
#include <string.h>
#include "et08.h"
#include "usart.h"
Brain_t Brain;
uint8_t RobotToBrainTimeBuffer[50];
uint8_t RobotToBrainChassisTimeBuffer[22];
#define HOLDER_MODE 1

#define AUTOAIM_Q_SELECT 1
#define AUTOAIM_VERSION 2

uint8_t Brain_Autoaim_Callback(uint8_t * recBuffer, uint16_t len)
{
	check_robot_state.Check_Usart.Check_vision_cnt = 0;
	Brain_Autoaim_DataUnpack(&Brain,recBuffer);
	return 0;
}

uint8_t Brain_Lidar_Callback(uint8_t * recBuffer, uint16_t len)
{
	check_robot_state.Check_Usart.Check_lidar_cnt = 0;
	Brain_Lidar_DataUnpack(&Brain,recBuffer);
	return 0;
}

void Brain_Autoaim_DataUnpack(Brain_t* brain ,uint8_t * recBuffer)
{
	#if AUTOAIM_VERSION == 1
	if (recBuffer[0] == 0xAA)
	{
		brain->Autoaim.Brain_Data.FrameType = recBuffer[1];
		brain->Autoaim.Brain_Data.FrameCoreID = recBuffer[2];

		if ((brain->Autoaim.Brain_Data.FrameType == 1) && recBuffer[12] == 0xDD) //< 解算偏转�??
		{
			brain->Autoaim.mode_cnt = 0;
			brain->Autoaim.mode = Lock;
			
			brain->Autoaim.Yaw_add = ((recBuffer[3] >> 6) == 0 ? 1 : -1) * ((float)((recBuffer[3] & 0x3f) * 100 + recBuffer[4]) / 100);
			brain->Autoaim.Pitch_add = ((recBuffer[5] >> 6) == 0 ? 1 : -1) * ((float)((recBuffer[5] & 0x3f) * 100 + recBuffer[6]) / 100);
			brain->Autoaim.Distance = (float)(recBuffer[7]) / 10;
			brain->Autoaim.IsFire_Autaim = recBuffer[8];
			brain->Autoaim.All_Sense = recBuffer[9];
			
			if(rc_Ctrl_et.rc.s2 == 2)
			{
				#if HOLDER_MODE == 1
				Holder.Yaw_S.Target_Angle = -Brain.Autoaim.Yaw_add + Holder.Yaw_S.Can_Angle;
				Holder.Pitch.Target_Angle = Brain.Autoaim.Pitch_add * 1.0f + Holder.Pitch.GYRO_Angle;
				#endif
				#if HOLDER_MODE == 2
				// 此时和自瞄的通信协�??为：原先表示增量的数�??变为位置的数�??
				Holder.Yaw_S.Target_Angle = Brain.Autoaim.Yaw_add;
				Holder.Pitch.Target_Angle = Brain.Autoaim.Pitch_add;
				#endif
			}
			if(ABS(Holder.Yaw_S.Target_Angle -Holder.Yaw_S.Can_Angle) < 0.8f && ABS(Holder.Pitch.Target_Angle - Holder.Pitch.GYRO_Angle) < 0.5f )
			{
				brain->Autoaim.IsFire = 1;
			}
			else if(ABS(Holder.Yaw_S.Target_Angle -Holder.Yaw_S.Can_Angle) < 0.8f && ABS(Holder.Pitch.Target_Angle - Holder.Pitch.GYRO_Angle) < 0.6f && (brain->Autoaim.Mode == Small_Buff || brain->Autoaim.Mode == Big_Buff))
				brain->Autoaim.IsFire = 1;
			else brain->Autoaim.IsFire = 0;
//			if(brain->Autoaim.Mode == Outpost)
//			{
//				ignore_outpost_cnt = 0;
//			}
		}
		else
		{
			brain->Autoaim.All_Sense = recBuffer[6];
			if (brain->Autoaim.mode_cnt >= 4)
			{
				brain->Autoaim.mode = Cruise;
				brain->Autoaim.IsFire = 0;
			}else 
			{
				brain->Autoaim.mode_cnt++;
			}
//			if(Top.Referee.game_time <= 390)
//			{
//				if(brain->Autoaim.Mode == Outpost)
//				{
//					ignore_outpost_cnt++;
//				}
//				if(ignore_outpost_cnt > 10000)
//					ignore_outpost = 1;
//				else
//					ignore_outpost = 0;
//			}
			
		}
	}
	#endif
	#if AUTOAIM_VERSION == 2
	if (recBuffer[0] == 'V' && recBuffer[1] == 'G' && recBuffer[27] == 'E' && recBuffer[28] == 'N')
	{
		brain->Autoaim.IsFire = recBuffer[2] == 2 && ( Holder.Yaw_S.PID.CorePID->Error < 0.5 && Holder.Pitch.PID.CorePID->Error < 0.4 )? 1 : 0;
        static uint16_t dislockCnt;
		//brain->Autoaim.mode = recBuffer[2] == 0 ? Cruise : Lock;
        if ( recBuffer[2] != 0 )
        {
            brain->Autoaim.mode = Lock;
            memcpy(&brain->Autoaim.Yaw, recBuffer+3, 4);
            brain->Autoaim.Yaw *= 180 / 3.1415f;
            memcpy(&brain->Autoaim.Yaw_vel, recBuffer+7, 4);
            brain->Autoaim.Yaw_vel *= 180 / 3.1415f;
            memcpy(&brain->Autoaim.Yaw_acc, recBuffer+11, 4);
            brain->Autoaim.Yaw_acc *= 180 / 3.1415f;
            memcpy(&brain->Autoaim.Pitch, recBuffer+15, 4);
            brain->Autoaim.Pitch *= -180 / 3.1415f;
            memcpy(&brain->Autoaim.Pitch_vel, recBuffer+19, 4);
            brain->Autoaim.Pitch_vel *= 180 / 3.1415f;
            memcpy(&brain->Autoaim.Pitch_acc, recBuffer+23, 4);
            brain->Autoaim.Pitch_acc *= 180 / 3.1415f;
//			brain->Autoaim.All_Sense = brain->Autoaim.Yaw_vel == 0 && brain->Autoaim.Yaw_acc == 0 && brain->Autoaim.Pitch_vel ==0 && brain->Autoaim.Pitch_acc == 0 && brain->Autoaim.mode == Lock ? 3 : 0;
            dislockCnt = 3;
			if(Top.Referee.game_time >= 360 && brain->Autoaim.Pitch <= 0.0f)
				brain->Autoaim.mode = Cruise;
        }
        else if ( brain->Autoaim.mode == Lock && dislockCnt > 0 )
        {
            dislockCnt --;
        }
        else 
        {
            brain->Autoaim.mode = Cruise;
        }
		

        if ( (rc_Ctrl_et.rc.s2 == 2 || Top.Referee.game_prograss == 4) && brain->Autoaim.mode != Cruise )
		{
			Holder.Yaw_S.Target_Angle = brain->Autoaim.Yaw;
			Holder.Pitch.Target_Angle = brain->Autoaim.Pitch ;
		}
		brain->Autoaim.All_Sense = brain->Autoaim.Yaw_vel == 0 && brain->Autoaim.Yaw_acc == 0 && brain->Autoaim.Pitch_vel ==0 && brain->Autoaim.Pitch_acc == 0 && brain->Autoaim.mode == Lock ? 3 : 0;
	}
	#endif
}


void Brain_Lidar_DataUnpack(Brain_t* brain ,uint8_t * recBuffer)
{
	if (recBuffer[0] == 0xAA)
	{
		brain->Lidar.Brain_Data.FrameType = recBuffer[1];

		brain->Lidar.movemode = recBuffer[2];

		if (brain->Lidar.Brain_Data.FrameType == BRAIN_TO_ROBOT_CMD) //< 解算偏转�??
		{
			brain->Lidar.vx = (((recBuffer[3] & 0x40) == 0) ? 1.0f : -1.0f) * (((float)((recBuffer[3] & 0x3F) * 100 + recBuffer[4]) / 100.0f)) ;
			brain->Lidar.vy = ((recBuffer[5] & 0x40) ? -1.0f : 1.0f) * ((float)((recBuffer[5] & 0x3f) * 100 + recBuffer[6]) / 100.0f) ;
			brain->Lidar.stance = recBuffer[9];
		}
	}
}

void RobotToBrain_Autoaim(float yaw,Brain_t* brain)//发给�??�??
{
	int16_t tmp0, tmp1, tmp2, tmp3;
	

#if AUTOAIM_Q_SELECT == 1
#if DMIMU_ENABLE
    EularAngleToQuaternion(IMU_S.Attitude.yaw,IMU_S.Attitude.pitch,IMU_S.Attitude.roll,IMU_S.Attitude.q);
	tmp0 = (int16_t)(IMU_S.Attitude.q[0] * 30000);
	tmp1 = -(int16_t)(IMU_S.Attitude.q[1] * 30000);
	tmp2 = -(int16_t)(IMU_S.Attitude.q[2] * 30000);
	tmp3 = (int16_t)(IMU_S.Attitude.q[3] * 30000);
#else  
	tmp0 = (int16_t)(INS_attitude->q[0] * 30000);
	tmp1 = -(int16_t)(INS_attitude->q[1] * 30000);
	tmp2 = -(int16_t)(INS_attitude->q[2] * 30000);
	tmp3 = (int16_t)(INS_attitude->q[3] * 30000);
#endif
#endif
#if AUTOAIM_Q_SELECT == 2
	float q[4];
	EularAngleToQuaternion(IMU_S.Attitude.yaw, -IMU_S.Attitude.roll, IMU_S.Attitude.pitch,q);
	float len = Sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] /= len;
    q[1] /= len;
    q[2] /= len;
	q[3] /= len;

	tmp0 = (int16_t)(q[0] * 30000);
	tmp1 = -(int16_t)(q[1] * 30000);
	tmp2 = -(int16_t)(q[2] * 30000);
	tmp3 = (int16_t)(q[3] * 30000);
#endif
#if AUTOAIM_Q_SELECT == 3
	tmp0 = (int16_t)((IMU_S.Attitude.q[0] - IMU_S.Attitude.q[3]) / sqrt(2) * 30000);
	tmp1 = (int16_t)((IMU_S.Attitude.q[1] + IMU_S.Attitude.q[2]) / sqrt(2) * 30000);
	tmp2 = (int16_t)((IMU_S.Attitude.q[2] - IMU_S.Attitude.q[1]) / sqrt(2) * 30000);
	tmp3 = (int16_t)((IMU_S.Attitude.q[3] + IMU_S.Attitude.q[0]) / sqrt(2) * 30000);
#endif

#if AUTOAIM_VERSION == 1
	RobotToBrainTimeBuffer[0] = 0xAA;
	RobotToBrainTimeBuffer[1] = 0x07;	// Type ;  //固定�??0x07
	RobotToBrainTimeBuffer[2] = 0x01;	// coreID;  //�??前固定为0x01
	RobotToBrainTimeBuffer[3] = 0x01;	// 索引，int16_t�??
	RobotToBrainTimeBuffer[4] = 0x01;

	RobotToBrainTimeBuffer[5] = (tim14.ClockTime >> 24);	// 定时器时间，int32_t�??
	RobotToBrainTimeBuffer[6] = ((tim14.ClockTime >> 16) & 0xff);
	RobotToBrainTimeBuffer[7] = ((tim14.ClockTime >> 8) & 0xff);
	RobotToBrainTimeBuffer[8] = ((tim14.ClockTime & 0xff));

	RobotToBrainTimeBuffer[9] = (Top.Referee.robot_id > 10) ? 1 : 0;
	RobotToBrainTimeBuffer[10] = tmp0 & 0xFF;	// 四元数q0，float�??
	RobotToBrainTimeBuffer[11] = tmp0 >> 8;
	RobotToBrainTimeBuffer[12] = tmp1 & 0xFF;
	RobotToBrainTimeBuffer[13] = tmp1 >> 8;
	RobotToBrainTimeBuffer[14] = tmp2 & 0xFF;	// 四元数q1，float�??
	RobotToBrainTimeBuffer[15] = tmp2 >> 8;
	RobotToBrainTimeBuffer[16] = tmp3 & 0xFF;
	RobotToBrainTimeBuffer[17] = tmp3 >> 8;
	RobotToBrainTimeBuffer[18] = 0x01;
	RobotToBrainTimeBuffer[19] = 0x01;

	RobotToBrainTimeBuffer[20] = brain->Autoaim.Mode;
	RobotToBrainTimeBuffer[21] = 0xDD;

	RobotToBrainTimeBuffer[22] = 0xDD;
	
	HAL_UART_Transmit_DMA(&huart2, RobotToBrainTimeBuffer, 23);

#endif
#if AUTOAIM_VERSION == 2
	RobotToBrainTimeBuffer[0] = 'G';
	RobotToBrainTimeBuffer[1] = 'V';
	RobotToBrainTimeBuffer[2] = brain->Autoaim.Mode;
//    float temp_yaw = IMU_S.Attitude.yaw / 360.0f * 2 * 3.14f;
#if DMIMU_ENABLE
//    float temp_yaw = -(mpu6050.Yaw - Holder.Motors.Yaw_S.Data.Angle) / 180.0f * 3.14f;
    float temp_yaw = IMU_S.Attitude.yaw / 180.0f * 3.14f;
    float temp_yaw_gyro = IMU_S.Attitude.gyro[2] / 180 * 3.14f;
    float temp_roll = IMU_S.Attitude.roll / 180 * 3.14f;
    float temp_roll_gyro = IMU_S.Attitude.gyro[0] / 180 * 3.14f;
    float temp_pitch = IMU_S.Attitude.pitch / 180 * 3.14f;
    float temp_pitch_gyro = IMU_S.Attitude.gyro[1] / 180 * 3.14f;
#else
	// float temp_yaw = -(mpu6050.Yaw - Holder.Motors.Yaw_S.Data.Angle) / 180.0f * 3.14f;
	// float temp_yaw = (INS_attitude->yaw ) / 180 * 3.14f;
    // float temp_yaw_gyro = INS_attitude->gyro[2] / 180 * 3.14f;
    // float temp_roll = INS_attitude->roll / 180 * 3.14f;
    // float temp_roll_gyro = INS_attitude->gyro[1] / 180 * 3.14f;
    // float temp_pitch = INS_attitude->pitch / 180 * 3.14f;
    // float temp_pitch_gyro = INS_attitude->gyro[0] / 180 * 3.14f;
#endif
	memcpy(RobotToBrainTimeBuffer+3,&temp_yaw,4);
	memcpy(RobotToBrainTimeBuffer+7,&temp_yaw_gyro,4);
	// memcpy(RobotToBrainTimeBuffer+11,&temp_roll,4);
	// memcpy(RobotToBrainTimeBuffer+15,&temp_roll_gyro,4);
	// memcpy(RobotToBrainTimeBuffer+19,&temp_pitch,4);
	memcpy(RobotToBrainTimeBuffer+11,&temp_pitch,4);
	memcpy(RobotToBrainTimeBuffer+15,&temp_pitch_gyro,4);
	memcpy(RobotToBrainTimeBuffer+19,&temp_roll,4);
    float temp_v = 22.0f;
	memcpy(RobotToBrainTimeBuffer+23,&temp_v,4);
//	RobotToBrainTimeBuffer[27] = AmmoBooster.Shoot_Plate.ShootNum;
//    RobotToBrainTimeBuffer[28] = AmmoBooster.Shoot_Plate.ShootNum;
	RobotToBrainTimeBuffer[29] = 'E';
	RobotToBrainTimeBuffer[30] = 'N';
	
	HAL_UART_Transmit_DMA(&huart2, RobotToBrainTimeBuffer, 31);

#endif
}

void RobotToBrain_Lidar(Brain_t* Brain)
{
	RobotToBrainChassisTimeBuffer[0] = 0xBB;
	
	RobotToBrainChassisTimeBuffer[1] = Top.Referee.game_time & 0xff; // referee2022.game_status.stage_remain_time
	RobotToBrainChassisTimeBuffer[2] = Top.Referee.game_time >> 8;
	RobotToBrainChassisTimeBuffer[3] = Top.Referee.robot_HP & 0xff;
	RobotToBrainChassisTimeBuffer[4] = Top.Referee.robot_HP >> 8;
	RobotToBrainChassisTimeBuffer[5] = Top.Referee.base_flag; //保护Base �??认为1
	RobotToBrainChassisTimeBuffer[6] = (Top.Referee.shoot_num <= 50) ? 0x01 : 0x00;
	RobotToBrainChassisTimeBuffer[7] = Top.Referee.lidar_target_state;
	RobotToBrainChassisTimeBuffer[8] = Top.Referee.outpost_flag;
	RobotToBrainChassisTimeBuffer[9] = 0xFF;
	
	HAL_UART_Transmit_DMA(&huart4, RobotToBrainChassisTimeBuffer,10);

}

void RobotToBrain(Brain_t *brain)
{
	RobotToBrain_Autoaim(0.0f,brain);
	if(tim14.ClockTime % 2 == 0)
	{
		RobotToBrain_Lidar(brain);	
	}
}

