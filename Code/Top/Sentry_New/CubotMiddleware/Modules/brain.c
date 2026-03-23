#include "brain.h"
Brain_t Brain;
float last_yaw_add;
float last_pitch_add;
uint8_t RobotToBrainTimeBuffer[50];
uint8_t RobotToBrainChassisTimeBuffer[22];
int flag_fire;
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
	if (recBuffer[0] == 0xAA)
	{
		brain->Autoaim.Brain_Data.FrameType = recBuffer[1];
		brain->Autoaim.Brain_Data.FrameCoreID = recBuffer[2];

		if ((brain->Autoaim.Brain_Data.FrameType == BRAIN_TO_ROBOT_CMD) && recBuffer[11] == 0xDD ) //< 解算偏转角
		{
//			brain->Autoaim.mode_cnt[Cruise] = 0;
//			brain->Autoaim.Use_Can_angle = 0 ;// Brain->Autoaim.Send_Can_angle[recBuffer[12]];
//			brain->Autoaim.Use_Gyro_angle = 0 ;// Brain->Autoaim.Send_Gyro_angle[recBuffer[12]];

			brain->Autoaim.mode = Lock;
			last_yaw_add = brain->Autoaim.Yaw_add;
			last_pitch_add = brain->Autoaim.Pitch_add;
			brain->Autoaim.Yaw_add = ((recBuffer[3] >> 6) == 0 ? 1 : -1) * ((float)((recBuffer[3] & 0x3f) * 100 + recBuffer[4]) / 100);
			brain->Autoaim.Pitch_add = ((recBuffer[5] >> 6) == 0 ? 1 : -1) * ((float)((recBuffer[5] & 0x3f) * 100 + recBuffer[6]) / 100);
			
			brain->Autoaim.Distance = (float)(recBuffer[7]) / 10;
//			brain->Autoaim.IsFire = ((float)(recBuffer[8])); 
//			stable = recBuffer[9];

			// Brain->Autoaim.Attack_state.camara_num = recBuffer[11];
			// if (Brain->Autoaim.Attack_state.camara_num >= 9)
			// 	Brain->Autoaim.Attack_state.camara_num -= 9;

			//			Brain->All_See.armorNumber[0] = Brain->Autoaim.camara_num;
			//			Brain->All_See.Distance[0]=Brain->Autoaim.Distance*100;
//			if ( brain->Autoaim.Yaw_add == 0 && brain->Autoaim.Pitch_add == 0) { 	// 丢失目标
//				brain->Autoaim.mode = Cruise;
//				Brain->Autoaim.change_mode_cnt = 0;
			if(rc_Ctrl.rc.s2 == 2)
			{
				Holder.Yaw_S.Target_Angle = -Brain.Autoaim.Yaw_add + Holder.Yaw_S.Can_Angle;
				Holder.Pitch.Target_Angle = Brain.Autoaim.Pitch_add * 1.15 + Holder.Pitch.GYRO_Angle;
			}
			
			if(ABS(Holder.Yaw_S.Target_Angle -Holder.Yaw_S.Can_Angle) < 0.3f && ABS(Holder.Pitch.Target_Angle - Holder.Pitch.GYRO_Angle) < 0.3f)
				brain->Autoaim.IsFire = 1;
			else brain->Autoaim.IsFire = 0;
		}else
		{
			brain->Autoaim.mode = Cruise;
		}
//			else{
//				if (Brain->Autoaim.mode == Cruise ) {
//					Brain->Autoaim.mode = Change;
//				}
//				else if (   Brain->Autoaim.mode == Change 
//							&& Brain->Autoaim.change_mode_cnt >= 1
//						){
//					Brain->Autoaim.mode = Lock;
//					Brain->Autoaim.change_mode_cnt = 0;
//					next_yaw1_tar = Holder.Yaw1.GYRO_Angle;
//					next_pitch_tar = Holder.Pitch.GYRO_Angle;
//				}
//				else if ( Brain->Autoaim.mode == Change ){
//					Brain->Autoaim.change_mode_cnt ++ ;
//				}
//			}

//			Brain->Autoaim.fire_flag = 0;
//			if (rc_Ctrl_et.rc.s2 == 2 
//				&& Brain->Autoaim.mode == Lock
//				)
//			{
//				if (Brain->Autoaim.change_mode_cnt >= 6) {
//					Brain->Autoaim.change_mode_cnt = 0;

//					if (fabs(Holder.Yaw1.Target_Angle - Holder.Yaw1.Can_Angle) < 0.8 && Brain->Autoaim.Mode == Outpost)
//						Brain->Autoaim.fire_flag = 1;
//					else if (fabs(Holder.Yaw1.Target_Angle - Holder.Yaw1.Can_Angle) < 0.4 && Brain->Autoaim.Mode == Autoaim)
//						Brain->Autoaim.fire_flag = 1;
//					else
//						Brain->Autoaim.fire_flag = 0;
//					// UsartDmaPrintf("%.2f\r\n", Brain->Autoaim.Yaw_add);
//					const float df = 3.0; // 低于此值则降低输入
//					if ( stable == 0 && fabs(Brain->Autoaim.Yaw_add) < df ){
//						Brain->Autoaim.Yaw_add = Brain->Autoaim.Yaw_add * (Brain->Autoaim.Yaw_add / df) * (Brain->Autoaim.Yaw_add / df) ;
//					}
//					// if ( fabs(Brain->Autoaim.Yaw_add) < 1 ){
//					// 	Brain->Autoaim.Yaw_add = Brain->Autoaim.Yaw_add * (Brain->Autoaim.Yaw_add / df) * (Brain->Autoaim.Yaw_add / df);
//					// }
//					next_yaw1_tar = Holder.Yaw1.Target_Angle + Brain->Autoaim.Yaw_add * 110;
//					// Holder.Yaw1.Target_Angle = Holder.Yaw1.Target_Angle + Brain->Autoaim.Yaw_add * 1.5;
//					next_pitch_tar = Holder.Pitch.Target_Angle + Brain->Autoaim.Pitch_add * 18;
//				}
//				else{
//					Brain->Autoaim.change_mode_cnt ++;
//					Holder.Yaw1.Target_Angle = LPFilter(next_yaw1_tar, &LPF_yaw_mpu);
//					Holder.Pitch.Target_Angle = LPFilter(next_pitch_tar, &LPF_pitch_mpu);
//				}

//			
//			else if ( rc_Ctrl_et.rc.s2 == 2 
//					&& brain->Autoaim.mode == Cruise){
//						// pitch sin t
//					}
	}
}


void Brain_Lidar_DataUnpack(Brain_t* brain ,uint8_t * recBuffer)
{
	if (recBuffer[0] == 0xAA)
	{
		brain->Lidar.Brain_Data.FrameType = recBuffer[1];

		brain->Lidar.movemode = recBuffer[2];

		if (brain->Lidar.Brain_Data.FrameType == BRAIN_TO_ROBOT_CMD) //< 解算偏转角
		{
			brain->Lidar.vx = (((recBuffer[3] & 0x40) == 0) ? 1.0f : -1.0f) * (((float)((recBuffer[3] & 0x3F) * 100 + recBuffer[4]) / 100.0f)) ;
			brain->Lidar.vy = ((recBuffer[5] & 0x40) ? -1.0f : 1.0f) * ((float)((recBuffer[5] & 0x3f) * 100 + recBuffer[6]) / 100.0f) ;
			// Brain->Lidar.angle_to_lidar=((recBuffer[7] >> 6) == 0 ? 1 : -1)*((float)((recBuffer[7]&0x3f)*100 + recBuffer[8])/100);
//			brain->Lidar.Arrive = recBuffer[9];
//			a222 = ((recBuffer[10] >> 6) == 0 ? 1 : -1) * ((float)((recBuffer[10] & 0x3f) * 100 + recBuffer[11]) / 100);
		}
	}
}

void RobotToBrain_Autoaim(float yaw,Brain_t* brain)//发给自瞄
{
	int16_t tmp0, tmp1, tmp2, tmp3;

//	ThisSecond++;
	tmp0 = (int16_t)(INS_attitude->q[0] * 30000);
	tmp1 = -(int16_t)(INS_attitude->q[1] * 30000);
	tmp2 = -(int16_t)(INS_attitude->q[2] * 30000);
	tmp3 = (int16_t)(INS_attitude->q[3] * 30000);

	RobotToBrainTimeBuffer[0] = 0xAA;
	RobotToBrainTimeBuffer[1] = 0x07;	// Type ;  //固定为0x07
	RobotToBrainTimeBuffer[2] = 0x01;	// coreID;  //目前固定为0x01
	RobotToBrainTimeBuffer[3] = 0x01;	// 索引，int16_t型
	RobotToBrainTimeBuffer[4] = 0x01;

	RobotToBrainTimeBuffer[5] = (tim14.ClockTime >> 24);	// 定时器时间，int32_t型
	RobotToBrainTimeBuffer[6] = ((tim14.ClockTime >> 16) & 0xff);
	RobotToBrainTimeBuffer[7] = ((tim14.ClockTime >> 8) & 0xff);
	RobotToBrainTimeBuffer[8] = ((tim14.ClockTime & 0xff));

	RobotToBrainTimeBuffer[9] = (referee2022.game_robot_status.robot_id > 10) ? 1 : 0;

	RobotToBrainTimeBuffer[10] = tmp0 & 0xFF;	// 四元数q0，float型
	RobotToBrainTimeBuffer[11] = tmp0 >> 8;
	RobotToBrainTimeBuffer[12] = tmp1 & 0xFF;
	RobotToBrainTimeBuffer[13] = tmp1 >> 8;
	RobotToBrainTimeBuffer[14] = tmp2 & 0xFF;	// 四元数q1，float型
	RobotToBrainTimeBuffer[15] = tmp2 >> 8;
	RobotToBrainTimeBuffer[16] = tmp3 & 0xFF;
	RobotToBrainTimeBuffer[17] = tmp3 >> 8;
//	if (brain->Autoaim.mode == Cruise || brain->Autoaim.mode == Change)
//	{
//		brain->Autoaim.Stand = 0;
//		cnt___ = 0;
//	}
//	else if (brain->Autoaim.mode == Lock)
//		cnt___++;
//	if (cnt___ >= 1)
//	{
//		brain->Autoaim.Stand = 1;
//		cnt___ = 0;
//	}

	RobotToBrainTimeBuffer[18] = 0x01; // 0是预测 1是跟随 4 ceres 静止或低速
	RobotToBrainTimeBuffer[19] = 0x01;
//  RobotToBrainTimeBuffer[19] = tmp4 >> 8;

	RobotToBrainTimeBuffer[20] = brain->Autoaim.Mode; // 1 是前哨站 0是普通
//	Armor_Ignore(brain);
	RobotToBrainTimeBuffer[21] = 0xDD; // 忽略装甲板

	RobotToBrainTimeBuffer[22] = 0xDD;
//  RobotToBrainTimeBuffer[23] = 0xDD;

//	brain->Autoaim.Send_Can_angle[RobotToBrainTimeBuffer[8]] = Holder.Yaw1.Can_Angle;
//	brain->Autoaim.Send_Gyro_angle[RobotToBrainTimeBuffer[8]] = Holder.Pitch.GYRO_Angle;

	HAL_UART_Transmit_DMA(&huart2, RobotToBrainTimeBuffer, 23);
}

void RobotToBrain_Lidar(Brain_t* Brain)
{
	//  x = referee2022.map_command_t.target_position_x * 100;
	//	y = referee2022.map_command_t.target_position_y * 100;
	RobotToBrainChassisTimeBuffer[0] = 0xAA;

	if (check_robot_state.Check_Usart.Check_receiver == 0)
	{
		RobotToBrainChassisTimeBuffer[1] = 1;
		RobotToBrainChassisTimeBuffer[2] = 1;
	}
	else if (referee2022.game_status.game_progress == 4)
	{
		RobotToBrainChassisTimeBuffer[1] = referee2022.game_status.stage_remain_time & 0xff; // referee2022.game_status.stage_remain_time
		RobotToBrainChassisTimeBuffer[2] = referee2022.game_status.stage_remain_time >> 8;
	}
	else
	{
		RobotToBrainChassisTimeBuffer[1] = 0;
		RobotToBrainChassisTimeBuffer[2] = 0;
	}
	if (rc_Ctrl_et.rc.s2 == 1)
	{
		RobotToBrainChassisTimeBuffer[1] = 1; // referee2022.game_status.stage_remain_time
		RobotToBrainChassisTimeBuffer[2] = 1;
	}
	// if(referee2022.game_robot_status.robot_id==0x07)
	// {
	//  outpost_self=referee2022.game_robot_hp.red_outpost_HP;
	//	outpost_enemy=referee2022.game_robot_hp.blue_outpost_HP;
	//}
	//	else if(referee2022.game_robot_status.robot_id==0x6b)
	//{
	//   outpost_self=referee2022.game_robot_hp.blue_outpost_HP;
	//		outpost_enemy=referee2022.game_robot_hp.red_outpost_HP;
	// }
	//	if(referee2022.bullet_remaining.bullet_remaining_num>0&&referee2022.game_robot_status.mains_power_shooter_output==0)
	//		shoot_flag=0;
	//	else shoot_flag=1;
	// if(referee2022.buff.defence_buff>0)defense_flag=1;
	//	else defense_flag=0;

	// change_position=kkk;
	RobotToBrainChassisTimeBuffer[3] = 0;  // change_position

	// Brain->Lidar.mode=kk1;
	//=Lidar_Fortress;
	// Brain->Lidar.mode=kk1;
	RobotToBrainChassisTimeBuffer[4] = Brain->Lidar.mode;
	//	RobotToBrainChassisTimeBuffer[5]  = y&0xff;
	//	RobotToBrainChassisTimeBuffer[6]  = y>>8;
	//	RobotToBrainChassisTimeBuffer[7]  = outpost_self&0xff;
	//	RobotToBrainChassisTimeBuffer[8]  = outpost_self>>8;
	//	RobotToBrainChassisTimeBuffer[9]  = outpost_enemy&0xff;
	//	RobotToBrainChassisTimeBuffer[10]  = outpost_enemy>>8;
	RobotToBrainChassisTimeBuffer[11] = referee2022.game_robot_status.remain_HP & 0xff;
	RobotToBrainChassisTimeBuffer[12] = referee2022.game_robot_status.remain_HP >> 8;
	//	if (RobotToBrainChassisTimeBuffer[11]==0xDD) RobotToBrainChassisTimeBuffer[11]=0xDE;

	//	RobotToBrainChassisTimeBuffer[13]  = referee2022.bullet_remaining.bullet_remaining_num&0xff;
	//	RobotToBrainChassisTimeBuffer[14]  = referee2022.bullet_remaining.bullet_remaining_num >> 8;
	//	RobotToBrainChassisTimeBuffer[15]  = shoot_flag;//referee2022.game_robot_status.mains_power_shooter_output;
	//	RobotToBrainChassisTimeBuffer[16]  = referee2022.bullet_remaining.money&0xff;
	//	RobotToBrainChassisTimeBuffer[17]  = referee2022.bullet_remaining.money >> 8;
	// RobotToBrainChassisTimeBuffer[18]  = defense_flag;
	//	RobotToBrainChassisTimeBuffer[19]  = lidar_station_flag;
	//	RobotToBrainChassisTimeBuffer[20]  = lidar_mode;
	RobotToBrainChassisTimeBuffer[21] = 0xDD;
	HAL_UART_Transmit_DMA(&huart4, RobotToBrainChassisTimeBuffer, 22);

}

void RobotToBrain(Brain_t *brain)
{
	RobotToBrain_Autoaim(0.0f,brain);
	if(tim14.ClockTime % 2 == 0)
	{
		RobotToBrain_Lidar(brain);	
	}

}

