#include "holder.h"
#include "et08.h"
#include "ins.h"
#include "mpu6050.h"
#include "driver_timer.h"
#include "user_lib.h"
#include "communication.h"
#include "filter.h"
Holder_t Holder;
float Yaw_TD = 200;
float Pitch_TD = 90;
int k = 0;
float fliter=0.9;
/**
  * @brief 云台初始化上板
  */
void HolderInit_Top(Holder_t* holder,DualPID_Object* pitch,DualPID_Object* yaw_s)
{
	DMiaoInit(&holder->Motors.Pitch,0x06,0x01,MIT);
	MotorInit(&holder->Motors.Yaw_S,4085,Motor6020,CAN1,0x205);
	DualPID_Init(&holder->Yaw_S.PID,yaw_s->ShellPID,yaw_s->CorePID);
	DualPID_Init(&holder->Pitch.PID,pitch->ShellPID,pitch->CorePID);
	holder->Pitch.Sensitivity = 0.00085f;		//-0.0015f
	holder->Yaw_S.Sensitivity = 0.00085f;
}
/**
  * @brief 云台控制
  */
void HolderControl_Top(Holder_t* holder,RC_Ctrl* rc_ctrl)
{
	if(rc_ctrl->rc.s2 == 1) {holder->Yaw_S.Target_Angle -= ((rc_ctrl->rc.ch2 - 1024) * holder->Yaw_S.Sensitivity);}
	holder->Pitch.Target_Angle += ((rc_ctrl->rc.ch3 - 1024) * holder->Pitch.Sensitivity);
	if(rc_ctrl->rc.s2 == 2 || Receive.Top.Referee.game_prograss == 4)
	{
		if(check_robot_state.Check_Usart.Check_vision == 1
			|| rc_Ctrl.rc.s2 == 2
		)
		{
			if(Brain.Autoaim.mode == Cruise)
			{
//				holder->Yaw_S.Target_Angle = 30 * sin(HAL_GetTick () / 200.0f);
				holder->Yaw_S.Target_Angle = 0;
				holder->Pitch.Target_Angle = 8 * sin(HAL_GetTick()/100.0f) - 7;
			}
			else if(Brain.Autoaim.mode == Lock)
		    {
				Holder_TD(&holder->Pitch,holder->Pitch.Target_Angle,Pitch_TD,0.001);
				Holder_TD(&holder->Yaw_S,holder->Yaw_S.Target_Angle ,Yaw_TD,0.001);
//				holder->Yaw_S.Target_Angle = fliter * holder->Yaw_S.Target_Angle  +(1-fliter) * holder->Yaw_S.Last_Target_Angle;
//				holder->Yaw_S.Last_Target_Angle  = holder->Yaw_S.Target_Angle; 
			}
			
		}
		
	}
	
	holder->Yaw_S.Can_Angle = holder->Motors.Yaw_S.Data.Angle;
	holder->Yaw_S.Can_AngleSpeed = holder->Motors.Yaw_S.Data.AngleSpeed;
	holder->Yaw_S.GYRO_Angle = INS_attitude->yaw;
	holder->Yaw_S.GYRO_AngleSpeed = INS_attitude->gyro[2] - mpu6050.mpu6050_Data.gyro[2];
	
	holder->Pitch.Can_Angle = holder->Motors.Pitch.angle;
	holder->Pitch.Can_AngleSpeed = holder->Motors.Pitch.speed_rpm;
	holder->Pitch.GYRO_Angle = -INS_attitude->roll;
	holder->Pitch.GYRO_AngleSpeed = -INS_attitude->gyro[1];
	
	holder->Yaw_S.Target_Angle = float_constrain(holder->Yaw_S.Target_Angle,-38,38);
	holder->Pitch.Target_Angle = float_constrain(holder->Pitch.Target_Angle,-30,30);
	
	holder->Yaw_S.Target_Angle = LPFilter( holder->Yaw_S.Target_Angle, &LPF_yaw_mpu );
	holder->Pitch.Target_Angle = LPFilter( holder->Pitch.Target_Angle, &LPF_pitch_mpu );
	
	holder->Yaw_S.Target_Angle = float_constrain(holder->Yaw_S.Target_Angle,-38,38);
	holder->Pitch.Target_Angle = float_constrain(holder->Pitch.Target_Angle,-30,30);
	
	if(rc_ctrl->rc.s2 == 2 && Brain.Autoaim.mode == Lock)
	{
		holder->Motors.Yaw_S.Data.Output = k * holder->Yaw_S.v2 + BasePID_SpeedControl(holder->Yaw_S.PID.CorePID,
			BasePID_AngleControl(holder->Yaw_S.PID.ShellPID,holder->Yaw_S.v1,holder->Yaw_S.Can_Angle),holder->Yaw_S.GYRO_AngleSpeed);
		holder->Motors.Pitch.motor_output = BasePID_SpeedControl(holder->Pitch.PID.CorePID,
			BasePID_AngleControl(holder->Pitch.PID.ShellPID,holder->Pitch.v1,holder->Pitch.GYRO_Angle),holder->Pitch.GYRO_AngleSpeed);
	}
	else
	{
		holder->Motors.Yaw_S.Data.Output = BasePID_SpeedControl(holder->Yaw_S.PID.CorePID,
			BasePID_AngleControl(holder->Yaw_S.PID.ShellPID,holder->Yaw_S.Target_Angle,holder->Yaw_S.Can_Angle),holder->Yaw_S.GYRO_AngleSpeed);
		holder->Motors.Pitch.motor_output = BasePID_SpeedControl(holder->Pitch.PID.CorePID,
			BasePID_AngleControl(holder->Pitch.PID.ShellPID,holder->Pitch.Target_Angle,holder->Pitch.GYRO_Angle),holder->Pitch.GYRO_AngleSpeed);
	}
	
	
	holder->Motors.Pitch.motor_output = float_constrain(holder->Motors.Pitch.motor_output,-50,50);
	DMiaoMitControl(&holder->Motors.Pitch,0,0,0,0,holder->Motors.Pitch.motor_output * 0.05f);
	MotorFillData(&holder->Motors.Yaw_S,holder->Motors.Yaw_S.Data.Output);
//	holder->Motors.Yaw_S.Data.Output = fliter * holder->Motors.Yaw_S.Data.Output  +(1-fliter) * holder->Motors.Yaw_S.Data.Last_Output;
//	holder->Motors.Yaw_S.Data.Last_Output  = holder->Motors.Yaw_S.Data.Output; 
}


float Holder_TD(struct Holder_Motor_Info* holder_info,float Expect,float r,float h)
{
    double fh= -r * r * (holder_info->v1 - Expect) - 2 * r * holder_info->v2;
    holder_info->v1 += holder_info->v2 * h;
    holder_info->v2 += fh * h;
		return holder_info->v1;
}
//void HolderInit(Holder_t* holder,DualPID_Object* pitch,DualPID_Object* yaw_m,DualPID_Object* yaw_s)
//{
//	// MotorInit(&holder->Motors.Pitch,6993,Motor6020,CAN1,0x206);
//	MotorInit(&holder->Motors.Yaw_S,5924,Motor6020,CAN1,0x207);
//	// MotorInit(&holder->Motors.Yaw_M,5645,Motor6020,CAN1,0x205);

//	DualPID_Init(&holder->Pitch.PID,pitch->ShellPID,pitch->CorePID);
//	DualPID_Init(&holder->Yaw_M.PID,yaw_m->ShellPID,yaw_m->CorePID);
//	DualPID_Init(&holder->Yaw_S.PID,yaw_s->ShellPID,yaw_s->CorePID);
//	
//	holder->Pitch.Sensitivity = 0.00085f;//-0.0015f
//	holder->Yaw_M.Sensitivity = 0.0010f;//0.003f 0.0015
//	holder->Yaw_S.Sensitivity = 0.0010f;
//	holder->Cruise_Mode.Yaw_S_sense=0.00325;
//	holder->Cruise_Mode.Pitch_sense=0.0065;
//	// holder->up_litmit=29;
//	// holder->down_litmit=-35;//-35
//	// holder->right_litmit=-85;
//	// holder->left_litmit=75;
//}

//void Holder_Control(Holder_t* holder,RC_Ctrl_ET* rc_ctrl)
//{
//	holder->Pitch.Target_Angle += ((rc_ctrl->rc.ch3 -1024) * holder->Pitch.Sensitivity);
//	if(rc_ctrl->rc.s2 != 1) holder->Yaw_M.Target_Angle += ((rc_ctrl->rc.ch2 - 1024) * holder->Yaw_M.Sensitivity);
//	else if(rc_ctrl->rc.s2 == 1) holder->Yaw_S.Target_Angle += ((rc_ctrl->rc.ch2 - 1024) * holder->Yaw_S.Sensitivity);
//	
//	// if (holder->Yaw_S.Can_Angle > 61) holder->down_litmit = -19;
//	// else holder->down_litmit = -35;
//	
//	// holder->Pitch.Can_Angle = holder->Motors.Pitch.Data.Angle;
//	// holder->Yaw_M.Can_Angle = holder->Motors.Yaw_M.Data.Angle;
//	holder->Yaw_S.Can_Angle = holder->Motors.Yaw_S.Data.Angle;
//	// holder->Pitch.Can_AngleSpeed = holder->Motors.Pitch.Data.SpeedRPM;
//	// holder->Yaw_M.Can_AngleSpeed = holder->Motors.Yaw_M.Data.SpeedRPM/2;//减速比1：2
//	holder->Yaw_S.Can_AngleSpeed = holder->Motors.Yaw_S.Data.SpeedRPM;
//	
//	holder->Pitch.GYRO_Angle = -(INS_attitude->pitch);
//	holder->Yaw_M.GYRO_Angle = mpu6050.Yaw_total_angle;
//	holder->Yaw_S.GYRO_Angle = INS_attitude->yaw;
//	holder->Yaw_M.GYRO_AngleSpeed = -150.0f * mpu6050.mpu6050_Data.gyro[2] * 0.001f * 50.0f * 3.0f;
//	holder->Pitch.GYRO_AngleSpeed = -(INS_attitude->gyro[0] * 0.001f) * 150.0f * 57.32f;
//	holder->Yaw_S.GYRO_AngleSpeed = ((INS_attitude->gyro[2] - mpu6050.mpu6050_Data.gyro[2]) * 0.001f) * 150.0f * 50.0f * 2.0f * (-1.0f);
//	
//	// if(tim14.ClockTime%100==0 && holder->Yaw_Fllow_Mode.Flag_Fllow == 1)//大云台跟随
//	// {
//	// 	if(holder->Yaw_S.Target_Angle > (holder->left_litmit-10)) 
//	// 	{
//	// 		holder->Yaw_M.Target_Angle += 60.0f;
//	// 		holder->Yaw_Fllow_Mode.Flag_Fllow = 0;
//	// 	}			
//	// 	else if(holder->Yaw_S.Target_Angle < (holder->right_litmit+10))	
//	// 	{
//	// 		holder->Yaw_M.Target_Angle -= 60.0f;
//	// 		holder->Yaw_Fllow_Mode.Flag_Fllow = 0;
//	// 	}
//	// }
//	// if(holder->Yaw_Fllow_Mode.Flag_Fllow==0) holder->Yaw_Fllow_Mode.Lock_cnt++;
//	// if(holder->Yaw_Fllow_Mode.Lock_cnt>=500) {holder->Yaw_Fllow_Mode.Lock_cnt=0;holder->Yaw_Fllow_Mode.Flag_Fllow=1;}
//	//限幅
//	// holder->Yaw_S.Target_Angle = float_constrain(holder->Yaw_S.Target_Angle,holder->right_litmit,holder->left_litmit);	
//	// holder->Pitch.Target_Angle = float_constrain(holder->Pitch.Target_Angle,holder->down_litmit,holder->up_litmit);
//	
//	// holder->Motors.Yaw_M.Data.Output = BasePID_SpeedControl(holder->Yaw_M.PID.CorePID,
//		// BasePID_AngleControl(holder->Yaw_M.PID.ShellPID,holder->Yaw_M.Target_Angle,holder->Yaw_M.GYRO_Angle),holder->Yaw_M.GYRO_AngleSpeed);
//	
//	holder->Motors.Yaw_S.Data.Output = BasePID_SpeedControl(holder->Yaw_S.PID.CorePID,
//		BasePID_AngleControl(holder->Yaw_S.PID.ShellPID,holder->Yaw_S.Target_Angle,holder->Yaw_S.Can_Angle),holder->Yaw_S.GYRO_AngleSpeed);
//	
//	// holder->Motors.Pitch.Data.Output = BasePID_SpeedControl(holder->Pitch.PID.CorePID,
//		// BasePID_AngleControl(holder->Pitch.PID.ShellPID,holder->Pitch.Target_Angle,holder->Pitch.GYRO_Angle),holder->Pitch.GYRO_AngleSpeed);
//	
//	// MotorFillData(&holder->Motors.Yaw_M,holder->Motors.Yaw_M.Data.Output);
//	MotorFillData(&holder->Motors.Yaw_S,holder->Motors.Yaw_S.Data.Output);
//	// MotorFillData(&holder->Motors.Pitch,holder->Motors.Pitch.Data.Output);
//}


