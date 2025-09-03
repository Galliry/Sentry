#include "holder.h"
#include "et08.h"
#include "ins.h"
#include "mpu6050.h"
#include "driver_timer.h"
#include "user_lib.h"

Holder_t Holder;

/**
  * @brief 云台初始化
  */
void HolderInit(Holder_t* holder,DualPID_Object* pitch,DualPID_Object* yaw_m,DualPID_Object* yaw_s)
{
	MotorInit(&holder->Motors6020.Pitch,6993,Motor6020,CAN1,0x206);
	MotorInit(&holder->Motors6020.Yaw_S,5924,Motor6020,CAN1,0x205);
	MotorInit(&holder->Motors6020.Yaw_M,5645,Motor6020,CAN2,0x205);

	DualPID_Init(&holder->Pitch.PID,pitch->ShellPID,pitch->CorePID);
	DualPID_Init(&holder->Yaw_M.PID,yaw_m->ShellPID,yaw_m->CorePID);
	DualPID_Init(&holder->Yaw_S.PID,yaw_s->ShellPID,yaw_s->CorePID);
	
	holder->Pitch.Sensitivity = 0.00085f;//-0.0015f
	holder->Yaw_M.Sensitivity = 0.0010f;//0.003f 0.0015
	holder->Yaw_S.Sensitivity = 0.0010f;
	holder->Cruise_Mode.Yaw_S_sense=0.00325;
	holder->Cruise_Mode.Pitch_sense=0.0065;
	holder->up_litmit=29;
	holder->down_litmit=-35;//-35
	holder->right_litmit=-85;
	holder->left_litmit=75;
}

void Holder_Control(Holder_t* holder,RC_Ctrl_ET* rc_ctrl)
{
	holder->Pitch.Target_Angle += ((rc_ctrl->rc.ch3 -1024) * holder->Pitch.Sensitivity);
	if(rc_ctrl->rc.s2 != 1) holder->Yaw_M.Target_Angle += ((rc_ctrl->rc.ch2 - 1024) * holder->Yaw_M.Sensitivity);
	else if(rc_ctrl->rc.s2 == 1) holder->Yaw_S.Target_Angle += ((rc_ctrl->rc.ch2 - 1024) * holder->Yaw_S.Sensitivity);
	
	if (holder->Yaw_S.Can_Angle > 61) holder->down_litmit = -19;
	else holder->down_litmit = -35;
	
	holder->Pitch.Can_Angle = holder->Motors6020.Pitch.Data.Angle;
	holder->Yaw_M.Can_Angle = holder->Motors6020.Yaw_M.Data.Angle;
	holder->Yaw_S.Can_Angle = holder->Motors6020.Yaw_S.Data.Angle;
	holder->Pitch.Can_AngleSpeed = holder->Motors6020.Pitch.Data.SpeedRPM;
	holder->Yaw_M.Can_AngleSpeed = holder->Motors6020.Yaw_M.Data.SpeedRPM/2;//减速比1：2
	holder->Yaw_S.Can_AngleSpeed = holder->Motors6020.Yaw_S.Data.SpeedRPM;
	
	holder->Pitch.GYRO_Angle = -(INS_attitude->pitch);
	holder->Yaw_M.GYRO_Angle = mpu6050.Yaw_total_angle;
	holder->Yaw_S.GYRO_Angle = INS_attitude->yaw;
	holder->Yaw_M.GYRO_AngleSpeed = -150 * mpu6050.mpu6050_Data.gyro[2] * 0.001 * 50 * 3;
	holder->Pitch.GYRO_AngleSpeed = -(INS_attitude->gyro[0] * 0.001) * 150 * 57.32;
	holder->Yaw_S.GYRO_AngleSpeed = ((INS_attitude->gyro[2] - mpu6050.mpu6050_Data.gyro[2]) * 0.001) * 150 * 50 * 2 * (-1);
	
	if(tim14.ClockTime%100==0 && holder->Yaw_Fllow_Mode.Flag_Fllow == 1)//大云台跟随
	{
		if(holder->Yaw_S.Target_Angle > (holder->left_litmit-10)) 
		{
			holder->Yaw_M.Target_Angle += 60.0f;
			holder->Yaw_Fllow_Mode.Flag_Fllow = 0;
		}			
		else if(holder->Yaw_S.Target_Angle < (holder->right_litmit+10))	
		{
			holder->Yaw_M.Target_Angle -= 60.0f;
			holder->Yaw_Fllow_Mode.Flag_Fllow = 0;
		}
	}
	if(holder->Yaw_Fllow_Mode.Flag_Fllow==0) holder->Yaw_Fllow_Mode.Lock_cnt++;
	if(holder->Yaw_Fllow_Mode.Lock_cnt>=500) {holder->Yaw_Fllow_Mode.Lock_cnt=0;holder->Yaw_Fllow_Mode.Flag_Fllow=1;}
	//限幅
	holder->Yaw_S.Target_Angle = float_constrain(holder->Yaw_S.Target_Angle,holder->right_litmit,holder->left_litmit);	
	holder->Pitch.Target_Angle = float_constrain(holder->Pitch.Target_Angle,holder->down_litmit,holder->up_litmit);
	
	holder->Motors6020.Yaw_M.Data.Output = BasePID_SpeedControl(holder->Yaw_M.PID.CorePID,
		BasePID_AngleControl(holder->Yaw_M.PID.ShellPID,holder->Yaw_M.Target_Angle,holder->Yaw_M.GYRO_Angle),holder->Yaw_M.GYRO_AngleSpeed);
	
	holder->Motors6020.Yaw_S.Data.Output = BasePID_SpeedControl(holder->Yaw_S.PID.CorePID,
		BasePID_AngleControl(holder->Yaw_S.PID.ShellPID,holder->Yaw_S.Target_Angle,holder->Yaw_S.Can_Angle),holder->Yaw_S.GYRO_AngleSpeed);
	
	holder->Motors6020.Pitch.Data.Output = BasePID_SpeedControl(holder->Pitch.PID.CorePID,
		BasePID_AngleControl(holder->Pitch.PID.ShellPID,holder->Pitch.Target_Angle,holder->Pitch.GYRO_Angle),holder->Pitch.GYRO_AngleSpeed);
	
	MotorFillData(&holder->Motors6020.Yaw_M,holder->Motors6020.Yaw_M.Data.Output);
	MotorFillData(&holder->Motors6020.Yaw_S,holder->Motors6020.Yaw_S.Data.Output);
	MotorFillData(&holder->Motors6020.Pitch,holder->Motors6020.Pitch.Data.Output);
}















