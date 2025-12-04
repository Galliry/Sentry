#include "check.h"
#include "et08.h"
#include "user_lib.h"
#include "dr16.h"

Check_Robot_State check_robot_state={
	.Check_Usart.Check_receiver_cnt=300
};

FPS tim14_FPS={
.Receiver_cnt= 0,
.Referee_cnt = 0,
.Vision_cnt = 0,
.Gyro_FPS= 0,
.Lidar_cnt=0
};

void RobotOnlineState(Check_Robot_State* CheckRobotState,RC_Ctrl_ET* rc_ctrl,RC_Ctrl* rc_ctrl_dr16)
{
	CheckRobotState->Check_Usart.Check_receiver_cnt++;
	CheckRobotState->Check_Usart.Check_referee_cnt++;
	rc_ctrl_dr16->online_cnt++;
	
	CheckRobotState->Check_Usart.Check_referee_cnt= int16_constrain(CheckRobotState->Check_Usart.Check_referee_cnt,0,200);
	CheckRobotState->Check_Usart.Check_referee = (CheckRobotState->Check_Usart.Check_referee_cnt > 100) ? 0 : 1;
	
	CheckRobotState->Check_Usart.Check_receiver_cnt=int16_constrain(CheckRobotState->Check_Usart.Check_receiver_cnt,0,200);
	if(CheckRobotState->Check_Usart.Check_receiver_cnt > 100){rc_ctrl->isOnline=0;CheckRobotState->Check_Usart.Check_receiver=0;}else{if(rc_ctrl->rc.sA==1&&rc_ctrl->rc.ch2!=1)rc_ctrl->isOnline=1;else rc_ctrl->isOnline=0;CheckRobotState->Check_Usart.Check_receiver=1;}
	if(rc_ctrl_dr16->online_cnt < 30){rc_ctrl_dr16->is_online = 1;}else {rc_ctrl_dr16->is_online = 0;}
}

/*¸÷Ä£¿éÖ¡ÂÊ¼ì²â*/
void FPS_Check(FPS * fps)
{
	fps->Receiver_FPS=fps->Receiver_cnt;
	fps->Referee_FPS=fps->Referee_cnt;
	fps->Vision_FPS=fps->Vision_cnt;
	fps->Lidar_FPS= fps->Lidar_cnt;
	fps->Gyro_FPS=fps->Gyro_cnt;
	fps->Camera_FPS=fps->Camera_cnt;
	fps->Lidar_cnt=0;
	fps->Receiver_cnt=0;
	fps->Referee_cnt=0;
	fps->Vision_cnt=0;
	fps->Gyro_cnt= 0;
	fps->Camera_cnt=0;
//	Motor_CheckFPS();
}







