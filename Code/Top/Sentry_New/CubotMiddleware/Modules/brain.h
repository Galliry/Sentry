#ifndef BRAIN_H__
#define BRAIN_H__
#include "stm32h7xx.h"
#include "driver_usart.h"
#include "check.h"
#include "ins.h"
#include "driver_timer.h"
#include "referee.h"
#include "holder.h"
#include "user_lib.h"
#include "communication.h"

#define Brain_rxBufferLengh 50
#define Lidar_rxBufferLengh 50 

typedef enum
{
	BRAIN_TO_ROBOT_CMD   = 1,	 //< 0b0001
	BRAIN_TO_ROBOT_HINT  = 2,	 //< 0b0010
	ROBOT_TO_BRAIN_QUEST = 3,  //< 0b0011
	ROBOT_TO_BRAIN_LOG   = 4,	 //< 0b0100
	ROBOT_TO_BRAIN_CMD   = 5
}BrainFrameType;

typedef enum {
    Single_IDLE = 0,   
    Single_Arrive,	
    Single_Find,    
    Single_Exit,    
}Singe_state;

typedef enum
{
	Autoaim = 0,             
	Outpost = 1,
	Single = 2
}
Brain_mode;		//三种模式，切换相关一部分在回调函数中，一部分在contro_logic中

typedef enum
{
	Cruise = 0,             
	Lock = 1,
	Change = 2
}Brain_Autoaim_mode; 

typedef enum
{
	Lidar_Arrive = 0,
	Lidar_OnlyMove = 1,
	Lidar_RevolveMove = 2,
}Brain_Lidar_mode;

typedef enum
{
	None=0,
	Found=1,
	Wait=2
}Brain_All_See_mode;

typedef struct
{ 
	uint8_t FrameType;
	uint8_t FrameCoreID;
}CubotBrain_t;

typedef struct
{
	
	struct{
		Brain_Autoaim_mode mode; 	//下位机模式
		uint16_t mode_cnt;
		CubotBrain_t Brain_Data;
		Brain_mode  Mode; 	//上位机模式
		float Pitch_add;
		float Yaw_add;
		float Distance;
		uint8_t IsFire;
		uint8_t Rune_Flag;
	}Autoaim;
	 
	struct{
		Brain_Lidar_mode mode;
		CubotBrain_t Brain_Data;
		uint8_t	movemode;
		float vx;
        float vy;
		uint8_t Posture;
		float MyPosition_x;
		float MyPosition_y;
	}Lidar;
	 
	struct
	{
		Brain_All_See_mode mode;
		uint16_t mode_cnt[3];
		uint8_t Find_size;
		uint8_t Camera_Index[10];
		uint8_t armorNumber[10];
		float Pitch_add[10];
		float Yaw_add[10];
		float Distance[10];//mm
	}All_See;
	
}Brain_t;
  
extern Brain_t Brain;

uint8_t Brain_Autoaim_Callback(uint8_t * recBuffer, uint16_t len);
uint8_t Brain_Lidar_Callback(uint8_t * recBuffer, uint16_t len);

void Brain_Autoaim_DataUnpack(Brain_t* brain ,uint8_t * recBuffer);//解包自瞄数据
void Brain_Lidar_DataUnpack(Brain_t* brain ,uint8_t * recBuffer);//解包雷达数据

void RobotToBrain(Brain_t* brain);
void RobotToBrain_Autoaim(float yaw,Brain_t* brain);//发给自瞄	
void RobotToBrain_Lidar(Brain_t* brain);//发给雷达

void Change_BrainMode(Brain_t* brain);
#endif


