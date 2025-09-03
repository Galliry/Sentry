#ifndef _ALL_CHASSIS_
#define _ALL_CHASSIS_
#include "stdint.h"
#include "pid.h"
#include "motor.h"
#define min(a, b) ((a) < (b) ? (a) : (b))
#define AtR 0.0174532f
typedef struct
{
	struct
	{
		float theoretical_power_sum;	//
		float target_require_power_sum; // 直接进行速度PID计算后得到的控制电流值之和，用以表示功率实际值
		float scaling_ratio;			// 缩放比例 = 功率理论值 / 功率实际值
		float initial_give_power[4];	//
		float scaled_give_power[4];
		struct
		{
			float torque_term_k1; // k1
			float speed_term_k2;  // k2
			float CONSTANT_COEFFICIENT;
			float TORQUE_COEFFICIENT;
		} coefficient; // 相关系数
		struct
		{
			uint8_t powermngmt_chassis_out; // 电管底盘接口的输出
			uint16_t energy_buffer;			// 缓冲能量
			uint16_t max_power;				// 功率上限
			float real_time_power;			// 实时功率
		} refereeData;
	} Power;
	struct
	{
		Motor motor[4];
		SinglePID_t RunPID[4];
		SinglePID_t FollowPID;
	} Motors;
	//< 控制底盘运动所需要的所有数据
	struct
	{
		float Vx;	  //< 前后运动的速度
		float Vy;	  //< 左右运动的速度
		float Vomega; //< 旋转的角速度
		float brain_vx;
		float brain_vy;
		float Vx_Sensitivity;
		float Vy_Sensitivity;
		float Vomega_Sensitivity;
		struct
		{
			uint8_t flag_up_down_slope;
			uint8_t flag_up_up_slope;
			uint8_t flag_up_stat_slope;
			uint8_t flag_down_stat_slope;
			uint8_t flag_down_up_slope;
			uint8_t flag_down_down_slope;
		} Slope_Flag;
		float angle_to_lidar;
	} Movement;
}AllChassis;

extern AllChassis allchassis;
void AllChassisInit(AllChassis* chassis,SinglePID_t* run_pid,SinglePID_t* follow_pid);
#endif
