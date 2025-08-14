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
		float target_require_power_sum; // ֱ�ӽ����ٶ�PID�����õ��Ŀ��Ƶ���ֵ֮�ͣ����Ա�ʾ����ʵ��ֵ
		float scaling_ratio;			// ���ű��� = ��������ֵ / ����ʵ��ֵ
		float initial_give_power[4];	//
		float scaled_give_power[4];
		struct
		{
			float torque_term_k1; // k1
			float speed_term_k2;  // k2
			float CONSTANT_COEFFICIENT;
			float TORQUE_COEFFICIENT;
		} coefficient; // ���ϵ��
		struct
		{
			uint8_t powermngmt_chassis_out; // ��ܵ��̽ӿڵ����
			uint16_t energy_buffer;			// ��������
			uint16_t max_power;				// ��������
			float real_time_power;			// ʵʱ����
		} refereeData;
	} Power;
	struct
	{
		Motor motor[4];
		SinglePID_t RunPID[4];
		SinglePID_t FollowPID;
	} Motors;
	//< ���Ƶ����˶�����Ҫ����������
	struct
	{
		float Vx;	  //< ǰ���˶����ٶ�
		float Vy;	  //< �����˶����ٶ�
		float Vomega; //< ��ת�Ľ��ٶ�
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
