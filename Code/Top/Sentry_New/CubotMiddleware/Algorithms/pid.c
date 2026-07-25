#include "pid.h"
#include "et08.h"

SinglePID_t pid_load_angle;
SinglePID_t pid_load_speed;
SinglePID_t pid_friction0;
SinglePID_t pid_friction1;
SinglePID_t pid_yaw_m_angle;
SinglePID_t pid_yaw_m_speed;
SinglePID_t pid_yaw_s_angle;
SinglePID_t pid_yaw_s_speed;
SinglePID_t pid_pitch_angle;
SinglePID_t pid_pitch_speed;
DualPID_Object pid_yaw_m = {.ShellPID = &pid_yaw_m_angle, .CorePID = &pid_yaw_m_speed};
DualPID_Object pid_yaw_s = {.ShellPID = &pid_yaw_s_angle, .CorePID = &pid_yaw_s_speed};
DualPID_Object pid_pitch = {.ShellPID = &pid_pitch_angle, .CorePID = &pid_pitch_speed};
SinglePID_t pid_run;
SinglePID_t pid_follow;
SinglePID_t pid_turn_speed[4];
SinglePID_t pid_turn_angle[4];
DualPID_Object pid_turn[4] = {
    {.ShellPID = &pid_turn_angle[0], .CorePID = &pid_turn_speed[0]}, // 电机0
    {.ShellPID = &pid_turn_angle[1], .CorePID = &pid_turn_speed[1]}, // 电机1
    {.ShellPID = &pid_turn_angle[2], .CorePID = &pid_turn_speed[2]}, // 电机2
    {.ShellPID = &pid_turn_angle[3], .CorePID = &pid_turn_speed[3]}, // 电机3
};

/**
  * @brief  限幅
  */
float AmplitudeLimit(float input,float amplitude)
{
 if(input<-amplitude)
  return -amplitude;
 else if(input>amplitude)
  return amplitude;
 else return input;
}

/**
 * @brief 单环PID初始化
 */
void BasePID_Init(SinglePID_t *base_pid, float kp, float ki, float kd, float detach, float LPF_frequency)
{
    base_pid->KiPartDetachment = detach;

    base_pid->dt = 0.001;
    base_pid->Kp = kp;
    base_pid->Ki = ki;
    base_pid->Kd = kd;

    base_pid->KpPart = 0;
    base_pid->KiPart = 0;
    base_pid->KdPart = 0;

    base_pid->LPF_Dpart.filter_coefficient = base_pid->dt/( (1.0f/2.0f*3.1415f*LPF_frequency) + base_pid->dt );
    base_pid->LPF_Dpart.last_output = 0;
}

/**
 * @brief 双环PID初始化
 */
void DualPID_Init(DualPID_Object *dual_pid, SinglePID_t *ShellPID, SinglePID_t *CorePID)
{
    dual_pid->ShellPID = ShellPID;
    dual_pid->CorePID = CorePID;
}

/**
 * @brief 单环比例积分速度控制
 */
float BasePID_SpeedControl(SinglePID_t *base_pid, float target, float feedback)
{
 base_pid->dt=0.001;
 base_pid->Error = target - feedback;

//比例项
 base_pid->KpPart = base_pid->Error * base_pid->Kp;

//带反计算抗积分饱和的积分项
 base_pid->KiPart += base_pid->Error * base_pid->Ki * base_pid->dt + base_pid->anti_windup;

//积分分离
 if(fabs(base_pid->Error) > base_pid->KiPartDetachment || fabs(base_pid->Error) < 0.05f || rc_Ctrl_et.isOnline == 0 )
  base_pid->KiPart = 0;

//测量值微分（避免微分冲击）
 base_pid->KdPart_raw = (feedback - base_pid->Last_Measure) / base_pid->dt;
 LPFilter(base_pid->KdPart_raw,&base_pid->LPF_Dpart);
 base_pid->KdPart = base_pid->Kd * base_pid->LPF_Dpart.output;
 base_pid->Last_Measure = feedback;

//总输出
 base_pid->Out_Raw = base_pid->KpPart + base_pid->KiPart + base_pid->KdPart;
 base_pid->Out = base_pid->KpPart + base_pid->KiPart + base_pid->KdPart;
 AmplitudeLimit(base_pid->Out,25000);

//反计算抗积分饱和
 if(base_pid->KiPart != 0.0f)
  base_pid->anti_windup = (base_pid->Out - base_pid->Out_Raw) * (1.0f/base_pid->KiPart) * 0.5f; 
 else
  base_pid->anti_windup = 0.0f;

 return base_pid->Out;
}

/**
 * @brief 单环比例积分角度控制
 */
float BasePID_AngleControl(SinglePID_t *base_pid, float target, float feedback)
{
 base_pid->dt=0.001;
 base_pid->Error = target - feedback;

//比例项
 base_pid->KpPart = base_pid->Error * base_pid->Kp;

//带反计算抗积分饱和的积分项
 base_pid->KiPart += base_pid->Error * base_pid->Ki * base_pid->dt + base_pid->anti_windup;

//积分分离
 if(fabs(base_pid->Error) > base_pid->KiPartDetachment || fabs(base_pid->Error) < 0.05f || rc_Ctrl_et.isOnline == 0)
  base_pid->KiPart = 0;

//测量值微分（避免微分冲击）
 base_pid->KdPart_raw = (feedback - base_pid->Last_Measure) / base_pid->dt;
 LPFilter(base_pid->KdPart_raw,&base_pid->LPF_Dpart);
 base_pid->KdPart = base_pid->Kd * base_pid->LPF_Dpart.output;
 base_pid->Last_Measure = feedback;

//总输出
 base_pid->Out_Raw = base_pid->KpPart + base_pid->KiPart + base_pid->KdPart;
 base_pid->Out = base_pid->KpPart + base_pid->KiPart + base_pid->KdPart;
 AmplitudeLimit(base_pid->Out,25000);

//反计算抗积分饱和
 if(base_pid->KiPart != 0.0f)
  base_pid->anti_windup = (base_pid->Out - base_pid->Out_Raw) * (1.0f/base_pid->KiPart) * 0.5f; 
 else
  base_pid->anti_windup = 0.0f;

 return base_pid->Out;
}

/**
 * @brief 单独给舵轮用的角度环
 */
float BasePID_AngleControl_Swerve(SinglePID_t *base_pid, float target, float feedback)
{
    base_pid->Error = target - feedback;
    if (base_pid->Error > 180)
    {
        base_pid->Error -= 360;
    }
    else if (base_pid->Error < -180)
    {
        base_pid->Error += 360;
    }
    base_pid->KpPart = base_pid->Error * base_pid->Kp;
    base_pid->KiPart += base_pid->Error * base_pid->Ki;

    if (fabs(base_pid->Error) > base_pid->KiPartDetachment || fabs(base_pid->Error) < 0.02f)
        base_pid->KiPart = 0;

    base_pid->KdPart = (-1) * base_pid->Kd * (base_pid->Error - base_pid->LastError);
    base_pid->Out = base_pid->KpPart + base_pid->KiPart + base_pid->KdPart;
    base_pid->LastError = base_pid->Error;
    return base_pid->Out;
}
void PID_Init(void)
{
    BasePID_Init(&pid_load_angle, 7, 0, 0, 0, 30.0f);         // 拨弹盘角度
    BasePID_Init(&pid_load_speed, 100,0,0,0,30.0f);// 拨弹盘速度
    BasePID_Init(&pid_friction0, 10, 1.5, 2, 0 ,30.0f); // 摩擦轮
    BasePID_Init(&pid_friction1, 10, 1.5, 2, 0 , 30.0f);
    BasePID_Init(&pid_pitch_angle, 0.34, 0.34, -0.3, 3.2, 30.0f); // 云台 0.5 0.005 -0.005 1.5
    BasePID_Init(&pid_pitch_speed, 0.35, 4, 0, 0.5 , 30.0f);
    BasePID_Init(&pid_yaw_s_angle, 0.4, 8, 0.3, 2 , 30.0f);//0.8 0.0045 -4 打符PID
    BasePID_Init(&pid_yaw_s_speed, 2800, 0, 1000, 2 , 30.0f); // 2200
    BasePID_Init(&pid_run, 20, 0, 0, 0, 30.0f);   // 底盘运动 20
    BasePID_Init(&pid_follow, 0, 0, 0, 0, 30.0f); // 底盘跟随
    for (int i = 0; i < 4; i++)
    {
        BasePID_Init(&pid_turn_angle[i], -35, -0.5, -20, 10, 30.0f); // 底盘舵向电机 -35 -0.5 -40 10
        BasePID_Init(&pid_turn_speed[i], 10, 0, 0, 0, 30.0f);        // 10 0 0 0
    }
}
