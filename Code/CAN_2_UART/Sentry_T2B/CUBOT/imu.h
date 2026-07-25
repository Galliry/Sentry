#ifndef __IMU_H__
#define __IMU_H__

#include "main.h"

// 暴露给外部调用的接口
void IMU_Bridge_Init(void);
void IMU_Bridge_Process(void);
void User_CAN_Test_Tx(void);
void IMU_Startup_Sequence(void);
#endif /* __IMU_H__ */
