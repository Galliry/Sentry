#ifndef __MPU6050__H
#define __MPU6050__H
#include "stm32h7xx_hal.h"
#include "driver_counter.h"
#include "ins.h"
#include "driver_iic.h"
// 需手动校准，修改数据
#define MPU6050_GxOFFSET       -0.0100490293f
#define MPU6050_GyOFFSET       -0.00745377829f
#define MPU6050_GzOFFSET       0.0170729998
#define MPU6050_gNORM          9.27340317f

#define MPU6050_ADDRESS       0xD0 // 从机地址0x68左移一位得到
#define MPU6050_RA_WHO_AM_I   0x75
#define MPU6050_ACC_OUT       0x3B // MPU6050加速度数据寄存器地址
#define MPU6050_GYRO_OUT      0x43 // MPU6050陀螺仪数据寄存器地址

#define MPU6050_ACCEL_2G_SEN  0.0005975341796875f
#define MPU6050_ACCEL_4G_SEN  0.0011958740234375f
#define MPU6050_ACCEL_8G_SEN  0.002391748046875f
#define MPU6050_ACCEL_16G_SEN 0.00478349609375f

#define MPU6050_GYRO_2000_SEN 0.00106526443603169529841533860381f
#define MPU6050_GYRO_1000_SEN 0.00053263221801584764920766930190693f //转弧度/s
#define MPU6050_GYRO_500_SEN  0.00026631610900792382460383465095346f
#define MPU6050_GYRO_250_SEN  0.00013315805450396191230191732547673f
#define RtA 57.324841f
typedef struct
{
    IMU_InitData_t mpu6050_Data;
    uint8_t WriteState;
		float Yaw;
		float Pitch;
		float Roll;
		float Yaw_total_angle;	  
} MPU6050_t;

uint8_t MPU6050_Init(IMU_InitData_t *mpu6050_data);
void MPU6050_Read(IMU_InitData_t *mpu6050_data);
void IMUupdate(IMU_InitData_t *mpu6050_data);
extern MPU6050_t mpu6050;

#endif

