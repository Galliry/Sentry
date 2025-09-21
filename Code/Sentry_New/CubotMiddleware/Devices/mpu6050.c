#include "mpu6050.h"
#include "i2c.h"
#include "user_lib.h"
#include "kalman_filter.h"



MPU6050_t mpu6050={.mpu6050_Data.i2cHandler=&hi2c2};
float MPU6050_ACCEL_SEN = MPU6050_ACCEL_4G_SEN;
float MPU6050_GYRO_SEN  = MPU6050_GYRO_1000_SEN;
int flag_1=1;
int16_t mpu6050_cali_count = 0;

// 写数据到MPU6050寄存器
static uint8_t MPU6050_WriteByte(uint8_t reg_add, uint8_t reg_dat,I2C_HandleTypeDef *hi2c)
{
    return Sensors_I2C_WriteRegister(MPU6050_ADDRESS, reg_add, 1, &reg_dat,hi2c);
}

// 从MPU6050寄存器读取数据
static uint8_t MPU6050_ReadData(uint8_t reg_add, uint8_t *Read, uint8_t num,I2C_HandleTypeDef *hi2c)
{

    return Sensors_I2C_ReadRegister(MPU6050_ADDRESS, reg_add, num, Read,hi2c);
}
static uint8_t MPU6050_ReadData_1(uint8_t reg_add, uint8_t *Read, uint8_t num,I2C_HandleTypeDef *hi2c)
{
    return Sensors_I2C_ReadRegister_1(MPU6050_ADDRESS, reg_add, num, Read,hi2c);
}
static void MPU6050_Calibrate_Offset(IMU_InitData_t *mpu6050_data);
uint8_t TAddr = 0;
uint8_t MPU6050_Init(IMU_InitData_t *mpu6050_data)
{

    MPU6050_WriteByte(0x6B, 0x01,mpu6050_data->i2cHandler); // 休眠
    DWT_Delay_s(0.5);
    MPU6050_WriteByte(0x6B, 0x00,mpu6050_data->i2cHandler); // 解除休眠状态0x00
    DWT_Delay_s(0.05);
    MPU6050_ReadData(MPU6050_RA_WHO_AM_I, &TAddr, 1,mpu6050_data->i2cHandler);
    if (TAddr != 0x68)
        return 1;
    MPU6050_WriteByte(0x6B, 0x00,mpu6050_data->i2cHandler); // 解除休眠状态0x00
    DWT_Delay_s(0.05);
    MPU6050_WriteByte(0x19, 0x00,mpu6050_data->i2cHandler); // 采样频率（1KHz）
    DWT_Delay_s(0.05);
    MPU6050_WriteByte(0x1A, 0x03,mpu6050_data->i2cHandler); // 低通滤波
    DWT_Delay_s(0.05);
    MPU6050_WriteByte(0x1B, 0x10,mpu6050_data->i2cHandler); // 陀螺仪量程，当寄存器0x1B的值为0x10时，陀螺仪量程为
    DWT_Delay_s(0.05);
    MPU6050_WriteByte(0x1C, 0x09,mpu6050_data->i2cHandler); // 加速度量程，当寄存器0x1C的值为0x09时，加速度量程为
    DWT_Delay_s(0.05);

	MPU6050_Calibrate_Offset(mpu6050_data);
	
    return 0;
}


// 防止零漂
void MPU6050_Calibrate_Offset(IMU_InitData_t *mpu6050_data)
{
    float startTime;
    uint16_t CaliTimes = 1000;
    float gyroDiff[3], gNormDiff;

	uint8_t accBuf[6]  = {0};
    uint8_t gyroBuf[6] = {0};
    int16_t mpu6050_raw_temp;
    float gyroMax[3], gyroMin[3];
    float gNormTemp = 0.0f, gNormMax = 0.0f, gNormMin = 0.0f;

    startTime = DWT_GetTimeline_s();
    do {
		if (DWT_GetTimeline_s() - startTime > 12) {
            // 在线校准超时，使用预置值校准
            mpu6050_data->gyro_offset[0] = MPU6050_GxOFFSET;
            mpu6050_data->gyro_offset[1] = MPU6050_GyOFFSET;
            mpu6050_data->gyro_offset[2] = MPU6050_GzOFFSET;
            mpu6050_data->g_norm         = MPU6050_gNORM;
            break;
        }

        DWT_Delay_s(0.005);
        mpu6050_data->g_norm         = 0;
        mpu6050_data->gyro_offset[0] = 0;
        mpu6050_data->gyro_offset[1] = 0;
        mpu6050_data->gyro_offset[2] = 0;

        for (uint16_t i = 0; i < CaliTimes; ++i) {
            MPU6050_ReadData(MPU6050_ACC_OUT, accBuf, 6,mpu6050_data->i2cHandler);
            mpu6050_raw_temp       = (int16_t)(accBuf[0] << 8) | accBuf[1];
            mpu6050_data->accel[0] = mpu6050_raw_temp * MPU6050_ACCEL_SEN;
            mpu6050_raw_temp       = (int16_t)(accBuf[2] << 8) | accBuf[3];
            mpu6050_data->accel[1] = mpu6050_raw_temp * MPU6050_ACCEL_SEN;
            mpu6050_raw_temp       = (int16_t)(accBuf[4] << 8) | accBuf[5];
            mpu6050_data->accel[2] = mpu6050_raw_temp * MPU6050_ACCEL_SEN;

            gNormTemp = sqrtf(mpu6050_data->accel[0] * mpu6050_data->accel[0] +
                              mpu6050_data->accel[1] * mpu6050_data->accel[1] +
                              mpu6050_data->accel[2] * mpu6050_data->accel[2]);
            mpu6050_data->g_norm += gNormTemp;

            MPU6050_ReadData(MPU6050_GYRO_OUT, gyroBuf, 6,mpu6050_data->i2cHandler);
            mpu6050_raw_temp      = (int16_t)(gyroBuf[0] << 8) | gyroBuf[1];
            mpu6050_data->gyro[0] = mpu6050_raw_temp * MPU6050_GYRO_SEN;
            mpu6050_data->gyro_offset[0] += mpu6050_data->gyro[0];
            mpu6050_raw_temp      = (int16_t)(gyroBuf[2] << 8) | gyroBuf[3];
            mpu6050_data->gyro[1] = mpu6050_raw_temp * MPU6050_GYRO_SEN;
            mpu6050_data->gyro_offset[1] += mpu6050_data->gyro[1];
            mpu6050_raw_temp      = (int16_t)(gyroBuf[4] << 8) | gyroBuf[5];
            mpu6050_data->gyro[2] = mpu6050_raw_temp * MPU6050_GYRO_SEN;
            mpu6050_data->gyro_offset[2] += mpu6050_data->gyro[2];

            DWT_Delay_s(0.0005);
        }

        mpu6050_data->g_norm /= (float)CaliTimes;
        for (uint8_t i = 0; i < 3; ++i)
            mpu6050_data->gyro_offset[i] /= (float)CaliTimes;

        mpu6050_cali_count++;

    } while (
             fabsf(mpu6050_data->g_norm - 9.8f) > 6.0f ||
             fabsf(mpu6050_data->gyro_offset[0]) > 1.0f ||
       
             fabsf(mpu6050_data->gyro_offset[2]) > 0.08f);
    // 若出不了while，说明校准条件恶劣，超时则使用预置值校准
    mpu6050_data->accel_scale = 9.81f / mpu6050_data->g_norm;
}


float q0_1 = 1.0f, q1_1 = 0.0f, q2_1 = 0.0f, q3_1 = 0.0f;
int8_t YawRoundCount;
void IMUupdate(IMU_InitData_t *mpu6050_data)
{
	MPU6050_t* mpu6050  =(MPU6050_t*)mpu6050_data ;
   
	float  halfT,norm,vx,vy,vz,ex,ey,ez,tempq0,tempq1,tempq2,tempq3,now,dt;
	static uint32_t DWT_Count = 0;
	static float YawAngleLast;
	dt =DWT_GetDeltaT(&DWT_Count); 
	//mpu6050_data->gyro[2]*=1.58;
    tempq0 = q0_1 + ( - q3_1*mpu6050_data->gyro[2])*dt*0.5f;
    tempq1 = q1_1 + ( q2_1*mpu6050_data->gyro[2] )*dt*0.5f;
    tempq2 = q2_1 + ( - q1_1*mpu6050_data->gyro[2] )*dt*0.5f;
    tempq3 = q3_1 + (q0_1*mpu6050_data->gyro[2] )*dt*0.5f;

    // 四元数规范化
    norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
    q0_1= tempq0 * norm;
    q1_1 = tempq1 * norm;
    q2_1 = tempq2 * norm;
    q3_1= tempq3 * norm;

    

	mpu6050->Yaw=-atan2(2 * q1_1 * q2_1+ 2 * q0_1* q3_1, -2 * q2_1*q2_1- 2 * q3_1 * q3_1 + 1)*RtA; // yaw        -pi----pi
    mpu6050->Roll= -asin(-2 * q1_1 * q3_1+ 2 * q0_1 * q2_1)*RtA; // pitch    -pi/2    --- pi/2 
//		//>pitch轴角度
    mpu6050->Pitch= -atan2(2 * q2_1* q3_1 + 2 * q0_1* q1_1, -2 * q1_1 * q1_1 - 2 * q2_1* q2_1+ 1)* RtA; 
	if (mpu6050->Yaw -YawAngleLast > 180.0f)
    {
        YawRoundCount--;
    }
    else if (mpu6050->Yaw -YawAngleLast < -180.0f)
    {
        YawRoundCount++;
    }
	mpu6050->Yaw_total_angle = 360.0f * YawRoundCount + mpu6050->Yaw;
	YawAngleLast = mpu6050->Yaw;
		// roll       -pi-----pi        -pi-----pi 
}	

/**
 * @brief   读取陀螺仪数据
 */
void MPU6050_Read(IMU_InitData_t *mpu6050_data)
{
	static uint8_t Buf[14];
	static int16_t mpu6050_raw_temp;
	MPU6050_t* mpu6050  =(MPU6050_t*)mpu6050_data ;

	MPU6050_ReadData(MPU6050_GYRO_OUT+4, Buf, 2,mpu6050_data->i2cHandler);

	mpu6050_data->gyro[0] = 0;


	mpu6050_data->gyro[1] = 0;
	mpu6050_raw_temp      = (int16_t)(Buf[0] << 8) | Buf[1];
	mpu6050_data->Raw_gyro[2]=mpu6050_raw_temp ;

	mpu6050_data->gyro[2] = mpu6050_raw_temp * MPU6050_GYRO_SEN - mpu6050_data->gyro_offset[2];

	if (fabs(mpu6050_data->gyro[2]/MPU6050_GYRO_SEN)<30.0) mpu6050_data->gyro[2]=0;
}
