#ifndef _BMI088_H_ // 防止重复包含
#define _BMI088_H_

#include "ins.h"
#include "driver_spi.h"
#include "driver_counter.h"
#include "bmi088_regNdef.h"

// 需手动校准，修改数据
#define BMI088_GxOFFSET (-0.00418030564f)
#define BMI088_GyOFFSET (-0.0052329856f)
#define BMI088_GzOFFSET (-0.000287621311f)
#define BMI088_gNORM    10.2522392f

#define BMI088_USE_SPI

typedef struct {
    SPI_Slave_t bmi088Accel;
    SPI_Slave_t bmi088Gyro;
    IMU_InitData_t bmi088_Data;
} BMI088_t;

/**
 * @brief 初始化BMI088,传入连接的SPI总线handle,以及是否进行在线标定
 *
 * @param bmi088_SPI handle
 * @param calibrate  1为进行在线标定,0使用离线数据
 * @return uint8_t   成功则返回BMI088_NO_ERROR
 */
extern uint8_t BMI088_Init(struct _IMU_InitData_t * imu_initdata_t);

/**
 * @brief 加速计初始化
 *
 * @return uint8_t
 */
extern uint8_t BMI088_AccInit(void);

/**
 * @brief 陀螺仪初始化
 *
 * @return uint8_t
 */
extern uint8_t BMI088_GyroInit(void);

/**
 * @brief 读取一次BMI088的数据,包括gyro和accel
 *
 * @param bmi088_data 传入BMI088实例(结构体)
 */
extern void BMI088_Read(IMU_InitData_t *bmi088_data);

extern BMI088_t bmi088;
void BMI088_IMUupdate(IMU_InitData_t *mpu6050_data);
void Bmi088_InitQuaternion();
#if defined(BMI088_USE_SPI)

#elif defined(BMI088_USE_IIC)

#endif

#endif
