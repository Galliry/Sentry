/**
 ******************************************************************************
 * @file    dmimu.h
 * @brief   DM-IMU-L1 六轴IMU模块驱动 (裸机版)
 *
 * @note    使用CAN主动模式: IMU上电后, MCU发一次 change_to_active,
 *          IMU即在StdId=mst_id上自动推送4种数据帧:
 *            [01 Temp   AccX  AccY  AccZ ]  加速度 + 温度
 *            [02 0x00 GyroX GyroY GyroZ]  角速度
 *            [03 0x00 Pitch  Yaw  Roll ]  欧拉角
 *            [04 <四元数14bit×4紧凑编码>]  四元数
 *
 *          CAN ID默认使用制造商出厂值: can_id=0x58, mst_id=0x59
 *
 *          通信协议参考: DM-IMU-L1使用说明书V1.2
 ******************************************************************************
 */

#ifndef __DMIMU_H__
#define __DMIMU_H__

#include "driver_can.h"

/* ========================== CAN 数据范围常量 ========================== */

#define DM_IMU_ACCEL_CAN_MAX   (235.2f)
#define DM_IMU_ACCEL_CAN_MIN   (-235.2f)
#define DM_IMU_GYRO_CAN_MAX    (34.88f)
#define DM_IMU_GYRO_CAN_MIN    (-34.88f)
#define DM_IMU_PITCH_CAN_MAX   (90.0f)
#define DM_IMU_PITCH_CAN_MIN   (-90.0f)
#define DM_IMU_ROLL_CAN_MAX    (180.0f)
#define DM_IMU_ROLL_CAN_MIN    (-180.0f)
#define DM_IMU_YAW_CAN_MAX     (180.0f)
#define DM_IMU_YAW_CAN_MIN     (-180.0f)
#define DM_IMU_QUAT_CAN_MAX    (1.0f)
#define DM_IMU_QUAT_CAN_MIN    (-1.0f)

/* ========================== 协议常量 ========================== */

#define DM_IMU_CAN_CMD_CC      0xCCu   /* CAN命令帧头 */
#define DM_IMU_CAN_READ        0x00u   /* 读寄存器 */
#define DM_IMU_CAN_WRITE       0x01u   /* 写寄存器 */

/* 数据帧类型 (Data[0]) */
#define DM_IMU_FRAME_ACCEL      0x01u
#define DM_IMU_FRAME_GYRO       0x02u
#define DM_IMU_FRAME_EULER      0x03u
#define DM_IMU_FRAME_QUATERNION 0x04u

/* 寄存器地址 */
#define DM_IMU_REG_ACCEL        0x01u
#define DM_IMU_REG_GYRO         0x02u
#define DM_IMU_REG_EULER        0x03u
#define DM_IMU_REG_QUATERNION   0x04u
#define DM_IMU_REG_CHANGE_ACTIVE 0x0Bu

/* ========================== 结构体 ========================== */

typedef struct
{
    /* --- 传感器数据 (由 CAN Rx ISR 写入) --- */
    float    accel[3];          /* 加速度: X, Y, Z (m/s^2) */
    float    gyro[3];           /* 角速度: X, Y, Z (rad/s) */
    float    euler[3];          /* 欧拉角: Pitch, Yaw, Roll (度) */
    float    quaternion[4];     /* 四元数: w, x, y, z */
    float    temperature;       /* 温度 (摄氏度) */

    /* --- 在线检测 --- */
    uint32_t last_update_tick;  /* 最后一次收到数据的时间戳 (HAL_GetTick) */
    uint32_t offline_threshold; /* 掉线判定阈值 (ms), 默认 500 */
    uint8_t  online;            /* 在线标志: 0=掉线, 1=在线 */

    /* --- 参数 --- */
    uint16_t can_id;            /* MCU→IMU 的标准帧ID (默认 0x58) */
    uint16_t mst_id;            /* IMU→MCU 的标准帧ID (默认 0x59) */

    /* --- 初始化状态机 --- */
    uint32_t init_delay_start;  /* 延迟起始 tick */
    uint8_t  init_done;         /* 初始化完成标志 */
} DM_IMU_t;

/* ========================== 全局实例 ========================== */

extern DM_IMU_t dm_imu;

/* ========================== API 声明 ========================== */

/**
 * @brief  初始化 DM-IMU 结构体
 * @param  imu     IMU 实例指针
 * @param  can_id  MCU→IMU 的 CAN StdId (建议 0x58)
 * @param  mst_id  IMU→MCU 的 CAN StdId (建议 0x59)
 */
void DM_IMU_Init(DM_IMU_t *imu, uint16_t can_id, uint16_t mst_id);

/**
 * @brief  尝试发送 active 模式命令 (一次性, 带延迟)
 * @note   在 TIM14_Task 中每 1ms 调用, 内部自动延迟 ~500ms 后发送一次
 * @param  can  CAN 总线对象
 * @param  imu  IMU 实例指针
 */
void DM_IMU_TrySendActive(CAN_Object *can, DM_IMU_t *imu);

/**
 * @brief  CAN 接收回调 (ISR 上下文)
 * @note   在 CAN2_rxCallBack 中调用
 * @param  imu     IMU 实例指针
 * @param  rx_buf  CAN 接收缓冲区
 */
void DM_IMU_CAN_Callback(DM_IMU_t *imu, CAN_RxBuffer *rx_buf);

/**
 * @brief  检测 IMU 是否在线
 * @note   在 TIM14_Task 中周期性调用
 * @param  imu  IMU 实例指针
 */
void DM_IMU_CheckOnline(DM_IMU_t *imu);

#endif /* __DMIMU_H__ */
