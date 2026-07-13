/**
 ******************************************************************************
 * @file    dmimu.c
 * @brief   DM-IMU-L1 六轴IMU模块驱动实现 (裸机版)
 *
 * @note    使用CAN主动模式:
 *          - 上电延迟 ~500ms 后发一次 change_to_active 命令
 *          - IMU 收到后在 mst_id 上自动推送加速度/角速度/欧拉角/四元数
 *          - CAN Rx 在 ISR 上下文中解析, 消费者在 TIM14_Task 中读取
 *
 *          参考: D:\Code\RoboMaster\CMake\Base-By-DM_MC02 (已验证通信)
 *          协议: DM-IMU-L1使用说明书V1.2
 ******************************************************************************
 */

#include "dmimu.h"
#include "user_lib.h"
#include "driver_can.h"
#include <string.h>

/* ========================== 全局实例 ========================== */

DM_IMU_t dm_imu = {0};

/* ========================== 延迟时间定义 ========================== */

#define DM_IMU_INIT_DELAY_MS  500u   /* 上电后等待 IMU 启动的时间 */

/* ========================== 初始化 ========================== */

void DM_IMU_Init(DM_IMU_t *imu, uint16_t can_id, uint16_t mst_id)
{
    if (imu == NULL) return;

    memset(imu, 0, sizeof(DM_IMU_t));

    imu->can_id            = can_id;
    imu->mst_id            = mst_id;
    imu->offline_threshold = 500;
    imu->init_delay_start   = 0;  /* 第一次 TrySendActive 时设置 */
    imu->init_done         = 0;
}

/* ========================== CAN 发送主动模式命令 ========================== */

/**
 * @brief  向 IMU 发送 change_to_active 命令
 *
 *  帧格式 (StdId=can_id, DLC=8):
 *    [0xCC] [0x0B] [0x01] [0xDD] [0x01 0x00 0x00 0x00]
 */
static void DM_IMU_SendActiveCmd(CAN_Object *can, DM_IMU_t *imu)
{
    CAN_TxBuffer tx;
    memset(&tx, 0, sizeof(tx));

    tx.Identifier  = imu->can_id;
    tx.Data[0]     = DM_IMU_CAN_CMD_CC;
    tx.Data[1]     = DM_IMU_REG_CHANGE_ACTIVE;
    tx.Data[2]     = DM_IMU_CAN_WRITE;
    tx.Data[3]     = 0xDD;
    tx.Data[4]     = 0x01;  /* 1 = 主动模式 */

    CAN_Send(can, &tx);
}

/**
 * @brief  尝试发送 active 模式命令 (一次性, 带延迟)
 *
 *  调用时机: 在 TIM14_Task (1kHz) 中每毫秒调用
 *  行为:
 *    - 首次调用记录当前 tick, 不做任何事
 *    - 等待 ~500ms (DM_IMU_INIT_DELAY_MS) 后发送一次 change_to_active
 *    - 发送后设置 init_done=1, 后续调用立即返回
 */
void DM_IMU_TrySendActive(CAN_Object *can, DM_IMU_t *imu)
{
    if (can == NULL || imu == NULL) return;
    if (imu->init_done) return;  /* 已发送过, 直接返回 */

    uint32_t now = HAL_GetTick();

    /* 首次调用: 记录起始时间 */
    if (imu->init_delay_start == 0)
    {
        imu->init_delay_start = now;
        return;
    }

    /* 等待延迟时间到 */
    if ((now - imu->init_delay_start) < DM_IMU_INIT_DELAY_MS)
        return;

    /* 发送一次 change_to_active, 然后标记完成 */
    DM_IMU_SendActiveCmd(can, imu);
    imu->init_done = 1;
}

/* ========================== CAN 数据帧解析 ========================== */

/**
 * @brief  解析 CAN 加速度数据帧
 *  帧格式: [01] [Temp] [AccX_L AccX_H AccY_L AccY_H AccZ_L AccZ_H]
 */
static void DM_IMU_ParseAccel(DM_IMU_t *imu, uint8_t *data)
{
    int16_t raw[3];

    raw[0] = (int16_t)(data[2] | (data[3] << 8));
    raw[1] = (int16_t)(data[4] | (data[5] << 8));
    raw[2] = (int16_t)(data[6] | (data[7] << 8));

    imu->temperature = (float)data[1];
    imu->accel[0]    = uint_to_float(raw[0], DM_IMU_ACCEL_CAN_MIN, DM_IMU_ACCEL_CAN_MAX, 16);
    imu->accel[1]    = uint_to_float(raw[1], DM_IMU_ACCEL_CAN_MIN, DM_IMU_ACCEL_CAN_MAX, 16);
    imu->accel[2]    = uint_to_float(raw[2], DM_IMU_ACCEL_CAN_MIN, DM_IMU_ACCEL_CAN_MAX, 16);
    imu->last_update_tick = HAL_GetTick();
}

/**
 * @brief  解析 CAN 角速度数据帧
 *  帧格式: [02] [00] [GyroX_L GyroX_H GyroY_L GyroY_H GyroZ_L GyroZ_H]
 */
static void DM_IMU_ParseGyro(DM_IMU_t *imu, uint8_t *data)
{
    int16_t raw[3];

    raw[0] = (int16_t)(data[2] | (data[3] << 8));
    raw[1] = (int16_t)(data[4] | (data[5] << 8));
    raw[2] = (int16_t)(data[6] | (data[7] << 8));

    imu->gyro[0] = uint_to_float(raw[0], DM_IMU_GYRO_CAN_MIN, DM_IMU_GYRO_CAN_MAX, 16);
    imu->gyro[1] = uint_to_float(raw[1], DM_IMU_GYRO_CAN_MIN, DM_IMU_GYRO_CAN_MAX, 16);
    imu->gyro[2] = uint_to_float(raw[2], DM_IMU_GYRO_CAN_MIN, DM_IMU_GYRO_CAN_MAX, 16);
    imu->last_update_tick = HAL_GetTick();
}

/**
 * @brief  解析 CAN 欧拉角数据帧
 *  帧格式: [03] [00] [Pitch_L Pitch_H Yaw_L Yaw_H Roll_L Roll_H]
 */
static void DM_IMU_ParseEuler(DM_IMU_t *imu, uint8_t *data)
{
    int16_t raw[3];

    raw[0] = (int16_t)(data[2] | (data[3] << 8));  /* Pitch */
    raw[1] = (int16_t)(data[4] | (data[5] << 8));  /* Yaw   */
    raw[2] = (int16_t)(data[6] | (data[7] << 8));  /* Roll  */

    imu->euler[0] = uint_to_float(raw[0], DM_IMU_PITCH_CAN_MIN, DM_IMU_PITCH_CAN_MAX, 16);
    imu->euler[1] = uint_to_float(raw[1], DM_IMU_YAW_CAN_MIN,   DM_IMU_YAW_CAN_MAX,   16);
    imu->euler[2] = uint_to_float(raw[2], DM_IMU_ROLL_CAN_MIN,  DM_IMU_ROLL_CAN_MAX,  16);
    imu->last_update_tick = HAL_GetTick();
}

/**
 * @brief  解析 CAN 四元数数据帧 (14bit 紧凑编码)
 *  帧格式: [04] [W[13:6]] [W[5:0]|X[13:12]] [X[11:4]] [X[3:0]|Y[13:10]] [Y[9:2]] [Y[1:0]|Z[13:8]] [Z[7:0]]
 *
 *  位提取 (参考制造商示例代码):
 *    w = (data[1] << 6) | ((data[2] & 0xF8) >> 2)
 *    x = ((data[2] & 0x03) << 12) | (data[3] << 4) | ((data[4] & 0xF0) >> 4)
 *    y = ((data[4] & 0x0F) << 10) | (data[5] << 2) | ((data[6] & 0xC0) >> 6)
 *    z = ((data[6] & 0x3F) << 8)  |  data[7]
 */
static void DM_IMU_ParseQuaternion(DM_IMU_t *imu, uint8_t *data)
{
    int w, x, y, z;

    w = ((int)data[1] << 6) | ((data[2] & 0xF8) >> 2);
    x = ((data[2] & 0x03) << 12) | ((int)data[3] << 4) | ((data[4] & 0xF0) >> 4);
    y = ((data[4] & 0x0F) << 10) | ((int)data[5] << 2) | ((data[6] & 0xC0) >> 6);
    z = ((data[6] & 0x3F) << 8)  |  (int)data[7];

    imu->quaternion[0] = uint_to_float(w, DM_IMU_QUAT_CAN_MIN, DM_IMU_QUAT_CAN_MAX, 14);
    imu->quaternion[1] = uint_to_float(x, DM_IMU_QUAT_CAN_MIN, DM_IMU_QUAT_CAN_MAX, 14);
    imu->quaternion[2] = uint_to_float(y, DM_IMU_QUAT_CAN_MIN, DM_IMU_QUAT_CAN_MAX, 14);
    imu->quaternion[3] = uint_to_float(z, DM_IMU_QUAT_CAN_MIN, DM_IMU_QUAT_CAN_MAX, 14);
    imu->last_update_tick = HAL_GetTick();
}

/* ========================== CAN 接收回调 (ISR 上下文) ========================== */

void DM_IMU_CAN_Callback(DM_IMU_t *imu, CAN_RxBuffer *rx_buf)
{
    if (imu == NULL || rx_buf == NULL) return;

    /* 仅处理 mst_id 的帧 (IMU 的回传帧) */
    if (rx_buf->Header.Identifier != imu->mst_id) return;

    switch (rx_buf->Data[0])
    {
        case DM_IMU_FRAME_ACCEL:
            DM_IMU_ParseAccel(imu, rx_buf->Data);
            break;
        case DM_IMU_FRAME_GYRO:
            DM_IMU_ParseGyro(imu, rx_buf->Data);
            break;
        case DM_IMU_FRAME_EULER:
            DM_IMU_ParseEuler(imu, rx_buf->Data);
            break;
        case DM_IMU_FRAME_QUATERNION:
            DM_IMU_ParseQuaternion(imu, rx_buf->Data);
            break;
        default:
            break;
    }
}

/* ========================== 在线检测 ========================== */

void DM_IMU_CheckOnline(DM_IMU_t *imu)
{
    if (imu == NULL) return;

    if ((HAL_GetTick() - imu->last_update_tick) > imu->offline_threshold)
        imu->online = 0;
    else
        imu->online = 1;
}
