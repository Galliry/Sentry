#include "dm_imu_rs485.h"
#include <string.h>

DM_IMU_rs485_t IMU_M;

void imu_rs485_startup_config(DM_IMU_rs485_t *imu,uint8_t slave_id,imu_baudrate_rs485_e baud)
{
    // 1. 初始化结构体和底层接口
    imu_rs485_init(imu, slave_id);
    
    // 2. 强制切换为被动模式 (应答模式)[cite: 1]
    imu_rs485_set_mode_passive(imu);
    HAL_Delay(10); // 等待指令发送与执行
    
    // 3. 修改波特率[cite: 1]
    imu_rs485_set_baud(imu, baud);
    HAL_Delay(10);
    
    // 4. 保存参数[cite: 1]
    // 提醒：如果上位机未提前配好波特率，此处的保存指令可能因波特率不同步而失效。
//    imu_rs485_save_parameters(imu);
//    HAL_Delay(10);
}

void imu_rs485_init(DM_IMU_rs485_t *imu, uint8_t slave_id)
{
    if (imu == NULL) return;
    imu->Buffer.slave_id = slave_id;
}

/**
 * @brief 底层 24 字节帧发送函数[cite: 1]
 */
static void imu_rs485_send_frame(DM_IMU_rs485_t *imu, uint8_t type, uint8_t reg, uint8_t rw, uint32_t data0)
{
    if (imu == NULL) return;

    // 根据 485 应答式帧格式构建 24 字节包[cite: 1] 
    
    imu->Buffer.transbuffer[0] = RS485_FRAME_HEAD;       // 0xA5[cite: 1]
    imu->Buffer.transbuffer[1] = type;                   // 0x0C (指令) 或 0x0D (数据)[cite: 1]
    imu->Buffer.transbuffer[2] = imu->Buffer.slave_id;   // 目标从机 ID[cite: 1]
    imu->Buffer.transbuffer[3] = reg;                    // 寄存器[cite: 1]
    imu->Buffer.transbuffer[4] = rw;                     // 0x00(读) 或 0x01(写)[cite: 1]
    
    // 拷贝参数到 Data0，Data1-3 保持为 0[cite: 1]
    memcpy(&imu->Buffer.transbuffer[5], &data0, 4);

    imu->Buffer.transbuffer[21] = 0x00;                  // 应答位[cite: 1]
    imu->Buffer.transbuffer[22] = 0x00;                  // 保留字节[cite: 1]
    imu->Buffer.transbuffer[23] = RS485_FRAME_TAIL;      // 0x5A[cite: 1]

    // 调用 HAL 库阻塞发送指令 (自动收发模块无需控制 DIR 引脚)
    HAL_UART_Transmit_DMA(&huart5, imu->Buffer.transbuffer, 24);
}


// ==========================================
// 轮询请求数据 API
// ==========================================

void imu_rs485_request_accel(DM_IMU_rs485_t *imu) {
    imu_rs485_send_frame(imu, RS485_TYPE_DATA, DATA_REG_ACCEL, RS485_OP_READ, 0);//[cite: 1]
}

void imu_rs485_request_gyro(DM_IMU_rs485_t *imu) {
    imu_rs485_send_frame(imu, RS485_TYPE_DATA, DATA_REG_GYRO, RS485_OP_READ, 0);//[cite: 1]
}

void imu_rs485_request_euler(DM_IMU_rs485_t *imu) {
    imu_rs485_send_frame(imu, RS485_TYPE_DATA, DATA_REG_EULER, RS485_OP_READ, 0);//[cite: 1]
}

void imu_rs485_request_quat(DM_IMU_rs485_t *imu) {
    imu_rs485_send_frame(imu, RS485_TYPE_DATA, DATA_REG_QUAT, RS485_OP_READ, 0);//[cite: 1]
}


// ==========================================
// 常用控制指令 API
// ==========================================

// 强制切换为被动(应答)模式并保存
void imu_rs485_set_mode_passive(DM_IMU_rs485_t *imu) {
    // 写入 0 代表进入被动模式[cite: 1]
    imu_rs485_send_frame(imu, RS485_TYPE_CMD, CMD_REG_MODE_SWITCH, RS485_OP_WRITE, 0);//[cite: 1]
}

void imu_rs485_save_parameters(DM_IMU_rs485_t *imu) {
    imu_rs485_send_frame(imu, RS485_TYPE_CMD, CMD_REG_SAVE, RS485_OP_WRITE, 0);//[cite: 1]
}

void imu_rs485_set_zero(DM_IMU_rs485_t *imu) {
    imu_rs485_send_frame(imu, RS485_TYPE_CMD, CMD_REG_ZERO, RS485_OP_WRITE, 0);//[cite: 1]
}

void imu_rs485_gyro_calibration(DM_IMU_rs485_t *imu) {
    imu_rs485_send_frame(imu, RS485_TYPE_CMD, CMD_REG_CALI_GYRO, RS485_OP_WRITE, 0);//[cite: 1]
}

void imu_rs485_set_baud(DM_IMU_rs485_t *imu, imu_baudrate_rs485_e baud) {
    // 写入波特率序号到寄存器 0x07[cite: 1]
    imu_rs485_send_frame(imu, RS485_TYPE_CMD, CMD_REG_BAUDRATE, RS485_OP_WRITE, (uint32_t)baud);//[cite: 1]
}

// ==========================================
// 串口 DMA 滑动窗口解析函数
// ==========================================
uint8_t DM_IMU_Callback(uint8_t* pData,uint16_t size)
{
	imu_rs485_data_unpack(&IMU_M,pData,size);
	return 0;
}
void imu_rs485_data_unpack(DM_IMU_rs485_t *imu, uint8_t* pData, uint16_t size)
{
    // 485 应答式帧固定长度为 24 字节[cite: 1]
    if (imu == NULL || pData == NULL || size < 24) return;
    
    uint16_t i = 0;
    
    // 滑动窗口寻找帧头 0xA5 和帧尾 0x5A[cite: 1]
    while (i + 24 <= size)
    {
        if (pData[i] == RS485_FRAME_HEAD && pData[i + 23] == RS485_FRAME_TAIL)//[cite: 1]
        {
            // 校验 ID 是否匹配当前实例[cite: 1]
            if (pData[i + 2] == imu->Buffer.slave_id)
            {
                uint8_t type = pData[i + 1];//[cite: 1]
                uint8_t reg  = pData[i + 3];//[cite: 1]
                
                // 仅解析目标为“数据”的反馈帧[cite: 1]
                if (type == RS485_TYPE_DATA)
                {
                    if (reg == DATA_REG_ACCEL) {
                        memcpy(&imu->Attitude.accel[0], &pData[i + 5], 4);//[cite: 1]
                        memcpy(&imu->Attitude.accel[1], &pData[i + 9], 4);//[cite: 1]
                        memcpy(&imu->Attitude.accel[2], &pData[i + 13], 4);//[cite: 1]
                    } 
                    else if (reg == DATA_REG_GYRO) {
                        memcpy(&imu->Attitude.gyro[0], &pData[i + 5], 4);//[cite: 1]
                        memcpy(&imu->Attitude.gyro[1], &pData[i + 9], 4);//[cite: 1]
                        memcpy(&imu->Attitude.gyro[2], &pData[i + 13], 4);//[cite: 1]
                    } 
                    else if (reg == DATA_REG_EULER) {
                        memcpy(&imu->Attitude.roll,  &pData[i + 5], 4);//[cite: 1]
                        memcpy(&imu->Attitude.pitch, &pData[i + 9], 4);//[cite: 1]
                        memcpy(&imu->Attitude.yaw,   &pData[i + 13], 4);//[cite: 1]
                    } 
                    else if (reg == DATA_REG_QUAT) {
                        memcpy(&imu->Attitude.quaternion[0], &pData[i + 5], 4);//[cite: 1]
                        memcpy(&imu->Attitude.quaternion[1], &pData[i + 9], 4);//[cite: 1]
                        memcpy(&imu->Attitude.quaternion[2], &pData[i + 13], 4);//[cite: 1]
                        memcpy(&imu->Attitude.quaternion[3], &pData[i + 17], 4);//[cite: 1]
                    }
                }
                
                i += 24; // 解析成功，跳过这一完整帧
                continue;
            }
        }
        i++; // 未找到或者解析失败，向后滑一个字节
    }
}