#ifndef __DM_IMU_RS485_H
#define __DM_IMU_RS485_H

#include "driver_usart.h" // 引入您的串口驱动[cite: 8]

// ==========================================
// 485 通信协议定义[cite: 1]
// ==========================================
#define RS485_FRAME_HEAD    0xA5
#define RS485_FRAME_TAIL    0x5A

#define RS485_TYPE_CMD      0x0C  // 类型：指令[cite: 1]
#define RS485_TYPE_DATA     0x0D  // 类型：数据[cite: 1]

#define RS485_OP_READ       0x00  // 读操作[cite: 1]
#define RS485_OP_WRITE      0x01  // 写操作[cite: 1]

// 指令域寄存器 (Type = 0x0C)[cite: 1]
typedef enum {
    CMD_REG_REBOOT          = 0x00, // 重启 IMU
    CMD_REG_SAVE            = 0x01, // 保存参数
    CMD_REG_ZERO            = 0x02, // 角度置零
    CMD_REG_CALI_GYRO       = 0x03, // 启动陀螺静态校准
    CMD_REG_CALI_ACCEL      = 0x04, // 启动加计六面校准
    CMD_REG_RESTORE         = 0x05, // 恢复出厂设置
    CMD_REG_MODE_SWITCH     = 0x06, // 主被动模式切换 (0:被动 1:主动)
    CMD_REG_BAUDRATE        = 0x07, // 485波特率配置
    CMD_REG_TX_DELAY        = 0x08, // 主动模式发送间隔
    CMD_REG_COM_TYPE        = 0x09  // 通信类型
} imu_485_cmd_reg_e;

// 数据域寄存器 (Type = 0x0D)[cite: 1]
typedef enum {
    DATA_REG_ACCEL          = 0x00, // 加速度数据
    DATA_REG_GYRO           = 0x01, // 角速度数据
    DATA_REG_EULER          = 0x02, // 欧拉角数据
    DATA_REG_QUAT           = 0x03  // 四元数数据
} imu_485_data_reg_e;


typedef enum
{
	RS485_BAUD_9600=0,
	RS485_BAUD_115200,
	RS485_BAUD_230400,
	RS485_BAUD_460800,
	RS485_BAUD_500K,
	RS485_BAUD_921600,
	RS485_BAUD_1M,
	RS485_BAUD_1500K,
	RS485_BAUD_2M,
	RS485_BAUD_2500K,
	RS485_BAUD_3M,
	RS485_BAUD_3500K,
	RS485_BAUD_4M
}imu_baudrate_rs485_e;

typedef struct
{
    struct
    {
        uint8_t slave_id;         // 该 IMU 的 485 从机 ID
		uint8_t transbuffer[24];
    } Buffer;
    
    struct
    {
        float accel[3];         // 加速度 (X, Y, Z)
        float gyro[3];          // 角速度 (X, Y, Z)
        float roll;             // 滚转角
        float pitch;            // 俯仰角
        float yaw;              // 航向角
        float quaternion[4];    // 四元数 (W, X, Y, Z)
    } Attitude;
    
} DM_IMU_rs485_t;


// --- API 声明 ---

// 1. 初始化与解析
void imu_rs485_init(DM_IMU_rs485_t *imu, uint8_t slave_id);
void imu_rs485_data_unpack(DM_IMU_rs485_t *imu, uint8_t* pData, uint16_t size);
uint8_t DM_IMU_Callback(uint8_t* pData,uint16_t size);
void imu_rs485_startup_config(DM_IMU_rs485_t *imu,uint8_t slave_id,imu_baudrate_rs485_e baud);
// 2. 数据请求函数 (用于轮询)
void imu_rs485_request_accel(DM_IMU_rs485_t *imu);
void imu_rs485_request_gyro(DM_IMU_rs485_t *imu);
void imu_rs485_request_euler(DM_IMU_rs485_t *imu);
void imu_rs485_request_quat(DM_IMU_rs485_t *imu);
extern DM_IMU_rs485_t IMU_M;;
// 3. 常用控制指令
void imu_rs485_set_mode_passive(DM_IMU_rs485_t *imu);
void imu_rs485_save_parameters(DM_IMU_rs485_t *imu);
void imu_rs485_set_zero(DM_IMU_rs485_t *imu);
void imu_rs485_gyro_calibration(DM_IMU_rs485_t *imu);
void imu_rs485_set_baud(DM_IMU_rs485_t *imu, imu_baudrate_rs485_e baud);
#endif