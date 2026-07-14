#ifndef __DM_IMU_H
#define __DM_IMU_H

#include "driver_can.h" 

#define ACCEL_CAN_MAX 	(235.2f)
#define ACCEL_CAN_MIN	(-235.2f)
#define GYRO_CAN_MAX	(34.88f)
#define GYRO_CAN_MIN	(-34.88f)
#define PITCH_CAN_MAX	(90.0f)
#define PITCH_CAN_MIN	(-90.0f)
#define ROLL_CAN_MAX	(180.0f)
#define ROLL_CAN_MIN	(-180.0f)
#define YAW_CAN_MAX		(180.0f)
#define YAW_CAN_MIN 	(-180.0f)
#define TEMP_MIN		(0.0f)
#define TEMP_MAX		(60.0f)
#define Quaternion_MIN	(-1.0f)
#define Quaternion_MAX	(1.0f)

#define CMD_READ 0
#define CMD_WRITE 1

typedef enum
{
	COM_USB=0,
	COM_RS485,
	COM_CAN,
	COM_VOFA
}imu_com_port_e;

typedef enum
{
	CAN_BAUD_1M=0,
	CAN_BAUD_500K,
	CAN_BAUD_400K,
	CAN_BAUD_250K,
	CAN_BAUD_200K,
	CAN_BAUD_100K,
	CAN_BAUD_50K,
	CAN_BAUD_25K
}imu_baudrate_e;

typedef enum 
{
	REBOOT_IMU=0,
	ACCEL_DATA,
	GYRO_DATA,
	EULER_DATA,
	QUAT_DATA,
	SET_ZERO,
	ACCEL_CALI,
	GYRO_CALI,
	MAG_CALI,
	CHANGE_COM,
	SET_DELAY,
	CHANGE_ACTIVE,
	SET_BAUD,
	SET_CAN_ID,
	SET_MST_ID,
	DATA_OUTPUT_SELECTION,
	SAVE_PARAM=254,
	RESTORE_SETTING=255
}reg_id_e;

// 嵌套结构的 IMU 结构体
typedef struct
{
	struct
	{
		uint8_t can_id;           // CAN ID (IMU设备ID)
		uint8_t mst_id;           // 主机 ID
		CAN_Object can_obj;      // 底层驱动对象指针 (如 &can1)
		CAN_TxBuffer tx_buf;      // 发送缓存区结构
	}Buffer;
	
	struct
	{
		float accel[3];         // 加速度
		float gyro[3];          // 角速度
		float roll;             // 滚转角
		float pitch;            // 俯仰角
		float yaw;              // 偏航角
		float yaw_total_angle;  // 最终解算得到的角度,以及yaw转动的总角度(方便多圈控制)
		float q[4];             // 四元数
		float cur_temp;         // 当前温度
	}Attitude;
	
} DM_IMU_t;


// --- 初始化接口 ---
void DM_IMU_Init(DM_IMU_t *imu, uint8_t can_id, uint8_t mst_id, CAN_Object can_obj);
void DM_IMU_Run(DM_IMU_t *imu);
// --- 指令控制接口 ---
void imu_write_reg(DM_IMU_t *imu, uint8_t reg_id, uint32_t data);
void imu_read_reg(DM_IMU_t *imu, uint8_t reg_id);
void imu_reboot(DM_IMU_t *imu);
void imu_accel_calibration(DM_IMU_t *imu);
void imu_gyro_calibration(DM_IMU_t *imu);
void imu_change_com_port(DM_IMU_t *imu, imu_com_port_e port);
void imu_set_active_mode_delay(DM_IMU_t *imu, uint32_t delay);
void imu_change_to_active(DM_IMU_t *imu);
void imu_change_to_request(DM_IMU_t *imu);
void imu_set_baud(DM_IMU_t *imu, imu_baudrate_e baud);
void imu_set_can_id(DM_IMU_t *imu, uint8_t can_id);
void imu_set_mst_id(DM_IMU_t *imu, uint8_t mst_id);
void imu_save_parameters(DM_IMU_t *imu);
void imu_restore_settings(DM_IMU_t *imu);
void imu_request_accel(DM_IMU_t *imu);
void imu_request_gyro(DM_IMU_t *imu);
void imu_request_euler(DM_IMU_t *imu);
void imu_request_quat(DM_IMU_t *imu);

// --- 解析接口 ---
void IMU_UpdateAccel(DM_IMU_t *imu, uint8_t* pData);
void IMU_UpdateGyro(DM_IMU_t *imu, uint8_t* pData);
void IMU_UpdateEuler(DM_IMU_t *imu, uint8_t* pData);
void IMU_UpdateQuaternion(DM_IMU_t *imu, uint8_t* pData);
void IMU_UpdateData(DM_IMU_t *imu, CAN_RxBuffer *rxBuffer);
extern DM_IMU_t IMU_M;
extern DM_IMU_t IMU_S;
#endif