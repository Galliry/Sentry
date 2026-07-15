#include "DM_imu.h"
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "user_lib.h"
DM_IMU_t IMU_M;
DM_IMU_t IMU_S;
// ==========================================
// 初始化设备
// ==========================================
void DM_IMU_Init(DM_IMU_t *imu, uint8_t can_id, uint8_t mst_id, CAN_Object can_obj)
{
	if(imu == NULL) return;
	
	imu->Buffer.can_id = can_id;
	imu->Buffer.mst_id = mst_id;
	imu->Buffer.can_obj = can_obj;
}

void DM_IMU_Run(DM_IMU_t *imu)
{
	imu_change_com_port(imu, COM_CAN);
   
    imu_set_baud(imu, CAN_BAUD_1M);
	
    imu_set_active_mode_delay(imu, 500);
	
    imu_change_to_active(imu);
}

// 单个陀螺仪校准
uint8_t DM_IMU_Calibration(DM_IMU_t *imu)
{
	imu->state.InitFlag = 0;
	uint8_t triedCnt = 0;
	const float stillGyro = 0.02f;
	do 
	{
		// 校准前检查是否处于静止状态
		uint32_t checkEndTick = HAL_GetTick() + 1000;
		while ( HAL_GetTick() < checkEndTick )
		{
			// 如果认为不符合静止条件则不发送零飘校准指令
			if ( fabs(imu->Attitude.gyro[0]) > stillGyro 
				|| fabs(imu->Attitude.gyro[1]) > stillGyro 
				|| fabs(imu->Attitude.gyro[2]) > stillGyro )
			{
				goto NextTurn;
			}
		}
		// 发送校准指令
		imu_gyro_calibration(imu);
		// 校准需要数秒，需要等待其校准完成
		float temp_gyro[3];
		float temp_euler[3];
		for (int i = 0; i < 3; i ++)
		{
			temp_gyro[i] = imu->Attitude.gyro[i];
		}
		temp_euler[0] = imu->Attitude.pitch;
		temp_euler[1] = imu->Attitude.yaw;
		temp_euler[2] = imu->Attitude.roll;
		// 比对重启前的数据
		// 不能够用 IMU_isOnline来判定是否已经完成校准
		// 因为校准过程中被测量保持不变但是can仍会发送过时数据，LastTick依旧正常增加
		while ( temp_gyro[0] == imu->Attitude.gyro[0]
			||  temp_gyro[1] == imu->Attitude.gyro[1]
			||  temp_gyro[2] == imu->Attitude.gyro[2]
			||  temp_euler[0] == imu->Attitude.pitch
			||  temp_euler[1] == imu->Attitude.yaw
			||  temp_euler[2] == imu->Attitude.roll) HAL_Delay(10);

		// 检查校准效果
		HAL_Delay(1500);
		float Yaw0 = imu->Attitude.yaw;
		float YawPError = 0; // 最大正向偏差
		float YawNError = 0; // 最大负向偏差
		const uint32_t deltaTime = 15000;
		float YawError = 0;
		checkEndTick = HAL_GetTick() + deltaTime;
		while ( HAL_GetTick() < checkEndTick )
		{
			YawError = imu->Attitude.yaw - Yaw0;
			if (YawError > 180) YawError -= 360;
			if (YawError < -180) YawError += 360;
			
			if( YawError > YawPError ) {
				YawPError = YawError;
			}
			if( YawError < YawNError ) {
				YawNError = YawError;
			}
		}
		if ( (YawPError>-YawNError?YawPError:-YawNError) < deltaTime * ( 6.0f/7/60/1000 ) )
		{
			// 符合要求则跳出，完成校准
			imu->state.InitFlag = 1;
			return 1;
		}
		else {
			// 不符合要求则再次尝试，最多尝试3次校准
			triedCnt += 10;
		}

		NextTurn:
		triedCnt ++;
		HAL_Delay(10);
	}while( triedCnt < 39);

	return 0;
}

// 双陀螺仪同时校准
uint8_t DM_IMUs_Calibration(DM_IMU_t *imu1, DM_IMU_t *imu2)
{
	imu1->state.InitFlag = 0;
	imu2->state.InitFlag = 0;
	uint8_t triedCnt1 = 0;
	uint8_t triedCnt2 = 0;
	const float stillGyro = 0.025f;
	do 
	{
		// 校准前检查是否处于静止状态
		uint32_t checkEndTick = HAL_GetTick() + 1000;
		uint8_t stillFlag = 0x00;
		while ( HAL_GetTick() < checkEndTick )
		{
			// 如果认为不符合静止条件则不发送零飘校准指令
			if ( imu1->state.InitFlag == 0 && (stillFlag & 0x01) == 0)
			{
				if ( fabs(imu1->Attitude.gyro[0]) > stillGyro 
					|| fabs(imu1->Attitude.gyro[1]) > stillGyro 
					|| fabs(imu1->Attitude.gyro[2]) > stillGyro )
				{
					stillFlag |= 0x01;
					triedCnt1 ++;
				}
			}
			if ( imu2->state.InitFlag == 0 && (stillFlag & (0x01 << 1)) == 0)
			{
				if ( fabs(imu2->Attitude.gyro[0]) > stillGyro 
					|| fabs(imu2->Attitude.gyro[1]) > stillGyro 
					|| fabs(imu2->Attitude.gyro[2]) > stillGyro )
				{
					stillFlag |= (0x01<<1);
					triedCnt2 ++;
				}
			}
			if (stillFlag == 0x03 ) // 0x03 = 0x01 & (0x01<<1) 两个陀螺仪都没静止
			{
				goto NextTurn;
			}
		}

		// 发送校准指令
		if ( imu1->state.InitFlag == 0 && (stillFlag & 0x01) == 0)
		{
			imu_gyro_calibration(imu1);
		}
		if ( imu2->state.InitFlag == 0 && (stillFlag & 0x02) == 0)
		{
			imu_gyro_calibration(imu2);
		}

		// 校准需要数秒，需要等待其校准完成
		float temp1_gyro[3];
		float temp1_euler[3];
		for (int i = 0; i < 3; i ++)
		{
			temp1_gyro[i] = imu1->Attitude.gyro[i];
		}
		temp1_euler[0] = imu1->Attitude.pitch;
		temp1_euler[1] = imu1->Attitude.yaw;
		temp1_euler[2] = imu1->Attitude.roll;

		float temp2_gyro[3];
		float temp2_euler[3];
		for (int i = 0; i < 3; i ++)
		{
			temp2_gyro[i] = imu2->Attitude.gyro[i];
		}
		temp2_euler[0] = imu2->Attitude.pitch;
		temp2_euler[1] = imu2->Attitude.yaw;
		temp2_euler[2] = imu2->Attitude.roll;
		// 比对重启前的数据
		// 不能够用 IMU_isOnline来判定是否已经完成校准
		// 因为校准过程中被测量保持不变但是can仍会发送过时数据，LastTick依旧正常增加
		uint8_t rebootFlag = 0x00;
		while ( rebootFlag != 0x03 ) 
			{
				if (imu1->state.InitFlag == 1)
				{
					rebootFlag |= 0x01;
				}
				else if (temp1_gyro[0] != imu1->Attitude.gyro[0]
					&&   temp1_gyro[1] != imu1->Attitude.gyro[1]
					&&   temp1_gyro[2] != imu1->Attitude.gyro[2]
					&&   temp1_euler[0] != imu1->Attitude.pitch
					&&   temp1_euler[1] != imu1->Attitude.yaw
					&&   temp1_euler[2] != imu1->Attitude.roll)
				{
					rebootFlag |= 0x01;
				}
				if (imu2->state.InitFlag == 1)
				{
					rebootFlag |= 0x01 << 1;
				}
				else if (temp2_gyro[0] != imu2->Attitude.gyro[0]
					&&   temp2_gyro[1] != imu2->Attitude.gyro[1]
					&&   temp2_gyro[2] != imu2->Attitude.gyro[2]
					&&   temp2_euler[0] != imu2->Attitude.pitch
					&&   temp2_euler[1] != imu2->Attitude.yaw
					&&   temp2_euler[2] != imu2->Attitude.roll)
				{
					rebootFlag |= 0x01 << 1;
				}
				HAL_Delay(10);
			}

		// 检查校准效果
		HAL_Delay(1500);
		float Yaw1 = imu1->Attitude.yaw;
		float Yaw2 = imu2->Attitude.yaw;
		float Yaw1PError = 0; // 最大正向偏差
		float Yaw1NError = 0; // 最大负向偏差
		float Yaw2PError = 0;
		float Yaw2NError = 0;
		const uint32_t deltaTime = 15000;
		float Yaw1Error = 0;
		float Yaw2Error = 0;
		checkEndTick = HAL_GetTick() + deltaTime;
		while ( HAL_GetTick() < checkEndTick )
		{
			// 即使有已经完成校准的陀螺仪依旧可以再次检查一遍
			Yaw1Error = imu1->Attitude.yaw - Yaw1;
			if (Yaw1Error > 180) Yaw1Error -= 360;
			if (Yaw1Error < -180) Yaw1Error += 360;
			Yaw2Error = imu2->Attitude.yaw - Yaw2;
			if (Yaw2Error > 180) Yaw2Error -= 360;
			if (Yaw2Error < -180) Yaw2Error += 360;
			
			if( Yaw1Error > Yaw1PError ) {
				Yaw1PError = Yaw1Error;
			}
			if( Yaw1Error < Yaw1NError ) {
				Yaw1NError = Yaw1Error;
			}
			if( Yaw2Error > Yaw2PError ) {
				Yaw2PError = Yaw2Error;
			}
			if( Yaw2Error < Yaw2NError ) {
				Yaw2NError = Yaw2Error;
			}
		}
		if ( (Yaw1PError>-Yaw1NError?Yaw1PError:-Yaw1NError) < deltaTime * ( 10.0f/7/60/1000 ) )
		{
			// 符合要求则跳出，完成校准
			imu1->state.InitFlag = 1;
		}
		else {
			// 不符合要求则再次尝试，最多尝试3次校准
			triedCnt1 += 10;
		}
		if ( (Yaw2PError>-Yaw2NError?Yaw2PError:-Yaw2NError) < deltaTime * ( 10.0f/7/60/1000 ) )
		{
			// 符合要求则跳出，完成校准
			imu2->state.InitFlag = 1;
		}
		else {
			// 不符合要求则再次尝试，最多尝试3次校准
			triedCnt2 += 10;
		}

		NextTurn:
		HAL_Delay(10);
	}while( !(imu1->state.InitFlag && imu2->state.InitFlag) && (triedCnt1 < 39 || triedCnt2 < 39) );

	return (imu1->state.InitFlag && imu2->state.InitFlag);
}

/*
    发送底层指令
*/
static void imu_send_cmd(DM_IMU_t* imu, uint8_t reg_id, uint8_t ac, uint32_t data)
{
	// 安全校验：确认结构体不为空，且绑定的 CAN 驱动对象句柄已初始化
	if(imu->Buffer.can_obj.Handle == NULL)
		return;
	
	// 打包数据区
	uint8_t buf[8] = {0xCC, reg_id, ac, 0xDD, 0, 0, 0, 0};
	memcpy(buf + 4, &data, 4);
	
	// 填充发送缓存区结构体
	imu->Buffer.tx_buf.Identifier = imu->Buffer.can_id;
	memcpy(imu->Buffer.tx_buf.Data, buf, 8);
	
	// 调用底层 CAN 驱动发送 (can_obj已经是 CAN_Object* 类型，直接传入即可)
	CAN_Send(&imu->Buffer.can_obj, &imu->Buffer.tx_buf);
}


/**
 * @brief  向 IMU 发送“写”指令到底层驱动
 * @param  imu     IMU 实例句柄
 * @param  reg_id  目标寄存器地址 (例如 REBOOT_IMU)
 * @param  data    要写入的数据内容
 */
void imu_write_reg(DM_IMU_t *imu, uint8_t reg_id, uint32_t data)
{
	imu_send_cmd(imu, reg_id, CMD_WRITE, data);
}

/**
 * @brief  向 IMU 发送“读”指令到底层驱动 (常用于请求被动模式下的数据)
 * @param  imu     IMU 实例句柄
 * @param  reg_id  目标寄存器地址 (例如 ACCEL_DATA)
 */
void imu_read_reg(DM_IMU_t *imu, uint8_t reg_id)
{
	imu_send_cmd(imu, reg_id, CMD_READ, 0);
}

/**
 * @brief  重启 IMU 模块
 * @note   操作寄存器 0x00 (W)[cite: 1]
 */
void imu_reboot(DM_IMU_t *imu)
{
	imu_write_reg(imu, REBOOT_IMU, 0);
}

/**
 * @brief  启动加速度计六面校准
 * @note   操作寄存器 0x06 (W)[cite: 1]，启动后需观察指示灯黄灯闪烁进行六面翻转[cite: 1]
 */
void imu_accel_calibration(DM_IMU_t *imu)
{
	imu_write_reg(imu, ACCEL_CALI, 0);
}

/**
 * @brief  启动陀螺仪静态校准
 * @note   操作寄存器 0x07 (W)[cite: 1]，启动后 IMU 需保持静止，校准完成后会自动重启[cite: 1]
 */
void imu_gyro_calibration(DM_IMU_t *imu)
{
	imu_write_reg(imu, GYRO_CALI, 0);
}

/**
 * @brief  切换目标通信接口
 * @note   操作寄存器 0x09 (RW)[cite: 1]
 * @param  port  目标接口：COM_USB=0, COM_RS485=1, COM_CAN=2, COM_VOFA=3[cite: 1]
 */
void imu_change_com_port(DM_IMU_t *imu, imu_com_port_e port)
{
	imu_write_reg(imu, CHANGE_COM, (uint8_t)port);
}

/**
 * @brief  设置主动发送模式下的发送间隔
 * @note   操作寄存器 0x0A (RW)[cite: 1]
 * @param  delay 间隔时间，单位参考说明书波特率与频率映射
 */
void imu_set_active_mode_delay(DM_IMU_t *imu, uint32_t delay)
{
	imu_write_reg(imu, SET_DELAY, delay);
}

/**
 * @brief  切换为主动发送模式
 * @note   操作寄存器 0x0B (RW)[cite: 1]，写入 1。IMU 将按照设定的频率主动向总线广播数据
 */
void imu_change_to_active(DM_IMU_t *imu)
{
	imu_write_reg(imu, CHANGE_ACTIVE, 1);
}

/**
 * @brief  切换为请求应答模式（被动模式）
 * @note   操作寄存器 0x0B (RW)[cite: 1]，写入 0。必须主机发送请求帧后，IMU 才会返回一条数据
 */
void imu_change_to_request(DM_IMU_t *imu)
{
	imu_write_reg(imu, CHANGE_ACTIVE, 0);
}

/**
 * @brief  修改 CAN / 485 通信波特率
 * @note   操作寄存器 0x0C (RW)[cite: 1]
 * @param  baud  波特率序号，例如 CAN_BAUD_1M = 0[cite: 1]
 */
void imu_set_baud(DM_IMU_t *imu, imu_baudrate_e baud)
{
	imu_write_reg(imu, SET_BAUD, (uint8_t)baud);
}

/**
 * @brief  设置 IMU 设备的 CAN ID
 * @note   操作寄存器 0x0D (RW)[cite: 1]。修改后，后续发送给该 IMU 的控制指令需要发往这个新 ID[cite: 1]
 * @param  can_id  目标设备 ID
 */
void imu_set_can_id(DM_IMU_t *imu, uint8_t can_id)
{
	imu_write_reg(imu, SET_CAN_ID, can_id);
}

/**
 * @brief  设置主机的 MST ID
 * @note   操作寄存器 0x0E (RW)[cite: 1]。这是 IMU 发送应答帧或主动输出时使用的标识 ID[cite: 1]
 * @param  mst_id  目标主机 ID
 */
void imu_set_mst_id(DM_IMU_t *imu, uint8_t mst_id)
{
	imu_write_reg(imu, SET_MST_ID, mst_id);
}

/**
 * @brief  保存当前参数
 * @note   操作寄存器 0xFE (W)[cite: 1]。对通信端口、波特率、ID 等进行修改后务必执行此指令，防止掉电丢失[cite: 1]
 */
void imu_save_parameters(DM_IMU_t *imu)
{
	imu_write_reg(imu, SAVE_PARAM, 0);
}

/**
 * @brief  恢复出厂设置
 * @note   操作寄存器 0xFF (W)[cite: 1]
 */
void imu_restore_settings(DM_IMU_t *imu)
{
	imu_write_reg(imu, RESTORE_SETTING, 0);
}

/**
 * @brief  向 IMU 发起一次读取“加速度数据”的请求
 * @note   操作寄存器 0x01 (R)[cite: 1]。发送后底层的接收中断中将捕获到一帧包含温度和三轴加速度的应答数据[cite: 1]
 */
void imu_request_accel(DM_IMU_t *imu)
{
	imu_read_reg(imu, ACCEL_DATA);
}

/**
 * @brief  向 IMU 发起一次读取“角速度数据”的请求
 * @note   操作寄存器 0x02 (R)[cite: 1]
 */
void imu_request_gyro(DM_IMU_t *imu)
{
	imu_read_reg(imu, GYRO_DATA);
}

/**
 * @brief  向 IMU 发起一次读取“欧拉角数据”的请求
 * @note   操作寄存器 0x03 (R)[cite: 1]。返回数据包含 Pitch、Yaw、Roll 映射值[cite: 1]
 */
void imu_request_euler(DM_IMU_t *imu)
{
	imu_read_reg(imu, EULER_DATA);
}

/**
 * @brief  向 IMU 发起一次读取“四元数数据”的请求
 * @note   操作寄存器 0x04 (R)[cite: 1]
 */
void imu_request_quat(DM_IMU_t *imu)
{
	imu_read_reg(imu, QUAT_DATA);
}

void IMU_UpdateAccel(DM_IMU_t *imu, uint8_t* pData)
{
	if(imu == NULL || pData == NULL) return;
	
	uint16_t accel[3];
	accel[0] = pData[3]<<8 | pData[2];
	accel[1] = pData[5]<<8 | pData[4];
	accel[2] = pData[7]<<8 | pData[6];
	
	imu->Attitude.accel[0] = uint_to_float(accel[0], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
	imu->Attitude.accel[1] = uint_to_float(accel[1], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
	imu->Attitude.accel[2] = uint_to_float(accel[2], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);

	imu->state.LastTick = HAL_GetTick();
}

void IMU_UpdateGyro(DM_IMU_t *imu, uint8_t* pData)
{
	if(imu == NULL || pData == NULL) return;
	
	uint16_t gyro[3];
	gyro[0] = pData[3]<<8 | pData[2];
	gyro[1] = pData[5]<<8 | pData[4];
	gyro[2] = pData[7]<<8 | pData[6];
	
	imu->Attitude.gyro[0] = uint_to_float(gyro[0], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
	imu->Attitude.gyro[1] = uint_to_float(gyro[1], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
	imu->Attitude.gyro[2] = uint_to_float(gyro[2], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);

	imu->state.LastTick = HAL_GetTick();
}

void IMU_UpdateEuler(DM_IMU_t *imu, uint8_t* pData)
{
	if(imu == NULL || pData == NULL) return;
	
	int euler[3];
	euler[0] = pData[3]<<8 | pData[2];
	euler[1] = pData[5]<<8 | pData[4];
	euler[2] = pData[7]<<8 | pData[6];
	
	imu->Attitude.pitch = uint_to_float(euler[0], PITCH_CAN_MIN, PITCH_CAN_MAX, 16);
	imu->Attitude.yaw   = uint_to_float(euler[1], YAW_CAN_MIN, YAW_CAN_MAX, 16);
	imu->Attitude.roll  = uint_to_float(euler[2], ROLL_CAN_MIN, ROLL_CAN_MAX, 16);

	imu->state.LastTick = HAL_GetTick();
}

void IMU_UpdateQuaternion(DM_IMU_t *imu, uint8_t* pData)
{
	if(imu == NULL || pData == NULL) return;
	
	int w = pData[1]<<6 | ((pData[2]&0xF8)>>2);
	int x = (pData[2]&0x03)<<12 | (pData[3]<<4) | ((pData[4]&0xF0)>>4);
	int y = (pData[4]&0x0F)<<10 | (pData[5]<<2) | (pData[6]&0xC0)>>6;
	int z = (pData[6]&0x3F)<<8  | pData[7];
	
	imu->Attitude.q[0] = uint_to_float(w, Quaternion_MIN, Quaternion_MAX, 14);
	imu->Attitude.q[1] = uint_to_float(x, Quaternion_MIN, Quaternion_MAX, 14);
	imu->Attitude.q[2] = uint_to_float(y, Quaternion_MIN, Quaternion_MAX, 14);
	imu->Attitude.q[3] = uint_to_float(z, Quaternion_MIN, Quaternion_MAX, 14);

	imu->state.LastTick = HAL_GetTick();
}

void IMU_UpdateData(DM_IMU_t *imu, CAN_RxBuffer *rxBuffer)
{
	
	if(rxBuffer->Header.Identifier == imu->Buffer.mst_id)
	{
		uint8_t* pData = rxBuffer->Data; // 提取数据段
		
		switch(pData[0])
		{
			case 1:
				IMU_UpdateAccel(imu, pData);
				break;
			case 2:
				IMU_UpdateGyro(imu, pData);
				break;
			case 3:
				IMU_UpdateEuler(imu, pData);
				break;
			case 4:
				IMU_UpdateQuaternion(imu, pData);
				break;
			default:
				break;
		}
	}
}

uint8_t IMU_isOnline(DM_IMU_t *imu)
{
	if (HAL_GetTick() - imu->state.LastTick > 20 ) return 0;
	return 1;
}

uint8_t IMU_isInit(DM_IMU_t *imu)
{
	if (imu->state.InitFlag) return 1;
}
