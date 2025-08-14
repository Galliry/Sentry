/**@file  driver_iic.c
* @brief    �弶֧�ְ������ڹ����������ļ����û��ص��ض���
* @details  ��Ҫ�����������ڹ��������ṩ���ڳ�ʼ�����û��ص��ض���
* @author      RyanJiao  any question please send mail to 1095981200@qq.com

* @date        2022-2-20
* @version     V1.1
* @copyright    Copyright (c) 2021-2121  �й���ҵ��ѧCUBOTս��
**********************************************************************************
* @attention
* Ӳ��ƽ̨: STM32H750VBT \n
* SDK�汾��-++++
* @par �޸���־:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2021-8-12  <td>1.0      <td>RyanJiao  <td>������ʼ�汾
* </table>
*
**********************************************************************************
*/
#include "driver_iic.h"



static void I2C2_Error(uint8_t Addr,I2C_HandleTypeDef *hi2c)
{
	/* �ָ�I2C�Ĵ���ΪĬ��ֵ */
	HAL_I2C_DeInit(hi2c);
	/* ���³�ʼ��I2C���� */
	HAL_I2C_Init(hi2c);
}


/**
  * @brief  д�Ĵ����������ṩ���ϲ�Ľӿ�
  */
int Sensors_I2C_WriteRegister(unsigned char slave_addr,
	unsigned char reg_addr,
	unsigned short len,
	unsigned char* data_ptr,I2C_HandleTypeDef *hi2c)
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Write(hi2c, slave_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data_ptr, len, I2Cx_FLAG_TIMEOUT);
	/* ���ͨѶ״̬ */
	if (status != HAL_OK)
	{
		/* ���߳����� */
		I2C2_Error(slave_addr,hi2c);
	}
	while (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY)
	{

	}
	/* ���SENSOR�Ƿ����������һ�ζ�д���� */
	while (HAL_I2C_IsDeviceReady(hi2c, slave_addr, I2Cx_FLAG_TIMEOUT, I2Cx_FLAG_TIMEOUT) == HAL_TIMEOUT);
	/* �ȴ�������� */
	while (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY)
	{

	}
	return status;
}

/**
  * @brief  ���Ĵ����������ṩ���ϲ�Ľӿ�
  */
int Sensors_I2C_ReadRegister(unsigned char slave_addr,
	unsigned char reg_addr,
	unsigned short len,
	unsigned char* data_ptr,I2C_HandleTypeDef *hi2c)
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Read(hi2c, slave_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data_ptr, len, I2Cx_FLAG_TIMEOUT);
	/* ���ͨѶ״̬ */
	if (status != HAL_OK)
	{
		/* ���߳����� */
		I2C2_Error(slave_addr,hi2c);
	}
	while (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY)
	{

	}
	/* ���SENSOR�Ƿ����������һ�ζ�д���� */
	while (HAL_I2C_IsDeviceReady(hi2c, slave_addr, I2Cx_FLAG_TIMEOUT, I2Cx_FLAG_TIMEOUT) == HAL_TIMEOUT);
	/* �ȴ�������� */
	while (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY)
	{

	}
	return status;
}
int Sensors_I2C_ReadRegister_1(unsigned char slave_addr,
	unsigned char reg_addr,
	unsigned short len,
	unsigned char* data_ptr,I2C_HandleTypeDef *hi2c)
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Read_IT(&hi2c2, slave_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data_ptr, len);

	return status;
}