/**@file     driver_can.c
* @brief     �����㣬CAN���������ļ�
* @details   ��Ҫ����CAN��������ʼ����ָ���жϻص���������ͬCAN_ID�µ������շ�����
* @date      2021-8-12
* @version   V1.2
* @copyright  Copyright (c) 2021-2121  �й���ҵ��ѧCUBOTս��
**********************************************************************************
* @attention
* Ӳ��ƽ̨: STM32H750VBT \n
* SDK�汾��-++++
* @par �޸���־:
* <table>
* <tr><th>Date       <th>Version  <th>Author    <th>Description
* <tr><td>2021-8-12  <td>1.0      <td>RyanJiao  <td>������ʼ�汾
* <tr><td>2021-10-9  <td>1.0      <td>RyanJiao  <td>�淶��������ȷ����CAN_TxBuffer�ṹ
* </table>
*
**********************************************************************************
 ==============================================================================
                          How to use this driver  
 ==============================================================================

	����driver_can.h
	
	1. ���� (*CAN_RxCpltCallback)(CAN_RxBuffer* rxBuffer) ���͵��û��ص�
	
	1. ����CANx_Init() �� ���?�� �û�����Ľ��ջص�����?������CAN�ṹ��  ���ص������жԽ��յ������ݽ��� IDʶ�� �� �ϲ����㣩

	2. ����CAN_Open() ����ʵ�����Ľṹ�壬����can�豸 
	
	3. Ӧ�ò���?CAN_TxBuffer �����ͻ������ṹ�壩����������͵��ֽ����ݺ�Ŀ��ID
	
	4. ����CAN_Send���� can�豸�ṹ�� �� TxBuffer�ṹ�壬�����ݷ��ͳ�ȥ

  ********************************************************************************
	* @attention
	* Ӳ��ƽ̨: STM32H750VBT \n
	* SDK�汾��-++++
  * if you had modified this file, please make sure your code does not have many 
  * bugs, update the version NO., write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  ********************************************************************************			
  CAN�豸���� 
		
		1.������:

		2.FIFO����: ���ο����ϣ�https://blog.csdn.net/flydream0/article/details/8155942 ��
		
		  ��bxCAN���յ����ģ��������������˺󣬻Ὣ���Ĵ洢��FIFO�С�ÿ���������鶼�����һ��FIFO��--�� FIFO0��FIFO1
			���FIFOΪ3��������ȣ�ÿ��FIFO������������ɣ�������ȫ��Ӳ������������ԼCPU��Դ������  FIFO0������ 3 x mailbox
			��������֤�����ݵ�һ���ԡ�Ӧ�ó���ֻ��ͨ����ȡFIFO������䣬����ȡFIFO�������յ��ı��ġ�
			  
			 FIFO�������״̬����״̬���Һ�?״̬���Һ�2״̬���Һ�3״̬�����״�?
				
			�ڳ�ʼ��״̬ʱ��FIFO�Ǵ��ڿ�״̬�ģ������յ�һ������ʱ��������Ĵ洢��FIFO�ڲ�
			�������У���ʱ��FIFO��״̬��ɹҺ�?״̬�����Ӧ�ó���ȡ�������Ϣ����FIFO�ָ���״̬��
      ���ڼ���FIFO���ڹҺ�1״̬�����ѽ��յ�һ�����ģ���Ӧ�ó���û���ü�ȡ�߽��յ��ı��ģ�
			��ʱ���ٴν��յ�һ�����ģ���ôFIFO����ɹҺ�?״̬���Դ����ƣ�����FIFO����3�����䣬ֻ�ܻ���3������
			����ˣ������յ�?�����ģ������ڼ�Ӧ�ó����δȡ���κα��ģ�ʱ����ʱFIFO������������һ������ʱ��
			���޷��ٴ洢����ʱFIFO��������״̬��
				
		  STM32����CAN������ص��ж���������?
			�����жϣ�ÿ��bxCAN���յ�һ������ʱ����һ���жϡ�
			FIFO���жϣ���FIFO��ʱ�����洢��3������ʱ�������жϡ�
			FIFO����жϣ���FIFO���ʱ�������жϡ�?
***********************************************************************************/
#include "driver_can.h"

//< ��ʼ������ͷ��
CAN_Object can1 = {
			.DevicesList = {&(can1.DevicesList),&(can1.DevicesList)}
};

CAN_Object can2= {
			.DevicesList = {&(can2.DevicesList),&(can2.DevicesList)}
};

volatile uint32_t can1_tx_fail_cnt = 0;
volatile uint32_t can2_tx_fail_cnt = 0;
volatile uint32_t fdcan2_irq_cnt = 0;
volatile uint32_t fdcan2_rxfifo_cb_cnt = 0;
volatile uint32_t fdcan2_error_cb_cnt = 0;


/**
  * @brief  CAN��ʼ����������ͽ��ջص�������CAN�豸�ṹ��
  */
void CANx_Init(FDCAN_HandleTypeDef* handle, CAN_RxCpltCallback rxCallback)
{
	//< ��ʼ��can1
	if (handle->Instance == FDCAN1)
	{
		can1.Handle = handle;
		can1.RxCpltCallback = rxCallback;
	}
	
	//< ��ʼ��can2
	if (handle->Instance == FDCAN2)
	{
		can2.Handle = handle;
		can2.RxCpltCallback = rxCallback;
	}
}


/**
  * @brief CAN�豸��ʼ�������ù�����Ϊ�գ�ʹ��fifo0���յ�����Ϣ�ж� ��ע���û��ص�
  */
void CAN_Open(CAN_Object* can) 
{ 
  FDCAN_FilterTypeDef filter;                   	//< �����ֲ����� can�������ṹ��
	filter.IdType       = FDCAN_STANDARD_ID;       	//< id����Ϊ��׼id
	filter.FilterIndex  = 0;                      	//< ��ֵɸѡ���ı�ţ���׼idѡ��0-127
	filter.FilterType   = FDCAN_FILTER_MASK;       	//< ���ù���ģʽΪ����ģʽ
	filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; 	//< ���������˵����ݴ洢�� fifo0
	filter.FilterID1    = 0x000;                   	//< ɸѡ����id
	filter.FilterID2    = 0x000;
	
	HAL_FDCAN_ConfigFilter(can->Handle, &filter);
	HAL_FDCAN_ConfigGlobalFilter(can->Handle,
	                             FDCAN_ACCEPT_IN_RX_FIFO0,
	                             FDCAN_ACCEPT_IN_RX_FIFO0,
	                             FDCAN_FILTER_REMOTE,
	                             FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ActivateNotification(can->Handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);  // ʹ��fifo0���յ�����Ϣ�ж�
	
  HAL_FDCAN_Start(can->Handle);                   //< ʹ��can
}


/**
  * @brief CAN���ͺ���, ��CAN_Object�µ�txBuffer�е�data���ͳ�ȥ
  */
uint8_t CAN_Send(CAN_Object* can, CAN_TxBuffer* txBuffer)
{
	FDCAN_TxHeaderTypeDef txHeader;
	txHeader.Identifier = txBuffer->Identifier;
	txHeader.IdType = FDCAN_STANDARD_ID;
	txHeader.TxFrameType = FDCAN_DATA_FRAME;
	txHeader.DataLength = FDCAN_DLC_BYTES_8;
	txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txHeader.BitRateSwitch = FDCAN_BRS_OFF;
	txHeader.FDFormat = FDCAN_CLASSIC_CAN;
	txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	txHeader.MessageMarker = 0x00;
	
	if(HAL_FDCAN_AddMessageToTxFifoQ(can->Handle, &txHeader, txBuffer->Data) != HAL_OK)
	{
		if (can->Handle->Instance == FDCAN1)
			can1_tx_fail_cnt++;
		else if (can->Handle->Instance == FDCAN2)
			can2_tx_fail_cnt++;
		return 0;
	}
	else
	{
		return 1;
	}		
}



/**
  * @brief CAN�豸�������ص�
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* handle, uint32_t RxFifo0ITs)
{
	CAN_RxBuffer rxBuffer;
	
	if (handle->Instance == FDCAN1)
	{
		if (HAL_FDCAN_GetRxMessage(handle, FDCAN_RX_FIFO0, &rxBuffer.Header, rxBuffer.Data) != HAL_ERROR)
		{			
			can1.RxCpltCallback(&rxBuffer);
		}
	}
	
 if(handle->Instance == FDCAN2)
	{
		fdcan2_rxfifo_cb_cnt++;
		if(HAL_FDCAN_GetRxMessage(handle, FDCAN_RX_FIFO0, &rxBuffer.Header, rxBuffer.Data) != HAL_ERROR)
		{
			can2.RxCpltCallback(&rxBuffer);
		}
	}
}

/**
  * @brief  FDCAN Error callback - override HAL weak default
  */
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan->Instance == FDCAN2)
        fdcan2_error_cb_cnt++;
}

