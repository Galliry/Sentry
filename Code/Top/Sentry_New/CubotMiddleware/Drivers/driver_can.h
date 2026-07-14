#ifndef DRV_CAN_H_
#define DRV_CAN_H_
#include "stm32h7xx_hal.h"
#include "fdcan.h"	
#include "linux_list.h"


/**
	* @brief	CAN�豸��ö��
	*/
typedef enum 
{
	CAN1 = 0x01U,
	CAN2 = 0x02U
}CanNumber;


/**
	* @brief	CAN���ջ�����
	*/
typedef struct
{
	FDCAN_RxHeaderTypeDef	Header;
	uint8_t	Data[8];
}CAN_RxBuffer;


/**
	* @brief	CAN���ͻ�����
	*/
typedef struct
{
	uint32_t Identifier;
	uint8_t		Data[8];
}CAN_TxBuffer;


/**
  * @brief   CAN�û��ص�����
	* @param[in] rxBuffer 	CAN���ջ�����
  */
typedef uint8_t(*CAN_RxCpltCallback)(CAN_RxBuffer* rxBuffer);


/**
	* @brief	CAN�豸����
	*/
typedef struct
{
	FDCAN_HandleTypeDef* 	Handle;
	list_t					DevicesList;
	CAN_RxCpltCallback		RxCpltCallback;
}CAN_Object;


/**
  * @brief  CAN��ʼ����������ͽ��ջص�������CAN�ṹ��
  * @param[in]  handle		        ���ھ��
  */
void CANx_Init(FDCAN_HandleTypeDef* handle, CAN_RxCpltCallback rxCallback);


/**
  * @brief ��CAN�豸�����ù�����Ϊ�գ�ʹ��fifo0���յ�����Ϣ�жϣ�ע���û��ص�
  * @param[in]	can	CAN�豸
  */
void CAN_Open(CAN_Object* can);


/**
  * @brief ͨ��CAN�豸��������
  *	@param[in] txBuffer CAN�ķ��ͻ�����
	* 
  */
uint8_t CAN_Send(CAN_Object* can, CAN_TxBuffer* txBuffer);


/**
  * @brief CAN�豸�������ص�
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* handle, uint32_t RxFifo0ITs);



extern CAN_Object can1;
extern CAN_Object can2;

extern volatile uint32_t can1_tx_fail_cnt;
extern volatile uint32_t can2_tx_fail_cnt;
extern volatile uint32_t fdcan2_irq_cnt;
extern volatile uint32_t fdcan2_rxfifo_cb_cnt;
extern volatile uint32_t fdcan2_error_cb_cnt;


#endif



///*�ӿ���͵���� ������Ϣ�жϵ��ж�*/
//#define FDCAN_RX_FIFO0_MASK (FDCAN_IR_RF0L | FDCAN_IR_RF0F | FDCAN_IR_RF0W | FDCAN_IR_RF0N)



