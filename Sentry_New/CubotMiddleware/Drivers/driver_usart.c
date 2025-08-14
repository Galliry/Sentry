/**@file  driver_usart.c
* @brief   �����㣬���ڹ����������ļ����û��ص��ض���
* @details  ��Ҫ�����������ڹ��������ṩ���ڳ�ʼ�����û��ص��ض���
* @author      RyanJiao  any question please send mail to 1095981200@qq.com						 
* @date        2021-8-23
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
 ==============================================================================
                          How to use this driver  
 ==============================================================================
 
	���driver_can.h

	1. ��UARTx_Init() �� ��� �� �û�����Ľ��ջص����� ������UART�ṹ��  ���ص������жԽ��յ������ݽ��� IDʶ�� �� �ϲ����㣩

  2. �û���д UART_RxBuffer������ Ŀ�껺������ַ �� ���ݳ��ȡ�

	3. ����UART_Open() ���� UART_Object �� �û���д UART_RxBuffer��
	
	4. �� UART_Idle_Handler ��ӵ� stm32H7xx_it.c �� USARTx_IRQHandler() �У������û���д��ͬһ�� UART_RxBuffer ��
	
	5. Ӧ�ò��д UART_TxBuffer �����ͻ������ṹ�壩������������ֽ������׵�ַ���ֽڳ���
	
	6. ��UART_Send()���� UART�豸�ṹ�� �� UART_TxBuffer�ṹ�壬�����ݷ��ͳ�ȥ

  ********************************************************************************
	* @attention
	* Ӳ��ƽ̨: STM32H750VBT \n
	* SDK�汾��-++++
  * if you had modified this file, please make sure your code does not have many 
  * bugs, update the version NO., write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  ********************************************************************************
	��DMA��NDTR�Ĵ���������
	This register can be written only
	when the stream is disabled. When the stream is enabled, this register is read-only,
	indicating the remaining data items to be transmitted. This register decrements after each
	DMA transfer.
	ָ����DMA�д������ʣ�����ݸ��� ÿ��DMA������ɺ��Զ���һ
	�ο��ֲ��ж�idle�����жϴ���������������
	
*/
#include "driver_usart.h"
//#include "referee.h"
#include "hardware_config.h"
#include "stdio.h"
#include "stdarg.h"
UART_Object uart1;
UART_Object uart2;
UART_Object uart3;
UART_Object uart4;
UART_Object uart5;
UART_Object uart6;
UART_Object uart7;
UART_Object uart8;
UART_Object* Find_UART(UART_HandleTypeDef *huart);


uint8_t Usart_TxBuffer[128];
uint8_t Usart_TxBuffer_[128];
void UsartDmaPrintf(const char *format,...)
{
	uint16_t len;
	va_list args;	
	va_start(args,format);
	len = vsnprintf((char*)Usart_TxBuffer,sizeof(Usart_TxBuffer)+1,(char*)format,args);
	va_end(args);
	HAL_UART_Transmit_DMA(&huart7, Usart_TxBuffer, len);
}
void UsarttoWifi(const char *format,...)
{
	uint16_t len;
	va_list args;	
	va_start(args,format);
	len = vsnprintf((char*)Usart_TxBuffer_,sizeof(Usart_TxBuffer_)+1,(char*)format,args);
	va_end(args);
	HAL_UART_Transmit_DMA(&huart8, Usart_TxBuffer_, len);
}

/**
  * @brief   ���ڳ�ʼ����������ͽ��ջص����������ڽṹ��
  */
void UARTx_Init(UART_HandleTypeDef* handle, UART_RxIdleCallback rxIdleCallback)
{

	UART_Object *uart;
	uart=Find_UART(handle);
	uart->Handle=handle;
	if (rxIdleCallback!=NULL) 
		{
			uart->RxIdleCallback=rxIdleCallback;
			__HAL_UART_CLEAR_IDLEFLAG(uart->Handle);
			__HAL_UART_ENABLE_IT(uart->Handle, UART_IT_IDLE); 
		 HAL_UART_Receive_DMA(uart->Handle, uart->uart_RxBuffer,200);		
    }
		
		
}




/**
  * @brief  �����豸�жϺ�����ִ���ж�DMA���������ô����û��ص����� 
  */
void UART_Idle_Handler(UART_HandleTypeDef *huart)
{
	uint16_t usart_rx_num;
	UART_Object *uart;
	uart=Find_UART(huart);
	 if ((__HAL_UART_GET_FLAG(uart->Handle, UART_FLAG_IDLE) != RESET)  &&uart->RxIdleCallback!=NULL)
    {

        HAL_UART_DMAStop(uart->Handle);          
        __HAL_UART_CLEAR_IDLEFLAG(uart->Handle); 
        __HAL_UART_CLEAR_OREFLAG(uart->Handle);
        usart_rx_num = 200- ((DMA_Stream_TypeDef *)uart->Handle->hdmarx->Instance)->NDTR;

			if (uart->is_first_idle==0)  {uart->is_first_idle=1;}
			else uart->RxIdleCallback(uart->uart_RxBuffer, usart_rx_num); //<�û��ص�
                
  
        HAL_UART_DMAResume(uart->Handle);
        HAL_UART_Receive_DMA(uart->Handle, uart->uart_RxBuffer,200);
    }
}

/**
  * @brief  ����ת�����ַ���ASCii�� 
  */
uint8_t* itoa(int  num, uint8_t* str, int radix)
{
    uint8_t index[]="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"; //<������
    unsigned unum;																	  			//<���Ҫת���������ľ���ֵ,ת�������������Ǹ���
    int i=0,j,k;																					  //<i����ָʾ�����ַ�����Ӧλ��ת��֮��i��ʵ�����ַ����ĳ��ȣ�ת����˳��������ģ��������������k����ָʾ����˳��Ŀ�ʼλ��;j����ָʾ����˳��ʱ�Ľ�����
																														//<��ȡҪת���������ľ���ֵ
    if(radix==10&&num<0)																		//<Ҫת����ʮ�����������Ǹ���
    {
        unum=(unsigned)-num;																//<��num�ľ���ֵ����unum
        str[i++]='-';																				//<���ַ�����ǰ������Ϊ'-'�ţ�����������1
    }
    else unum=(unsigned)num;																//<����numΪ����ֱ�Ӹ�ֵ��unum
		
    do																							  			//<ת�����֣�ע��ת�����������
    {
        str[i++]=index[unum%(unsigned)radix];							//<ȡunum�����һλ��������Ϊstr��Ӧλ��ָʾ������1
        unum/=radix;																				//<unumȥ�����һλ
 
    }while(unum);																						//<ֱ��unumΪ0�˳�ѭ��
 
    str[i]='\0';																						//<���ַ���������'\0'�ַ���c�����ַ�����'\0'������
 
																														//<��˳���������
    if(str[0]=='-') k=1;																		//<����Ǹ��������Ų��õ������ӷ��ź��濪ʼ����
    else k=0;																								//<���Ǹ�����ȫ����Ҫ����
 
    uint8_t temp;																						//<��ʱ��������������ֵʱ�õ�
    for(j=k;j<=(i-1)/2;j++)																	//<ͷβһһ�Գƽ�����i��ʵ�����ַ����ĳ��ȣ��������ֵ�ȳ�����1
    {
        temp=str[j];																				//<ͷ����ֵ����ʱ����
        str[j]=str[i-1+k-j];																//<β����ֵ��ͷ��
        str[i-1+k-j]=temp;																	//<����ʱ������ֵ(��ʵ����֮ǰ��ͷ��ֵ)����β��
    }
    return str;																							//<����ת������ַ���
 
}
UART_Object* Find_UART(UART_HandleTypeDef *huart)
{
	if (huart==&huart1) return &uart1;
	else if (huart==&huart2) return &uart2;
	else if (huart==&huart3) return &uart3;
	else if (huart==&huart4) return &uart4;
	else if (huart==&huart5) return &uart5;
	else if (huart==&huart6) return &uart6;
	else if (huart==&huart7) return &uart7;
	else if (huart==&huart8) return &uart8;
}