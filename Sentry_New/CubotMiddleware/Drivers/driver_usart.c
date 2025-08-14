/**@file  driver_usart.c
* @brief   驱动层，串口管理器配置文件，用户回调重定义
* @details  主要包括构建串口管理器，提供串口初始化和用户回调重定义
* @author      RyanJiao  any question please send mail to 1095981200@qq.com						 
* @date        2021-8-23
* @version     V1.1
* @copyright    Copyright (c) 2021-2121  中国矿业大学CUBOT战队
**********************************************************************************
* @attention
* 硬件平台: STM32H750VBT \n
* SDK版本：-++++
* @par 修改日志:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2021-8-12  <td>1.0      <td>RyanJiao  <td>创建初始版本
* </table>
*
**********************************************************************************
 ==============================================================================
                          How to use this driver  
 ==============================================================================
 
	添加driver_can.h

	1. 调UARTx_Init() 将 句柄 和 用户定义的接收回调函数 拷贝至UART结构体  （回调函数中对接收到的数据进行 ID识别 和 合并解算）

  2. 用户编写 UART_RxBuffer，填入 目标缓存区地址 和 数据长度。

	3. 调用UART_Open() 传入 UART_Object 和 用户编写 UART_RxBuffer。
	
	4. 将 UART_Idle_Handler 添加到 stm32H7xx_it.c 的 USARTx_IRQHandler() 中，调用用户编写的同一个 UART_RxBuffer 。
	
	5. 应用层编写 UART_TxBuffer （发送缓存区结构体），填入待发送字节数组首地址和字节长度
	
	6. 调UART_Send()传入 UART设备结构体 和 UART_TxBuffer结构体，将数据发送出去

  ********************************************************************************
	* @attention
	* 硬件平台: STM32H750VBT \n
	* SDK版本：-++++
  * if you had modified this file, please make sure your code does not have many 
  * bugs, update the version NO., write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  ********************************************************************************
	对DMA中NDTR寄存器描述：
	This register can be written only
	when the stream is disabled. When the stream is enabled, this register is read-only,
	indicating the remaining data items to be transmitted. This register decrements after each
	DMA transfer.
	指明了DMA中待传输的剩余数据个数 每次DMA传输完成后自动减一
	参考手册中对idle空闲中断触发条件的描述：
	
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
  * @brief   串口初始化，将句柄和接收回调拷贝至串口结构体
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
  * @brief  串口设备中断函数，执行中断DMA操作，调用串口用户回调函数 
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
			else uart->RxIdleCallback(uart->uart_RxBuffer, usart_rx_num); //<用户回调
                
  
        HAL_UART_DMAResume(uart->Handle);
        HAL_UART_Receive_DMA(uart->Handle, uart->uart_RxBuffer,200);
    }
}

/**
  * @brief  整型转换成字符串ASCii码 
  */
uint8_t* itoa(int  num, uint8_t* str, int radix)
{
    uint8_t index[]="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"; //<索引表
    unsigned unum;																	  			//<存放要转换的整数的绝对值,转换的整数可能是负数
    int i=0,j,k;																					  //<i用来指示设置字符串相应位，转换之后i其实就是字符串的长度；转换后顺序是逆序的，有正负的情况，k用来指示调整顺序的开始位置;j用来指示调整顺序时的交换。
																														//<获取要转换的整数的绝对值
    if(radix==10&&num<0)																		//<要转换成十进制数并且是负数
    {
        unum=(unsigned)-num;																//<将num的绝对值赋给unum
        str[i++]='-';																				//<在字符串最前面设置为'-'号，并且索引加1
    }
    else unum=(unsigned)num;																//<若是num为正，直接赋值给unum
		
    do																							  			//<转换部分，注意转换后是逆序的
    {
        str[i++]=index[unum%(unsigned)radix];							//<取unum的最后一位，并设置为str对应位，指示索引加1
        unum/=radix;																				//<unum去掉最后一位
 
    }while(unum);																						//<直至unum为0退出循环
 
    str[i]='\0';																						//<在字符串最后添加'\0'字符，c语言字符串以'\0'结束。
 
																														//<将顺序调整过来
    if(str[0]=='-') k=1;																		//<如果是负数，符号不用调整，从符号后面开始调整
    else k=0;																								//<不是负数，全部都要调整
 
    uint8_t temp;																						//<临时变量，交换两个值时用到
    for(j=k;j<=(i-1)/2;j++)																	//<头尾一一对称交换，i其实就是字符串的长度，索引最大值比长度少1
    {
        temp=str[j];																				//<头部赋值给临时变量
        str[j]=str[i-1+k-j];																//<尾部赋值给头部
        str[i-1+k-j]=temp;																	//<将临时变量的值(其实就是之前的头部值)赋给尾部
    }
    return str;																							//<返回转换后的字符串
 
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