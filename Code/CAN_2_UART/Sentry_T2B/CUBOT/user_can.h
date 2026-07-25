#ifndef __USER_CAN_H__
#define __USER_CAN_H__
#include "main.h"

void User_CAN_Init(void);

/* CAN→UART方向：从FIFO取一条消息，通过UART DMA发送 */
void CAN2UART_TryTransmit(void);

/* UART DMA TX 忙标志（跨模块共享，user_usart.c发送错误通知前需检查） */
extern volatile uint8_t uart_tx_dma_busy;

/* 获取CAN→UART方向FIFO状态（调试/监控用） */
uint16_t CAN2UART_GetBufferedCount(void);
uint16_t CAN2UART_GetOverflowCnt(void);
uint8_t  CAN2UART_IsOffline(void);

#endif
