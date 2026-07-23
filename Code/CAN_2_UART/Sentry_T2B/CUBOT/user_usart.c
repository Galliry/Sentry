#include "user_usart.h"
#include "usart.h"
#include "can.h"
static uint8_t uart3_rx_buffer[9];
static uint8_t can_err_mailbox_full[9] = {0xFF, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE};
static uint8_t can_err_add_msg_fail[9] = {0xFF, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD};

void User_USART3_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart3_rx_buffer, 9);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART3)
    {
		if(Size == 9)
		{
			if(uart3_rx_buffer[0] == 1 || uart3_rx_buffer[0] == 2 || uart3_rx_buffer[0] == 3 || uart3_rx_buffer[0] == 4)
				User_USART3_Process();
		}
		
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart3_rx_buffer, 9);
    }
}

void User_USART3_Process(void)
{
        // 检查 CAN 的发送邮箱是否有空位
        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0)
        {
            CAN_TxHeaderTypeDef txHeader;
            uint32_t txMailbox;

            // 组装 CAN 报文头
            txHeader.StdId =  0x100 + uart3_rx_buffer[0]; // 第一个字节作为 ID
            txHeader.ExtId = 0;
            txHeader.IDE = CAN_ID_STD;
            txHeader.RTR = CAN_RTR_DATA;
            txHeader.DLC = 8;                    // 剩下的8个字节作为数据
            txHeader.TransmitGlobalTime = DISABLE;

            if (HAL_CAN_AddTxMessage(&hcan, &txHeader, &uart3_rx_buffer[1], &txMailbox) != HAL_OK)
            {
                HAL_UART_Transmit_DMA(&huart3, can_err_add_msg_fail, 9);
            }
        }
        else
        {
            HAL_UART_Transmit_DMA(&huart3, can_err_mailbox_full, 9);
        }
}

