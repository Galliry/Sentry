#include "user_can.h"
#include "can.h"    
#include "usart.h"  

static uint8_t uart3_tx_buffer[9];

/**
 * @brief  用户 CAN 初始化函数
 * @note   包含过滤器配置、启动 CAN 和开启接收中断
 */
void User_CAN_Init(void)
{
   CAN_FilterTypeDef canFilterConfig;

    // 1. 硬件级精准拦截：配置 16位列表模式，只接收 0x101, 0x102, 0x103, 0x104
    canFilterConfig.FilterBank = 0;
    canFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;   // 列表模式（不再是掩码）
    canFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;  // 16位位宽（可以塞下4个ID）
    
    // 在 16位模式下，标准 ID 需要左移 5 位对齐到寄存器的高 11 位
    canFilterConfig.FilterIdHigh = 0x105 << 5;            // 允许 ID 1
    canFilterConfig.FilterIdLow = 0x106 << 5;             // 允许 ID 2
    canFilterConfig.FilterMaskIdHigh = 0x105 << 5;        // 允许 ID 3
    canFilterConfig.FilterMaskIdLow = 0x106 << 5;         // 允许 ID 4

    canFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    canFilterConfig.FilterActivation = ENABLE;
    canFilterConfig.SlaveStartFilterBank = 14; 

    // 应用过滤器配置
    HAL_CAN_ConfigFilter(&hcan, &canFilterConfig);

    // 2. 启动 CAN 外设
    HAL_CAN_Start(&hcan);

    // 3. 激活 CAN 的 FIFO0 接收中断
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief  重写 HAL 库的 CAN FIFO0 接收中断回调函数
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
    {
        if (rxHeader.IDE == CAN_ID_STD && rxHeader.RTR == CAN_RTR_DATA)
        {
            uart3_tx_buffer[0] = (uint8_t)(rxHeader.StdId - 0x100); 
            
            for (uint8_t i = 0; i < 8; i++)
            {
                uart3_tx_buffer[i + 1] = rxData[i];
            }
            if (HAL_UART_Transmit_DMA(&huart3, uart3_tx_buffer, 9) != HAL_OK)
            {
				
            }
        }
    }
}

