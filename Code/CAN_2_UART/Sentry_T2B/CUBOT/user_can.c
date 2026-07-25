///**
// ******************************************************************************
// * @file    user_can.c
// * @brief   CAN侧用户代码 —— CAN→UART 离线重发逻辑
// * @note
// *          - CAN消息到达 → Push到FIFO → 尝试UART DMA发送
// *          - DMA发送完成回调 → Pop下一条 → 链式发送直到FIFO空
// *          - main循环中CAN2UART_TryTransmit()作为备用重发入口
// ******************************************************************************
// */

//#include "user_can.h"
//#include "fifo.h"
//#include "can.h"
//#include "usart.h"

///* ------------------------- 静态变量 --------------------------- */

///* CAN→UART方向消息FIFO */
//static MsgFIFO_t fifo_can2uart;

///* UART DMA TX 缓冲区（须为静态或全局，DMA传输期间不可释放） */
//static uint8_t uart3_tx_buffer[FIFO_MSG_SIZE];

///* UART DMA TX 忙标志（extern声明在user_can.h，供user_usart.c检查） */
//volatile uint8_t uart_tx_dma_busy = 0;

///* ------------------------- 初始化 ----------------------------- */

///**
// * @brief  用户CAN初始化
// * @note   配置CAN过滤器、启动CAN、使能FIFO0接收中断
// */
//void User_CAN_Init(void)
//{
//    CAN_FilterTypeDef canFilterConfig;

//    /* 初始化CAN→UART FIFO */
//    FIFO_Init(&fifo_can2uart);

//    /* 硬件过滤器：16位列表模式，只接收 0x105, 0x106 */
//    canFilterConfig.FilterBank = 0;
//    canFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
//    canFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;

//    canFilterConfig.FilterIdHigh      = 0x105U << 5;
//    canFilterConfig.FilterIdLow       = 0x106U << 5;
//    canFilterConfig.FilterMaskIdHigh  = 0x105U << 5;
//    canFilterConfig.FilterMaskIdLow   = 0x106U << 5;

//    canFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
//    canFilterConfig.FilterActivation     = ENABLE;
//    canFilterConfig.SlaveStartFilterBank = 14;

//    HAL_CAN_ConfigFilter(&hcan, &canFilterConfig);

//    /* 启动CAN */
//    HAL_CAN_Start(&hcan);

//    /* 使能CAN FIFO0消息挂起中断 */
//    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
//}

///* ------------------ CAN接收回调 ------------------------------- */

///**
// * @brief  HAL回调：CAN FIFO0收到消息
// * @note   在 USB_LP_CAN1_RX0_IRQHandler 中断上下文中调用
// */
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//    CAN_RxHeaderTypeDef rxHeader;
//    uint8_t rxData[8];

//    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
//    {
//        if (rxHeader.IDE == CAN_ID_STD && rxHeader.RTR == CAN_RTR_DATA)
//        {
//            uint8_t msg[FIFO_MSG_SIZE];

//            /* 组装9字节消息：通道ID + 8字节CAN数据 */
//            msg[0] = (uint8_t)(rxHeader.StdId - 0x100U);
//            for (uint8_t i = 0; i < 8U; i++)
//            {
//                msg[i + 1U] = rxData[i];
//            }

//            /* 压入FIFO（满时自动丢弃最旧，溢出计数+1） */
//            FIFO_Push(&fifo_can2uart, msg);

//            /* 立即尝试发送 */
//            CAN2UART_TryTransmit();
//        }
//    }
//}

///* ------------------ UART DMA TX 完成回调 ---------------------- */

///**
// * @brief  HAL回调：UART DMA发送完成
// * @note   在 DMA1_Channel2_IRQHandler 中断上下文中调用（优先级0，最高）
// *         发送完成后自动Pop下一条消息继续传输（链式发送）
// */
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if (huart->Instance == USART3)
//    {
//        /* DMA传输结束，清除忙标志 */
//        uart_tx_dma_busy = 0;

//        /* 链式发送：继续处理FIFO中排队的消息 */
//        CAN2UART_TryTransmit();
//    }
//}

///* ------------------ CAN→UART 发送调度 ------------------------ */

///**
// * @brief  CAN→UART TryTransmit
// * @note   从中断和main循环均可调用，内部通过DMA忙标志防止竞争
// *         优先级：DMA TX完成回调(0) > CAN RX回调(2) > main循环
// *         如果DMA忙，消息留在FIFO中，由DMA完成回调自动触发链式发送
// */
//void CAN2UART_TryTransmit(void)
//{
//    /* 如果DMA正在传输中，等待完成回调触发链式发送 */
//    if (uart_tx_dma_busy)
//    {
//        return;
//    }

//    /* 从FIFO取一条消息 */
//    if (FIFO_Pop(&fifo_can2uart, uart3_tx_buffer))
//    {
//        /* 启动DMA传输 */
//        if (HAL_UART_Transmit_DMA(&huart3, uart3_tx_buffer, FIFO_MSG_SIZE) == HAL_OK)
//        {
//            uart_tx_dma_busy = 1;
//        }
//        else
//        {
//            /* DMA启动失败（异常情况），消息已从FIFO取出，
//             * 只能丢弃。此情况理论上极少发生 */
//        }
//    }
//    /* FIFO为空，无需操作 */
//}

///* ------------------ 状态查询接口 ----------------------------- */

///**
// * @brief  获取CAN→UART方向FIFO中缓冲的消息数
// */
//uint16_t CAN2UART_GetBufferedCount(void)
//{
//    return FIFO_Count(&fifo_can2uart);
//}

///**
// * @brief  获取CAN→UART方向溢出计数
// */
//uint16_t CAN2UART_GetOverflowCnt(void)
//{
//    return FIFO_OverflowCnt(&fifo_can2uart);
//}

///**
// * @brief  查询CAN→UART方向是否离线
// */
//uint8_t CAN2UART_IsOffline(void)
//{
//    return FIFO_IsFull(&fifo_can2uart) ? 1U : 0U;
//    /* CAN→UART方向无真正的"离线"概念（UART TX不等待ACK），
//     * FIFO满意味着消费速度落后于生产速度，可视为逻辑离线 */
//}
