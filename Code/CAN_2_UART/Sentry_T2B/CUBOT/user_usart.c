/**
 ******************************************************************************
 * @file    user_usart.c
 * @brief   UART侧用户代码 —— UART→CAN 离线重发逻辑
 * @note
 *          - UART空闲中断收到9字节帧 → Push到FIFO → 尝试CAN发送
 *          - CAN mailbox满或HAL错误 → 消息留在FIFO → main循环持续重试
 *          - CAN bus-off → 自动恢复（ABOM使能）→ 恢复后继续发送FIFO积压
 *          - 连续发送失败超阈值 → 标记CAN离线，FIFO满时丢弃最旧+UART错误通知
 ******************************************************************************
 */

#include "user_usart.h"
#include "fifo.h"
#include "usart.h"
#include "can.h"
#include "user_can.h"   /* uart_tx_dma_busy 跨模块共享 */

/* ------------------------- 静态变量 --------------------------- */

/* UART→CAN方向消息FIFO */
static MsgFIFO_t fifo_uart2can;

/* UART DMA RX 缓冲区 */
static uint8_t uart3_rx_buffer[FIFO_MSG_SIZE];

/* CAN TX错误反馈包（预定义，DMA传输期间不可变） */
static const uint8_t can_err_overflow[9]   = {0xFF, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE};
static const uint8_t can_err_tx_fail[9]    = {0xFF, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD};

/* CAN bus-off恢复相关 */
static volatile uint8_t  can_was_busoff  = 0;  /* 标记是否曾检测到bus-off */

/* ------------------------- 初始化 ----------------------------- */

void User_USART3_Init(void)
{
    FIFO_Init(&fifo_uart2can);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart3_rx_buffer, FIFO_MSG_SIZE);
}

/* ------------------ UART接收回调 ------------------------------ */

/**
 * @brief  HAL回调：UART空闲中断（一帧接收完成）
 * @note   在 USART3_IRQHandler 中断上下文中调用（优先级1）
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART3)
    {
        if (Size == FIFO_MSG_SIZE)
        {
            /* 只处理通道ID 1~4 的消息（UART→CAN方向） */
            if (uart3_rx_buffer[0] == 1U || uart3_rx_buffer[0] == 2U ||
                uart3_rx_buffer[0] == 3U || uart3_rx_buffer[0] == 4U)
            {
                User_USART3_Process();
            }
        }

        /* 重新使能DMA接收（准备下一帧） */
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart3_rx_buffer, FIFO_MSG_SIZE);
    }
}

/* ------------------ UART→CAN 数据处理 ------------------------- */

/**
 * @brief  处理收到的UART帧：推入FIFO并尝试CAN发送
 * @note   此函数原始版本直接调用HAL_CAN_AddTxMessage，
 *         改为FIFO缓冲后，在此Push并触发TryTransmit
 */
void User_USART3_Process(void)
{
    /* 推入UART→CAN FIFO */
    if (!FIFO_Push(&fifo_uart2can, uart3_rx_buffer))
    {
        /* FIFO已满 → 溢出错误通知（仅当UART DMA空闲时发送） */
        if (!uart_tx_dma_busy)
        {
            if (HAL_UART_Transmit_DMA(&huart3, (uint8_t *)can_err_overflow, 9) == HAL_OK)
            {
                uart_tx_dma_busy = 1;  /* 标记DMA忙，防止CAN2UART抢占 */
            }
        }
    }

    /* 尝试立即发送 */
    UART2CAN_TryTransmit();
}

/* ------------------ UART→CAN 发送调度 ------------------------- */

/**
 * @brief  UART→CAN TryTransmit
 * @note   从main循环和UART RX回调中调用
 *         1. 检测CAN bus-off并尝试恢复
 *         2. 检测离线标记并判断是否到了重试时间
 *         3. 检查mailbox空闲数，逐条发送FIFO中的消息
 *         4. 记录成功/失败，更新离线状态
 */
void UART2CAN_TryTransmit(void)
{
    /* ---- 1. CAN bus-off 检测 ---- */
    /* 直接读ESR寄存器（不依赖错误中断使能） */
    if (hcan.Instance->ESR & CAN_ESR_BOFF)
    {
        if (!can_was_busoff)
        {
            can_was_busoff  = 1;
        }
        /* ABOM已使能（CubeMX配置CAN_ABOM=ENABLE），
         * 硬件会在检测到128次11个隐性位后自动恢复。
         * 此处不手动Stop/Start，避免干扰自动恢复流程。 */
        return;  /* bus-off期间不尝试发送 */
    }
    else
    {
        /* CAN正常，清除bus-off标记 */
        if (can_was_busoff)
        {
            can_was_busoff = 0;
            /* CAN已恢复，重置失败计数，开始发送积压数据 */
            FIFO_ResetFailCnt(&fifo_uart2can);
        }
    }

    /* ---- 2. 离线重试间隔控制 ---- */
    if (FIFO_CheckOffline(&fifo_uart2can, CAN_BUSOFF_RECOVER_MS))
    {
        /* 仍在离线等待期，跳过本次重试 */
        return;
    }

    /* ---- 3. 发送FIFO中所有消息（直到mailbox满或FIFO空） ---- */
    while (!FIFO_IsEmpty(&fifo_uart2can))
    {
        /* 检查CAN mailbox空闲数 */
        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0U)
        {
            /* mailbox全满，下次重试 */
            FIFO_IncFailCnt(&fifo_uart2can);
            break;
        }

        /* 取一条消息 */
        uint8_t msg[FIFO_MSG_SIZE];
        if (!FIFO_Pop(&fifo_uart2can, msg))
        {
            break;  /* 理论上不会发生（刚检查过非空），防御性编程 */
        }

        /* 构造CAN帧头 */
        CAN_TxHeaderTypeDef txHeader;
        txHeader.StdId              = 0x100U + msg[0];   /* 通道ID → CAN ID */
        txHeader.ExtId              = 0;
        txHeader.IDE                = CAN_ID_STD;
        txHeader.RTR                = CAN_RTR_DATA;
        txHeader.DLC                = 8;
        txHeader.TransmitGlobalTime = DISABLE;

        uint32_t txMailbox;
        if (HAL_CAN_AddTxMessage(&hcan, &txHeader, &msg[1], &txMailbox) == HAL_OK)
        {
            /* 发送成功：重置连续失败计数，标记在线 */
            FIFO_ResetFailCnt(&fifo_uart2can);
        }
        else
        {
            /* HAL发送失败 → 消息无法回退到FIFO（已Pop），只能丢弃 */
            /* 尝试通过UART发送错误通知（仅当DMA空闲时） */
            if (!uart_tx_dma_busy)
            {
                if (HAL_UART_Transmit_DMA(&huart3, (uint8_t *)can_err_tx_fail, 9) == HAL_OK)
                {
                    uart_tx_dma_busy = 1;
                }
            }
            FIFO_IncFailCnt(&fifo_uart2can);
            break;  /* 遇到硬错误，停止继续尝试 */
        }
    }
}

/* ------------------ 状态查询接口 ----------------------------- */

/**
 * @brief  获取UART→CAN方向FIFO中缓冲的消息数
 */
uint16_t UART2CAN_GetBufferedCount(void)
{
    return FIFO_Count(&fifo_uart2can);
}

/**
 * @brief  获取UART→CAN方向溢出计数
 */
uint16_t UART2CAN_GetOverflowCnt(void)
{
    return FIFO_OverflowCnt(&fifo_uart2can);
}

/**
 * @brief  查询UART→CAN方向是否离线
 */
uint8_t UART2CAN_IsOffline(void)
{
    return fifo_uart2can.is_offline;
}
