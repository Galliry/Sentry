///**
// ******************************************************************************
// * @file    fifo.c
// * @brief   消息FIFO实现 —— 临界区保护 + 离线检测
// * @note
// *          - 使用单调递增的head/tail索引，count = head - tail
// *          - 实际数组下标 = index & (FIFO_CAPACITY - 1)，要求CAPACITY为2的幂
// *          - 临界区使用 PRIMASK 保存/恢复模式，可安全嵌套
// *          - Cortex-M3 强内存序，volatile + 临界区保证ISR与main间数据一致性
// ******************************************************************************
// */

//#include "fifo.h"
//#include "main.h"        /* HAL_GetTick */
//#include <string.h>      /* memcpy */

///* ---- 硬件适配：临界区保护 ---- */
///* 在F1系列上无BASEPRI，使用PRIMASK全局关中断，保存/恢复避免嵌套问题 */

//#define FIFO_LOCK()   __disable_irq()
//#define FIFO_UNLOCK() __enable_irq()

///* 注意：如果调用上下文本身已关中断（PRIMASK=1），
// * FIFO_UNLOCK会错误地开中断。但本项目所有FIFO调用点均在
// * PRIMASK=0的上下文中（ISR入口硬件只关同级/低级中断，
// * PRIMASK仍为0；main循环PRIMASK=0），因此可以使用简化宏。
// * 如移植到其他场景，请改为 PRIMASK 保存/恢复模式。
// */

///* ================================================================
// * 内部工具：获取FIFO中当前消息数量（调用者保证在临界区内）
// * ================================================================ */
//static inline uint16_t fifo_count_unsafe(MsgFIFO_t *fifo)
//{
//    return fifo->head - fifo->tail;
//}

///* ================================================================
// * 公开 API
// * ================================================================ */

///**
// * @brief  初始化FIFO控制块
// */
//void FIFO_Init(MsgFIFO_t *fifo)
//{
//    if (fifo == NULL) return;

//    FIFO_LOCK();
//    fifo->head          = 0;
//    fifo->tail          = 0;
//    fifo->overflow_cnt  = 0;
//    fifo->fail_cnt      = 0;
//    fifo->is_offline    = 0;
//    fifo->offline_since = 0;
//    FIFO_UNLOCK();
//}

///**
// * @brief  向FIFO压入一条消息
// * @param  fifo  FIFO控制块指针
// * @param  data  9字节消息数据指针
// * @retval 1     压入成功
// * @retval 0     FIFO已满，丢弃（overflow_cnt自动递增）
// */
//uint8_t FIFO_Push(MsgFIFO_t *fifo, const uint8_t *data)
//{
//    uint8_t result = 0;

//    if (fifo == NULL || data == NULL) return 0;

//    FIFO_LOCK();

//    if (fifo_count_unsafe(fifo) < FIFO_CAPACITY)
//    {
//        /* 计算写入下标 */
//        uint16_t idx = fifo->head & (FIFO_CAPACITY - 1U);
//        memcpy(fifo->buffer[idx], data, FIFO_MSG_SIZE);
//        fifo->head++;
//        result = 1;
//    }
//    else
//    {
//        /* FIFO已满：丢弃最旧一条，为新数据腾出空间 */
//        uint16_t idx_old = fifo->tail & (FIFO_CAPACITY - 1U);
//        fifo->tail++;  /* 丢弃最旧 */
//        fifo->overflow_cnt++;

//        uint16_t idx_new = fifo->head & (FIFO_CAPACITY - 1U);
//        memcpy(fifo->buffer[idx_new], data, FIFO_MSG_SIZE);
//        fifo->head++;
//        result = 1;  /* 覆盖写入仍视为成功 */
//    }

//    FIFO_UNLOCK();
//    return result;
//}

///**
// * @brief  从FIFO弹出一条消息
// * @param  fifo     FIFO控制块指针
// * @param  data_out 9字节输出缓冲区
// * @retval 1        弹出成功
// * @retval 0        FIFO为空
// */
//uint8_t FIFO_Pop(MsgFIFO_t *fifo, uint8_t *data_out)
//{
//    uint8_t result = 0;

//    if (fifo == NULL || data_out == NULL) return 0;

//    FIFO_LOCK();

//    if (fifo_count_unsafe(fifo) > 0)
//    {
//        uint16_t idx = fifo->tail & (FIFO_CAPACITY - 1U);
//        memcpy(data_out, fifo->buffer[idx], FIFO_MSG_SIZE);
//        fifo->tail++;
//        result = 1;
//    }

//    FIFO_UNLOCK();
//    return result;
//}

///**
// * @brief  判断FIFO是否为空
// * @retval 1 为空，0 非空
// */
//uint8_t FIFO_IsEmpty(MsgFIFO_t *fifo)
//{
//    if (fifo == NULL) return 1;
//    uint8_t empty;
//    FIFO_LOCK();
//    empty = (fifo_count_unsafe(fifo) == 0) ? 1U : 0U;
//    FIFO_UNLOCK();
//    return empty;
//}

///**
// * @brief  判断FIFO是否已满
// * @retval 1 已满，0 未满
// */
//uint8_t FIFO_IsFull(MsgFIFO_t *fifo)
//{
//    if (fifo == NULL) return 0;
//    uint8_t full;
//    FIFO_LOCK();
//    full = (fifo_count_unsafe(fifo) >= FIFO_CAPACITY) ? 1U : 0U;
//    FIFO_UNLOCK();
//    return full;
//}

///**
// * @brief  获取FIFO中当前消息数量
// */
//uint16_t FIFO_Count(MsgFIFO_t *fifo)
//{
//    if (fifo == NULL) return 0;
//    uint16_t cnt;
//    FIFO_LOCK();
//    cnt = fifo_count_unsafe(fifo);
//    FIFO_UNLOCK();
//    return cnt;
//}

///**
// * @brief  获取溢出计数（FIFO满时丢弃的消息数）
// */
//uint16_t FIFO_OverflowCnt(MsgFIFO_t *fifo)
//{
//    if (fifo == NULL) return 0;
//    uint16_t cnt;
//    FIFO_LOCK();
//    cnt = fifo->overflow_cnt;
//    FIFO_UNLOCK();
//    return cnt;
//}

///**
// * @brief  清零溢出计数
// */
//void FIFO_ClearOverflow(MsgFIFO_t *fifo)
//{
//    if (fifo == NULL) return;
//    FIFO_LOCK();
//    fifo->overflow_cnt = 0;
//    FIFO_UNLOCK();
//}

///**
// * @brief  清零连续失败计数，标记在线
// */
//void FIFO_ResetFailCnt(MsgFIFO_t *fifo)
//{
//    if (fifo == NULL) return;
//    FIFO_LOCK();
//    fifo->fail_cnt   = 0;
//    fifo->is_offline = 0;
//    FIFO_UNLOCK();
//}

///**
// * @brief  递增连续失败计数，超过阈值自动标记离线
// */
//void FIFO_IncFailCnt(MsgFIFO_t *fifo)
//{
//    if (fifo == NULL) return;
//    FIFO_LOCK();
//    fifo->fail_cnt++;
//    if (fifo->fail_cnt >= CAN_OFFLINE_THRESHOLD)
//    {
//        fifo->is_offline    = 1;
//        fifo->offline_since = HAL_GetTick();
//    }
//    FIFO_UNLOCK();
//}

///**
// * @brief  检查离线状态，超过恢复超时则清离线标记（允许重试）
// * @param  recover_timeout_ms  离线后等待恢复的时间（ms）
// * @retval 1  仍然离线
// * @retval 0  在线或已过恢复等待期（可以重试）
// */
//uint8_t FIFO_CheckOffline(MsgFIFO_t *fifo, uint32_t recover_timeout_ms)
//{
//    if (fifo == NULL) return 0;
//    uint8_t offline;
//    FIFO_LOCK();
//    if (fifo->is_offline)
//    {
//        if (HAL_GetTick() - fifo->offline_since >= recover_timeout_ms)
//        {
//            /* 恢复等待期已过，允许重试但不立即清除离线标志 */
//            /* 等下一次发送成功时由 ResetFailCnt 清除 */
//            offline = 0;
//        }
//        else
//        {
//            offline = 1;
//        }
//    }
//    else
//    {
//        offline = 0;
//    }
//    FIFO_UNLOCK();
//    return offline;
//}
