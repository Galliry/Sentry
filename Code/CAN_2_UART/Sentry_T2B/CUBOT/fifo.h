///**
// ******************************************************************************
// * @file    fifo.h
// * @brief   消息FIFO循环缓冲区 —— 用于CAN<->UART桥接的离线缓存与重发
// * @note    每条消息固定9字节，容量16条（可配置）
// ******************************************************************************
// */

//#ifndef __FIFO_H__
//#define __FIFO_H__

//#include "stdint.h"

///* ------------------------- 可配置参数 ------------------------- */

//#define FIFO_MSG_SIZE   9U          /* 每条消息固定9字节 */
//#define FIFO_CAPACITY   16U         /* FIFO最大容量，须为2的幂 */

///* ---------------- UART→CAN 离线检测阈值 ---------------------- */
//#define CAN_OFFLINE_THRESHOLD   8U  /* 连续发送失败超过此次数，标记CAN离线 */
//#define CAN_BUSOFF_RECOVER_MS   100U /* bus-off后等待恢复时间(ms) */

///* ------------------------- 数据结构 --------------------------- */

///**
// * @brief 消息FIFO控制块
// * @note  head/tail单调递增，取模得实际下标；count = head - tail
// */
//typedef struct {
//    uint8_t  buffer[FIFO_CAPACITY][FIFO_MSG_SIZE]; /* 数据缓冲区 */
//    volatile uint16_t head;       /* 写入索引（递增不归零） */
//    volatile uint16_t tail;       /* 读取索引（递增不归零） */
//    volatile uint16_t overflow_cnt;  /* FIFO满时丢弃计数 */
//    volatile uint16_t fail_cnt;      /* 连续发送失败计数 */
//    volatile uint8_t  is_offline;    /* 硬件离线标志（1=离线） */
//    volatile uint32_t offline_since; /* 进入离线时的tick（用于恢复等待） */
//} MsgFIFO_t;

///* ------------------------- API 函数 --------------------------- */

//void    FIFO_Init       (MsgFIFO_t *fifo);
//uint8_t FIFO_Push       (MsgFIFO_t *fifo, const uint8_t *data);
//uint8_t FIFO_Pop        (MsgFIFO_t *fifo, uint8_t *data_out);
//uint8_t FIFO_IsEmpty    (MsgFIFO_t *fifo);
//uint8_t FIFO_IsFull     (MsgFIFO_t *fifo);
//uint16_t FIFO_Count     (MsgFIFO_t *fifo);
//uint16_t FIFO_OverflowCnt(MsgFIFO_t *fifo);
//void    FIFO_ClearOverflow(MsgFIFO_t *fifo);

///* 离线/失败状态管理 */
//void    FIFO_ResetFailCnt (MsgFIFO_t *fifo);
//void    FIFO_IncFailCnt   (MsgFIFO_t *fifo);
//uint8_t FIFO_CheckOffline (MsgFIFO_t *fifo, uint32_t recover_timeout_ms);

//#endif /* __FIFO_H__ */
