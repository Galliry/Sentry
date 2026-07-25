#include "imu.h"
#include "can.h"    // 提供 hcan (根据你实际工程的命名，可能是 hcan1)
#include "usart.h"  // 提供 huart3
#include "string.h" // 提供 memcpy

/* ================== 环形队列定义 ================== */
// 定义单包数据结构 (1字节ID + 8字节数据 = 9字节)
typedef struct {
    uint8_t data[9];
} UART_Packet_t;

// 定义环形队列长度 (最多缓存 64 帧，约600字节，完全满足高频并发需求)
#define TX_QUEUE_SIZE 64

static UART_Packet_t tx_queue[TX_QUEUE_SIZE];
static volatile uint16_t queue_head = 0; // 写指针 (CAN接收中断用)
static volatile uint16_t queue_tail = 0; // 读指针 (主循环用)

/* ================== 初始化函数 ================== */
/**
 * @brief  IMU 桥接模块初始化
 */
void IMU_Bridge_Init(void)
{
    CAN_FilterTypeDef canFilterConfig;

    // 1. 退回 32位掩码全接收模式 (总线上只有 IMU，直接全部放行)
    canFilterConfig.FilterBank = 0;
    canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;   
    canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;  
    canFilterConfig.FilterIdHigh = 0x0000;                
    canFilterConfig.FilterIdLow = 0x0000;
    canFilterConfig.FilterMaskIdHigh = 0x0000;            
    canFilterConfig.FilterMaskIdLow = 0x0000;
    canFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;  
    canFilterConfig.FilterActivation = ENABLE;            
    canFilterConfig.SlaveStartFilterBank = 14;            

    HAL_CAN_ConfigFilter(&hcan, &canFilterConfig);

    // 2. 启动 CAN
    HAL_CAN_Start(&hcan);

    // 3. 激活 CAN 接收中断
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/* ================== 中断回调 (生产者) ================== */
/**
 * @brief  重写 CAN FIFO0 接收中断回调函数
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    // 提取 CAN 报文
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
    {
        // 确保是标准数据帧
        if (rxHeader.IDE == CAN_ID_STD && rxHeader.RTR == CAN_RTR_DATA)
        {
            // [核心过滤]：只放行 0x02(角速度) 和 0x03(欧拉角)[cite: 1]
            if (rxData[0] == 0x02 || rxData[0] == 0x03)
            {
                uint16_t next_head = (queue_head + 1) % TX_QUEUE_SIZE;
                
                // 如果队列没满，存入数据
                if (next_head != queue_tail)
                {
                    // 字节0：存放 CAN ID 的低 8 位 (即 Master ID)
                    tx_queue[queue_head].data[0] = (uint8_t)(rxHeader.StdId & 0xFF);
                    
                    // 字节1~8：存放 IMU 原始数据
                    memcpy(&tx_queue[queue_head].data[1], rxData, 8);
                    
                    // 更新写指针
                    queue_head = next_head;
                }
            }
        }
    }
}

/* ================== 主循环任务 (消费者) ================== */
/**
 * @brief  处理队列并将数据通过串口发出
 * @note   在 main 函数的 while(1) 中调用
 */
void IMU_Bridge_Process(void)
{
    // 检查队列是否有数据待发
    if (queue_head != queue_tail)
    {
        // 检查串口硬件是否空闲 (必须是 READY 才能启动下一次 DMA)
        if (huart3.gState == HAL_UART_STATE_READY)
        {
            // 启动 DMA 发送 9 字节
            HAL_UART_Transmit_DMA(&huart3, tx_queue[queue_tail].data, 9);
            
            // 数据移交 DMA 后，更新读指针
            queue_tail = (queue_tail + 1) % TX_QUEUE_SIZE;
        }
    }
}

/* ================== 串口错误防护 ================== */
/**
 * @brief  重写串口错误回调函数
 * @note   防止外部干扰导致串口发生 ORE/FE 错误后永久死锁
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        // 强行终止异常状态，复位外设状态机
        HAL_UART_AbortTransmit(huart);
        HAL_UART_AbortReceive(huart);
    }
}


void User_CAN_Test_Tx(void)
{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    uint8_t testData[8] = {0xAA, 0xBB, 0xCC, 0xDD, 0x11, 0x22, 0x33, 0x44};

    // 配置报文头
    txHeader.StdId = 0x98;             // 测试用 ID
    txHeader.ExtId = 0;                 // 不使用扩展 ID
    txHeader.IDE = CAN_ID_STD;          // 标准帧
    txHeader.RTR = CAN_RTR_DATA;        // 数据帧
    txHeader.DLC = 8;                   // 发送 8 个字节
    txHeader.TransmitGlobalTime = DISABLE;

    // 检查是否有空闲的发送邮箱 (防止邮箱满导致死机)
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0)
    {
        // 尝试发送
        HAL_CAN_AddTxMessage(&hcan, &txHeader, testData, &txMailbox);
    }
}


/* ================== IMU 主动配置 (F103作为主机) ================== */

/**
 * @brief  F103 底层发送指令给 IMU
 * @param  reg_id  寄存器地址
 * @param  ac      操作类型 (0:读, 1:写)
 * @param  data    数据内容 (32位)
 */
static void IMU_Send_Cmd(uint8_t reg_id, uint8_t ac, uint32_t data)
{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    uint8_t txData[8];

    // 配置报文头：目标是 IMU 的接收 ID 0x98
    txHeader.StdId = 0x98;
    txHeader.ExtId = 0;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = 8;
    txHeader.TransmitGlobalTime = DISABLE;

    // 拼装协议头：[0xCC, 寄存器, 读写, 0xDD]
    txData[0] = 0xCC;
    txData[1] = reg_id;
    txData[2] = ac;
    txData[3] = 0xDD;
    
    // 拼装数据体：小端序
    txData[4] = (uint8_t)(data & 0xFF);
    txData[5] = (uint8_t)((data >> 8) & 0xFF);
    txData[6] = (uint8_t)((data >> 16) & 0xFF);
    txData[7] = (uint8_t)((data >> 24) & 0xFF);

    // 检查是否有空闲邮箱
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0)
    {
        HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox);
    }
}

/**
 * @brief  F103 唤醒并配置 IMU 序列
 * @note   复刻了官方代码中的 DM_IMU_Run 逻辑
 */
void IMU_Startup_Sequence(void)
{
    // 延时等待 IMU 上电稳定
    HAL_Delay(100);

    // 1. 切换通信端口为 CAN (COM_CAN = 2) [寄存器: 0x09]
    IMU_Send_Cmd(0x09, 1, 2);
    HAL_Delay(10);
    
    // 2. 设置波特率 (CAN_BAUD_1M = 0, 如果你的 F103 是 500k，这里就填 1) [寄存器: 0x0C]
    // 注意：修改波特率可能导致后续指令发不通，建议 IMU 默认和 F103 匹配即可。这里以 1M 为例。
    IMU_Send_Cmd(0x0C, 1, 0); 
    HAL_Delay(10);
    
    // 3. 设置主动发送频率间隔 (1000) [寄存器: 0x0A]
    IMU_Send_Cmd(0x0A, 1, 1000);
    HAL_Delay(10);
    
    // 4. 切换为主动发送模式 (ACTIVE = 1) [寄存器: 0x0B]
    IMU_Send_Cmd(0x0B, 1, 1);
    HAL_Delay(10);
}
