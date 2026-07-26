/**
 ******************************************************************************
 * @file    imu.c
 * @brief   IMU CAN-UART bridge with cold-boot recovery and error handling
 * @note
 *          - CAN RX ISR → ring queue → UART DMA TX (chain-send)
 *          - CAN error callback: bus-off / error-passive auto-recovery
 *          - UART error callback: abort + resume (no permanent stall)
 *          - IMU startup sequence: CAN pre-check, extended delay, retry rounds
 ******************************************************************************
 */

#include "imu.h"
#include "can.h"
#include "usart.h"
#include "string.h"

/* ================== Ring Buffer ================== */

/* 1-byte ID + 8-byte payload = 9-byte frame */
typedef struct {
    uint8_t data[9];
} UART_Packet_t;

/* 64-frame deep (~600 bytes), sufficient for transient bursts */
#define TX_QUEUE_SIZE 64

static UART_Packet_t tx_queue[TX_QUEUE_SIZE];
static volatile uint16_t queue_head = 0; /* Write index (CAN RX ISR)        */
static volatile uint16_t queue_tail = 0; /* Read index  (main loop / TX ISR) */

/* ---- Shared state flags (ISR + main loop) ---- */
static volatile uint8_t uart_tx_dma_busy = 0; /* UART DMA TX in progress    */
static volatile uint8_t imu_can_offline = 0;  /* CAN bus error / bus-off    */

/* ---- Retry configuration ---- */
#define IMU_CMD_RETRY_MAX   3   /* Retries per command                      */
#define IMU_CMD_RETRY_DELAY 5   /* ms between retries                      */
#define IMU_STARTUP_ROUNDS  2   /* Full-sequence retries before giving up  */

/* ================== Initialization ================== */

/**
 * @brief  Initialize IMU bridge: CAN full-mask filter, start CAN, enable IRQ
 */
void IMU_Bridge_Init(void)
{
    CAN_FilterTypeDef canFilterConfig;

    /* 32-bit mask mode — accept ALL frames (only IMU on this CAN bus) */
    canFilterConfig.FilterBank = 0;
    canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    canFilterConfig.FilterIdHigh = 0x0000;
    canFilterConfig.FilterIdLow  = 0x0000;
    canFilterConfig.FilterMaskIdHigh = 0x0000;
    canFilterConfig.FilterMaskIdLow  = 0x0000;
    canFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    canFilterConfig.FilterActivation     = ENABLE;
    canFilterConfig.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(&hcan, &canFilterConfig);

    /* Start CAN peripheral and enable RX FIFO0 pending interrupt */
    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/* ================== CAN Error Callback (ISR context) ================== */

/**
 * @brief  CAN error ISR — detect bus-off / severe error-passive and reset
 * @note   ABOM is enabled (CubeMX), so hardware auto-recovers from bus-off
 *         after 128×11 recessive bits. This callback resets HAL state and
 *         re-enables interrupts once the controller is operational again.
 *         Called from USB_LP_CAN1_RX0_IRQHandler (priority 2).
 */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance != CAN1) return;

    uint32_t esr = hcan->Instance->ESR;

    if (esr & CAN_ESR_BOFF)
    {
        /* Bus-Off: fully disconnected. Reset controller. */
        imu_can_offline = 1;
        HAL_CAN_Stop(hcan);
        HAL_CAN_Start(hcan);
        HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
    else if (esr & CAN_ESR_EPVF)
    {
        /* Error-Passive: TEC >= 128. If near bus-off threshold, reset. */
        uint32_t tec = (esr >> CAN_ESR_TEC_Pos) & 0xFFU;
        if (tec >= 192)
        {
            imu_can_offline = 1;
            HAL_CAN_Stop(hcan);
            HAL_CAN_Start(hcan);
            HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
        }
    }
}

/* ================== CAN RX Interrupt Callback (ISR context) ================== */

/**
 * @brief  CAN FIFO0 message pending — filter IMU data and push to ring queue
 * @note   Called from USB_LP_CAN1_RX0_IRQHandler (priority 2).
 *         Only forwards type 0x02 (accel) and 0x03 (euler) frames.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
    {
        if (rxHeader.IDE == CAN_ID_STD && rxHeader.RTR == CAN_RTR_DATA)
        {
            /* Only forward recognized IMU data types */
            if (rxData[0] == 0x02 || rxData[0] == 0x03)
            {
                uint16_t next_head = (queue_head + 1) % TX_QUEUE_SIZE;

                /* Drop if queue full (head catches tail) */
                if (next_head != queue_tail)
                {
                    /* Byte0 = CAN StdId low byte (Master ID) */
                    tx_queue[queue_head].data[0] = (uint8_t)(rxHeader.StdId & 0xFFU);
                    /* Bytes 1~8 = raw IMU payload */
                    memcpy(&tx_queue[queue_head].data[1], rxData, 8);
                    queue_head = next_head;
                }
            }
        }
    }
}

/* ================== UART DMA TX Complete Callback (ISR context) ================== */

/**
 * @brief  DMA TX complete — chain-send next queued frame automatically
 * @note   Called from DMA1_Channel2_IRQHandler (priority 0 — highest).
 *         Chain-sending from ISR minimizes inter-frame gap vs. main-loop polling.
 *         The busy flag prevents the main loop from racing with this ISR.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        uart_tx_dma_busy = 0;

        /* Chain: if more data queued, start next DMA immediately */
        if (queue_head != queue_tail)
        {
            if (HAL_UART_Transmit_DMA(huart, tx_queue[queue_tail].data, 9) == HAL_OK)
            {
                uart_tx_dma_busy = 1;
                queue_tail = (queue_tail + 1) % TX_QUEUE_SIZE;
            }
            /* On failure: leave busy=0; main loop will retry */
        }
    }
}

/* ================== Main Loop: UART TX Polling (safety net) ================== */

/**
 * @brief  Process queued IMU frames: send via UART DMA if idle
 * @note   Called from main() while(1). Serves as safety net if DMA chain
 *         breaks (e.g. TX complete ISR missed or DMA error).
 *         Skips TX entirely when CAN is offline (data would pile up uselessly).
 */
void IMU_Bridge_Process(void)
{
    if (imu_can_offline)
    {
        return;  /* CAN down — don't forward stale data */
    }

    if (queue_head != queue_tail)
    {
        /* Only start if DMA idle AND UART peripheral is ready.
         * gState check is redundant with busy flag but provides defense
         * against HAL state corruption after error recovery. */
        if (!uart_tx_dma_busy && huart3.gState == HAL_UART_STATE_READY)
        {
            if (HAL_UART_Transmit_DMA(&huart3, tx_queue[queue_tail].data, 9) == HAL_OK)
            {
                uart_tx_dma_busy = 1;
                queue_tail = (queue_tail + 1) % TX_QUEUE_SIZE;
            }
        }
    }
}

/* ================== UART Error Callback (ISR context) ================== */

/**
 * @brief  UART error handler — abort and let gState recover to READY
 * @note   Called from USART3_IRQHandler (priority 1).
 *         After Abort, gState returns to HAL_UART_STATE_READY,
 *         so the next IMU_Bridge_Process() call can resume transmission.
 *         The current frame is lost (tail already advanced), which is
 *         acceptable for a real-time bridge.
 *
 *         WARNING: Do NOT call HAL_Delay here — ISR context with priority 1
 *         cannot be interrupted by SysTick (priority 15).
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        /* Abort current TX/RX — resets gState to READY */
        HAL_UART_AbortTransmit(huart);
        HAL_UART_AbortReceive(huart);

        /* Clear DMA busy flag so main loop can restart transmission */
        uart_tx_dma_busy = 0;
    }
}

/* ================== CAN Test TX (debug) ================== */

/**
 * @brief  Send a test CAN frame (ID 0x98, 8-byte pattern)
 * @note   Debug helper; not used in normal operation.
 */
void User_CAN_Test_Tx(void)
{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    uint8_t testData[8] = {0xAA, 0xBB, 0xCC, 0xDD, 0x11, 0x22, 0x33, 0x44};

    txHeader.StdId              = 0x98;
    txHeader.ExtId              = 0;
    txHeader.IDE                = CAN_ID_STD;
    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.DLC                = 8;
    txHeader.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0)
    {
        HAL_CAN_AddTxMessage(&hcan, &txHeader, testData, &txMailbox);
    }
}

/* ================== IMU CAN Command (with retry) ================== */

/**
 * @brief  Send a configuration command to IMU via CAN with retry logic
 * @param  reg_id  IMU register address
 * @param  ac      Access code (0: read, 1: write)
 * @param  data    32-bit payload (little-endian)
 * @retval 1       Command accepted into CAN mailbox
 * @retval 0       Failed after all retries (CAN marked offline)
 * @note   Called from main context (not ISR) — HAL_Delay is safe.
 */
static uint8_t IMU_Send_Cmd(uint8_t reg_id, uint8_t ac, uint32_t data)
{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    uint8_t  txData[8];
    uint8_t  retry;

    /* Build CAN header: target IMU at StdId 0x98 */
    txHeader.StdId              = 0x98;
    txHeader.ExtId              = 0;
    txHeader.IDE                = CAN_ID_STD;
    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.DLC                = 8;
    txHeader.TransmitGlobalTime = DISABLE;

    /* Assemble protocol header + payload */
    txData[0] = 0xCC;
    txData[1] = reg_id;
    txData[2] = ac;
    txData[3] = 0xDD;
    txData[4] = (uint8_t)(data & 0xFFU);
    txData[5] = (uint8_t)((data >> 8)  & 0xFFU);
    txData[6] = (uint8_t)((data >> 16) & 0xFFU);
    txData[7] = (uint8_t)((data >> 24) & 0xFFU);

    for (retry = 0; retry < IMU_CMD_RETRY_MAX; retry++)
    {
        /* ---- Pre-flight: check CAN controller health ---- */
        uint32_t esr = hcan.Instance->ESR;

        if (esr & CAN_ESR_BOFF)
        {
            /* Bus-off — reset controller before attempting send */
            HAL_CAN_Stop(&hcan);
            HAL_CAN_Start(&hcan);
            HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
            HAL_Delay(10);  /* Let controller stabilize */
        }

        /* ---- Attempt send ---- */
        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0)
        {
            if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox) == HAL_OK)
            {
                /* Accepted into mailbox — CAN controller is functional */
                return 1;
            }
        }

        /* Wait before retry */
        HAL_Delay(IMU_CMD_RETRY_DELAY);
    }

    /* All retries exhausted */
    imu_can_offline = 1;
    return 0;
}

/* ================== IMU Startup Sequence ================== */

/**
 * @brief  Power-on IMU handshake: configure via CAN with error resilience
 * @note   Handles cold-boot where IMU may not be ready on the CAN bus yet:
 *         1. Pre-check CAN error counters, reset if unhealthy
 *         2. Extended 300ms delay for IMU boot (DM series needs ~200ms)
 *         3. Each command retried up to 3× with 5ms gap
 *         4. 2 consecutive failures → restart full sequence (max 2 rounds)
 *         5. All rounds exhausted → mark CAN offline, main loop skips TX
 */
void IMU_Startup_Sequence(void)
{
    uint8_t round;

    /* ======== Phase 0: CAN controller health check ======== */
    uint32_t esr = hcan.Instance->ESR;
    uint32_t tec = (esr >> CAN_ESR_TEC_Pos) & 0xFFU;

    if ((esr & CAN_ESR_BOFF) || tec >= 96)
    {
        /* CAN controller in bad state (possibly from previous run or
         * ESD event) — full reset before attempting IMU config. */
        HAL_CAN_Stop(&hcan);
        HAL_CAN_Start(&hcan);
        HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
        imu_can_offline = 0;  /* Fresh start; prove otherwise on failure */
        HAL_Delay(10);
    }

    /* ======== Phase 1: IMU power-up stabilization ======== */
    /* Extended from 100ms to 300ms:
     * - DM-series IMU has its own MCU boot (~80ms) + sensor init (~120ms)
     * - F103 boots fastest; in cold-start, IMU may still be initializing */
    HAL_Delay(300);

    /* ======== Phase 2: Configuration with retry rounds ======== */
    for (round = 0; round < IMU_STARTUP_ROUNDS; round++)
    {
        uint8_t fail_count = 0;
        uint8_t ok;

        /* 1. Switch comm port to CAN: COM_CAN = 2  [Reg 0x09] */
        ok = IMU_Send_Cmd(0x09, 1, 2);
        HAL_Delay(10);
        fail_count = ok ? 0 : (fail_count + 1);
        if (fail_count >= 2) goto startup_retry;

        /* 2. Set baud rate: CAN_BAUD_1M = 0  [Reg 0x0C] */
        ok = IMU_Send_Cmd(0x0C, 1, 0);
        HAL_Delay(10);
        fail_count = ok ? 0 : (fail_count + 1);
        if (fail_count >= 2) goto startup_retry;

        /* 3. Set auto-report rate: 1000 Hz  [Reg 0x0A] */
        ok = IMU_Send_Cmd(0x0A, 1, 1000);
        HAL_Delay(10);
        fail_count = ok ? 0 : (fail_count + 1);
        if (fail_count >= 2) goto startup_retry;

        /* 4. Switch to active report mode: ACTIVE = 1  [Reg 0x0B] */
        ok = IMU_Send_Cmd(0x0B, 1, 1);
        HAL_Delay(10);
        fail_count = ok ? 0 : (fail_count + 1);
        if (fail_count >= 2) goto startup_retry;

        /* ---- All 4 commands succeeded — startup complete ---- */
        imu_can_offline = 0;
        return;

startup_retry:
        /* Pause before retrying entire sequence */
        HAL_Delay(100);
    }

    /* ---- All rounds exhausted ---- */
    imu_can_offline = 1;
    /* Main loop will skip TX until CAN recovers (via error callback or
     * external reset). Data arriving during offline is enqueued normally;
     * if queue fills, oldest frames are silently dropped. */
}
