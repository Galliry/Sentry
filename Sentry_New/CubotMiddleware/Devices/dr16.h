#include "stm32h7xx_hal.h"
#include "usart.h"
#include "driver_usart.h"

#define DR16_rxBufferLengh 18	 //< dr16���ջ��������ݳ���
#define Key_Filter_Num 7       //< ������������˲�ʱ��(ms)
#define ACE_SENSE  0.12f //0.05f
#define ACE_SHACHE 0.009f
#define PARK_SENSE 0.005f

/**
  * @brief  ���ջ�������������, ����rcң�������ݡ�mouse������ݺ� keyflag ��������
  */
typedef struct
{
    struct
    {
        uint16_t sw;
        uint16_t ch0;
        uint16_t ch1;
        uint16_t ch2;
        uint16_t ch3;
        uint8_t s1;
        uint8_t s2;
        uint8_t s1_last;
        uint8_t s2_last;
    } rc;
    uint8_t isUnpackaging; // ����״̬��־λ����������в���ȡ����
    uint8_t is_online;
    int16_t online_cnt;
	int16_t FPS;
} RC_Ctrl;

void DR16_DataUnpack(RC_Ctrl *rc_ctrl, uint8_t *recBuffer);
void DR16Init(RC_Ctrl *rc_ctrl);
uint8_t DR16_Callback(uint8_t *recBuffer, uint16_t len);

extern UART_RxBuffer uart1_buffer;
extern RC_Ctrl rc_Ctrl;
extern uint8_t DR16_RxBuffer0[36];
extern uint8_t DR16_RxBuffer1[36];
