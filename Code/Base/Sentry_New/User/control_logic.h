#ifndef CONTROLLOGIC_H_
#define CONTROLLOGIC_H_
#include "stm32h7xx_hal.h"
#include "motor.h"
#include "interboard.h"
#include "Supercap.h"
#include <stdint.h>
uint8_t CAN1_rxCallBack(CAN_RxBuffer* rxBuffer);
uint8_t CAN2_rxCallBack(CAN_RxBuffer* rxBuffer);

extern uint8_t beHit;

void TIM14_Task(void);
void TIM13_Task(void);

#endif



