#ifndef _COMMUNICATION_H
#define _COMMUNICATION_H
#include "driver_can.h"
#include "dr16.h"
#include "et08.h"
#include "mpu6050.h"
#include "check.h"
#include "brain.h"
typedef struct 
{
    uint8_t TransData[50];
}Transmit_t;

typedef struct 
{
    struct 
    {
        struct 
        {
            uint8_t Status;
            uint8_t StatusCnt;
        }Online_check;
    }Top;
    
}Receive_t;

extern Transmit_t Transmit;
extern Receive_t Receive;
void Trans_forToptoBase(RC_Ctrl* rc_ctrl);
// void SolutionData_FromTop(CAN_RxBuffer* rxBuffer);
#endif
