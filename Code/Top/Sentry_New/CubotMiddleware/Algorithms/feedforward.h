#ifndef __FEEDFORWARD_H_
#define __FEEDFORWARD_H_

#include "stm32h7xx_hal.h"


typedef struct {
    double K;//增益
    double tau;//时间常数
    double Ts;//采样周期
    double ref_prev;
    double u;//输出

} FeedforwardController;

void Feedforward_init(FeedforwardController *c, double K, double tau, double Ts);
double Feedforward_one_step(FeedforwardController *c, double ref);


#endif