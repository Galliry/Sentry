#include "pid.h"
#include "dr16.h"
#include "filter.h"
#include "feedforward.h"

#include <stdio.h>

FeedforwardController   YawFFC = {
    .K = 0,
    .tau = 1,
    .Ts = 0.001,
};

void Feedforward_init(FeedforwardController *c, double K, double tau, double Ts)
{
 c->K = K;
 c->tau = tau;
 c->Ts = Ts;
 c->ref_prev = 0.0f;
}
/*一阶电机系统*/
double Feedforward_one_step(FeedforwardController *c, double ref)
{
 double dref = (ref - c->ref_prev) / c->Ts;
 c->u = (c->tau / c->K) * dref + (1.0 / c->K) * ref;

 c->ref_prev = ref;

 return c->u;
}

