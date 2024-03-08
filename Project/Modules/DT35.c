#include "DT35.h"
#include "DwtClock.h"
#include "math_bsp.h"


void DT35_Init(DT35_t* dt35)
{
    if(!DwtClock_IsInited()) DwtClock_Init();
}

void DT35_FDCAN_Callback(FdcanBspReceive_t* rxmsg, DT35_t* dt35)
{
    if(rxmsg->hfdcan == 0 || rxmsg->hfdcan->Instance != dt35->hfdcan->Instance) return;
    if(rxmsg->RxMessage.IdType != FDCAN_STANDARD_ID || rxmsg->RxMessage.Identifier != dt35->id) return;
    //解析原始数据
    dt35->rxval = *(int32_t*)(rxmsg->RxData);
    //进行距离滤波和映射
    dt35->distfilter.sum -= dt35->distfilter.buf[dt35->distfilter.iter];
    dt35->distfilter.buf[dt35->distfilter.iter++] = dt35->rxval;
    dt35->distfilter.sum += dt35->rxval;
    dt35->distfilter.iter %= DT35_DISTFILTER_BUFSIZE;
    float origdist = (float)dt35->distfilter.sum / (float)DT35_DISTFILTER_BUFSIZE;
    dt35->distance = origdist * dt35->distfilter.map_k + dt35->distfilter.map_b;
    //进行速度滤波和映射
    float timespan = (float)DwtClock_GetDwtTime() - dt35->speedfilter.lasttime;
    dt35->speedfilter.lasttime = (float)DwtClock_GetDwtTime();
    dt35->speedfilter.buf[dt35->speedfilter.iter] = (origdist - dt35->speedfilter.lastrx) / timespan;
    dt35->speedfilter.lastrx = origdist;
    dt35->speedfilter.iter = (dt35->speedfilter.iter + 1) % DT35_SPEDFILTER_BUFSIZE;
    float sortedbuf[DT35_SPEDFILTER_BUFSIZE];
    for(uint32_t i = 0; i < DT35_SPEDFILTER_BUFSIZE; i++) sortedbuf[i] = dt35->speedfilter.buf[i];
    MathBsp_QuickSort_float(sortedbuf, DT35_SPEDFILTER_BUFSIZE);
    //抛弃最大1/4和最小1/4的数据，取中间1/2的数据求和
    float new_speed = 0;
    for(uint32_t i = DT35_SPEDFILTER_BUFSIZE / 4; i < (DT35_SPEDFILTER_BUFSIZE / 2) + (DT35_SPEDFILTER_BUFSIZE / 4); i++) new_speed += sortedbuf[i];
    new_speed /= (float)(DT35_SPEDFILTER_BUFSIZE / 2);
    new_speed = dt35->speedfilter.lastfilt * dt35->speedfilter.inertia + new_speed * (1.0f - dt35->speedfilter.inertia);
    dt35->speedfilter.lastfilt = new_speed;
    new_speed = new_speed * dt35->speedfilter.map_k + dt35->speedfilter.map_b;
    if(MathBsp_Abs(new_speed) < MathBsp_Abs(dt35->speedfilter.deadband)) dt35->speed = 0;
    else dt35->speed = new_speed;
}

int32_t DT35_GetRxValue(DT35_t* dt35)
{
    return dt35->rxval;
}
float DT35_GetDistance(DT35_t* dt35)
{
    return dt35->distance;
}
float DT35_GetSpeed(DT35_t* dt35)
{
    return dt35->speed;
}



