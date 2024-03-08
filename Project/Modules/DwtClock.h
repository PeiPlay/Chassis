#ifndef __DWT_CLOCK_H
#define __DWT_CLOCK_H
#define DWT_CLOCK

#include "main.h"

//Dwt时钟初始化
void DwtClock_Init(void);
//得到Dwt计数值
uint64_t DwtClock_GetDwtCnt(void);
//得到Dwt微秒数
uint64_t DwtClock_GetDwtUs(void);
//得到Dwt毫秒数
uint64_t DwtClock_GetDwtMs(void);
//得到Dwt秒数
double DwtClock_GetDwtTime(void);
//Dwt延时微秒
void DwtClock_DelayUs(uint64_t us);
//Dwt延时毫秒
void DwtClock_DelayMs(uint64_t ms);
//Dwt溢出保护(为防止两次调用DwtClock_GetDwtCnt的时间差大于Dwt溢出周期，需要调用该函数来保护性地调用DwtClock_GetDwtCnt)
void DwtClock_OverflowProtect(void);
//Dwt数据转换
double DwtClock_ConvertCnt2Time(uint64_t cnt);
//Dwt时钟是否初始化
uint8_t DwtClock_IsInited(void);

#endif
