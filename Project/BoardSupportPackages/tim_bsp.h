#pragma once 
#include "tim.h"
#include "main.h"

//开启定时器
void TimBsp_Start(TIM_HandleTypeDef *htim);
//关闭定时器
void TimBsp_Shutdown(TIM_HandleTypeDef *htim);
//得到计数器自增频率
uint32_t TimBsp_GetCounterFreq(TIM_HandleTypeDef *htim);
//计算合适的分频系数和重装载值
void TimBsp_CalcPscAndAtr(TIM_HandleTypeDef *htim, double freq, uint32_t *psc, uint32_t *atr);
