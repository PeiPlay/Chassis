#include "TimedInterrupt.h"
#include "tim_bsp.h"
#include <stdio.h>

typedef enum 
{
    TimedInterrupt_ErrorCode_UndefinedTimer = 0,
    TimedInterrupt_ErrorCode_IntPeriodError,
} TimedInterrupt_ErrorCode_t;

void TimedInterrupt_ErrorHandler(TimedInterrupt_ErrorCode_t error_code)
{
    Error_Handler();
}
//返回中断周期的计数值
uint64_t _TimedInterrupt_GetIntPeriodCnt(TimedInterrupt_t* timedInterrupt)
{
    return ((uint64_t)timedInterrupt->intPeriod * (uint64_t)(TimBsp_GetCounterFreq(timedInterrupt->htim) / 1000000));
}
void _TimedInterrupt_Shutdown(TimedInterrupt_t* timedInterrupt)
{
    TimedInterrupt_Timer_t htim = timedInterrupt->htim;
    HAL_TIM_Base_Stop_IT(htim);
    HAL_TIM_Base_Stop(htim);
    __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
    __HAL_TIM_SET_COUNTER(htim, 0);
}
void _TimedInterrupt_Start(TimedInterrupt_t* timedInterrupt)
{
    _TimedInterrupt_Shutdown(timedInterrupt);
    TimedInterrupt_Timer_t htim = timedInterrupt->htim;
    HAL_TIM_Base_Start_IT(htim);
    HAL_TIM_Base_Start(htim);
}

void TimedInterrupt_Init(TimedInterrupt_t* timedInterrupt)
{
    uint64_t intPeriodCnt = _TimedInterrupt_GetIntPeriodCnt(timedInterrupt);
    if(intPeriodCnt == 0 || intPeriodCnt > 0x100000000)
        TimedInterrupt_ErrorHandler(TimedInterrupt_ErrorCode_IntPeriodError);
    //计算定时器的分频系数和自动重装值
    uint32_t prescaler, period;
    uint64_t best_product = 0;
    for (uint32_t i = 1; i <= 65536; ++i) {
        uint32_t j = intPeriodCnt / i;
        if (j > 65536) continue;
        uint64_t product = i * j;
        if (product > best_product) {
            best_product = product;
            prescaler = i;
            period = j;
        }
		else if(product == intPeriodCnt)
			break;
    }
    //初始化定时器
    TimedInterrupt_Timer_t htim = timedInterrupt->htim;
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    HAL_TIM_Base_Stop_IT(htim);
    HAL_TIM_Base_Stop(htim);
    __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
    __HAL_TIM_SET_COUNTER(htim, 0);

    HAL_TIM_Base_DeInit(htim);
    htim->Init.Prescaler = prescaler - 1; //设置分频系数
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;        //向上计数
    htim->Init.Period = period - 1;                              //设置自动重装载值
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  //关闭自动重装载
    if (HAL_TIM_Base_Init(htim) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    _TimedInterrupt_Start(timedInterrupt);
}

void TimedInterrupt_TIM_Callback(TimedInterrupt_Timer_t htim, TimedInterrupt_t* timedInterrupt)
{
    if(htim->Instance != timedInterrupt->htim->Instance)
        return;
    if(timedInterrupt->mode == TimedInterrupt_Mode_Obstructed)
    {
        _TimedInterrupt_Shutdown(timedInterrupt);
        for(uint16_t i = 0; i < timedInterrupt->taskCount; ++i)
            timedInterrupt->taskBuff[i]();
        _TimedInterrupt_Start(timedInterrupt);
    }
    else
    {
        _TimedInterrupt_Start(timedInterrupt);
        for(uint16_t i = 0; i < timedInterrupt->taskCount; ++i)
            timedInterrupt->taskBuff[i]();
    }
}
void TimedInterrupt_AddTask(TimedInterrupt_t* timedInterrupt, TimedInterrupt_TaskFunction_t task)
{
    if(timedInterrupt->taskCount >= TIMED_INTERRUPT_MAX_TASKS)
        Error_Handler();
    timedInterrupt->taskBuff[timedInterrupt->taskCount++] = task;
}
void TimedInterrupt_RemoveTask(TimedInterrupt_t* timedInterrupt, TimedInterrupt_TaskFunction_t task)
{
    for(uint16_t i = 0; i < timedInterrupt->taskCount; ++i)
    {
        if(timedInterrupt->taskBuff[i] == task)
        {
            for(uint16_t j = i; j < timedInterrupt->taskCount - 1; ++j)
                timedInterrupt->taskBuff[j] = timedInterrupt->taskBuff[j + 1];
            --timedInterrupt->taskCount;
            return;
        }
    }
}
