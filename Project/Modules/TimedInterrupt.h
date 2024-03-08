#pragma once
#include "main.h"
#include "tim.h"

#define TIMED_INTERRUPT_MAX_TASKS 20

typedef void (*TimedInterrupt_TaskFunction_t)(void);
typedef TIM_HandleTypeDef* TimedInterrupt_Timer_t;
typedef enum 
{
    TimedInterrupt_Mode_Independent = 0,//时钟独立模式，进入中断时立刻开始下一轮计时，与此同时执行任务
    TimedInterrupt_Mode_Obstructed,     //时钟阻塞模式，所有任务执行完毕后才开始下一轮计时
} TimedInterrupt_Mode_t;
typedef struct {
    TimedInterrupt_Timer_t htim;
    TimedInterrupt_TaskFunction_t taskBuff[TIMED_INTERRUPT_MAX_TASKS];
    uint32_t intPeriod;//中断周期us
    uint16_t mode:1;
    uint16_t taskCount:15;
} TimedInterrupt_t;
//初始化定时中断
void TimedInterrupt_Init(TimedInterrupt_t* timedInterrupt);
//设置定时器中断回调
void TimedInterrupt_TIM_Callback(TimedInterrupt_Timer_t htim, TimedInterrupt_t* timedInterrupt);
//向定时中断中添加任务
void TimedInterrupt_AddTask(TimedInterrupt_t* timedInterrupt, TimedInterrupt_TaskFunction_t task);
//从定时中断中移除任务
void TimedInterrupt_RemoveTask(TimedInterrupt_t* timedInterrupt, TimedInterrupt_TaskFunction_t task);
