#pragma once
#include "main.h" 
typedef struct
{
    float kp;
    float ki;
    float kd;

    float last_err;                 //上一次误差
    float integral;                 //积分
    float target;                   //目标值
	float measure;                  //测量值
    float output;                   //输出值

    float output_max;               //输出最大值
    float output_min;               //输出最小值
    float integral_max;             //积分最大累计值
    float integral_min;             //积分最小累计值

    float integral_startzone;       //积分分离
    float integral_deadband;        //误差死区
} Pid_t;
//根据测量值计算PID输出
float Pid_Calculate(Pid_t* pid, float meas);
//设置PID目标值
void Pid_SetTarget(Pid_t* pid, float targ);
//清除PID积分
void Pid_ClearAccumulation(Pid_t* pid);
