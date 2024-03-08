#pragma once
#include "main.h"
#include "fdcan_bsp.h"
#define COMMUNICATION_TASK_FIFO_BUFSIZE 20

typedef enum 
{
    Communication_BoardID_All = 0,
    Communication_BoardID_Chassis,
    Communication_BoardID_Seedling,
    Communication_BoardID_BallFetcher,
    Communication_BoardID_BallShooter,
    Communication_BoardID_Gyro,
    Communication_BoardID_DT35Left,
    Communication_BoardID_DT35Right,
    Communication_BoardID_DT35Back,
} Communication_BoardID_t;

typedef struct 
{
    Communication_BoardID_t senderId;   //发送者ID
    Communication_BoardID_t receiverId; //接收者ID
    uint8_t taskCode;       //任务码
    uint8_t dataLength;     //数据长度
    uint8_t data[8];        //数据
    uint8_t isExecuted;     //是否已经执行
} Communication_Task_t;

typedef struct 
{
    Communication_Task_t buffer[COMMUNICATION_TASK_FIFO_BUFSIZE];
    uint32_t head;
    uint32_t tail;
    uint32_t task_cnt;      //当前fifo中的任务数量
    uint32_t operate_cnt;   //当前对应的Task周期执行计数
    uint32_t offline_cnt;   //当前对应的Task离线计数
} Communication_TaskFifo_t;

typedef struct
{
    FDCAN_HandleTypeDef* hfdcan;
    Communication_TaskFifo_t taskFifo;
    uint32_t operate_period;   //周期执行时间
    uint32_t offline_retry;   //离线重发次数
} Communication_t;

uint8_t Communication_AddTask(Communication_t* commu, Communication_Task_t* task);
void Communication_Init(Communication_t* commu);
void Communication_TaskThread(Communication_t* commu);
__weak void Communication_FDCAN_Callback(FdcanBspReceive_t* rxmsg, Communication_t* commu);
