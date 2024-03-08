#pragma once
#include "main.h"
#include "fdcan_bsp.h"


#define DT35_DISTFILTER_BUFSIZE  4    
#define DT35_SPEDFILTER_BUFSIZE  6  


//采用递推均值滤波
typedef struct 
{
    uint32_t iter;  
    int32_t sum;
    int32_t buf[DT35_DISTFILTER_BUFSIZE];   
    float map_k;
    float map_b;
} DT35_DistanceFilter_t;
//采用递推均值滤波和低通滤波
typedef struct 
{
    uint32_t iter;  
    float buf[DT35_SPEDFILTER_BUFSIZE];
    float lastfilt;
    float lastrx;
    float lasttime;
    float inertia;                          //惯性系数
    float deadband;                         //死区       
    float map_k;
    float map_b;
} DT35_SpeedFilter_t;


typedef struct 
{
    FDCAN_HandleTypeDef *hfdcan;
    uint32_t id;
    int32_t rxval;     //原始数据
    float distance;     //滤波并映射的距离
    float speed;        //滤波并映射的速度

    DT35_DistanceFilter_t distfilter;
    DT35_SpeedFilter_t speedfilter;
} DT35_t;

void DT35_Init(DT35_t* dt35);
void DT35_FDCAN_Callback(FdcanBspReceive_t* rxmsg, DT35_t* dt35);
int32_t DT35_GetRxValue(DT35_t* dt35);
float DT35_GetDistance(DT35_t* dt35);
float DT35_GetSpeed(DT35_t* dt35);
