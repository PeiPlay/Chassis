#pragma once
#include "main.h"
#include "Pid.h"

#define ROUTE_MAX_POINT_IN_PATH 10
#define ROUTE_MAX_PATH 20 + 1       //最大路径数，需要保证最后一个路径为NULL


typedef struct
{
    float x_out, y_out, z_out;
} Route_PidOutput_t;
typedef struct 
{
    struct 
    {
        float kp, ki, kd;                   // 比例、积分、微分系数
        float measure, target, output;      // 测量值、目标值、输出值
        float integral, lastErr;            // 积分、上一次误差
        float startzone, deadband;          //积分分离，误差死区
        float integralLimit, outputLimit;   //积分限幅，输出限幅
        float approach;                     //接近目标点的距离
    } x, y, z;
    uint32_t    approch_cnt;                //到达目标点计数
    uint32_t    approch_check;              //approch_cnt累计到approch_check，认为到达目标点
} Route_Point_t;//164字节

typedef struct 
{
    Route_Point_t point[ROUTE_MAX_POINT_IN_PATH];
    uint32_t point_total;
    uint32_t point_iter;
} Route_Path_t;

typedef struct
{
    Route_Path_t* path[ROUTE_MAX_PATH];
    uint32_t path_total;
    uint32_t path_iter;
} Route_t;
Route_Path_t*  Route_GetCurrentPath(Route_t* route);
Route_Point_t* Route_GetCurrentPoint(Route_t* route);
void Route_PointPidCalculate(Route_Point_t* point, float x_measure, float y_measure, float z_measure); 
Route_PidOutput_t Route_RunOperation(Route_t* route, float x_measure, float y_measure, float z_measure);


