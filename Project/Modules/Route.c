#include "Route.h"
#include "math_bsp.h"

/*
*   获取当前路径
*/
Route_Path_t*  Route_GetCurrentPath(Route_t* route)
{
    return route->path[route->path_iter];
}
/*
*   获取当前点
*/
Route_Point_t* Route_GetCurrentPoint(Route_t* route)
{
    Route_Path_t* path = Route_GetCurrentPath(route);
    return &path->point[path->point_iter];
}
/*
*   判断当前点是否为路径的起点
*/
uint8_t Route_IsCurrentPointStartingPoint(Route_t* route)
{
    Route_Path_t* path = Route_GetCurrentPath(route);
    return path->point_iter == 0;
}
/*
*   判断当前点是否为路径的终点
*/
uint8_t Route_IsCurrentPointEndingPoint(Route_t* route)
{
    Route_Path_t* path = Route_GetCurrentPath(route);
    return path->point_iter == path->point_total - 1;
}
/*
*   计算PID
*/
void Route_PointPidCalculate(Route_Point_t* point, float x_measure, float y_measure, float z_measure)
{
    point->x.measure = x_measure; float x_nowErr = point->x.target - x_measure;
    point->y.measure = y_measure; float y_nowErr = point->y.target - y_measure;
    point->z.measure = z_measure; float z_nowErr = point->z.target - z_measure;
    float temp;
    if(MathBsp_Abs(x_nowErr) <= MathBsp_Abs(point->x.startzone))
    {
        if(MathBsp_Abs(x_nowErr) >= MathBsp_Abs(point->x.deadband))
            temp = point->x.integral + x_nowErr * point->x.kp;
//      else 
//          point->x.integral += 0;
    }
//  else
//      point->x.integral += 0;
    point->x.integral = MathBsp_Clamp(temp, -point->x.integralLimit, point->x.integralLimit);
    temp = point->x.kp * x_nowErr + point->x.integral + point->x.kd * (x_nowErr - point->x.lastErr);
    point->x.output = MathBsp_Clamp(temp, -point->x.outputLimit, point->x.outputLimit);
    point->x.lastErr = x_nowErr;

    if(MathBsp_Abs(y_nowErr) <= MathBsp_Abs(point->y.startzone))
    {
        if(MathBsp_Abs(y_nowErr) >= MathBsp_Abs(point->y.deadband))
            temp = point->y.integral + y_nowErr * point->y.kp;
//      else
//          point->y.integral += 0;
    }
//  else
//      point->y.integral += 0;
    point->y.integral = MathBsp_Clamp(temp, -point->y.integralLimit, point->y.integralLimit);
    temp = point->y.kp * y_nowErr + point->y.integral + point->y.kd * (y_nowErr - point->y.lastErr);
    point->y.output = MathBsp_Clamp(temp, -point->y.outputLimit, point->y.outputLimit);
    point->y.lastErr = y_nowErr;

    if(MathBsp_Abs(z_nowErr) <= MathBsp_Abs(point->z.startzone))
    {
        if(MathBsp_Abs(z_nowErr) >= MathBsp_Abs(point->z.deadband))
            temp = point->z.integral + z_nowErr * point->z.kp;
//      else
//          point->z.integral += 0;
    }
//  else
//      point->z.integral += 0;
    point->z.integral = MathBsp_Clamp(temp, -point->z.integralLimit, point->z.integralLimit);
    temp = point->z.kp * z_nowErr + point->z.integral + point->z.kd * (z_nowErr - point->z.lastErr);
    point->z.output = MathBsp_Clamp(temp, -point->z.outputLimit, point->z.outputLimit);
    point->z.lastErr = z_nowErr;
}
/*
*   检查是否到达目标点
*/
void _Route_CheckApproch(Route_Point_t* point)
{
    if( MathBsp_Abs(point->x.measure - point->x.target) <= point->x.approach &&
        MathBsp_Abs(point->y.measure - point->y.target) <= point->y.approach &&
        MathBsp_Abs(point->z.measure - point->z.target) <= point->z.approach)
        point->approch_cnt++;
    else
        point->approch_cnt = 0;
}
/*
*   清零PID和approch_cnt
*/
void _Route_PointAccumulationClear(Route_Point_t* point)
{
    point->x.integral = 0;
    point->y.integral = 0;
    point->z.integral = 0;
    point->x.lastErr = 0;
    point->y.lastErr = 0;
    point->z.lastErr = 0;
    point->approch_cnt = 0;
}
/*
*   切换到下一个点
*/
Route_Point_t* _Route_SwitchNextPoint(Route_t* route)
{
    Route_Path_t* path = Route_GetCurrentPath(route);
    if(path->point_iter < path->point_total - 1)
    {
        path->point_iter++;
        return &path->point[path->point_iter];
    }
    else if(route->path_iter < route->path_total - 1)
    {
        route->path_iter++;
        return &route->path[route->path_iter]->point[route->path[route->path_iter]->point_iter];
    }
    else
    {
        return NULL;
    }
}
/*
    对所有非当前运行点的PID和approch_cnt进行清零
    可以调用以方便调试
*/
void _Route_ResetDormantPoints(Route_t* route)
{
    for(uint32_t i = 0; i < route->path_total; i++)
    {
        if(i == route->path_iter)
        {
            for(uint32_t j = 0; j < route->path[i]->point_total; j++)
            {
                if (j == route->path[i]->point_iter)
                    continue;
                _Route_PointAccumulationClear(&route->path[i]->point[j]);
            }
            continue;
        }
        for(uint32_t j = 0; j < route->path[i]->point_total; j++)
        {
            _Route_PointAccumulationClear(&route->path[i]->point[j]);
        }
    }
}
/*
*   运行路径
*/
Route_PidOutput_t Route_RunOperation(Route_t* route, float x_measure, float y_measure, float z_measure)
{
    _Route_ResetDormantPoints(route);
    Route_PidOutput_t output;
    Route_Point_t* point = Route_GetCurrentPoint(route);
    Route_PointPidCalculate(point, x_measure, y_measure, z_measure);
    output.x_out = point->x.output;
    output.y_out = point->y.output;
    output.z_out = point->z.output;
    _Route_CheckApproch(point);
    if (point->approch_cnt > point->approch_check)
    {
        _Route_PointAccumulationClear(point);
        if(_Route_SwitchNextPoint(route) == NULL)
        {
            output.x_out = 0;
            output.y_out = 0;
            output.z_out = 0;
        }
    }
    return output;
}
