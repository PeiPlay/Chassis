#include "Pid.h"

float _Pid_Abs(float x)
{
    return x > 0 ? x : -x;
}

float _Pid_Limit(float x, float upper, float lower)
{
    return x > upper ? upper : (x < lower ? lower : x);
}

inline float Pid_Calculate(Pid_t* pid, float meas)
{
    //计算误差值
	pid->measure = meas;
    float curr_err = pid->target - pid->measure;
    //积分分离和误差死区
    if(_Pid_Abs(curr_err) <= pid->integral_startzone)
    {
        if(_Pid_Abs(curr_err) >= pid->integral_deadband)
            pid->integral += curr_err * pid->ki;
    //  else
    //      pid->integral += 0;
    }
    else
        pid->integral = 0;
    //积分限幅
    pid->integral = _Pid_Limit(pid->integral, pid->integral_max, pid->integral_min);
    //计算输出值
    pid->output = pid->kp * curr_err + pid->integral + pid->kd * (curr_err - pid->last_err);
    //输出限幅
    pid->output = _Pid_Limit(pid->output, pid->output_max, pid->output_min);
    //更新上一次误差值
    pid->last_err = curr_err;
    //返回输出值
    return pid->output;
}
inline void Pid_SetTarget(Pid_t* pid, float targ)
{
    pid->target = targ;
}
inline void Pid_ClearAccumulation(Pid_t* pid)
{
    pid->integral = 0;
    pid->last_err = 0;
}
