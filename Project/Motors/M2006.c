#include "M2006.h"
#include "fdcan_bsp.h"
#include "Config.h"
void M2006_SetSpeed(M2006_t* m2006, double spd)
{
    if(m2006->group == 1)
        M2006G1_SetSpeed(m2006->id, spd);
    else if(m2006->group == 2)
        M2006G2_SetSpeed(m2006->id, spd);
}
void M2006_SetPosition(M2006_t* m2006, double pos)
{
    if(m2006->group == 1)
        M2006G1_SetPosition(m2006->id, pos);
    else if(m2006->group == 2)
        M2006G2_SetPosition(m2006->id, pos);

}
void M2006_SetZero(M2006_t* m2006)
{
    if(m2006->group == 1)
        M2006G1_SetZero(m2006->id);
    else if(m2006->group == 2)
        M2006G2_SetZero(m2006->id);

}
int64_t M2006_GetPosition(M2006_t* m2006)
{
    if(m2006->group == 1)
        return M2006G1_GetPosition(m2006->id);
    else if(m2006->group == 2)
        return M2006G2_GetPosition(m2006->id);
    return 0;
}
int64_t M2006_GetSpeed(M2006_t* m2006)
{
    if(m2006->group == 1)
        return M2006G1_GetSpeed(m2006->id);
    else if(m2006->group == 2)
        return M2006G2_GetSpeed(m2006->id);
    return 0;
}
void M2006_SetOffset(M2006_t* m2006, int64_t offset)
{
    if(m2006->group == 1)
        M2006G1_SetOffset(m2006->id, offset);
    else if(m2006->group == 2)
        M2006G2_SetOffset(m2006->id, offset);
}

#define M2006G1_TxID1 0x200
#define M2006G1_TxID2 0x1FF
#define M2006G1_ID_BEGIN 0x201    //电机ID起始值
#define M2006G1_ID_END 0x208      //电机ID结束值
#define M2006G1_REDUCTION 36
#define _Absolut(x) ((x) > 0 ? (x) : -(x))//绝对值
#define _Limit(x, max, min) ((x) > (max) ? (max) : ((x) < (min) ? (min) : (x)))//限幅
static uint8_t M2006G1_TxData[16] = {0}; //FDCAN发送数据1
static FDCAN_HandleTypeDef* M2006G1_hfdcan;
M2006G1_t M2006G1[8] = 
{
    {
        .pid_speed.kp = 8,
        .pid_speed.ki = 1,
        .pid_speed.kd = 0.2,
        .pid_speed.output_limit_max = 14000,
        .pid_speed.output_limit_min = -14000,
        .pid_speed.integral_limit_max = 14000,
        .pid_speed.integral_limit_min = -14000,
        .pid_speed.integral_deadband = 10,

        .pid_position.kp = 10,
        .pid_position.ki = 0.4,
        .pid_position.kd = 0.1,
        .pid_position.output_limit_max = 14000,
        .pid_position.output_limit_min = -14000,
        .pid_position.integral_limit_max = 20000,
        .pid_position.integral_limit_min = -20000,
        .pid_position.integral_deadband = 0,
    },
    {
        .pid_speed.kp = 5,
        .pid_speed.ki = 0.02,
        .pid_speed.kd = 0.2,
        .pid_speed.output_limit_max = 14000,
        .pid_speed.output_limit_min = -14000,
        .pid_speed.integral_limit_max = 14000,
        .pid_speed.integral_limit_min = -14000,
        .pid_speed.integral_deadband = 10,
		.pid_speed.integral_startzone = 1000,
		
        .pid_position.kp = 10,
        .pid_position.ki = 0.4,
        .pid_position.kd = 0.1,
        .pid_position.output_limit_max = 20000,
        .pid_position.output_limit_min = -20000,
        .pid_position.integral_limit_max = 20000,
        .pid_position.integral_limit_min = -20000,
        .pid_position.integral_deadband = 0,
    },
    {
        .pid_speed.kp = 8,
        .pid_speed.ki = 1,
        .pid_speed.kd = 0.2,
        .pid_speed.output_limit_max = 14000,
        .pid_speed.output_limit_min = -14000,
        .pid_speed.integral_limit_max = 14000,
        .pid_speed.integral_limit_min = -14000,
        .pid_speed.integral_deadband = 10,

        .pid_position.kp = 10,
        .pid_position.ki = 0.4,
        .pid_position.kd = 0.1,
        .pid_position.output_limit_max = 14000,
        .pid_position.output_limit_min = -14000,
        .pid_position.integral_limit_max = 20000,
        .pid_position.integral_limit_min = -20000,
        .pid_position.integral_deadband = 0,
    },
    {
        .pid_speed.kp = 8,
        .pid_speed.ki = 1,
        .pid_speed.kd = 0.2,
        .pid_speed.output_limit_max = 14000,
        .pid_speed.output_limit_min = -14000,
        .pid_speed.integral_limit_max = 14000,
        .pid_speed.integral_limit_min = -14000,
        .pid_speed.integral_deadband = 10,

        .pid_position.kp = 10,
        .pid_position.ki = 0.4,
        .pid_position.kd = 0.1,
        .pid_position.output_limit_max = 14000,
        .pid_position.output_limit_min = -14000,
        .pid_position.integral_limit_max = 20000,
        .pid_position.integral_limit_min = -20000,
        .pid_position.integral_deadband = 0,
    },
    {
        .pid_speed.kp = 8,
        .pid_speed.ki = 1,
        .pid_speed.kd = 0.2,
        .pid_speed.output_limit_max = 14000,
        .pid_speed.output_limit_min = -14000,
        .pid_speed.integral_limit_max = 14000,
        .pid_speed.integral_limit_min = -14000,
        .pid_speed.integral_deadband = 10,

        .pid_position.kp = 10,
        .pid_position.ki = 0.4,
        .pid_position.kd = 0.1,
        .pid_position.output_limit_max = 14000,
        .pid_position.output_limit_min = -14000,
        .pid_position.integral_limit_max = 20000,
        .pid_position.integral_limit_min = -20000,
        .pid_position.integral_deadband = 0,
    },
    {
        .pid_speed.kp = 8,
        .pid_speed.ki = 1,
        .pid_speed.kd = 0.2,
        .pid_speed.output_limit_max = 25000,
        .pid_speed.output_limit_min = -25000,
        .pid_speed.integral_limit_max = 25000,
        .pid_speed.integral_limit_min = -25000,
        .pid_speed.integral_deadband = 10,

        .pid_position.kp = 10,
        .pid_position.ki = 0.4,
        .pid_position.kd = 0.1,
        .pid_position.output_limit_max = 25000,
        .pid_position.output_limit_min = -25000,
        .pid_position.integral_limit_max = 25000,
        .pid_position.integral_limit_min = -25000,
        .pid_position.integral_deadband = 0,
    },
    {
        .pid_speed.kp = 8,
        .pid_speed.ki = 1,
        .pid_speed.kd = 0.2,
        .pid_speed.output_limit_max = 14000,
        .pid_speed.output_limit_min = -14000,
        .pid_speed.integral_limit_max = 14000,
        .pid_speed.integral_limit_min = -14000,
        .pid_speed.integral_deadband = 10,

        .pid_position.kp = 10,
        .pid_position.ki = 0.4,
        .pid_position.kd = 0.1,
        .pid_position.output_limit_max = 14000,
        .pid_position.output_limit_min = -14000,
        .pid_position.integral_limit_max = 20000,
        .pid_position.integral_limit_min = -20000,
        .pid_position.integral_deadband = 0,
    },
    {
        .pid_speed.kp = 8,
        .pid_speed.ki = 1,
        .pid_speed.kd = 0.2,
        .pid_speed.output_limit_max = 14000,
        .pid_speed.output_limit_min = -14000,
        .pid_speed.integral_limit_max = 14000,
        .pid_speed.integral_limit_min = -14000,
        .pid_speed.integral_deadband = 10,

        .pid_position.kp = 10,
        .pid_position.ki = 0.4,
        .pid_position.kd = 0.1,
        .pid_position.output_limit_max = 14000,
        .pid_position.output_limit_min = -14000,
        .pid_position.integral_limit_max = 20000,
        .pid_position.integral_limit_min = -20000,
        .pid_position.integral_deadband = 0,
    },
};

void _M2006G1_SetCurrent(M2006G1_t* m2006, double current)
{
    uint16_t index = m2006 - M2006G1;
    if(index > 7 || m2006 == NULL) return;
    current = _Limit(current, 10000, -10000);
    int temp = (int)current;
	//temp = 9000;
    M2006G1_TxData[index * 2] = (uint8_t)((temp >> 8) & 0xff);
    M2006G1_TxData[index * 2 + 1] = (uint8_t)(temp & 0xff);
}
void _M2006G1_SetSpeed(M2006G1_t* m2006, double target_spd)
{
    uint16_t index = m2006 - M2006G1;
    if(index > 7 || m2006 == NULL) return;
    m2006->pid_speed.last_error = m2006->pid_speed.curr_error;
    m2006->pid_speed.curr_error = target_spd - (double)m2006->status.speed;
    //积分分离和误差死区
    if(_Absolut(m2006->pid_speed.curr_error) <= m2006->pid_speed.integral_startzone)
    {
        if(_Absolut(m2006->pid_speed.curr_error) <= m2006->pid_speed.integral_deadband)
            m2006->pid_speed.integral += 0;
        else
            m2006->pid_speed.integral += m2006->pid_speed.curr_error * m2006->pid_speed.ki;
    }
    else
        m2006->pid_speed.integral = 0;
    //积分限幅
    m2006->pid_speed.integral = _Limit(m2006->pid_speed.integral, m2006->pid_speed.integral_limit_max, m2006->pid_speed.integral_limit_min);
    //PID计算
    double p_calc = m2006->pid_speed.kp * m2006->pid_speed.curr_error;
    double i_calc = m2006->pid_speed.integral;
    double d_calc = m2006->pid_speed.kd * (m2006->pid_speed.curr_error - m2006->pid_speed.last_error);

    m2006->pid_speed.output = p_calc + i_calc + d_calc;
    //输出限幅
    m2006->pid_speed.output = _Limit(m2006->pid_speed.output, m2006->pid_speed.output_limit_max, m2006->pid_speed.output_limit_min);

    _M2006G1_SetCurrent(m2006, m2006->pid_speed.output);
}
void _M2006G1_SetPosition(M2006G1_t* m2006, double target_pos)
{
    uint16_t index = m2006 - M2006G1;
    if(index > 7 || m2006 == NULL) return;
    m2006->pid_position.last_error = m2006->pid_position.curr_error;
    m2006->pid_position.curr_error = target_pos - (double)m2006->status.output_position;

    //积分分离和误差死区
    if(_Absolut(m2006->pid_position.curr_error) <= m2006->pid_position.integral_startzone)
    {
        if(_Absolut(m2006->pid_position.curr_error) <= m2006->pid_position.integral_deadband)
            m2006->pid_position.integral += 0;
        else
            m2006->pid_position.integral += m2006->pid_position.curr_error * m2006->pid_position.ki;
    }
    else
        m2006->pid_position.integral = 0;
    //积分限幅
    m2006->pid_position.integral = _Limit(m2006->pid_position.integral, m2006->pid_position.integral_limit_max, m2006->pid_position.integral_limit_min);
    //PID计算
    double p_calc = m2006->pid_position.kp * m2006->pid_position.curr_error;
    double i_calc = m2006->pid_position.integral;
    double d_calc = m2006->pid_position.kd * (m2006->pid_position.curr_error - m2006->pid_position.last_error);

    m2006->pid_position.output = p_calc + i_calc + d_calc;
    //输出限幅
    m2006->pid_position.output = _Limit(m2006->pid_position.output, m2006->pid_position.output_limit_max, m2006->pid_position.output_limit_min);

    _M2006G1_SetSpeed(m2006, m2006->pid_position.output);
}
void _M2006G1_FDCAN_Transmit(FDCAN_HandleTypeDef* hfdcan, uint8_t *pData, uint16_t ID)
{
    FdcanBsp_Transmit_Classic_StdData(hfdcan, ID, pData, FDCAN_DLC_BYTES_8);
}
void _M2006G1_MotorControl(M2006G1_t* m2006)
{
    uint16_t index = m2006 - M2006G1;
    if(m2006 == NULL || index > 7)
        return;
    if(index > 3)
        _M2006G1_FDCAN_Transmit(M2006G1_hfdcan, &((uint8_t*)M2006G1_TxData)[8], M2006G1_TxID2);
    else
        _M2006G1_FDCAN_Transmit(M2006G1_hfdcan, &((uint8_t*)M2006G1_TxData)[0], M2006G1_TxID1);
}
void M2006G1_Init(FDCAN_HandleTypeDef* hc)
{
    M2006G1_hfdcan = hc;
}
void M2006G1_FDCAN_Callback(FdcanBspReceive_t* rxmsg)
{

    if(rxmsg->hfdcan == 0 || rxmsg->hfdcan->Instance != M2006G1_hfdcan->Instance) return;
    if(rxmsg->RxMessage.IdType != FDCAN_STANDARD_ID)
        return;
    if(rxmsg->RxMessage.Identifier < M2006G1_ID_BEGIN || rxmsg->RxMessage.Identifier > M2006G1_ID_END)
        return;
    uint8_t index = rxmsg->RxMessage.Identifier - M2006G1_ID_BEGIN;
    M2006G1[index].status.position = (uint16_t)((rxmsg->RxData[0] << 8) | rxmsg->RxData[1]);
    M2006G1[index].status.speed = (int16_t)(rxmsg->RxData[2] << 8) | rxmsg->RxData[3];
    M2006G1[index].status.torque = (int16_t)(rxmsg->RxData[4] << 8) | rxmsg->RxData[5];

    //得到编码器值
    if(M2006G1[index].outerEncoderSource == NULL)
    {
        if ((int32_t)M2006G1[index].status.position - (int32_t)M2006G1[index].temp.last_encoder > 4095)                  //当前编码器数值从0突变到8191，刚好反转一圈
            M2006G1[index].temp.position_base -= 8191;                                                       //base减一圈
        else if ((int32_t)M2006G1[index].status.position - (int32_t)M2006G1[index].temp.last_encoder < -4095)          //当前编码器数值从8191突变到0，刚好正转一圈
            M2006G1[index].temp.position_base += 8191;                                                       //base加一圈
        M2006G1[index].temp.last_encoder = M2006G1[index].status.position;                                                  //更新上一次的角度值
        M2006G1[index].status.total_cnt = M2006G1[index].temp.position_base + M2006G1[index].status.position;     //总编码器值=base+当前编码器值
        M2006G1[index].status.output_position = ((M2006G1[index].status.total_cnt - M2006G1[index].status.zero_offset) / M2006G1_REDUCTION);
    }
    else
    {
        M2006G1[index].status.total_cnt = -M2006G1[index].outerEncoderSource();
        M2006G1[index].status.output_position = M2006G1[index].status.total_cnt;
		M2006G1[index].status.position = M2006G1[index].status.output_position;
	}
    
    if(M2006G1[index].status.mode == M2006G1_MODE_SPEED)
    {
        _M2006G1_SetSpeed(&M2006G1[index], M2006G1[index].status.target_speed);
    }
    else if(M2006G1[index].status.mode == M2006G1_MODE_POSITION)
    {
        _M2006G1_SetPosition(&M2006G1[index], M2006G1[index].status.target_position);
    }
    _M2006G1_MotorControl(&M2006G1[index]);
}
void M2006G1_SetSpeed(uint16_t index_from_1, double spd)
{
    if(index_from_1 > 8 || index_from_1 < 1) return;
    M2006G1[index_from_1 - 1].status.target_speed = spd;
    M2006G1[index_from_1 - 1].status.mode = M2006G1_MODE_SPEED;
}
void M2006G1_SetPosition(uint16_t index_from_1, double pos)
{
    if(index_from_1 > 8 || index_from_1 < 1) return;
    M2006G1[index_from_1 - 1].status.target_position = pos;
    M2006G1[index_from_1 - 1].status.mode = M2006G1_MODE_POSITION;
}
void M2006G1_SetZero(uint16_t index_from_1)
{
    if(index_from_1 > 8 || index_from_1 < 1) return;
    M2006G1[index_from_1 - 1].status.zero_offset = M2006G1[index_from_1 - 1].status.total_cnt;
}
int64_t M2006G1_GetPosition(uint16_t index_from_1)
{
	if(index_from_1 > 8 || index_from_1 < 1) return 0;
	return M2006G1[index_from_1 - 1].status.position;
}
int64_t M2006G1_GetSpeed(uint16_t index_from_1)
{
	if(index_from_1 > 8 || index_from_1 < 1) return 0;
	return M2006G1[index_from_1 - 1].status.speed;
}
void M2006G1_SetOffset(uint16_t index_from_1, int64_t offset)
{
    if(index_from_1 > 8 || index_from_1 < 1) return;
	M2006G1[index_from_1 - 1].status.zero_offset = offset;
}	



#define M2006G2_TxID1 0x200
#define M2006G2_TxID2 0x1FF
#define M2006G2_ID_BEGIN 0x201    //电机ID起始值
#define M2006G2_ID_END 0x208      //电机ID结束值
#define M2006G2_REDUCTION 36
static uint8_t M2006G2_TxData[16] = {0}; //FDCAN发送数据2
static FDCAN_HandleTypeDef* M2006G2_hfdcan;
M2006G2_t M2006G2[8] = 
{
    {
        .pid_speed.kp = 8,
        .pid_speed.ki = 1,
        .pid_speed.kd = 0.2,
        .pid_speed.output_limit_max = 14000,
        .pid_speed.output_limit_min = -14000,
        .pid_speed.integral_limit_max = 14000,
        .pid_speed.integral_limit_min = -14000,
        .pid_speed.integral_deadband = 10,

        .pid_position.kp = 10,
        .pid_position.ki = 0.4,
        .pid_position.kd = 0.1,
        .pid_position.output_limit_max = 14000,
        .pid_position.output_limit_min = -14000,
        .pid_position.integral_limit_max = 20000,
        .pid_position.integral_limit_min = -20000,
        .pid_position.integral_deadband = 0,
    },
    {
        .pid_speed.kp = 5,
        .pid_speed.ki = 0.02,
        .pid_speed.kd = 0.2,
        .pid_speed.output_limit_max = 14000,
        .pid_speed.output_limit_min = -14000,
        .pid_speed.integral_limit_max = 14000,
        .pid_speed.integral_limit_min = -14000,
        .pid_speed.integral_deadband = 10,
		.pid_speed.integral_startzone = 1000,
		
        .pid_position.kp = 10,
        .pid_position.ki = 0.4,
        .pid_position.kd = 0.1,
        .pid_position.output_limit_max = 20000,
        .pid_position.output_limit_min = -20000,
        .pid_position.integral_limit_max = 20000,
        .pid_position.integral_limit_min = -20000,
        .pid_position.integral_deadband = 0,
    },
    {
        .pid_speed.kp = 8,
        .pid_speed.ki = 1,
        .pid_speed.kd = 0.2,
        .pid_speed.output_limit_max = 14000,
        .pid_speed.output_limit_min = -14000,
        .pid_speed.integral_limit_max = 14000,
        .pid_speed.integral_limit_min = -14000,
        .pid_speed.integral_deadband = 10,

        .pid_position.kp = 10,
        .pid_position.ki = 0.4,
        .pid_position.kd = 0.1,
        .pid_position.output_limit_max = 14000,
        .pid_position.output_limit_min = -14000,
        .pid_position.integral_limit_max = 20000,
        .pid_position.integral_limit_min = -20000,
        .pid_position.integral_deadband = 0,
    },
    {
        .pid_speed.kp = 8,
        .pid_speed.ki = 1,
        .pid_speed.kd = 0.2,
        .pid_speed.output_limit_max = 14000,
        .pid_speed.output_limit_min = -14000,
        .pid_speed.integral_limit_max = 14000,
        .pid_speed.integral_limit_min = -14000,
        .pid_speed.integral_deadband = 10,

        .pid_position.kp = 10,
        .pid_position.ki = 0.4,
        .pid_position.kd = 0.1,
        .pid_position.output_limit_max = 14000,
        .pid_position.output_limit_min = -14000,
        .pid_position.integral_limit_max = 20000,
        .pid_position.integral_limit_min = -20000,
        .pid_position.integral_deadband = 0,      
    },
    {
        .pid_speed.kp = 8,
        .pid_speed.ki = 1,
        .pid_speed.kd = 0.2,
        .pid_speed.output_limit_max = 14000,
        .pid_speed.output_limit_min = -14000,
        .pid_speed.integral_limit_max = 14000,
        .pid_speed.integral_limit_min = -14000,
        .pid_speed.integral_deadband = 10,

        .pid_position.kp = 10,
        .pid_position.ki = 0.4,
        .pid_position.kd = 0.1,
        .pid_position.output_limit_max = 14000,
        .pid_position.output_limit_min = -14000,
        .pid_position.integral_limit_max = 20000,
        .pid_position.integral_limit_min = -20000,
        .pid_position.integral_deadband = 0,
    },
    {
        .pid_speed.kp = 8,
        .pid_speed.ki = 1,
        .pid_speed.kd = 0.2,
        .pid_speed.output_limit_max = 25000,
        .pid_speed.output_limit_min = -25000,
        .pid_speed.integral_limit_max = 25000,
        .pid_speed.integral_limit_min = -25000,
        .pid_speed.integral_deadband = 10,

        .pid_position.kp = 10,
        .pid_position.ki = 0.4,
        .pid_position.kd = 0.1,
        .pid_position.output_limit_max = 25000,
        .pid_position.output_limit_min = -25000,
        .pid_position.integral_limit_max = 25000,
        .pid_position.integral_limit_min = -25000,
        .pid_position.integral_deadband = 0,
    },
    {
        .pid_speed.kp = 8,
        .pid_speed.ki = 1,
        .pid_speed.kd = 0.2,
        .pid_speed.output_limit_max = 14000,
        .pid_speed.output_limit_min = -14000,
        .pid_speed.integral_limit_max = 14000,
        .pid_speed.integral_limit_min = -14000,
        .pid_speed.integral_deadband = 10,

        .pid_position.kp = 10,
        .pid_position.ki = 0.4,
        .pid_position.kd = 0.1,
        .pid_position.output_limit_max = 14000,
        .pid_position.output_limit_min = -14000,
        .pid_position.integral_limit_max = 20000,
        .pid_position.integral_limit_min = -20000,
        .pid_position.integral_deadband = 0,
    },
    {
        .pid_speed.kp = 8,
        .pid_speed.ki = 1,
        .pid_speed.kd = 0.2,
        .pid_speed.output_limit_max = 14000,
        .pid_speed.output_limit_min = -14000,
        .pid_speed.integral_limit_max = 14000,
        .pid_speed.integral_limit_min = -14000,
        .pid_speed.integral_deadband = 10,

        .pid_position.kp = 10,
        .pid_position.ki = 0.4,
        .pid_position.kd = 0.1,
        .pid_position.output_limit_max = 14000,
        .pid_position.output_limit_min = -14000,
        .pid_position.integral_limit_max = 20000,
        .pid_position.integral_limit_min = -20000,
        .pid_position.integral_deadband = 0,
    },
};

void _M2006G2_SetCurrent(M2006G2_t* m2006, double current)
{
    uint16_t index = m2006 - M2006G2;
    if(index > 7 || m2006 == NULL) return;
    current = _Limit(current, 10000, -10000);
    int temp = (int)current;
    M2006G2_TxData[index * 2] = (uint8_t)((temp >> 8) & 0xff);
    M2006G2_TxData[index * 2 + 1] = (uint8_t)(temp & 0xff);
}
void _M2006G2_SetSpeed(M2006G2_t* m2006, double target_spd)
{
    uint16_t index = m2006 - M2006G2;
    if(index > 7 || m2006 == NULL) return;
    m2006->pid_speed.last_error = m2006->pid_speed.curr_error;
    m2006->pid_speed.curr_error = target_spd - (double)m2006->status.speed;
    //积分分离和误差死区
    if(_Absolut(m2006->pid_speed.curr_error) <= m2006->pid_speed.integral_startzone)
    {
        if(_Absolut(m2006->pid_speed.curr_error) <= m2006->pid_speed.integral_deadband)
            m2006->pid_speed.integral += 0;
        else
            m2006->pid_speed.integral += m2006->pid_speed.curr_error * m2006->pid_speed.ki;
    }
    else
        m2006->pid_speed.integral = 0;
    //积分限幅
    m2006->pid_speed.integral = _Limit(m2006->pid_speed.integral, m2006->pid_speed.integral_limit_max, m2006->pid_speed.integral_limit_min);
    //PID计算
    double p_calc = m2006->pid_speed.kp * m2006->pid_speed.curr_error;
    double i_calc = m2006->pid_speed.integral;
    double d_calc = m2006->pid_speed.kd * (m2006->pid_speed.curr_error - m2006->pid_speed.last_error);

    m2006->pid_speed.output = p_calc + i_calc + d_calc;
    //输出限幅
    m2006->pid_speed.output = _Limit(m2006->pid_speed.output, m2006->pid_speed.output_limit_max, m2006->pid_speed.output_limit_min);

    _M2006G2_SetCurrent(m2006, m2006->pid_speed.output);
}
void _M2006G2_SetPosition(M2006G2_t* m2006, double target_pos)
{
    uint16_t index = m2006 - M2006G2;
    if(index > 7 || m2006 == NULL) return;
    m2006->pid_position.last_error = m2006->pid_position.curr_error;
    m2006->pid_position.curr_error = target_pos - (double)m2006->status.output_position;

    //积分分离和误差死区
    if(_Absolut(m2006->pid_position.curr_error) <= m2006->pid_position.integral_startzone)
    {
        if(_Absolut(m2006->pid_position.curr_error) <= m2006->pid_position.integral_deadband)
            m2006->pid_position.integral += 0;
        else
            m2006->pid_position.integral += m2006->pid_position.curr_error * m2006->pid_position.ki;
    }
    else
        m2006->pid_position.integral = 0;
    //积分限幅
    m2006->pid_position.integral = _Limit(m2006->pid_position.integral, m2006->pid_position.integral_limit_max, m2006->pid_position.integral_limit_min);
    //PID计算
    double p_calc = m2006->pid_position.kp * m2006->pid_position.curr_error;
    double i_calc = m2006->pid_position.integral;
    double d_calc = m2006->pid_position.kd * (m2006->pid_position.curr_error - m2006->pid_position.last_error);

    m2006->pid_position.output = p_calc + i_calc + d_calc;
    //输出限幅
    m2006->pid_position.output = _Limit(m2006->pid_position.output, m2006->pid_position.output_limit_max, m2006->pid_position.output_limit_min);

    _M2006G2_SetSpeed(m2006, m2006->pid_position.output);
}
void _M2006G2_FDCAN_Transmit(FDCAN_HandleTypeDef* hfdcan, uint8_t *pData, uint16_t ID)
{
    FdcanBsp_Transmit_Classic_StdData(hfdcan, ID, pData, FDCAN_DLC_BYTES_8);
}
void _M2006G2_MotorControl(M2006G2_t* m2006)
{
    uint16_t index = m2006 - M2006G2;
    if(m2006 == NULL || index > 7)
        return;
    if(index > 3)
        _M2006G2_FDCAN_Transmit(M2006G2_hfdcan, &((uint8_t*)M2006G2_TxData)[8], M2006G2_TxID2);
    else
        _M2006G2_FDCAN_Transmit(M2006G2_hfdcan, &((uint8_t*)M2006G2_TxData)[0], M2006G2_TxID1);
}
void M2006G2_Init(FDCAN_HandleTypeDef* hc)
{
    M2006G2_hfdcan = hc;
}
void M2006G2_FDCAN_Callback(FdcanBspReceive_t* rxmsg)
{
    if(rxmsg->hfdcan == 0 || rxmsg->hfdcan->Instance != M2006G2_hfdcan->Instance) return;
    if(rxmsg->RxMessage.IdType != FDCAN_STANDARD_ID)
        return;
    if(rxmsg->RxMessage.Identifier < M2006G2_ID_BEGIN || rxmsg->RxMessage.Identifier > M2006G2_ID_END)
        return;
    uint8_t index = rxmsg->RxMessage.Identifier - M2006G2_ID_BEGIN;

     M2006G2[index].status.position = (uint16_t)((rxmsg->RxData[0] << 8) | rxmsg->RxData[1]);
    M2006G2[index].status.speed = (int16_t)(rxmsg->RxData[2] << 8) | rxmsg->RxData[3];
    M2006G2[index].status.torque = (int16_t)(rxmsg->RxData[4] << 8) | rxmsg->RxData[5];

    //得到编码器值
    if(M2006G2[index].outerEncoderSource == NULL)
    {
        if ((int32_t)M2006G2[index].status.position - (int32_t)M2006G2[index].temp.last_encoder > 4095)                  //当前编码器数值从0突变到8191，刚好反转一圈
            M2006G2[index].temp.position_base -= 8191;                                                       //base减一圈
        else if ((int32_t)M2006G2[index].status.position - (int32_t)M2006G2[index].temp.last_encoder < -4095)          //当前编码器数值从8191突变到0，刚好正转一圈
            M2006G2[index].temp.position_base += 8191;                                                       //base加一圈
        M2006G2[index].temp.last_encoder = M2006G2[index].status.position;                                                  //更新上一次的角度值
        M2006G2[index].status.total_cnt = M2006G2[index].temp.position_base + M2006G2[index].status.position;     //总编码器值=base+当前编码器值
        M2006G2[index].status.output_position = ((M2006G2[index].status.total_cnt - M2006G2[index].status.zero_offset) / M2006G2_REDUCTION);
    }
    else
    {
        M2006G2[index].status.total_cnt = -M2006G2[index].outerEncoderSource();
        M2006G2[index].status.output_position = M2006G2[index].status.total_cnt;
		M2006G2[index].status.position = M2006G2[index].status.output_position;
	}

    if(M2006G2[index].status.mode == M2006G2_MODE_SPEED)
    {
        _M2006G2_SetSpeed(&M2006G2[index], M2006G2[index].status.target_speed);
    }
    else if(M2006G2[index].status.mode == M2006G2_MODE_POSITION)
    {
        _M2006G2_SetPosition(&M2006G2[index], M2006G2[index].status.target_position);
    }
    _M2006G2_MotorControl(&M2006G2[index]);
}
void M2006G2_SetSpeed(uint16_t index_from_1, double spd)
{
    if(index_from_1 > 8 || index_from_1 < 1) return;
    M2006G2[index_from_1 - 1].status.target_speed = spd;
    M2006G2[index_from_1 - 1].status.mode = M2006G2_MODE_SPEED;
}
void M2006G2_SetPosition(uint16_t index_from_1, double pos)
{
    if(index_from_1 > 8 || index_from_1 < 1) return;
    M2006G2[index_from_1 - 1].status.target_position = pos;
    M2006G2[index_from_1 - 1].status.mode = M2006G2_MODE_POSITION;
}
void M2006G2_SetZero(uint16_t index_from_1)
{
    if(index_from_1 > 8 || index_from_1 < 1) return;
    M2006G2[index_from_1 - 1].status.zero_offset = M2006G2[index_from_1 - 1].status.total_cnt;
}
int64_t M2006G2_GetPosition(uint16_t index_from_1)
{
	if(index_from_1 > 8 || index_from_1 < 1) return 0;
	return M2006G2[index_from_1 - 1].status.position;
}
int64_t M2006G2_GetSpeed(uint16_t index_from_1)
{
	if(index_from_1 > 8 || index_from_1 < 1) return 0;
	return M2006G2[index_from_1 - 1].status.speed;
}
void M2006G2_SetOffset(uint16_t index_from_1, int64_t offset)
{
    if(index_from_1 > 8 || index_from_1 < 1) return;
	M2006G2[index_from_1 - 1].status.zero_offset = offset;
}	
