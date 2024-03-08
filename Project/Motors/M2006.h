#pragma once
#include "main.h"
#include "fdcan.h"
#include "fdcan_bsp.h"


typedef struct 
{
    uint16_t id;
    uint16_t group;
} M2006_t;
void M2006_SetSpeed(M2006_t* m2006, double spd);
void M2006_SetPosition(M2006_t* m2006, double pos);
void M2006_SetZero(M2006_t* m2006);
int64_t M2006_GetPosition(M2006_t* m2006);
int64_t M2006_GetSpeed(M2006_t* m2006);
void M2006_MotorControl(M2006_t* m2006);
void M2006_SetOffset(M2006_t* m2006, int64_t offset);

typedef struct 
{
    double last_error;
    double curr_error;
    double integral;//积分

    double kp;
    double ki;
    double kd;

    double integral_limit_max;
    double integral_limit_min;
    double integral_deadband;
    double integral_startzone;

    double output_limit_max;
    double output_limit_min;
    double output;
} M2006G1_PID_t;
typedef struct 
{
    int64_t speed;
    int64_t position;
    int64_t torque;
    enum
    {
        M2006G1_MODE_SPEED,
        M2006G1_MODE_POSITION,
    } mode;
    int64_t total_cnt;
    int64_t zero_offset;
    int64_t output_position;

    double target_speed;
    double target_position;
    double target_torque;
	int64_t outer_position;

} M2006G1_Status_t;
typedef struct 
{
    int64_t position_base;
    int64_t last_encoder;
} M2006G1_Temp_t;
typedef struct 
{
    M2006G1_PID_t pid_speed;
    M2006G1_PID_t pid_position;
    M2006G1_Status_t status;
    M2006G1_Temp_t temp;
    int64_t (*outerEncoderSource)(void);
} M2006G1_t;
extern M2006G1_t M2006G1[8];

void M2006G1_Init(FDCAN_HandleTypeDef* hc);
void M2006G1_FDCAN_Callback(FdcanBspReceive_t* rxmsg);
void M2006G1_SetSpeed(uint16_t index_from_1, double spd);
void M2006G1_SetPosition(uint16_t index_from_1, double pos);
void M2006G1_SetZero(uint16_t index_from_1);
int64_t M2006G1_GetPosition(uint16_t index_from_1);
int64_t M2006G1_GetSpeed(uint16_t index_from_1);
void M2006G1_MotorControl(uint16_t index_from_1);
void M2006G1_SetOffset(uint16_t index_from_1, int64_t offset);


typedef struct 
{
    double last_error;
    double curr_error;
    double integral;//积分

    double kp;
    double ki;
    double kd;

    double integral_limit_max;
    double integral_limit_min;
    double integral_deadband;
    double integral_startzone;

    double output_limit_max;
    double output_limit_min;
    double output;
} M2006G2_PID_t;
typedef struct 
{
    int64_t speed;
    int64_t position;
    int64_t torque;
    enum
    {
        M2006G2_MODE_SPEED,
        M2006G2_MODE_POSITION,
    } mode;
    int64_t total_cnt;
    int64_t zero_offset;
    int64_t output_position;

    double target_speed;
    double target_position;
    double target_torque;
	

} M2006G2_Status_t;
typedef struct 
{
    int64_t position_base;
    int64_t last_encoder;
} M2006G2_Temp_t;
typedef struct 
{
    M2006G2_PID_t pid_speed;
    M2006G2_PID_t pid_position;
    M2006G2_Status_t status;
    M2006G2_Temp_t temp;
    int64_t (*outerEncoderSource)(void);
} M2006G2_t;
extern M2006G2_t M2006G2[8];

void M2006G2_Init(FDCAN_HandleTypeDef* hc);
void M2006G2_FDCAN_Callback(FdcanBspReceive_t* rxmsg);
void M2006G2_SetSpeed(uint16_t index_from_1, double spd);
void M2006G2_SetPosition(uint16_t index_from_1, double pos);
void M2006G2_SetZero(uint16_t index_from_1);
int64_t M2006G2_GetPosition(uint16_t index_from_1);
int64_t M2006G2_GetSpeed(uint16_t index_from_1);
void M2006G2_MotorControl(uint16_t index_from_1);
void M2006G2_SetOffset(uint16_t index_from_1, int64_t offset);

