#pragma once
#include "main.h"
#include "fdcan.h"
// 发送数据帧ID格式
#define FDCAN_TRANSMIT_ID_STD                   (0 << 0)
#define FDCAN_TRANSMIT_ID_EXT                   (1 << 0)
// 发送数据帧类型
#define FDCAN_TRANSMIT_FRAME_DATA               (0 << 1)
#define FDCAN_TRANSMIT_FRAME_REMOTE             (1 << 1)
// 发送数据帧类型
#define FDCAN_TRANSMIT_CAN_CLASSIC              (0 << 2)
#define FDCAN_TRANSMIT_CAN_FD                   (1 << 2)
/*
*   对传入的FDCAN外设进行默认初始化：
*       接收所有ID，FDCAN1的使用FIFO0缓冲区，其余使用FIFO1缓冲区
*/
void FdcanBsp_Init_Default(FDCAN_HandleTypeDef *hfdcan);
/*
* @brief  FDCAN发送数据
* @param  hfdcan: FDCAN句柄
* @param  transmit_Para: 为FDCAN_TRANSMIT_IDxxx | FDCAN_TRANSMIT_FRAMExxx | FDCAN_TRANSMIT_CANxxx之间的或组合，分别表示ID类型、帧类型、CAN类型
* @param  id: 数据帧ID
* @param  pData: 数据指针
* @param  dlcField: 数据长度：FDCAN需要特定的数据长度，如8字节数据长度对应FDCAN_DLC_BYTES_8，16字节数据长度对应FDCAN_DLC_BYTES_16
* @retval HAL_StatusTypeDef
*/
HAL_StatusTypeDef FdcanBsp_Transmit(FDCAN_HandleTypeDef *hfdcan, uint8_t transmit_Para, uint32_t id, void *pData, uint32_t dlcField);
HAL_StatusTypeDef FdcanBsp_Transmit_Classic_StdData(FDCAN_HandleTypeDef *hfdcan, uint32_t id, void *pData, uint32_t dlcField);
HAL_StatusTypeDef FdcanBsp_Transmit_Classic_StdRemote(FDCAN_HandleTypeDef *hfdcan, uint32_t id, void *pData, uint32_t dlcField);
HAL_StatusTypeDef FdcanBsp_Transmit_Classic_ExtData(FDCAN_HandleTypeDef *hfdcan, uint32_t id, void *pData, uint32_t dlcField);
HAL_StatusTypeDef FdcanBsp_Transmit_Classic_ExtRemote(FDCAN_HandleTypeDef *hfdcan, uint32_t id, void *pData, uint32_t dlcField);
HAL_StatusTypeDef FdcanBsp_Transmit_FD_StdData(FDCAN_HandleTypeDef *hfdcan, uint32_t id, void *pData, uint32_t dlcField);
HAL_StatusTypeDef FdcanBsp_Transmit_FD_StdRemote(FDCAN_HandleTypeDef *hfdcan, uint32_t id, void *pData, uint32_t dlcField);
HAL_StatusTypeDef FdcanBsp_Transmit_FD_ExtData(FDCAN_HandleTypeDef *hfdcan, uint32_t id, void *pData, uint32_t dlcField);
HAL_StatusTypeDef FdcanBsp_Transmit_FD_ExtRemote(FDCAN_HandleTypeDef *hfdcan, uint32_t id, void *pData, uint32_t dlcField);

typedef struct 
{
    FDCAN_HandleTypeDef *hfdcan;
    FDCAN_RxHeaderTypeDef RxMessage;
    uint8_t RxData[64];
    uint32_t RxFIFO;
} FdcanBspReceive_t;
/*
* @brief  FDCAN接收数据
* @param  hfdcan: FDCAN句柄
* @retval FdcanBspReceive_t 包含接收到的数据和数据帧信息
*/
FdcanBspReceive_t FdcanBsp_DequeueRxPackage(FDCAN_HandleTypeDef *hfdcan);
