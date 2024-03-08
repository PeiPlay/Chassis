#include "fdcan_bsp.h"
#define FDCAN_TRANSMIT_ID_TYPE                  0x1
#define FDCAN_TRANSMIT_FRAME_TYPE               0x2
#define FDCAN_TRANSMIT_CAN_TYPE                 0x4
/*
*   初始化对应FDCAN外设的标准帧接收过滤器
*   其中FDCAN1和FDCAN3使用FILTER_RANGE模式进行过滤，FilterID1和FilterID2表示通过的标识符范围
*   FDCAN2使用FILTER_MASK模式进行过滤，FilterID1和FilterID2表示半字掩码
*   FDCAN1的使用FIFO0缓冲区，FDCAN2和FDCAN3关联到FIFO1缓冲区
*/
void _FdcanBsp_InitStandardFrame(FDCAN_HandleTypeDef *hfdcan)
{
    if(HAL_OK != HAL_FDCAN_ConfigFilter(hfdcan, &(FDCAN_FilterTypeDef)
    {		
		.IdType = FDCAN_STANDARD_ID,
        .FilterConfig = (hfdcan->Instance == FDCAN1) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1,
		.FilterType = (hfdcan->Instance == FDCAN2) ? FDCAN_FILTER_MASK : FDCAN_FILTER_RANGE,
		.FilterID1 = 0x00,
        .FilterID2 = (hfdcan->Instance == FDCAN2) ? 0x00 : 0x7FF
    })) Error_Handler();
}
/*
*   初始化对应FDCAN外设的扩展帧接收过滤器
*   其中FDCAN1和FDCAN3使用FILTER_RANGE模式进行过滤，FilterID1和FilterID2表示通过的标识符范围
*   FDCAN2使用FILTER_MASK模式进行过滤，FilterID1和FilterID2表示半字掩码
*   FDCAN1的使用FIFO0缓冲区，FDCAN2和FDCAN3关联到FIFO1缓冲区
*/
void _FdcanBsp_InitExtendedFrame(FDCAN_HandleTypeDef *hfdcan)
{    
    if(HAL_OK != HAL_FDCAN_ConfigFilter(hfdcan, &(FDCAN_FilterTypeDef)
    {
        .IdType = FDCAN_EXTENDED_ID,
        .FilterConfig = (hfdcan->Instance == FDCAN1) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1,
        .FilterType = (hfdcan->Instance == FDCAN2) ? FDCAN_FILTER_MASK : FDCAN_FILTER_RANGE,
        .FilterID1 = 0x00,
        .FilterID2 = (hfdcan->Instance == FDCAN2) ? 0x00 : 0x1FFFFFFF
    })) Error_Handler();
}
/*
*   对传入的FDCAN外设进行默认初始化：
*       接收所有ID，FDCAN1的使用FIFO0缓冲区，其余使用FIFO1缓冲区
*/
void FdcanBsp_Init_Default(FDCAN_HandleTypeDef *hfdcan)
{
	if(hfdcan == NULL) Error_Handler();
    _FdcanBsp_InitStandardFrame(hfdcan);
    _FdcanBsp_InitExtendedFrame(hfdcan);
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, DISABLE, DISABLE) ||
        HAL_FDCAN_ActivateNotification(hfdcan, (hfdcan->Instance == FDCAN1) ? FDCAN_IT_RX_FIFO0_NEW_MESSAGE : FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) ||
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_BUS_OFF, 0)
    ) Error_Handler();
    HAL_FDCAN_Start(hfdcan);
}
HAL_StatusTypeDef FdcanBsp_Transmit(FDCAN_HandleTypeDef *hfdcan, uint8_t transmit_Para, uint32_t id, void *pData, uint32_t dlcField)
{
    FDCAN_TxHeaderTypeDef TxMessage = {0};
    TxMessage.Identifier            = id;
    TxMessage.DataLength            = dlcField;
    TxMessage.ErrorStateIndicator   = FDCAN_ESI_ACTIVE;
    TxMessage.TxEventFifoControl    = FDCAN_NO_TX_EVENTS;
    TxMessage.BitRateSwitch = FDCAN_BRS_ON;           
    TxMessage.IdType    	=   (transmit_Para & FDCAN_TRANSMIT_ID_TYPE)	? FDCAN_EXTENDED_ID     : FDCAN_STANDARD_ID;
    TxMessage.TxFrameType   =   (transmit_Para & FDCAN_TRANSMIT_FRAME_TYPE) ? FDCAN_REMOTE_FRAME    : FDCAN_DATA_FRAME;
    TxMessage.FDFormat      =   (transmit_Para & FDCAN_TRANSMIT_CAN_TYPE)   ? FDCAN_FD_CAN          : FDCAN_CLASSIC_CAN;
    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, pData);
}
HAL_StatusTypeDef FdcanBsp_Transmit_Classic_StdData(FDCAN_HandleTypeDef *hfdcan, uint32_t id, void *pData, uint32_t dlcField)
{
    return FdcanBsp_Transmit(hfdcan, FDCAN_TRANSMIT_ID_STD | FDCAN_TRANSMIT_FRAME_DATA | FDCAN_TRANSMIT_CAN_CLASSIC, id, pData, dlcField);
}
HAL_StatusTypeDef FdcanBsp_Transmit_Classic_StdRemote(FDCAN_HandleTypeDef *hfdcan, uint32_t id, void *pData, uint32_t dlcField)
{
    return FdcanBsp_Transmit(hfdcan, FDCAN_TRANSMIT_ID_STD | FDCAN_TRANSMIT_FRAME_REMOTE | FDCAN_TRANSMIT_CAN_CLASSIC, id, pData, dlcField);
}
HAL_StatusTypeDef FdcanBsp_Transmit_Classic_ExtData(FDCAN_HandleTypeDef *hfdcan, uint32_t id, void *pData, uint32_t dlcField)
{
    return FdcanBsp_Transmit(hfdcan, FDCAN_TRANSMIT_ID_EXT | FDCAN_TRANSMIT_FRAME_DATA | FDCAN_TRANSMIT_CAN_CLASSIC, id, pData, dlcField);
}
HAL_StatusTypeDef FdcanBsp_Transmit_Classic_ExtRemote(FDCAN_HandleTypeDef *hfdcan, uint32_t id, void *pData, uint32_t dlcField)
{
    return FdcanBsp_Transmit(hfdcan, FDCAN_TRANSMIT_ID_EXT | FDCAN_TRANSMIT_FRAME_REMOTE | FDCAN_TRANSMIT_CAN_CLASSIC, id, pData, dlcField);
}
HAL_StatusTypeDef FdcanBsp_Transmit_FD_StdData(FDCAN_HandleTypeDef *hfdcan, uint32_t id, void *pData, uint32_t dlcField)
{
    return FdcanBsp_Transmit(hfdcan, FDCAN_TRANSMIT_ID_STD | FDCAN_TRANSMIT_FRAME_DATA | FDCAN_TRANSMIT_CAN_FD, id, pData, dlcField);
}
HAL_StatusTypeDef FdcanBsp_Transmit_FD_StdRemote(FDCAN_HandleTypeDef *hfdcan, uint32_t id, void *pData, uint32_t dlcField)
{
    return FdcanBsp_Transmit(hfdcan, FDCAN_TRANSMIT_ID_STD | FDCAN_TRANSMIT_FRAME_REMOTE | FDCAN_TRANSMIT_CAN_FD, id, pData, dlcField);
}
HAL_StatusTypeDef FdcanBsp_Transmit_FD_ExtData(FDCAN_HandleTypeDef *hfdcan, uint32_t id, void *pData, uint32_t dlcField)
{
    return FdcanBsp_Transmit(hfdcan, FDCAN_TRANSMIT_ID_EXT | FDCAN_TRANSMIT_FRAME_DATA | FDCAN_TRANSMIT_CAN_FD, id, pData, dlcField);
}
HAL_StatusTypeDef FdcanBsp_Transmit_FD_ExtRemote(FDCAN_HandleTypeDef *hfdcan, uint32_t id, void *pData, uint32_t dlcField)
{
    return FdcanBsp_Transmit(hfdcan, FDCAN_TRANSMIT_ID_EXT | FDCAN_TRANSMIT_FRAME_REMOTE | FDCAN_TRANSMIT_CAN_FD, id, pData, dlcField);
}

FdcanBspReceive_t FdcanBsp_DequeueRxPackage(FDCAN_HandleTypeDef *hfdcan)
{
    FdcanBspReceive_t rxmsg = {0};
    if(hfdcan == NULL) return rxmsg;
    rxmsg.hfdcan = hfdcan;
    rxmsg.RxFIFO = (hfdcan->Instance == FDCAN1) ? FDCAN_RX_FIFO0 : FDCAN_RX_FIFO1;
    HAL_FDCAN_GetRxMessage(hfdcan, rxmsg.RxFIFO, &rxmsg.RxMessage, rxmsg.RxData);
    return rxmsg;
}
