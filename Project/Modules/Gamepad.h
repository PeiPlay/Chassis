#pragma once
#include "main.h"
#include "spi.h"


#define NRF24L01_INIT (nRF24L01_t){                     \
    .ce_pin = GPIO_PIN_2,                               \
    .ce_port = GPIOD,                                   \
    .nss_pin = GPIO_PIN_15,                             \
    .nss_port = GPIOA,                                  \
    .irq_pin = GPIO_PIN_3,                              \
    .irq_port = GPIOB,                                  \
    .hspi = &hspi2,                                     \
    .receive_address.high = {0x22,0x2C,0x01,0x28},      \
    .receive_address.p1 = 0x29,                         \
    .receive_address.p2 = 0x01,                         \
    .receive_address.p3 = 0x02,                         \
    .receive_address.p4 = 0x03,                         \
    .receive_address.p5 = 0x04,                         \
    .transmit_address = {0x00, 0x22,0x24,0x2C,0x01},    \
    .rf_channel = 6                                     \
}
typedef uint8_t nRF24L01_ReceiveAddrHigh_t[4];
typedef uint8_t nRF24L01_ReceiveAddrLow_t;
typedef uint8_t nRF24L01_TransmitAddr_t[5];
typedef struct 
{
    nRF24L01_ReceiveAddrHigh_t high;
    nRF24L01_ReceiveAddrLow_t p1;
    nRF24L01_ReceiveAddrLow_t p2;
    nRF24L01_ReceiveAddrLow_t p3;
    nRF24L01_ReceiveAddrLow_t p4;
    nRF24L01_ReceiveAddrLow_t p5;
} nRF24L01_ReceiveAddress_t;
typedef struct
{
    uint8_t buffer[32];
    uint8_t length;
} nRF24L01_ReceivedData_t;

typedef uint16_t nRF24L01_CE_Pin_t;
typedef GPIO_TypeDef* nRF24L01_CE_Port_t;
typedef uint16_t nRF24L01_NSS_Pin_t;
typedef GPIO_TypeDef* nRF24L01_NSS_Port_t;
typedef uint16_t nRF24L01_IRQ_Pin_t;
typedef GPIO_TypeDef* nRF24L01_IRQ_Port_t;
typedef SPI_HandleTypeDef* nRF24L01_SPI_t;
typedef struct 
{
    uint64_t check_buf[2];
    uint32_t check_index;
} nRF24L01_Check_t;

typedef struct 
{
    nRF24L01_CE_Pin_t ce_pin;
    nRF24L01_CE_Port_t ce_port;
    nRF24L01_NSS_Pin_t nss_pin;
    nRF24L01_NSS_Port_t nss_port;
    nRF24L01_IRQ_Pin_t irq_pin;
    nRF24L01_IRQ_Port_t irq_port;
    nRF24L01_SPI_t hspi;
    nRF24L01_Check_t check;                     //连接性校验
    enum 
    {
        Nrfsignal_Strong,
        Nrfsignal_Weak,
        Nrfsignal_Disconnected
    } signal;
    nRF24L01_ReceiveAddress_t receive_address;  //接收地址
    nRF24L01_TransmitAddr_t transmit_address;   //发送地址
    uint8_t rf_channel;                         //射频通道
    nRF24L01_ReceivedData_t receive_data[6];    //接收到的数据
} nRF24L01_t;


typedef enum 
{
    _DisConnected,
    _Connected
} nRF24L01_Connectivity;    //NRF连接状态


extern nRF24L01_t nRF24L01;
void nRF24L01_Init(void);                                                   //NRF初始化函数
nRF24L01_Connectivity nRF24L01_CheckConnectivity(void);                     //检查NRF是否连接（读写寄存器）
void nRF24L01_IRQ_EXTI_Callback(uint16_t GPIO_Pin);                         //NRF中断回调函数，用于接收数据
void nRF24L01_Tansmit(uint8_t* tx_buf, uint8_t length);   //发送数据
void nRF24L01_SetTransmitAddress(uint8_t* transmit_addr);   //设置发送地址


/***********************发送数据包的定义***********************/
typedef enum{
    Gamepad_Idle,
    Gamepad_ValueModify,

/*自定义的Task枚举*/

	
/*自定义的Task枚举*/
//功能按键的默认Task枚举
    Gamepad_FunctionKey_Key1_Click,
    Gamepad_FunctionKey_Key1_MultiClicks,
    Gamepad_FunctionKey_Key1_Long,
    Gamepad_FunctionKey_Key2_Click,
    Gamepad_FunctionKey_Key2_MultiClicks,
    Gamepad_FunctionKey_Key2_Long,
    Gamepad_FunctionKey_Key3_Click,
    Gamepad_FunctionKey_Key3_MultiClicks,
    Gamepad_FunctionKey_Key3_Long,
    Gamepad_FunctionKey_Key4_Click,
    Gamepad_FunctionKey_Key4_MultiClicks,
    Gamepad_FunctionKey_Key4_Long,
    Gamepad_FunctionKey_Key5_Click,
    Gamepad_FunctionKey_Key5_MultiClicks,
    Gamepad_FunctionKey_Key5_Long,
    Gamepad_FunctionKey_Key6_Click,
    Gamepad_FunctionKey_Key6_MultiClicks,
    Gamepad_FunctionKey_Key6_Long,
    Gamepad_FunctionKey_Key7_Click,
    Gamepad_FunctionKey_Key7_MultiClicks,
    Gamepad_FunctionKey_Key7_Long,
    Gamepad_FunctionKey_Key8_Click,
    Gamepad_FunctionKey_Key8_MultiClicks,
    Gamepad_FunctionKey_Key8_Long,
} TaskID_t;
//键盘状态数据包
typedef struct __attribute__((packed))
{
    uint8_t fk_0:1;
    uint8_t fk_1:1;
    uint8_t fk_2:1;
    uint8_t fk_3:1;
    uint8_t fk_4:1;
    uint8_t fk_5:1;
    uint8_t fk_6:1;
    uint8_t fk_7:1;
} FunctionKeyStatus_t;
typedef struct __attribute__((packed))
{
    uint8_t sw_0:1;
    uint8_t sw_1:1;
    uint8_t sw_2:1;
    uint8_t sw_3:1;
    uint8_t sw_4:1;
    uint8_t sw_5:1;
    uint8_t sw_6:1;
    uint8_t sw_7:1;
} SwitchStatus_t;
typedef struct __attribute__((packed))
{
    uint16_t nk_0:1;
    uint16_t nk_1:1;
    uint16_t nk_2:1;
    uint16_t nk_3:1;
    uint16_t nk_4:1;
    uint16_t nk_5:1;
    uint16_t nk_6:1;
    uint16_t nk_7:1;
    uint16_t nk_8:1;
    uint16_t nk_9:1;
    uint16_t nk_10:1;
    uint16_t nk_11:1;
    uint16_t nk_12:1;
    uint16_t nk_13:1;
    uint16_t nk_14:1;
    uint16_t nk_15:1;
} NormalKeyStatus_t;
typedef struct __attribute__((packed))
{
    int8_t right_x;
    int8_t right_y;
    int8_t left_x;
    int8_t left_y;
} RockerStatus_t;
typedef struct __attribute__((packed))
{
    FunctionKeyStatus_t functionKeys;
    SwitchStatus_t switches;
    NormalKeyStatus_t normalKeys;
    RockerStatus_t rockers;
} KeyBoardState_t;
//手柄任务数据包
typedef struct __attribute__((packed))
{
    uint8_t type:1;
    uint8_t index:7;
    union __attribute__((packed))
    {
        int32_t int32;
        float float32;
    } value;    
} PlugIn_Gamepad_ValueModify_t;
typedef uint16_t GamepadTaskID_t;
typedef union __attribute__((packed))
{
    int8_t int8;
    uint8_t uint8;
    int16_t int16;
    uint16_t uint16;
    int32_t int32;
    uint32_t uint32;
    float float32;
    double double64;
    int64_t int64;
    uint64_t uint64;
    uint8_t data_array[16];
    char string[16];
    //默认插件的数据类型
    PlugIn_Gamepad_ValueModify_t value_modify;
    /*可自定义不超过16字节的结构体放入该区域*/


    /*可自定义不超过16字节的结构体放入该区域*/
} GamepadTaskData_t;
typedef struct __attribute__((packed))
{
    GamepadTaskID_t task_id;
    GamepadTaskData_t task_data;
} GamepadTask_t;
typedef struct __attribute__((packed))
{
    KeyBoardState_t keyBoard;
    GamepadTask_t gamepadTask;
    uint8_t crc_check;
    uint8_t reserved[32 - (sizeof(KeyBoardState_t) + sizeof(GamepadTask_t) + 1)];
} DataPackage_Gamepad_t;

extern DataPackage_Gamepad_t GamepadPackage;
KeyBoardState_t* Gamepad_GetKeyBoardState(void);
GamepadTask_t* Gamepad_GetGamepadTask(void);
GamepadTaskData_t* Gamepad_GetTaskData(void);
FunctionKeyStatus_t* Gamepad_GetFunctionKeys(void);
SwitchStatus_t* Gamepad_GetSwitches(void);
NormalKeyStatus_t* Gamepad_GetNormalKeys(void);
RockerStatus_t* Gamepad_GetRockers(void);
void Gamepad_nRF24L01_ReceiveCallback(DataPackage_Gamepad_t* recv);
