#include "Gamepad.h"
#include <string.h>

DataPackage_Gamepad_t GamepadPackage;

/*调用一下函数方便获取元素*/
KeyBoardState_t* Gamepad_GetKeyBoardState(void)
{
    return &GamepadPackage.keyBoard;
}
GamepadTask_t* Gamepad_GetGamepadTask(void)
{
    return &GamepadPackage.gamepadTask;
}
GamepadTaskData_t* Gamepad_GetTaskData(void)
{
    return &GamepadPackage.gamepadTask.task_data;
}
FunctionKeyStatus_t* Gamepad_GetFunctionKeys(void)
{
    return &GamepadPackage.keyBoard.functionKeys;
}
SwitchStatus_t* Gamepad_GetSwitches(void)
{
    return &GamepadPackage.keyBoard.switches;
}
NormalKeyStatus_t* Gamepad_GetNormalKeys(void)
{
    return &GamepadPackage.keyBoard.normalKeys;
}
RockerStatus_t* Gamepad_GetRockers(void)
{
    return &GamepadPackage.keyBoard.rockers;
}

uint8_t _Calculate_CRC(uint8_t* data, uint32_t length) {
    uint8_t crc = 0;
    for(uint32_t i = 0; i < length; i++) {
        crc ^= data[i];
        for(uint8_t j = 0; j < 8; j++) {
            if(crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void Gamepad_nRF24L01_ReceiveCallback(DataPackage_Gamepad_t* recv)
{
    if(recv->crc_check == _Calculate_CRC((uint8_t*)recv, sizeof(KeyBoardState_t) + sizeof(GamepadTask_t))) {
        memcpy(&GamepadPackage, recv, sizeof(DataPackage_Gamepad_t));
    } 
    /*自定义在接收到数据后的中断回调代码*/
    
}


//nRF24L01寄存器操作命令
#define NRF_CMD_READ_REG                0x00  //读配置寄存器,低5位为寄存器地址
#define NRF_CMD_WRITE_REG               0x20  //写配置寄存器,低5位为寄存器地址
#define NRF_CMD_READ_RX_PAYLOAD         0x61  //读RX有效数据,1~32字节
#define NRF_CMD_WRITE_TX_PAYLOAD        0xA0  //写TX有效数据,1~32字节
#define NRF_CMD_FLUSH_TX                0xE1  //清除TX FIFO寄存器.发射模式下用
#define NRF_CMD_FLUSH_RX                0xE2  //清除RX FIFO寄存器.接收模式下用，,在传输应答信号过程中不应执行此指令
#define NRF_CMD_REUSE_TX_PAYLOAD        0xE3  //数据重发,重新使用上一包数据,CE为高,数据包被不断发送
#define NRF_CMD_READ_RX_PAYLOAD_WID     0x60  //读RX FIFO顶层R_RX_PLOAD载荷长度,若超出32字节则清除RX FIFO
#define NRF_CMD_WRITE_ACK_PAYLOAD       0xA8  //RX模式下,将PIPEppp(ppp的范围为000~101)中的AcK包与载荷一起写入,最多可以有3个挂起的ACK包.
#define NRF_CMD_WRITE_TX_PAYLOAD_NOACK  0xB0  //发送数据,但禁止自动应答,发射模式用
#define NRF_CMD_NOP                     0xFF  //空操作,可以用来读状态寄存器

#define NRF_REG_CONFIG                  0x00  //配置收发状态,CRC校验模式以及收发状态响应方式
#define NRF_REG_EN_AA                   0x01  //自动应答功能设置
#define NRF_REG_EN_RXADDR               0x02  //可用信道设置
#define NRF_REG_SETUP_AW                0x03  //收发地址宽度设置
#define NRF_REG_SETUP_RETR              0x04  //自动重发功能设置
#define NRF_REG_RF_CH                   0x05  //工作频率设置
#define NRF_REG_RF_SETUP                0x06  //发射速率、功耗功能设置0b0010 0111
#define NRF_REG_STATUS                  0x07  //状态寄存器（写1清除对应的中断）
#define NRF_REG_OBSERVE_TX              0x08  //发送监测功能
#define NRF_REG_CD                      0x09  //地址检测
#define NRF_REG_RX_ADDR_P0              0x0A  //频道0接收数据地址（通道0的地址 注： 位39到位0可以随意改 ）
#define NRF_REG_RX_ADDR_P1              0x0B  //频道1接收数据地址（通道1的地址 注： 位39到位0可以随意改 ）
#define NRF_REG_RX_ADDR_P2              0x0C  //频道2接收数据地址（通道2的地址，注：位39到位8同通道1,位7到位0可以随意改）
#define NRF_REG_RX_ADDR_P3              0x0D  //频道3接收数据地址（通道3的地址，注：位39到位8同通道1,位7到位0可以随意改）
#define NRF_REG_RX_ADDR_P4              0x0E  //频道4接收数据地址（通道4的地址，注：位39到位8同通道1,位7到位0可以随意改）
#define NRF_REG_RX_ADDR_P5              0x0F  //频道5接收数据地址（通道5的地址，注：位39到位8同通道1,位7到位0可以随意改）
#define NRF_REG_TX_ADDR                 0x10  //发送地址寄存器（发射地址，注： 位39到位0可以随意改）
#define NRF_REG_RX_PW_P0                0x11  //接收数据通道0有效数据宽度
#define NRF_REG_RX_PW_P1                0x12  //接收数据通道1有效数据宽度
#define NRF_REG_RX_PW_P2                0x13  //接收数据通道2有效数据宽度
#define NRF_REG_RX_PW_P3                0x14  //接收数据通道3有效数据宽度
#define NRF_REG_RX_PW_P4                0x15  //接收数据通道4有效数据宽度
#define NRF_REG_RX_PW_P5                0x16  //接收数据通道5有效数据宽度
#define NRF_REG_FIFO_STATUS             0x17  //FIFO栈入栈出状态寄存器设置（只读）
#define NRF_REG_DYNPD                   0x1C  //nrf24l01+ 动态数据长度寄存器
#define NRF_REG_FEATURE                 0x1D  //nrf24l01+ 功能寄存器

#define NRF_WID_RX_ADR             5        //5字节的地址宽度
#define NRF_WID_TX_ADR             5        //5字节的地址宽度
#define NRF_WID_RX_PLOAD           32       //32字节的用户数据宽度
#define NRF_WID_ACK_PAYLOAD        5        //5字节的用户数据宽度



nRF24L01_t nRF24L01;

typedef enum 
{
    _NRF_Transmiting,
    _NRF_Transmit_Success,
    _NRF_Transmit_Failed,
    _NRF_Transmit_Idle
} _NRF_Tx_Status;           //NRF发送状态

typedef enum 
{
    _Transmit,
    _Receive
} _nRF24L01_Mode;           //NRF工作模式



static void _Delay_us(volatile uint32_t us)//延时函数
{
    volatile uint8_t i = 0;
    while (us--)
    {
        i = 8;
        while (i--);
    }
}

void _CE_Low(void)//将CE置低，向nRF24L01发送控制信号
{
    HAL_GPIO_WritePin(nRF24L01.ce_port, nRF24L01.ce_pin, GPIO_PIN_RESET);
}
void _CE_High(void)//将CE置高，向nRF24L01发送控制信号
{
    HAL_GPIO_WritePin(nRF24L01.ce_port, nRF24L01.ce_pin, GPIO_PIN_SET);
}
void _CS_Low(void)//使能片选，开启SPI传输
{
    HAL_GPIO_WritePin(nRF24L01.nss_port, nRF24L01.nss_pin, GPIO_PIN_RESET);
}
void _CS_High(void)//失能片选，关闭SPI传输
{
    HAL_GPIO_WritePin(nRF24L01.nss_port, nRF24L01.nss_pin, GPIO_PIN_SET);
}

GPIO_PinState _IRQ_Read(void)//读取IRQ引脚的电平
{
    return HAL_GPIO_ReadPin(nRF24L01.irq_port, nRF24L01.irq_pin);
}

uint8_t _SPI_ByteExchange(uint8_t data)//SPI字节交换函数
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(nRF24L01.hspi, &data, &rx_data, 1, 100);
    return rx_data;
}
/*-----------------------SPI接口的命令设置-------------------------*/
/**********************************************************************	
*@function: 向指定的寄存器中写入数据
*@input:    reg_offset:寄存器地址，data:写入的数据
*@output:   status:状态寄存器值	
***********************************************************************/ 
uint8_t _NRF_Write_Register(uint8_t reg_offset, uint8_t data)//写寄存器
{
    uint8_t status;
    _CS_Low();
    status = _SPI_ByteExchange(NRF_CMD_WRITE_REG | reg_offset);
    _SPI_ByteExchange(data);
    _CS_High();
    return status;
}
/**********************************************************************	
*@function: 读出指定寄存器的值
*@input:    reg_offset:寄存器地址
*@output:   reg_val:寄存器的值
***********************************************************************/ 
uint8_t _NRF_Read_Register(uint8_t reg_offset)//读寄存器
{
    uint8_t reg_val;
    _CS_Low();
    _SPI_ByteExchange(NRF_CMD_READ_REG | reg_offset);
    reg_val = _SPI_ByteExchange(NRF_CMD_NOP);
    _CS_High();
    return reg_val;
}
/**********************************************************************	
*@function: 向NRF的TX FIFO中写入数据
*@input:    pBuf:写入数据的指针，len:写入数据的长度
*@output:   status:状态寄存器值
***********************************************************************/ 
uint8_t _NRF_Write_TX_Payload(uint8_t *pBuf, uint8_t len)//写TX Payload
{
    uint8_t status;
    _CS_Low();
    status = _SPI_ByteExchange(NRF_CMD_WRITE_TX_PAYLOAD);   //SPI发送写TX_PAYLOAD的命令
    while (len--)
    {
        _SPI_ByteExchange(*pBuf++); //SPI发送数据，依次写入到E01C的TX FIFO
    }
    _CS_High();
    return status;
}
/**********************************************************************	
*@function: 读RX有效数据：1-32字节。读操作全部从字节0开始。当读RX有效数据完成后，FFO寄存器中有效数据被清除。应用于接收模式下。
*@input:    pBuf:读出数据的指针，len:读出数据的长度
*@output:  status:状态寄存器值
***********************************************************************/ 
uint8_t _NRF_Read_RX_Payload(uint8_t *pBuf, uint8_t len)//读RX Payload
{
    uint8_t status;
    _CS_Low();
    status = _SPI_ByteExchange(NRF_CMD_READ_RX_PAYLOAD);    //SPI发送读RX_PAYLOAD的命令
    while (len--)
    {
        *pBuf++ = _SPI_ByteExchange(NRF_CMD_NOP);    //SPI读取数据，依次读出E01C的RX FIFO
    }
    _CS_High();
    return status;
}
/**********************************************************************	
*@function: 清除TX FIFO寄存器，应用于发射模式下。
*@input:    无
*@output:   status:状态寄存器值
***********************************************************************/ 
uint8_t _NRF_Flush_TX(void)//清空TX FIFO
{
    uint8_t status;
    _CS_Low();
    status = _SPI_ByteExchange(NRF_CMD_FLUSH_TX);   //SPI发送清空TX FIFO的命令
    _CS_High();
    return status;
}
/**********************************************************************	
*@function: 清除RX FIFO寄存器，应用于接收模式下。在传输应答信号过程中不应执行此指令。也就是说，若传输应答信号过程中执行此指令的话将使得应答信号不能被完整的传输。
*@input:    无
*@output:   status:状态寄存器值
***********************************************************************/ 
uint8_t _NRF_Flush_RX(void)//清空RX FIFO
{
    uint8_t status;
    _CS_Low();
    status = _SPI_ByteExchange(NRF_CMD_FLUSH_RX);   //SPI发送清空RX FIFO的命令
    _CS_High();
    return status;
}
/**********************************************************************	
*@function: 应用于发射端重新使用上一包发射的有效数据。当CE=1时，数据被不断重新发射。在发射数据包过程中必须禁止数据包重利用功能。
*@input:    无
*@output:   status:状态寄存器值
***********************************************************************/ 
uint8_t _NRF_Reuse_TX_Payload(void)//重复使用上一包数据
{
    uint8_t status;
    _CS_Low();
    status = _SPI_ByteExchange(NRF_CMD_REUSE_TX_PAYLOAD);    //SPI发送重复使用上一包数据的命令
    _CS_High();
    return status;
}
/**********************************************************************	
*@function: 空操作，可以用来读状态寄存器
*@input:    无
*@output:   status:状态寄存器值
***********************************************************************/ 
uint8_t _NRF_Nop(void)
{
    _CS_Low();
    uint8_t status = _SPI_ByteExchange(NRF_CMD_NOP);    //SPI发送重复使用上一包数据的命令
    _CS_High();
    return status;
}

uint8_t _NRF_Read_Buffer(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    _CS_Low();                               //使能SPI传输
    uint8_t status = _SPI_ByteExchange(NRF_CMD_READ_REG | reg);        //发送寄存器值(位置),并读取状态值
    while(len--)                            //读取数据
    {
        *pBuf++ = _SPI_ByteExchange(NRF_CMD_NOP);  //读出数据
    }
    _CS_High();                               //关闭SPI传输
    return status;   
}
uint8_t _NRF_Write_Buffer(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    _CS_Low();                               //使能SPI传输
    uint8_t status = _SPI_ByteExchange(NRF_CMD_WRITE_REG | reg);        //发送寄存器值(位置),并读取状态值
    while(len--)                            //读取数据
    {
        _SPI_ByteExchange(*pBuf++);         //读出数据
    }
    _CS_High();                               //关闭SPI传输
    return status;   
}

/*----------------------------------寄存器----------------------------------*/
/*高危操作，为避免内存对齐，结构体位域不能掺杂其他变量，各变量不得调换顺序，且必须为固定字节的倍数*/
//配置寄存器CONFIG
typedef struct {
    uint8_t prim_rx : 1;      //（第0位）1:接收模式, 0:发射模式
    uint8_t pwr_up : 1;       //（第1位）1:上电, 0:掉电
    uint8_t crc0 : 1;         //（第2位）CRC校验的模式：0：1字节(8位校验)，1：2字节(16位校验)
    uint8_t en_crc : 1;       //（第3位）是1否0使能CRC校验。如果EN_AA中任意一位被使能，CRC也必须被使能(1)
    uint8_t mask_max_rt : 1;  //（第4位）可屏蔽中断MAX_RT, 1:IRQ引脚不会产生MAX_RT中断 0:MAX_RT中断产生时IRQ引脚电平为低
    uint8_t mask_tx_ds : 1;   //（第5位）可屏蔽中断TX_DS, 1:IRQ引脚不会产生TX_DS中断 0:TX_DS中断产生时IRQ引脚电平为低
    uint8_t mask_rx_dr : 1;   //（第6位）可屏蔽中断RX_RD, 1:IRQ引脚不会产生RX_RD中断 0:RX_RD中断产生时IRQ引脚电平为低
    uint8_t reserved : 1;    //（第7位）保留位，不用
} _REG_CONFIG;//配置寄存器结构体
//自动应答使能寄存器EN_AA，此功能禁止后可与nRF2401通信
typedef struct {
    uint8_t enaa_p0 : 1;      //（第0位）通道0自动应答使能
    uint8_t enaa_p1 : 1;      //（第1位）通道1自动应答使能
    uint8_t enaa_p2 : 1;      //（第2位）通道2自动应答使能
    uint8_t enaa_p3 : 1;      //（第3位）通道3自动应答使能
    uint8_t enaa_p4 : 1;      //（第4位）通道4自动应答使能
    uint8_t enaa_p5 : 1;      //（第5位）通道5自动应答使能
    uint8_t reserved : 2;     //（第6、7位）保留位，不用
} _REG_EN_AA;//自动应答使能寄存器结构体
//接收地址使能寄存器EN_RXADDR
typedef struct {
    uint8_t erx_p0 : 1;       //（第0位）通道0接收使能
    uint8_t erx_p1 : 1;       //（第1位）通道1接收使能
    uint8_t erx_p2 : 1;       //（第2位）通道2接收使能
    uint8_t erx_p3 : 1;       //（第3位）通道3接收使能
    uint8_t erx_p4 : 1;       //（第4位）通道4接收使能
    uint8_t erx_p5 : 1;       //（第5位）通道5接收使能
    uint8_t reserved : 2;     //（第6、7位）保留位，不用
} _REG_EN_RXADDR;//接收地址使能寄存器结构体
//地址宽度寄存器SETUP_AW
typedef struct {
    uint8_t aw : 2;           //（第0、1位）RX/TX地址字段宽度设置：00：无效，01：3字节，10：4字节，11：5字节
    uint8_t reserved : 6;     //（第2-7位）保留位，不用
} _REG_SETUP_AW;//地址宽度寄存器结构体
//自动重发延时寄存器SETUP_RETR
typedef struct {
    uint8_t arc : 4;          //（第0-3位）自动重发计数器，最大重发次数为0-15
    uint8_t ard : 4;          //（第4-7位）自动重发延时，0：250us，1：500us，2：750us，3：1000us，4：1250us，5：1500us，6：1750us，7：2000us，8：2250us，9：2500us，10：2750us，11：3000us，12：3250us，13：3500us，14：3750us，15：4000us (+-86us)
} _REG_SETUP_RETR;//自动重发延时寄存器结构体
//射频通道寄存器RF_CH
typedef struct {
    uint8_t rf_ch : 7;        //（第0-6位）射频通道选择，2400+RF_CH MHz
    uint8_t reserved : 1;     //（第7位）保留位，不用
} _REG_RF_CH;//射频通道寄存器结构体
//射频设置寄存器RF_SETUP
typedef struct {
    uint8_t lna_hcurr : 1;    //（第0位）低噪声放大器增益，1：高增益，0：低增益
    uint8_t rf_pwr : 2;       //（第1、2位）发射功率，00：-18dBm，01：-12dBm，10：-6dBm，11：0dBm
    uint8_t rf_dr : 1;   //（第3位）空中速率，1：2Mbps，0：1Mbps
    uint8_t pll_lock : 1;     //（第4位）锁定PLL，1：锁定，0：未锁定
    uint8_t reserved : 3;     //（第5-7位）保留位，不用
} _REG_RF_SETUP;//射频设置寄存器结构体
//状态寄存器STATUS
typedef struct {
    uint8_t tx_full : 1;      //（第0位）TX FIFO满标志，1：满，0：未满
    uint8_t rx_p_no : 3;      //（第1-3位）接收数据通道号，000：通道0，001：通道1，010：通道2，011：通道3，100：通道4，101：通道5，110：未使用，111：RX FIFO为空
    uint8_t max_rt : 1;       //（第4位）达到最大重发次数中断，1：达到，0：未达到，如果MAX_RT被置位，则必须被清除才能继续发送数据包
    uint8_t tx_ds : 1;        //（第5位）数据发送完成中断，1：完成，0：未完成
    uint8_t rx_dr : 1;        //（第6位）接收到数据中断，1：接收到数据，0：未接收到数据
    uint8_t reserved : 1;     //（第7位）保留位，不用
} _REG_STATUS;//状态寄存器结构体
//发送检测寄存器OBSERVE_TX
typedef struct {
    uint8_t arc_cnt : 4;      //（第0-3位）自动重发计数器，最大重发次数为0-15
    uint8_t plos_cnt : 4;     //（第4-7位）丢包计数器。当写RF_CH寄存器时，该值被清零，当发射超过15次时，此寄存器溢出清零
} _REG_OBSERVE_TX;//发送检测寄存器结构体
//载波检测寄存器CD
typedef struct {
    uint8_t cd : 1;           //（第0位）载波检测，1：接收到的信号强度大于-64dBm，0：接收到的信号强度小于-64dBm
    uint8_t reserved : 7;     //（第1-7位）保留位，不用
} _REG_CD;//载波检测寄存器结构体
//接收数据通道0地址寄存器RX_ADDR_P0
typedef struct {
    uint8_t rx_addr_p0[5];    //（第0-39位）接收数据通道0地址，低字节在前，所写字节数由SETUP_AW寄存器决定
} _REG_RX_ADDR_P0;//接收数据通道0地址寄存器结构体
//接收数据通道1地址寄存器RX_ADDR_P1
typedef struct {
    uint8_t rx_addr_p1[5];    //（第0-39位）接收数据通道1地址，低字节在前，所写字节数由SETUP_AW寄存器决定
} _REG_RX_ADDR_P1;//接收数据通道1地址寄存器结构体
//接收数据通道2地址寄存器RX_ADDR_P2
typedef struct {
    uint8_t rx_addr_p2;       //（第0-7位）接收数据通道2地址，最低字节可设置，高字节必须与RX_ADDR_P1[39:8]相等
} _REG_RX_ADDR_P2;//接收数据通道2地址寄存器结构体
//接收数据通道3地址寄存器RX_ADDR_P3
typedef struct {
    uint8_t rx_addr_p3;       //（第0-7位）接收数据通道3地址，最低字节可设置，高字节必须与RX_ADDR_P1[39:8]相等
} _REG_RX_ADDR_P3;//接收数据通道3地址寄存器结构体
//接收数据通道4地址寄存器RX_ADDR_P4
typedef struct {
    uint8_t rx_addr_p4;       //（第0-7位）接收数据通道4地址，最低字节可设置，高字节必须与RX_ADDR_P1[39:8]相等
} _REG_RX_ADDR_P4;//接收数据通道4地址寄存器结构体
//接收数据通道5地址寄存器RX_ADDR_P5
typedef struct {
    uint8_t rx_addr_p5;       //（第0-7位）接收数据通道5地址，最低字节可设置，高字节必须与RX_ADDR_P1[39:8]相等
} _REG_RX_ADDR_P5;//接收数据通道5地址寄存器结构体
//发送地址寄存器TX_ADDR
typedef struct {
    uint8_t tx_addr[5];       //（第0-39位）发送地址，低字节在前
} _REG_TX_ADDR;//发送地址寄存器结构体
//接收数据通道有效数据宽度寄存器RX_PW_P0
typedef struct {
    uint8_t rx_pw_p0 : 6;     //（第0-5位）接收数据通道0有效数据宽度，0-32字节
    uint8_t reserved : 2;     //（第6、7位）保留位，不用
} _REG_RX_PW_P0;//接收数据通道有效数据宽度寄存器结构体
//接收数据通道有效数据宽度寄存器RX_PW_P1
typedef struct {
    uint8_t rx_pw_p1 : 6;     //（第0-5位）接收数据通道1有效数据宽度，0-32字节
    uint8_t reserved : 2;     //（第6、7位）保留位，不用
} _REG_RX_PW_P1;//接收数据通道有效数据宽度寄存器结构体
//接收数据通道有效数据宽度寄存器RX_PW_P2
typedef struct {
    uint8_t rx_pw_p2 : 6;     //（第0-5位）接收数据通道2有效数据宽度，0-32字节
    uint8_t reserved : 2;     //（第6、7位）保留位，不用
} _REG_RX_PW_P2;//接收数据通道有效数据宽度寄存器结构体
//接收数据通道有效数据宽度寄存器RX_PW_P3
typedef struct {
    uint8_t rx_pw_p3 : 6;     //（第0-5位）接收数据通道3有效数据宽度，0-32字节
    uint8_t reserved : 2;     //（第6、7位）保留位，不用
} _REG_RX_PW_P3;//接收数据通道有效数据宽度寄存器结构体
//接收数据通道有效数据宽度寄存器RX_PW_P4
typedef struct {
    uint8_t rx_pw_p4 : 6;     //（第0-5位）接收数据通道4有效数据宽度，0-32字节
    uint8_t reserved : 2;     //（第6、7位）保留位，不用
} _REG_RX_PW_P4;//接收数据通道有效数据宽度寄存器结构体
//接收数据通道有效数据宽度寄存器RX_PW_P5
typedef struct {
    uint8_t rx_pw_p5 : 6;     //（第0-5位）接收数据通道5有效数据宽度，0-32字节
    uint8_t reserved : 2;     //（第6、7位）保留位，不用
} _REG_RX_PW_P5;//接收数据通道有效数据宽度寄存器结构体
//FIFO状态寄存器FIFO_STATUS
typedef struct {
    uint8_t rx_empty : 1;     //（第0位）RX FIFO为空标志，1：空，0：非空
    uint8_t rx_full : 1;      //（第1位）RX FIFO满标志，1：满，0：未满
    uint8_t reserved2_3 : 2;     //（第2、3位）保留位，不用
    uint8_t tx_empty : 1;     //（第4位）TX FIFO为空标志，1：空，0：非空
    uint8_t tx_full : 1;      //（第5位）TX FIFO满标志，1：满，0：未满
    uint8_t tx_reuse : 1;     //（第6位）若TX_REUSE = 1则当CE为高电平时不断发送上一包数据。TX_REUSE通过SPI命令REUSE_TX_PL设置，通过W_TX_PAYLOAD或W_TX_PAYLOAD_NOACK清除
    uint8_t reserved7 : 1;     //（第7位）保留位，不用
} _REG_FIFO_STATUS;//FIFO状态寄存器结构体
typedef struct {
    uint8_t dpl_p0 : 1;//（第0位）通道0动态载荷长度，1：使能，0：禁止
    uint8_t dpl_p1 : 1;//（第1位）通道1动态载荷长度，1：使能，0：禁止
    uint8_t dpl_p2 : 1;//（第2位）通道2动态载荷长度，1：使能，0：禁止
    uint8_t dpl_p3 : 1;//（第3位）通道3动态载荷长度，1：使能，0：禁止
    uint8_t dpl_p4 : 1;//（第4位）通道4动态载荷长度，1：使能，0：禁止
    uint8_t dpl_p5 : 1;//（第5位）通道5动态载荷长度，1：使能，0：禁止
    uint8_t reserved : 2;//（第6、7位）保留位，不用
} _REG_DYNPD;//动态载荷长度寄存器//DPL_P<N>：启用数据通道<N>的动态载荷长度，需要EN_DPL和ENAA_P<N>
typedef struct 
{
    uint8_t en_dyn_ack : 1; //（第0位）允许W_TX_PAYLOAD_NOACK命令
    uint8_t en_ack_pay : 1; //（第1位）允许载荷带ACK
    uint8_t en_dpl : 1;     //（第2位）启用动态载荷长度
    uint8_t reversed : 5;   //（第3-7位）保留位，不用
}_REG_FEATURE;

//

uint8_t _NRF_Read_RegStruct_8bits(void* reg_struct, uint8_t reg_offset)//读寄存器结构体
{
    *((uint8_t*)reg_struct) = _NRF_Read_Register(reg_offset);
    return *((uint8_t*)reg_struct);
}

uint8_t _NRF_Write_RegStruct_8bits(void* reg_struct, uint8_t reg_offset)//写寄存器结构体
{
    return _NRF_Write_Register(reg_offset, *((uint8_t*)reg_struct));
}
uint8_t _NRF_Read_RegStruct_40bits(void* reg_struct, uint8_t reg_offset)//读寄存器结构体
{
    return _NRF_Read_Buffer(reg_offset, (uint8_t*)reg_struct, 5);
}
uint8_t _NRF_Write_RegStruct_40bits(void* reg_struct, uint8_t reg_offset)//写寄存器结构体
{
    return _NRF_Write_Buffer(reg_offset, (uint8_t*)reg_struct, 5);
}
_NRF_Tx_Status NRF_Transmit_Status;
void _nRF24L01_ModeSwitch(_nRF24L01_Mode mode)
{
	_CE_Low();
    _REG_CONFIG config;
    _NRF_Read_RegStruct_8bits(&config, NRF_REG_CONFIG);//读取配置寄存器
    if(mode == _Transmit)
        config.prim_rx = 0;//切换到发射模式
    else if(mode == _Receive)
        config.prim_rx = 1;//切换到接收模式
    _NRF_Write_RegStruct_8bits(&config, NRF_REG_CONFIG);//写入配置寄存器
	_CE_High();
}
void _nRF24L01_Set_Config(void)//配置寄存器
{
	_CE_Low();
    _NRF_Write_RegStruct_8bits(&(_REG_SETUP_AW){.aw = 3}, NRF_REG_SETUP_AW);

    _REG_RX_ADDR_P1 p1_struct;
    for(uint32_t i = 0; i < 4; i++)//填充P1-P5的高位相同地址
        p1_struct.rx_addr_p1[i + 1] = nRF24L01.receive_address.high[i];
    p1_struct.rx_addr_p1[0] = nRF24L01.receive_address.p1;  //填充P1的低位地址
    _NRF_Write_RegStruct_40bits(&p1_struct, NRF_REG_RX_ADDR_P1);//写入P1的地址
    _NRF_Write_RegStruct_8bits(&(_REG_RX_ADDR_P2){.rx_addr_p2 = nRF24L01.receive_address.p2}, NRF_REG_RX_ADDR_P2);
    _NRF_Write_RegStruct_8bits(&(_REG_RX_ADDR_P3){.rx_addr_p3 = nRF24L01.receive_address.p3}, NRF_REG_RX_ADDR_P3);
    _NRF_Write_RegStruct_8bits(&(_REG_RX_ADDR_P4){.rx_addr_p4 = nRF24L01.receive_address.p4}, NRF_REG_RX_ADDR_P4);
    _NRF_Write_RegStruct_8bits(&(_REG_RX_ADDR_P5){.rx_addr_p5 = nRF24L01.receive_address.p5}, NRF_REG_RX_ADDR_P5);

    _NRF_Write_RegStruct_8bits(&(_REG_EN_AA){.enaa_p0 = 1,.enaa_p1 = 1,.enaa_p2 = 1,.enaa_p3 = 1,.enaa_p4 = 1,.enaa_p5 = 1}, NRF_REG_EN_AA);
    _NRF_Write_RegStruct_8bits(&(_REG_EN_RXADDR){.erx_p0 = 1,.erx_p1 = 1,.erx_p2 = 1,.erx_p3 = 1,.erx_p4 = 1,.erx_p5 = 1}, NRF_REG_EN_RXADDR);
    _NRF_Write_RegStruct_8bits(&(_REG_SETUP_RETR){.arc = 15, .ard = 4}, NRF_REG_SETUP_RETR);
    _NRF_Write_RegStruct_8bits(&(_REG_RF_CH){.rf_ch = nRF24L01.rf_channel}, NRF_REG_RF_CH);
    _NRF_Write_RegStruct_8bits(&(_REG_RF_SETUP){.lna_hcurr = 1, .rf_pwr = 3, .rf_dr = 1}, NRF_REG_RF_SETUP);

    _NRF_Write_RegStruct_8bits(&(_REG_CONFIG){.prim_rx = 1, .pwr_up = 1, .crc0 = 1, .en_crc = 1}, NRF_REG_CONFIG);
    _NRF_Flush_TX();
    _NRF_Flush_RX();

    _NRF_Write_RegStruct_8bits(&(_REG_DYNPD){.dpl_p0 = 1,.dpl_p1 = 1,.dpl_p2 = 1,.dpl_p3 = 1,.dpl_p4 = 1,.dpl_p5 = 1}, NRF_REG_DYNPD);
    _NRF_Write_RegStruct_8bits(&(_REG_FEATURE){.en_dyn_ack = 0, .en_ack_pay = 1, .en_dpl = 1}, NRF_REG_FEATURE);

    _CE_High();
    NRF_Transmit_Status = _NRF_Transmit_Idle;
    _Delay_us(130);
    _nRF24L01_ModeSwitch(_Receive);
}
void nRF24L01_Init()
{
	nRF24L01 = NRF24L01_INIT;
    while(nRF24L01_CheckConnectivity() == _DisConnected);
    _nRF24L01_Set_Config();
	_REG_STATUS status;
    _NRF_Read_RegStruct_8bits(&status, NRF_REG_STATUS);//读取配置寄存器
    _NRF_Write_RegStruct_8bits(&status, NRF_REG_STATUS);//中断触发时status对应位置1，向对应的标志位写1清除中断标志
    _nRF24L01_ModeSwitch(_Receive);
}
void _nRF24L01_TransmitSuccess(void)
{//如果成功，那么将check_buf的index位写1
    uint64_t op = 1;
	
    if(nRF24L01.check.check_index > 63)
    {
        op <<= (nRF24L01.check.check_index - 64);
        nRF24L01.check.check_buf[1] |= op;
    }
    else
    {
        op <<= nRF24L01.check.check_index;
        nRF24L01.check.check_buf[0] |= op;
    }
    nRF24L01.check.check_index = (nRF24L01.check.check_index + 1) % 128;
}
void _nRF24L01_TransmitFailed(void)
{//如果失败，那么将check_buf的index位写0
    uint64_t op = 1;
    if(nRF24L01.check.check_index > 63)
    {
        op <<= (nRF24L01.check.check_index - 64);
        nRF24L01.check.check_buf[1] &= ~op;
    }
    else
    {
        op <<= nRF24L01.check.check_index;
        nRF24L01.check.check_buf[0] &= ~op;
    }
    nRF24L01.check.check_index = (nRF24L01.check.check_index + 1) % 128;
}
float _nRF24L01_CalculateConnectRate(void)
{//计算连接率
    uint32_t count = 0;
    uint64_t temp = nRF24L01.check.check_buf[0];
    while(temp)
    {
        count++;
        temp = temp & (temp - 1);
    }
    temp = nRF24L01.check.check_buf[1];
    while (temp)
    {
        count++;
        temp = temp & (temp - 1);
    }
    float persent = (float)count / 128.0f;

    if(persent > 0.95f)
        nRF24L01.signal = Nrfsignal_Strong;
    else if (persent > 0.70f)
        nRF24L01.signal = Nrfsignal_Weak;
    else
        nRF24L01.signal = Nrfsignal_Disconnected;
    return persent;
}

//尝试读写一个寄存器的值，如果读写成功，则认为nRF24L01连接正常，这里选择更改自动重发延时寄存器的值，验证后还原原始值
nRF24L01_Connectivity nRF24L01_CheckConnectivity(void)
{
    _REG_SETUP_RETR check_reg;
    _NRF_Read_RegStruct_8bits(&check_reg, NRF_REG_SETUP_RETR);//读取
    check_reg.ard = ~check_reg.ard;
    uint8_t temp = check_reg.ard;

    _NRF_Write_RegStruct_8bits(&check_reg, NRF_REG_SETUP_RETR);
    _NRF_Read_RegStruct_8bits(&check_reg, NRF_REG_SETUP_RETR);

    if(check_reg.ard != temp)
        return _DisConnected;
    else
    {
        check_reg.ard = ~check_reg.ard;
        _NRF_Write_RegStruct_8bits(&check_reg, NRF_REG_SETUP_RETR);
        return _Connected;
    }
}
//需要将此函数放入EXTI中断回调函数中
void nRF24L01_IRQ_EXTI_Callback(uint16_t GPIO_Pin)
{
	
    if(GPIO_Pin != nRF24L01.irq_pin)
        return;
    _REG_STATUS status;
    _NRF_Read_RegStruct_8bits(&status, NRF_REG_STATUS);//读取配置寄存器
    _NRF_Write_RegStruct_8bits(&status, NRF_REG_STATUS);//中断触发时status对应位置1，向对应的标志位写1清除中断标志
    _nRF24L01_ModeSwitch(_Receive);//切换到接收模式
    if(status.rx_dr && status.rx_p_no != 7)//如果成功接收到了数据
    {
        nRF24L01.receive_data[status.rx_p_no].length = _NRF_Read_Register(NRF_CMD_READ_RX_PAYLOAD_WID);//读取接收到的数据包长度
        if(nRF24L01.receive_data[status.rx_p_no].length)
        {
            _NRF_Read_Buffer(NRF_CMD_READ_RX_PAYLOAD, nRF24L01.receive_data[status.rx_p_no].buffer, nRF24L01.receive_data[status.rx_p_no].length);//读取接收到的数据包
            Gamepad_nRF24L01_ReceiveCallback((void*)nRF24L01.receive_data[status.rx_p_no].buffer);
            _NRF_Flush_RX();
        }
        else//如果数据包长度大于32，则说明数据包有误，应该丢弃
        {
            nRF24L01.receive_data[status.rx_p_no].length = 0;
            _NRF_Flush_RX();//清空接收缓冲区
        }
    }
    else if(status.tx_ds)//发送成功
    {
        _nRF24L01_TransmitSuccess();
        NRF_Transmit_Status = _NRF_Transmit_Success;
        _NRF_Flush_TX();//清空发送缓冲区
    }
    else if(status.max_rt)//达到最大重发次数
    {
        _nRF24L01_TransmitFailed();
        NRF_Transmit_Status = _NRF_Transmit_Failed;
        _NRF_Flush_TX();//清空发送缓冲区
    }
}
void nRF24L01_Tansmit(uint8_t* tx_buf, uint8_t length)
{
    NRF_Transmit_Status = _NRF_Transmiting;
    _nRF24L01_ModeSwitch(_Transmit);//切换到发射模式
    _CE_Low();
    _NRF_Write_Buffer(NRF_REG_TX_ADDR, nRF24L01.transmit_address, NRF_WID_TX_ADR);//写入发送地址
    _NRF_Write_Buffer(NRF_REG_RX_ADDR_P0, nRF24L01.transmit_address, NRF_WID_RX_ADR);//P0通道接收地址与发送地址相同，用于接收接收机回传的ACK
    _NRF_Write_Buffer(NRF_CMD_WRITE_TX_PAYLOAD, tx_buf, length);//写入发送数据包
    _CE_High();
	HAL_Delay(5);
}
void nRF24L01_SetTransmitAddress(uint8_t* transmit_addr)
{
    for(uint32_t i = 0; i < 5; i++)
        nRF24L01.transmit_address[i] = transmit_addr[i];
}
