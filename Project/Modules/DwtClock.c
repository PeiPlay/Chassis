#include "DwtClock.h"

volatile static uint64_t DwtClockFreq;//Dwt寄存器自增频率
volatile static uint64_t DwtRoundCnt;
volatile static uint8_t IsDwtInited = 0;
void DwtClock_Init(void)
{
    IsDwtInited = 1;
    DwtClockFreq = SystemCoreClock;
    DwtRoundCnt = 0u;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = (uint32_t)0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
inline uint64_t DwtClock_GetDwtCnt(void)
{
    volatile uint32_t currentDwtCnt = DWT->CYCCNT;
    volatile static uint32_t lastDwtCnt = 0u;
    if (currentDwtCnt < lastDwtCnt) {
        // 如果当前计数值小于上一次的计数值，说明发生了溢出
        DwtRoundCnt++;
    }
    lastDwtCnt = currentDwtCnt;
    return ((uint64_t)DwtRoundCnt << 32) | currentDwtCnt;
}
//得到Dwt微秒数
inline uint64_t DwtClock_GetDwtUs(void)
{
    return DwtClock_GetDwtCnt() / (DwtClockFreq / 1000000u);
}
//得到Dwt毫秒数
inline uint64_t DwtClock_GetDwtMs(void)
{
    return DwtClock_GetDwtCnt() / (DwtClockFreq / 1000u);
}
//得到Dwt秒数
double DwtClock_GetDwtTime(void)
{
    return (double)DwtClock_GetDwtCnt() / (double)DwtClockFreq;
}
//Dwt延时微秒
inline void DwtClock_DelayUs(uint64_t us)
{
    uint64_t start = DwtClock_GetDwtUs();
    while (DwtClock_GetDwtUs() - start < us);
}
//Dwt延时毫秒
inline void DwtClock_DelayMs(uint64_t ms)
{
    uint64_t start = DwtClock_GetDwtMs();
    while (DwtClock_GetDwtMs() - start < ms);
}
//Dwt溢出保护(为防止两次调用DwtClock_GetDwtCnt的时间差大于Dwt溢出周期，需要进行溢出保护)
inline void DwtClock_OverflowProtect(void)
{
    (void)DwtClock_GetDwtCnt();
}
//Dwt数据转换
inline double DwtClock_ConvertCnt2Time(uint64_t cnt)
{
    return cnt / (double)DwtClockFreq;
}
inline uint8_t DwtClock_IsInited(void)
{
    return IsDwtInited;
}
