#include "tim_bsp.h"


//开启定时器
void TimBsp_Start(TIM_HandleTypeDef *htim)
{
    HAL_TIM_Base_Start_IT(htim);
    HAL_TIM_Base_Start(htim);
}
//关闭定时器
void TimBsp_Shutdown(TIM_HandleTypeDef *htim)
{
    HAL_TIM_Base_Stop_IT(htim);
    HAL_TIM_Base_Stop(htim);
    __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
    __HAL_TIM_SET_COUNTER(htim, 0);
}
//得到计数器自增频率
uint32_t TimBsp_GetCounterFreq(TIM_HandleTypeDef *htim)
{    
    TIM_TypeDef* tim = htim->Instance;
#if defined(TIM1)
    if(tim == TIM1)
        goto _APB2_TIM_FREQ;
#endif
#if defined(TIM2)
    if(tim == TIM2)
        goto _APB1_TIM_FREQ;
#endif
#if defined(TIM3)
    if(tim == TIM3)
        goto _APB1_TIM_FREQ;
#endif
#if defined(TIM4)
    if(tim == TIM4)
        goto _APB1_TIM_FREQ;
#endif
#if defined(TIM5)
    if(tim == TIM5)
        goto _APB1_TIM_FREQ;
#endif
#if defined(TIM6)
    if(tim == TIM6)
        goto _APB1_TIM_FREQ;
#endif
#if defined(TIM7)
    if(tim == TIM7)
        goto _APB1_TIM_FREQ;
#endif
#if defined(TIM8)
    if(tim == TIM8)
        goto _APB2_TIM_FREQ;
#endif
#if defined(TIM9)
    if(tim == TIM9)
        goto _APB2_TIM_FREQ;
#endif
#if defined(TIM10)
    if(tim == TIM10)
        goto _APB2_TIM_FREQ;
#endif
#if defined(TIM11)
    if(tim == TIM11)
        goto _APB2_TIM_FREQ;
#endif
#if defined(TIM12)
    if(tim == TIM12)
        goto _APB1_TIM_FREQ;
#endif
#if defined(TIM13)
    if(tim == TIM13)
        goto _APB1_TIM_FREQ;
#endif
#if defined(TIM14)
    if(tim == TIM14)
        goto _APB1_TIM_FREQ;
#endif
#if defined(TIM15)
    if(tim == TIM15)
        goto _APB2_TIM_FREQ;
#endif
#if defined(TIM16)
    if(tim == TIM16)
        goto _APB2_TIM_FREQ;
#endif
#if defined(TIM17)
    if(tim == TIM17)
        goto _APB2_TIM_FREQ;
#endif
#if defined(TIM18)
    if(tim == TIM18)
        goto _APB1_TIM_FREQ;
#endif
#if defined(TIM19)
    if(tim == TIM19)
        goto _APB1_TIM_FREQ;
#endif
#if defined(TIM20)
    if(tim == TIM20)
        goto _APB1_TIM_FREQ;
#endif
    Error_Handler();

	uint32_t pclk;
//计算对应定时器的APB? timer clock的频率
_APB1_TIM_FREQ://APB1 timer clock的频率
    pclk = HAL_RCC_GetPCLK1Freq();
#if defined(RCC_D2CFGR_D2PPRE1)
    if((RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) == 0)
#else 
    if((RCC->CFGR & RCC_CFGR_PPRE1) == 0)
#endif
        return (pclk);
    else
        return(2 * pclk);
_APB2_TIM_FREQ://APB2 timer clock的频率
    pclk = HAL_RCC_GetPCLK2Freq();
#if defined(RCC_D2CFGR_D2PPRE2)
    if((RCC->D2CFGR & RCC_D2CFGR_D2PPRE2) == 0)
#else
    if((RCC->CFGR & RCC_CFGR_PPRE2) == 0)
#endif
        return (pclk);
    else
        return(2 * pclk);
}
//计算合适的分频系数和重装载值
void TimBsp_CalcPscAndAtr(TIM_HandleTypeDef *htim, double freq, uint32_t *psc, uint32_t *atr)
{
    uint64_t intPeriodCnt = (double)TimBsp_GetCounterFreq(htim) / freq;
    uint32_t prescaler, period;
    uint64_t best_product = 0;
    for (uint32_t i = 1; i <= 65536; ++i) {
        uint32_t j = intPeriodCnt / i;
        if (j > 65536) continue;
        uint64_t product = i * j;
        if (product > best_product) {
            best_product = product;
            prescaler = i;
            period = j;
        }
		else if(product == intPeriodCnt)
			break;
    }
    *psc = prescaler - 1;
    *atr = period - 1;
}

