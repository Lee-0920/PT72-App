/*
 * PumpTimer.c
 *
 *  Created on: 2016年5月31日
 *      Author: Administrator
 */

#include <LiquidDriver/PumpTimer.h>
#include "string.h"
#include "tracer/trace.h"
#include "SystemConfig.h"

#define PUMP_TIMER_PERIOD             1000-1  // 重载计数
#define PUMP_TIMER_PRESCALER          (90-1) // 预分频

#define PUMP_TIMERA_RCC                RCC_APB1Periph_TIM2
#define PUMP_TIMERA_RCC_CONFIG         RCC_APB1PeriphClockCmd(PUMP_TIMERA_RCC,ENABLE)
#define PUMP_TIMERA_IRQn               TIM2_IRQn
#define PUMP_TIMERA_IRQHANDLER         TIM2_IRQHandler
#define PUMP_TIMERA                    TIM2

#define PUMP_TIMERB_RCC                RCC_APB1Periph_TIM4
#define PUMP_TIMERB_RCC_CONFIG         RCC_APB1PeriphClockCmd(PUMP_TIMERB_RCC,ENABLE)
#define PUMP_TIMERB_IRQn               TIM4_IRQn
#define PUMP_TIMERB_IRQHANDLER         TIM4_IRQHandler
#define PUMP_TIMERB                    TIM4

#define PUMP_TIMERC_RCC                RCC_APB1Periph_TIM12
#define PUMP_TIMERC_RCC_CONFIG         RCC_APB1PeriphClockCmd(PUMP_TIMERC_RCC,ENABLE)
#define PUMP_TIMERC_IRQn               TIM8_BRK_TIM12_IRQn
#define PUMP_TIMERC_IRQHANDLER         TIM8_BRK_TIM12_IRQHandler
#define PUMP_TIMERC                    TIM12

#define PUMP_TIMERD_RCC                RCC_APB1Periph_TIM13
#define PUMP_TIMERD_RCC_CONFIG         RCC_APB1PeriphClockCmd(PUMP_TIMERD_RCC,ENABLE)
#define PUMP_TIMERD_IRQn               TIM8_UP_TIM13_IRQn
#define PUMP_TIMERD_IRQHANDLER         TIM8_UP_TIM13_IRQHandler
#define PUMP_TIMERD                     TIM13

//各个定时器中断服务程序回调函数指针
static void (*PumpTimerA_Handle)(void *obj) = NULL;
static void (*PumpTimerB_Handle)(void *obj) = NULL;
static void (*PumpTimerC_Handle)(void *obj) = NULL;
static void (*PumpTimerD_Handle)(void *obj) = NULL;

//传入中断服务程序回调函数参数
static void* s_timerAObj = NULL;
static void* s_timerBObj = NULL;
static void* s_timerCObj = NULL;
static void* s_timerDObj = NULL;

/**
 * @brief 配置4个定时器的中断向量管理器
 */
static void PumpTimer_NVICConfig(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_InitStructure.NVIC_IRQChannel = PUMP_TIMERA_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PUMP_TIMERA_PRIORITY;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = PUMP_TIMERB_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PUMP_TIMERB_PRIORITY;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = PUMP_TIMERC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PUMP_TIMERC_PRIORITY;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = PUMP_TIMERD_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PUMP_TIMERD_PRIORITY;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief 配置定时器的时钟
 */
static void PumpTimer_RCCConfig(void)
{
    PUMP_TIMERA_RCC_CONFIG;
    PUMP_TIMERB_RCC_CONFIG;
    PUMP_TIMERC_RCC_CONFIG;
    PUMP_TIMERD_RCC_CONFIG;
}

/**
 * @brief 配置定时器的工作参数
 */
static void PumpTimer_Configuration(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    memset(&TIM_TimeBaseStructure, 0, sizeof(TIM_TimeBaseStructure));

    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseStructure.TIM_Period = PUMP_TIMER_PERIOD;
    TIM_TimeBaseStructure.TIM_Prescaler = PUMP_TIMER_PRESCALER;
    TIM_TimeBaseInit(PUMP_TIMERA, &TIM_TimeBaseStructure);
    TIM_ClearFlag(PUMP_TIMERA, TIM_IT_Update); //避免定时器启动时进入中断服务程序
    TIM_ITConfig(PUMP_TIMERA, TIM_IT_Update, ENABLE);
    TIM_Cmd(PUMP_TIMERA, DISABLE);

    TIM_TimeBaseStructure.TIM_Period = PUMP_TIMER_PERIOD;
    TIM_TimeBaseStructure.TIM_Prescaler = PUMP_TIMER_PRESCALER;
    TIM_TimeBaseInit(PUMP_TIMERB, &TIM_TimeBaseStructure);
    TIM_ClearFlag(PUMP_TIMERB, TIM_IT_Update);
    TIM_ITConfig(PUMP_TIMERB, TIM_IT_Update, ENABLE);
    TIM_Cmd(PUMP_TIMERB, DISABLE);

    TIM_TimeBaseStructure.TIM_Period = PUMP_TIMER_PERIOD;
    TIM_TimeBaseStructure.TIM_Prescaler = PUMP_TIMER_PRESCALER;
    TIM_TimeBaseInit(PUMP_TIMERC, &TIM_TimeBaseStructure);
    TIM_ClearFlag(PUMP_TIMERC, TIM_IT_Update);
    TIM_ITConfig(PUMP_TIMERC, TIM_IT_Update, ENABLE);
    TIM_Cmd(PUMP_TIMERC, DISABLE);

    TIM_TimeBaseStructure.TIM_Period = PUMP_TIMER_PERIOD;
    TIM_TimeBaseStructure.TIM_Prescaler = PUMP_TIMER_PRESCALER;
    TIM_TimeBaseInit(PUMP_TIMERD, &TIM_TimeBaseStructure);
    TIM_ClearFlag(PUMP_TIMERD, TIM_IT_Update);
    TIM_ITConfig(PUMP_TIMERD, TIM_IT_Update, ENABLE);
    TIM_Cmd(PUMP_TIMERD, DISABLE);
}

/**
 * @brief 初始化定时器
 */
void PumpTimer_Init(void)
{
    PumpTimer_NVICConfig();
    PumpTimer_RCCConfig();
    PumpTimer_Configuration();
}

/**
 * @brief 定时器中断回调函数注册，使中断回调函数指向需要使用定时器的泵的运动算法处理
 * @param Handle 定时器中断回调函数指向的函数
 * @param obj 定时器中断回调函数传入的参数
 * @return TRUE 操作成功 FALSE 操作失败(所有定时器已用掉)
 */
Bool PumpTimer_RegisterHandle(TimerHandle Handle, void* obj)
{
    if (PumpTimerA_Handle == NULL)
    {
        PumpTimerA_Handle = Handle;
        s_timerAObj = obj;
        TRACE_CODE("\n###########Start TIMERA: ");
    }
    else if (PumpTimerB_Handle == NULL)
    {
        PumpTimerB_Handle = Handle;
        s_timerBObj = obj;
        TRACE_CODE("\n###########Start TIMERB: ");
    }
    else if (PumpTimerC_Handle == NULL)
    {
        PumpTimerC_Handle = Handle;
        s_timerCObj = obj;
        TRACE_CODE("\n###########Start TIMERC: ");
    }
    else if (PumpTimerD_Handle == NULL)
    {
        PumpTimerD_Handle = Handle;
        s_timerDObj = obj;
        TRACE_CODE("\n###########Start TIMERD: ");
    }
    else
    {
        return FALSE;
    }
    return TRUE;
}

/**
 * @brief 取消中断回调函数指向的泵的运动算法处理，释放中断回调函数和传入参数
 * @param Handle 定时器中断回调函数指向的函数
 * @return TRUE 操作成功 FALSE 操作失败(找不到使用的定时器)
 */
Bool PumpTimer_CancelRegisterHandle(void* obj)
{
    if (obj == s_timerAObj)
    {
        PumpTimerA_Handle = NULL;
        s_timerAObj = NULL;
        TRACE_CODE("\n###########Discon TIMERA: ");
    }
    else if (obj == s_timerBObj)
    {
        PumpTimerB_Handle = NULL;
        s_timerBObj = NULL;
        TRACE_CODE("\n###########Discon TIMERB: ");
    }
    else if (obj == s_timerCObj)
    {
        PumpTimerC_Handle = NULL;
        s_timerCObj = NULL;
        TRACE_CODE("\n###########Discon TIMERC: ");
    }
    else if (obj == s_timerDObj)
    {
        PumpTimerD_Handle = NULL;
        s_timerDObj = NULL;
        TRACE_CODE("\n###########Discon TIMERD: ");
    }
    else
    {
        return FALSE;
    }
    return TRUE;
}

/**
 * @brief 启动对应得定时器
 * @param obj 用于查找对应的定时器
 */
void PumpTimer_Start(void* obj)
{
    TIM_TypeDef* TIMx;

    if (obj == s_timerAObj)
    {
        TIMx = PUMP_TIMERA;
    }
    else if (obj == s_timerBObj)
    {
        TIMx = PUMP_TIMERB;
    }
    else if (obj == s_timerCObj)
    {
        TIMx = PUMP_TIMERC;
    }
    else if (obj == s_timerDObj)
    {
        TIMx = PUMP_TIMERD;
    }
    // 清计数器
    TIMx->CNT = 0;
    // 清除更新中断标志
    TIMx->SR &= (uint16_t) (~((uint16_t) TIM_IT_Update));
    TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);
    // 使能定时器
    TIMx->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief 停止对应得定时器
 * @param obj 用于查找对应的定时器
 */
void PumpTimer_Stop(void* obj)
{
    TIM_TypeDef* TIMx;
    if (obj == s_timerAObj)
    {
        TIMx = PUMP_TIMERA;
    }
    else if (obj == s_timerBObj)
    {
        TIMx = PUMP_TIMERB;
    }
    else if (obj == s_timerCObj)
    {
        TIMx = PUMP_TIMERC;
    }
    else if (obj == s_timerDObj)
    {
        TIMx = PUMP_TIMERD;
    }
    TIM_ITConfig(TIMx, TIM_IT_Update, DISABLE);
    TIMx->CR1 &= (uint16_t) (~((uint16_t) TIM_CR1_CEN));
}

/**
 * @brief 设置对应定时器的中断频率
 * @param obj 用于查找对应的定时器
 * @param speed 中断频率 单位Hz(注意频率不能小于15.2Hz)
 */
void PumpTimer_SpeedSetting(void* obj, float speed)
{
    TIM_TypeDef* TIMx;
    if (obj == s_timerAObj)
    {
        TIMx = PUMP_TIMERA;
    }
    else if (obj == s_timerBObj)
    {
        TIMx = PUMP_TIMERB;
    }
    else if (obj == s_timerCObj)
    {
        TIMx = PUMP_TIMERC;
    }
    else if (obj == s_timerDObj)
    {
        TIMx = PUMP_TIMERD;
    }
//     设置定时器重载计数
    TIM_SetAutoreload(TIMx, (uint16_t) (1000000 / speed) - 1);
}

void PUMP_TIMERA_IRQHANDLER(void)
{
    if ((PUMP_TIMERA->DIER & TIM_IT_Update)
            && (PUMP_TIMERA->SR & TIM_IT_Update))  //检查更新中断发生与否
    {
        // 清更新中断标志
        PUMP_TIMERA->SR &= (uint16_t) (~((uint16_t) TIM_IT_Update));
        PumpTimerA_Handle(s_timerAObj);
    }
}

void PUMP_TIMERB_IRQHANDLER(void)
{
    if ((PUMP_TIMERB->DIER & TIM_IT_Update)
            && (PUMP_TIMERB->SR & TIM_IT_Update))  //检查更新中断发生与否
    {
        // 清更新中断标志
        PUMP_TIMERB->SR &= (uint16_t) (~((uint16_t) TIM_IT_Update));
        PumpTimerB_Handle(s_timerBObj);
    }
}

void PUMP_TIMERC_IRQHANDLER(void)
{
    if ((PUMP_TIMERC->DIER & TIM_IT_Update)
            && (PUMP_TIMERC->SR & TIM_IT_Update))  //检查更新中断发生与否
    {
        // 清更新中断标志
        PUMP_TIMERC->SR &= (uint16_t) (~((uint16_t) TIM_IT_Update));
        PumpTimerC_Handle(s_timerCObj);
    }
}

void PUMP_TIMERD_IRQHANDLER(void)
{
    if ((PUMP_TIMERD->DIER & TIM_IT_Update)
            && (PUMP_TIMERD->SR & TIM_IT_Update))  //检查更新中断发生与否
    {
        // 清更新中断标志
        PUMP_TIMERD->SR &= (uint16_t) (~((uint16_t) TIM_IT_Update));
        PumpTimerD_Handle(s_timerDObj);
    }
}
