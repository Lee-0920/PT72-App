/*
 * PwmDriver.c
 *
 *  Created on: 2020年6月23日
 *      Author: Administrator
 */

#include "PwmDriver.h"
#include "Tracer/Trace.h"
#include "SystemConfig.h"
#include <math.h>

#define PWMDRIVER_TIMER_PERIOD         1000-1  // 重载计数
#define PWMDRIVER_TIMER_PRESCALER      (45-1) // 预分频

#define PWMDRIVER_TIMER_RCC                RCC_APB1Periph_TIM3
#define PWMDRIVER_TIMER_RCC_CONFIG         RCC_APB1PeriphClockCmd(PWMDRIVER_TIMER_RCC, ENABLE)
#define PWMDRIVER_TIMER_IRQn               TIM3_IRQn
#define PWMDRIVER_TIMER_IRQHANDLER         TIM3_IRQHandler
#define PWMDRIVER_TIMER                    TIM3

static void (*PwmDriver_Handle)(void*, Bool) = NULL;
volatile static Uint16 s_onTime = 0;
volatile static Uint16 s_offTime = 100;
volatile static Uint16 s_curTime = 0;
volatile static Bool s_curState = FALSE;
static void* s_device = NULL;

void PwmDriver_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOC,&GPIO_InitStructure);
    GPIO_SetBits(GPIOC, GPIO_Pin_9);
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    PWMDRIVER_TIMER_RCC_CONFIG;

    NVIC_InitStructure.NVIC_IRQChannel=PWMDRIVER_TIMER_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=PWM_TIMER_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_TimeBaseInitStructure.TIM_Period = PWMDRIVER_TIMER_PERIOD;
    TIM_TimeBaseInitStructure.TIM_Prescaler= PWMDRIVER_TIMER_PRESCALER;
    TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
    TIM_TimeBaseInit(PWMDRIVER_TIMER, &TIM_TimeBaseInitStructure);

    TIM_ITConfig(PWMDRIVER_TIMER, TIM_IT_Update,ENABLE);
    TIM_Cmd(PWMDRIVER_TIMER, DISABLE);
}

void PwmDriver_RegisterDeviceHandle(void* device, PwmDriverTimer_Handle handle)
{
    s_device = device;
    PwmDriver_Handle = handle;
}

Bool PwmDriver_SetPwm(float level)
{
    if(level < 1 && level > 0)
    {
        s_onTime = floor(level*100);
        s_offTime = 100 - s_onTime;
        TRACE_DEBUG("\nPWM set level =  %f, ontime = %d, offtime = %d", level, s_onTime, s_offTime);
        TIM_Cmd(PWMDRIVER_TIMER, ENABLE);
    }
    else if(level >= 1)
    {
        TIM_Cmd(PWMDRIVER_TIMER, DISABLE);
        s_onTime = 100;
        s_offTime = 0;
        s_curTime = 0;
        s_curState = TRUE;
        if(s_device != NULL && PwmDriver_Handle != NULL)
        {
            PwmDriver_Handle(s_device, s_curState);
        }
    }
    else if(level <= 0)
    {
        TIM_Cmd(PWMDRIVER_TIMER, DISABLE);
        s_onTime = 0;
        s_offTime = 100;
        s_curTime = 0;
        s_curState = FALSE;
        if(s_device != NULL && PwmDriver_Handle != NULL)
        {
            PwmDriver_Handle(s_device, s_curState);
        }
    }
    else
    {
        TRACE_INFO("\nPWM set level error");
        return FALSE;
    }

    return TRUE;
}

float PwmDriver_GetPwm(void)
{
    float level = (float)s_onTime/100;
    return level;
}

void PWMDRIVER_TIMER_IRQHANDLER(void)
{
    if(TIM_GetITStatus(PWMDRIVER_TIMER,TIM_IT_Update)==SET) //溢出中断
    {
        s_curTime++;

        if(s_curState == TRUE && s_curTime >= s_onTime)
        {
            s_curState = FALSE;
            s_curTime = 0;

            if(s_device != NULL && PwmDriver_Handle != NULL)
            {
                PwmDriver_Handle(s_device, s_curState);
            }
        }

        if(s_curState == FALSE && s_curTime >= s_offTime)
        {
            s_curState = TRUE;
            s_curTime = 0;

            if(s_device != NULL && PwmDriver_Handle != NULL)
            {
                PwmDriver_Handle(s_device, s_curState);
            }
        }
    }
    TIM_ClearITPendingBit(PWMDRIVER_TIMER,TIM_IT_Update);  //清除中断标志位
}

