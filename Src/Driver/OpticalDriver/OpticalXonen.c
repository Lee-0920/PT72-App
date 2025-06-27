/*
 * OpticalXonen.c
 *
 *  Created on: 2019年8月19日
 *      Author: Administrator
 */

//#include <OpticalDriver/OpticalLed.h>
#include <OpticalDriver/OpticalXonen.h>
#include <string.h>
#include <stdlib.h>
#include "Common/Types.h"
#include "SystemConfig.h"
#include "Tracer/Trace.h"
#include "FreeRTOS.h"
#include "task.h"

__IO Bool collectSyncFlag = FALSE;

typedef enum
{
    CONTROL_STATE_INIT = 0,                 // 初始化
    CONTROL_STATE_INTEGRAL_CLEAR,           // 积分清零
    CONTROL_STATE_INTERVAL_BEFORE_INTEGRAL, // 积分前的间隔
    CONTROL_STATE_INTEGRAL,                 // 积分
    CONTROL_STATE_XONEN_ON,                 // 氙灯打开
    CONTROL_STATE_XONEN_OFF,                // 氙灯关闭
    CONTROL_STATE_MAX

}ControlState;

// **************** 引脚定义 ****************
// 氙灯触发控制管脚
#define OPT_XONEN_TRIGGER_PIN                   GPIO_Pin_12
#define OPT_XONEN_TRIGGER_PORT                  GPIOC
#define OPT_XONEN_TRIGGER_RCC                   RCC_AHB1Periph_GPIOC
#define OPT_XONEN_TRIGGER_HIGH()                GPIO_ResetBits(OPT_XONEN_TRIGGER_PORT, OPT_XONEN_TRIGGER_PIN)
#define OPT_XONEN_TRIGGER_LOW()                 GPIO_SetBits  (OPT_XONEN_TRIGGER_PORT, OPT_XONEN_TRIGGER_PIN)


// 氙灯清零
#define OPT_XONEN_INTEGRAL_CLEAR_PIN            GPIO_Pin_5
#define OPT_XONEN_INTEGRAL_CLEAR_PORT           GPIOA
#define OPT_XONEN_INTEGRAL_CLEAR_RCC            RCC_AHB1Periph_GPIOA
#define OPT_XONEN_INTEGRAL_CLEAR_HIGH()         GPIO_ResetBits(OPT_XONEN_INTEGRAL_CLEAR_PORT, OPT_XONEN_INTEGRAL_CLEAR_PIN)
#define OPT_XONEN_INTEGRAL_CLEAR_LOW()          GPIO_SetBits(OPT_XONEN_INTEGRAL_CLEAR_PORT, OPT_XONEN_INTEGRAL_CLEAR_PIN)


// 氙灯积分
#define OPT_XONEN_INTEGRAL_PIN                  GPIO_Pin_4
#define OPT_XONEN_INTEGRAL_PORT                 GPIOA
#define OPT_XONEN_INTEGRAL_RCC                  RCC_AHB1Periph_GPIOA
#define OPT_XONEN_INTEGRAL_HIGH()               GPIO_ResetBits(OPT_XONEN_INTEGRAL_PORT, OPT_XONEN_INTEGRAL_PIN)
#define OPT_XONEN_INTEGRAL_LOW()                GPIO_SetBits(OPT_XONEN_INTEGRAL_PORT, OPT_XONEN_INTEGRAL_PIN)


// 定时器
#define OPT_XONEN_TIMER_RCC                     RCC_APB1Periph_TIM6
#define OPT_XONEN_TIMER_IRQn                    TIM6_DAC_IRQn
#define OPT_XONEN_IRQHANDLER                    TIM6_DAC_IRQHandler
#define OPT_XONEN_TIMER                         TIM6

// 氙灯驱动时间参数
#define TIME_INTEGRAL_CLEAR                     (72-1)          // 积分清零时间                          实为：120   单位：us
#define TIME_INTEGRAL_CLEAN_TO_TURN_ON_XONEN    (6-1)           // 积分清零至打开氙灯的时间    实为：10    单位：us
#define TIME_INTEGRAL                           (72-1)          // 积分时间                                  实为：120   单位：us
#define TIME_XONEN_ON                           (6000-1)        // 氙灯打开时间                           实为：10000 单位：us
#define TIME_XONEN_OFF                          (54000-1)       // 氙灯关闭时间                           实为：90000 单位：us
#define TIME_WAIT_SYNC                          (600-1)           // 等待同步时间                        实为：1000  单位：us


// 定时器
#define OPT_XONEN_TIMER_PERIOD                  (6000-1)    // 重载计数
#define OPT_XONEN_TIMER_PRESCALER               (150-1)     // 预分频

// DAC输出值
//#define OPT_XONEN_LIGHT_ADJUST_DAC_OUTPUT       (2048)      // 1个DAC值约为0.80587mV

// 氙灯触发控制
static Uint8 s_xonenTriggerControl = FALSE;

// 氙灯闪烁
static Uint8 s_xonenGlitter = FALSE;

#define XONEN_GLITTER_HZ      60

static void OpticalXonen_RCCConfiguration(void)
{
    // 使能I/O时钟
    RCC_AHB1PeriphClockCmd(
                            OPT_XONEN_TRIGGER_RCC |
							OPT_XONEN_INTEGRAL_CLEAR_RCC |
							OPT_XONEN_INTEGRAL_RCC, ENABLE );

    // 定时器
    RCC_APB1PeriphClockCmd(OPT_XONEN_TIMER_RCC, ENABLE);
}

static void OpticalXonen_NVICConfiguration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    // 变量初始化
    memset(&NVIC_InitStructure, 0, sizeof(NVIC_InitStructure));

    NVIC_InitStructure.NVIC_IRQChannel                   = OPT_XONEN_TIMER_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = OPT_XONEN_TIMER_IQR_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

static void OpticalXonen_GPIOConfiguration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    // 氙灯触发控制
    GPIO_InitStructure.GPIO_Pin     = OPT_XONEN_TRIGGER_PIN;
    GPIO_Init(OPT_XONEN_TRIGGER_PORT, &GPIO_InitStructure);

    // 积分清零
    GPIO_InitStructure.GPIO_Pin     = OPT_XONEN_INTEGRAL_CLEAR_PIN;
    GPIO_Init(OPT_XONEN_INTEGRAL_CLEAR_PORT, &GPIO_InitStructure);

    // 积分
    GPIO_InitStructure.GPIO_Pin     = OPT_XONEN_INTEGRAL_PIN;
    GPIO_Init(OPT_XONEN_INTEGRAL_PORT, &GPIO_InitStructure);
}

static void OpticalXonen_TimerConfiguration(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    // 变量初始化
    memset(&TIM_TimeBaseStructure, 0, sizeof(TIM_TimeBaseStructure));

    TIM_TimeBaseStructure.TIM_Period        = OPT_XONEN_TIMER_PERIOD;
    TIM_TimeBaseStructure.TIM_Prescaler     = OPT_XONEN_TIMER_PRESCALER;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(OPT_XONEN_TIMER, &TIM_TimeBaseStructure);
    TIM_ClearFlag(OPT_XONEN_TIMER, TIM_IT_Update);
    TIM_ITConfig(OPT_XONEN_TIMER, TIM_IT_Update, ENABLE);  // 使能计数中断
    TIM_Cmd(OPT_XONEN_TIMER, DISABLE);
}

/**
 * @brief I/O状态初始化
 */
static void OpticalXonen_CustomConfiguration(void)
{
    // 控制管脚初始化
    OPT_XONEN_TRIGGER_LOW();                    // 触发信号

    OPT_XONEN_INTEGRAL_CLEAR_HIGH();            // 积分清零信号
    OPT_XONEN_INTEGRAL_HIGH();                  // 积分信号
//    OPT_XONEN_INTEGRAL_LOW();
//    OPT_XONEN_INTEGRAL_CLEAR_LOW();

    // 打开定时器
    TIM_SetCounter(OPT_XONEN_TIMER, 0);
    TIM_Cmd(OPT_XONEN_TIMER, ENABLE);
}

/**
 * @brief 氙灯控制接口初始化
 */
void OpticalXonen_Init(void)
{
    OpticalXonen_RCCConfiguration();
    OpticalXonen_NVICConfiguration();
    OpticalXonen_GPIOConfiguration();
    OpticalXonen_TimerConfiguration();
    OpticalXonen_CustomConfiguration();
}

/**
 * @brief 打开氙灯
 * @details
 * @param
 * @param
 * @return
 */
void OpticalXonen_TurnOn(void)
{
    s_xonenTriggerControl = TRUE;
    s_xonenGlitter = FALSE;
}

/**
 * @brief 关闭氙灯
 * @details
 * @param
 * @param
 * @return
 */
void OpticalXonen_TurnOff(void)
{
    s_xonenTriggerControl = FALSE;
    s_xonenGlitter = FALSE;
}

/**
 * @brief 氙灯定频闪烁
 */
void OpticalXonen_Glitter(void)
{
    s_xonenTriggerControl = TRUE;
    s_xonenGlitter = TRUE;
}


/**
 * @brief 定时器中断处理程序
 */
void OPT_XONEN_IRQHANDLER(void)
{
    if(TIM_GetITStatus(OPT_XONEN_TIMER, TIM_IT_Update) != RESET)
    {
        // 氙灯控制状态
        static ControlState s_control_state = CONTROL_STATE_INIT;
        Uint16 countValue = 0;

        // 清更新中断标志
        TIM_ClearITPendingBit(OPT_XONEN_TIMER, TIM_IT_Update);

        switch(s_control_state)
        {
        // 初始化状态
        case CONTROL_STATE_INIT:
            collectSyncFlag = FALSE;


            OPT_XONEN_INTEGRAL_CLEAR_LOW();		// 积分清零控制管脚置低
            countValue = TIME_INTEGRAL_CLEAR;       // 120us
            s_control_state  = CONTROL_STATE_INTEGRAL_CLEAR;
            break;

        // 积分清零
        case CONTROL_STATE_INTEGRAL_CLEAR:
        	OPT_XONEN_INTEGRAL_CLEAR_HIGH();  // 积分清零控制管脚置高
            countValue = TIME_INTEGRAL_CLEAN_TO_TURN_ON_XONEN; //10us
            s_control_state = CONTROL_STATE_INTERVAL_BEFORE_INTEGRAL;
            break;

        // 积分清零至积分间隔
        case CONTROL_STATE_INTERVAL_BEFORE_INTEGRAL:
            // 氙灯
            if (TRUE == s_xonenTriggerControl)
            {
                OPT_XONEN_TRIGGER_HIGH();
            }

            OPT_XONEN_INTEGRAL_LOW();               // 积分
            countValue = TIME_INTEGRAL;             //120us
            s_control_state = CONTROL_STATE_INTEGRAL;
            break;

        // 积分
        case CONTROL_STATE_INTEGRAL:

        	OPT_XONEN_INTEGRAL_HIGH();
            countValue = TIME_XONEN_ON - TIME_INTEGRAL; // 10000us - 120us = 9.878ms
            s_control_state = CONTROL_STATE_XONEN_ON;
            break;

        // 氙灯打开
        case CONTROL_STATE_XONEN_ON:
            collectSyncFlag = TRUE;
            OPT_XONEN_TRIGGER_LOW();

            countValue = TIME_WAIT_SYNC;// 1ms
            s_control_state = CONTROL_STATE_XONEN_OFF;
            break;

        // 氙灯关闭
        case CONTROL_STATE_XONEN_OFF:
	        if(s_xonenGlitter == TRUE)
	        {
	            s_control_state = CONTROL_STATE_INIT;
	            countValue = TIME_WAIT_SYNC * 5;
	        }
	        else
	        {
	            if (collectSyncFlag == FALSE)
	            {
	                s_control_state = CONTROL_STATE_INIT;
	            }
	            countValue = TIME_WAIT_SYNC;// 1ms
			}
            break;

        default:
            OPT_XONEN_TRIGGER_LOW();
            OPT_XONEN_INTEGRAL_CLEAR_HIGH();
            OPT_XONEN_INTEGRAL_HIGH();
//            OPT_XONEN_INTEGRAL_LOW();
//            OPT_XONEN_INTEGRAL_CLEAR_LOW();

            countValue = TIME_XONEN_ON;
            s_control_state = CONTROL_STATE_INIT;
            break;
        }
        //TRACE_INFO("\nTimer6");
        // 装载计数值
        TIM_SetCounter(OPT_XONEN_TIMER, 0);
        TIM_SetAutoreload(OPT_XONEN_TIMER, countValue);
    }
}

