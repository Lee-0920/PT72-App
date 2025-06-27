/*
 * HardwareFlag.c
 *
 *  Created on: 2020年4月10日
 *      Author: Administrator
 */

#include "stm32f4xx.h"
#include <HardwareType.h>
#include "Tracer/Trace.h"
#include "TempDriver/TempADCollect.h"

static float s_refVoltage = 2.5;

/**
 * @brief 硬件板卡类型标记初始化
 */
void HardwareType_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

	Uint8 value = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4);

	Printf("\n***************************************************************");
    if(0 == value)
    {
    	Printf("\nHardware Versiont is V1.0");
    }
    else if(1 == value)
    {
    	Printf("\nHardware Versiont is V2.0");
    }
    else
    {
    	Printf("\nAD error ,No corresponding version");
    }
	Printf("\n***************************************************************");
}

float HardwareType_GetRefVoltage()
{
	return s_refVoltage;
}

/**
 * @brief 读取硬件版本标记
 * @return 标记值
 */
Uint8 HardwareType_GetValue(void)
{
    Uint8 value = 0;
    Uint8 AD = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4);

    if(0 == AD)
    {
    	value = 0;
    }

    if(1 == AD)
    {
    	value = 1;
    }

    return value;
}

