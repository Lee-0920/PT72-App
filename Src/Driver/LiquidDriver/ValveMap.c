/*
 * ValveMap.c
 *
 *  Created on: 2016年6月7日
 *      Author: Administrator
 */

#include <LiquidDriver/ValveMap.h>
#include "stm32f4xx.h"
#include "SolenoidValve/ValveManager.h"

//static Valve s_valveDir[2];

void ValveMap_Init(Valve *valve)
{
    Uint8 i;
    //上高压阀
    valve[0].pin = GPIO_Pin_0;
    valve[0].port = GPIOD;
    valve[0].rcc = RCC_AHB1Periph_GPIOD;
    ValveDriver_Init(&valve[0]);
    //下高压阀
    valve[1].pin = GPIO_Pin_1;
    valve[1].port = GPIOD;
    valve[1].rcc = RCC_AHB1Periph_GPIOD;
    ValveDriver_Init(&valve[1]);
    //阀接口3
	valve[2].pin = GPIO_Pin_2;
	valve[2].port = GPIOD;
	valve[2].rcc = RCC_AHB1Periph_GPIOD;
	ValveDriver_Init(&valve[2]);
	//阀接口4
	valve[3].pin = GPIO_Pin_3;
	valve[3].port = GPIOD;
	valve[3].rcc = RCC_AHB1Periph_GPIOD;
	ValveDriver_Init(&valve[3]);
    
    for(i = 0; i < SOLENOIDVALVECONF_TOTALVAlVES - ROTARY_TOTALVALVES; i++)
    {
    	ValveDriver_Control(&valve[i], VAlVE_CLOSE);
    }

}


