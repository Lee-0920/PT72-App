/*
 * PumpMap.c
 *
 *  Created on: 2016年5月30日
 *      Author: Administrator
 */
#include "stm32f4xx.h"
#include "PumpDriver.h"
#include "PumpMap.h"
#include "HardwareType.h"

void PumpMap_Init(Pump *pump)
{
	pump[0].driver.pinClock = GPIO_Pin_12;
	pump[0].driver.portClock = GPIOD;
	pump[0].driver.rccClock = RCC_AHB1Periph_GPIOD;

	pump[0].driver.pinDir = GPIO_Pin_11;
	pump[0].driver.portDir = GPIOD;
	pump[0].driver.rccDir = RCC_AHB1Periph_GPIOD;

	pump[0].driver.pinDiag = GPIO_Pin_10;
	pump[0].driver.portDiag = GPIOD;
	pump[0].driver.rccDiag = RCC_AHB1Periph_GPIOD;

	pump[0].driver.pinReset = GPIO_Pin_13;
	pump[0].driver.portReset = GPIOD;
	pump[0].driver.rccReset = RCC_AHB1Periph_GPIOD;

    PumpDriver_Init(&pump[0].driver);
    PumpDriver_PullLow(&pump[0].driver);
    PumpDriver_SetForwardLevel(&pump[0].driver,Bit_RESET);


}

