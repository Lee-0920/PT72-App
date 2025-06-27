/*
 * LiquidSensor.h
 *
 *  Created on: 2022年4月25日
 *      Author: hyz
 */

#include "LiquidSensor.h"
#include "Tracer/Trace.h"

#define SPIN_VALVE_SENSOR_NUM			2

static SpinValveSensor s_valveSensor[SPIN_VALVE_SENSOR_NUM] = {0};

static void LiquidSensor_DriverInit(SpinValveSensor spinValveSensor)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(spinValveSensor.rcc , ENABLE);

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Pin = spinValveSensor.pin;
    GPIO_Init(spinValveSensor.port, &GPIO_InitStructure);
}

static void LiquidSensor_MapInit()
{
	s_valveSensor[0].rcc 	= RCC_AHB1Periph_GPIOA;
	s_valveSensor[0].port 	= GPIOA;
	s_valveSensor[0].pin 	= GPIO_Pin_6;

	s_valveSensor[1].rcc 	= RCC_AHB1Periph_GPIOA;
	s_valveSensor[1].port 	= GPIOA;
	s_valveSensor[1].pin 	= GPIO_Pin_7;

//	s_valveSensor[2].rcc 	= RCC_AHB1Periph_GPIOC;
//	s_valveSensor[2].port 	= GPIOC;
//	s_valveSensor[2].pin 	= GPIO_Pin_2;
//
//	s_valveSensor[3].rcc 	= RCC_AHB1Periph_GPIOC;
//	s_valveSensor[3].port 	= GPIOC;
//	s_valveSensor[3].pin 	= GPIO_Pin_3;

}

/**************************************************************************************/

void LiquidSensor_Init()
{
	LiquidSensor_MapInit();
	for(Uint8 index=0; index<SPIN_VALVE_SENSOR_NUM; index++)
	{
		LiquidSensor_DriverInit(s_valveSensor[index]);
	}
}

Bool LiquidSensor_ReadSensorStatus(Uint8 index)
{
    if (GPIO_ReadInputDataBit(s_valveSensor[index].port, s_valveSensor[index].pin))
    {
    	//return TRUE;
    	return FALSE;
    }
    else
    {
        //return FALSE;
        return TRUE;
    }
}
