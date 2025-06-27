/*
 * SensorManager.c
 *
 *  Created on: 2022年4月25日
 *      Author: hyz
 */

#include "SensorManager.h"


void SensorManager_Init(void)
{
	LiquidSensor_Init();
}


Bool SensorManager_ReadSensorStatus(Uint8 index)
{
	return LiquidSensor_ReadSensorStatus(index);
}
