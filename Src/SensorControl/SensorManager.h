/*
 * SensorManager.h
 *
 *  Created on: 2022年4月25日
 *      Author: hyz
 */

#include "Common/Types.h"
#include "LiquidDriver/LiquidSensor.h"

#define SENSOR_MAX_NUM			SPIN_VALVE_SENSOR_NUM

void SensorManager_Init(void);
Bool SensorManager_ReadSensorStatus(Uint8 index);
