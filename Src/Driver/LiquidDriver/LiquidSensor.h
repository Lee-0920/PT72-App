/*
 * LiquidSensor.h
 *
 *  Created on: 2022年3月10日
 *      Author: hyz
 */

#ifndef SRC_LIQUIDSENSOR_H_
#define SRC_LIQUIDSENSOR_H_

#include "stm32f4xx.h"
#include "Common/Types.h"

typedef struct
{
    GPIO_TypeDef *port;
    Uint16 pin;
    Uint32 rcc;
}SpinValveSensor;

void LiquidSensor_Init();
Bool LiquidSensor_ReadSensorStatus(Uint8 index);

#endif /* SRC_LIQUIDSENSOR_H_ */

