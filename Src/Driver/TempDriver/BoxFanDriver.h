/*
 * BoxFanDriver.h
 *
 *  Created on: 2016年10月10日
 *      Author: LIANG
 */
#include "stm32f4xx.h"
#include "Common/Types.h"

#ifndef SRC_DRIVER_TEMPDRIVER_BOXFANDRIVER_H_
#define SRC_DRIVER_TEMPDRIVER_BOXFANDRIVER_H_

void BoxFanDriver_Init(void);
void BoxFanDriver_SetOutput(float level);
Bool BoxFanDriver_IsOpen(void);

#endif /* SRC_DRIVER_TEMPDRIVER_BOXFANDRIVER_H_ */
