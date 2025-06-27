/*
 * HardwareFlag.h
 *
 *  Created on: 2020年4月10日
 *      Author: Administrator
 */

#ifndef SRC_DRIVER_HARDWARETYPE_H_
#define SRC_DRIVER_HARDWARETYPE_H_

#include "Common/Types.h"

typedef enum
{
    Hardware_v10 = 0,
    Hardware_v20 = 1,
}HardwareType;

void HardwareType_Init(void);
Uint8 HardwareType_GetValue(void);
float HardwareType_GetRefVoltage();

#endif /* SRC_DRIVER_HARDWARETYPE_H_ */
