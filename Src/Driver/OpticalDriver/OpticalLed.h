/**
 * @file
 * @brief 光学信号LED驱动头文件。
 * @details 提供光学信号LED控制功能接口。
 * @version 1.1.0
 * @author kim.xiejinqiang
 * @date 2015-06-04
 */

#ifndef SRC_DRIVER_OPTICALLED_H_
#define SRC_DRIVER_OPTICALLED_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "Common/Types.h"

#define DAC_CHANNEL_NUM   1

#define DAC_MAX_LIMIT_VALUE  1.5

void OpticalLed_Init(void);
void OpticalLed_TurnOn(Uint8 index);
void OpticalLed_TurnOff(Uint8 index);
void OpticalLed_ChangeDACOut(Uint8 index, float valve);
void OpticalLed_InitParam(Uint8 index);
float OpticalLed_GetDefaultValue(Uint8 index);
Bool OpticalLed_SetDefaultValue(Uint8 index, float value);

#ifdef __cplusplus
}
#endif

#endif /* SRC_DRIVER_OPTICALLED_H_ */
