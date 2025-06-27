/*
 * OpticalXonen.h
 *
 *  Created on: 2019年8月19日
 *      Author: Administrator
 */

#ifndef SRC_DRIVER_OPTICALDRIVER_OPTICALXONEN_H_
#define SRC_DRIVER_OPTICALDRIVER_OPTICALXONEN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "Common/Types.h"
#include "stm32f4xx.h"

extern __IO Bool collectSyncFlag;

void OpticalXonen_Init(void);               // I/0 及定时器初始化
void OpticalXonen_TurnOn(void);             // 打开氙灯
void OpticalXonen_TurnOff(void);            // 关闭氙灯
void OpticalXonen_Glitter(void);             // 氙灯定频闪烁

#ifdef __cplusplus
}
#endif

#endif /* SRC_DRIVER_OPTICALDRIVER_OPTICALXONEN_H_ */
