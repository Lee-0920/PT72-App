/*
 * PumpTimer.h
 *
 *  Created on: 2016年5月31日
 *      Author: Administrator
 */

#ifndef SRC_DRIVER_PUMPTIMER_H_
#define SRC_DRIVER_PUMPTIMER_H_
#include "stm32f4xx.h"
#include "Common/Types.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef void (*TimerHandle)(void *obj);
void PumpTimer_Init(void);
void PumpTimer_Start(void* obj);
void PumpTimer_Stop(void* obj);
Bool PumpTimer_CancelRegisterHandle(void* obj);
Bool PumpTimer_RegisterHandle(TimerHandle Handle, void* obj);
void PumpTimer_SpeedSetting(void* obj, float speed);

#ifdef __cplusplus
}
#endif
#endif /* SRC_DRIVER_PUMPTIMER_H_ */
