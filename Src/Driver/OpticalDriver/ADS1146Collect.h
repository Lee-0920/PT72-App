/*
 * ADS1146Collect.h
 *
 *  Created on: 2019年8月19日
 *      Author: Administrator
 */

#ifndef SRC_DRIVER_OPTICALDRIVER_ADS1146COLLECT_H_
#define SRC_DRIVER_OPTICALDRIVER_ADS1146COLLECT_H_

#include "stm32f4xx.h"
#include "Common/Types.h"

#ifdef __cplusplus
extern "C" {
#endif

void ADS1146Collect_Init(void);
Uint16 ADS1146Collect_GetAD(Uint8 channel);
Uint16 ADS1146Collect_GetADWithSync(Uint8 channel);
Bool ADS1146Collect_GetDoubleChannelADWithSync(Uint16 *channel1Buff, Uint8 channel1, Uint16 *channel2Buff, Uint8 channel2, Uint8 len, Uint32 sampleTimeOut);
Bool ADS1146Collect_GetDoubleChannelAD(Uint16 *channel1Buff, Uint8 channel1, Uint16 *channel2Buff, Uint8 channel2, Uint8 len, Uint32 sampleTimeOut);
#ifdef __cplusplus
}
#endif

#endif /* SRC_DRIVER_OPTICALDRIVER_ADS1146COLLECT_H_ */
