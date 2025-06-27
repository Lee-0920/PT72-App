/*
 * PumpManager.h
 *
 *  Created on: 2016年5月31日
 *      Author: Administrator
 */

#ifndef SRC_PERISTALTICPUMP_PUMPMANAGER_H_
#define SRC_PERISTALTICPUMP_PUMPMANAGER_H_

#include <LiquidDriver/PumpDriver.h>
#include <LiquidDriver/PumpMap.h>
#include <LiquidDriver/PumpTimer.h>
#include "Pump.h"
#include "Common/Types.h"
#include "string.h"
#include "tracer/trace.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define PUMPMANAGER_TOTAL_PUMP 1

void PumpManager_Init(void);
void PumpManager_Restore(void);
Pump* PumpManager_GetPump(Uint8 index);
Uint16 PumpManager_GetTotalPumps(void);
PumpParam PumpManager_GetMotionParam(Uint8 index);
Uint16 PumpManager_SetMotionParam(Uint8 index, float acceleration, float maxSpeed, ParamFLashOp flashOp);
float PumpManager_GetFactor(Uint8 index);
float PumpManager_GetTempFactor(Uint8 index);
Uint16 PumpManager_SetFactor(Uint8 index, float factor);
Uint16 PumpManager_SetTempFactor(Uint8 index, float factor);
PumpStatus PumpManager_GetStatus(Uint8 index);
Bool PumpManager_ImmediatelyStop(Uint8 index);
float PumpManager_GetVolume(Uint8 index);
float PumpManager_GetAirVolume(Uint8 index);
Uint16 PumpManager_Start(Uint8 index, Direction dir, float volume, ParamFLashOp flashOp);
Uint16 PumpManager_Stop(Uint8 index);
void PumpManager_ChangeVolume(Uint8 index, float volume);
Bool PumpManager_SendEventOpen(Uint8 index);
Uint32 PumpManager_GetAlreadyStep(Uint8 index);
Uint8 PumpManager_GetSubdivision(Uint8 index);
void PumpManager_Reset(void);
#ifdef __cplusplus
}
#endif

#endif /* SRC_DRIVER_PUMPMANAGER_H_ */
