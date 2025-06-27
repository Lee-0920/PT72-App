/*
 * ValveManager.h
 *
 *  Created on: 2016年6月7日
 *      Author: Administrator
 */

#ifndef SRC_SOLENOIDVALVE_VALVEMANAGER_H_
#define SRC_SOLENOIDVALVE_VALVEMANAGER_H_

#include "Common/Types.h"

#define SOLENOIDVALVECONF_TOTALVAlVES         16
#define ROTARY_TOTALVALVES 					  12
#define SOLENOIDVALVE_MAX_MAP                 0x7FFF

void ValveManager_Init(void);
Uint16 ValveManager_GetTotalValves(void);
Bool ValveManager_SetValvesMap(Uint32 map);
Bool ValveManager_CtrlSpinValve(Uint8 cmd, Uint8 param1, Uint8 Param2);
Uint32 ValveManager_GetValvesMap(void);
bool valve_control(Uint8 index, bool isOpen);

#endif /* SRC_SOLENOIDVALVE_VALVEMANAGER_H_ */
