/*
 * SpinValveManager.h
 *
 *  Created on: 2019年9月11日
 *      Author: Administrator
 */

#ifndef SRC_SPINVALVE_SPINVALVE_H_
#define SRC_SPINVALVE_SPINVALVE_H_

#include "stm32f4xx.h"
#include "Common/Types.h"
#include "Driver/LiquidDriver/SpinValveDriver.h"

#define MAX_SPIN_VALVE_NUM 10

void SpinValve_Init(void);
Bool SpinValve_Reset(Bool waitEvent);
Bool SpinValve_Stop(Bool waitEvent);
Bool SpinValve_OpenValve(Uint8 index, Bool waitEvent);
Bool SpinValve_Control(Uint8 cmd, Uint8 param1, Uint8 param2, Bool waitEvent);
Uint8 SpinValve_GetCurrentValveIndex(void);
Uint16 SpinValve_GetTotalValves(void);
Uint8 SpinValve_GetStatus(void);
void SpinValve_CmdQueryTest(Uint8 cmd);
void SpinValve_CmdFactoryTest(Uint8 cmd, Uint32 param);
void SpinValve_CmdList(void);



#endif /* SRC_SPINVALVE_SPINVALVE_H_ */
