/*
 * CheckLeakingControl.h
 *
 *  Created on: 2021年10月11日
 *      Author: liwenqin
 */

#ifndef SRC_CHECKLEAKING_CHECKLEAKINGCONTROL_H_
#define SRC_CHECKLEAKING_CHECKLEAKINGCONTROL_H_

#include "Common/Types.h"


void CheckLeakingControl_Init(void);
Uint16 CheckLeakingControl_GetAD(void);
void CheckLeaking_SetCheckLeakingReportPeriod(float period);


#endif /* SRC_CHECKLEAKING_CHECKLEAKINGCONTROL_H_ */
