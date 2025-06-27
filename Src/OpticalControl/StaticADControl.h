/*
 * StaticADControl.h
 *
 *  Created on: 2018年11月21日
 *      Author: Administrator
 */

#ifndef SRC_OPTICALCONTROL_STATICADCONTROL_H_
#define SRC_OPTICALCONTROL_STATICADCONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include "Driver/System.h"

#define AD_CONTROLLER_NUM   4
#define LED_REF     0
#define LED_MEA     1
#define LED_METER1      2
#define LED_METER2      3
#define MCP4651_DEFAULTVALUE 256
#define MCP4651_DEFAULTVALUE_METER 40

void StaticADControl_InitDriver(void);
void StaticADControl_InitParam(void);
void StaticADControl_InitSetting(void);
void StaticADControl_Init(void);
Bool StaticADControl_Start(Uint8 index, Uint32 targetAD, Bool isExtLED);
void StaticADControl_Stop(void);
Uint16 StaticADControl_GetDefaultValue(Uint8 index, Bool isExtLED);
void StaticADControl_SetDefaultValue(Uint8 index, Uint16 value, Bool isExtLED);
Uint16 StaticADControl_GetRealValue(Uint8 index);
Bool StaticADControl_SetRealValue(Uint8 index, Uint16 value);
void StaticADControl_SendEventOpen(void);
void StaticADControl_SendEventClose(void);
Bool StaticADControl_IsValid(void);
Bool StaticADControl_ResetMeaLedParam(Bool isExtLED);
void StaticADControl_SetMultiple(float multiple);
//Bool StaticADControl_Gain_Set(int Gain);
//*****下位机测试使用*****
void StaticADControl_MCP4651Write(Uint8 index,Uint16 value);
void StaticADControl_MCP4651Read(Uint8 index);

#ifdef __cplusplus
}
#endif

#endif /* SRC_OPTICALCONTROL_STATICADCONTROL_H_ */
