/*
 * ThermostatDeviceManager.h
 *
 *  Created on: 2019年8月17日
 *      Author: Administrator
 */

#ifndef SRC_TEMPERATURECONTROL_THERMOSTATDEVICEMANAGER_H_
#define SRC_TEMPERATURECONTROL_THERMOSTATDEVICEMANAGER_H_


#include <TempDriver/ThermostatDevice.h>
#include <TempDriver/ThermostatDeviceMap.h>
#include <TempDriver/PwmDriver.h>
#include "Common/Types.h"

void ThermostatDeviceManager_Init();
char* ThermostatDeviceManager_GetName(Uint8 index);
Bool ThermostatDeviceManager_IsOpen(Uint8 index);
Bool ThermostatDeviceManager_SetOutput(Uint8 index, float level);
float ThermostatDeviceManager_GetMaxDutyCycle(Uint8 index);
Bool ThermostatDeviceManager_SetMaxDutyCycle(Uint8 index, float value);
void ThermostatDeviceManager_RestoreInit(void);

#endif /* SRC_TEMPERATURECONTROL_THERMOSTATDEVICEMANAGER_H_ */
