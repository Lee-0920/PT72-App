/*
 * ThermostatDeviceMap.h
 *
 *  Created on: 2019年8月16日
 *      Author: Administrator
 */

#ifndef SRC_DRIVER_TEMPDRIVER_THERMOSTATDEVICEMAP_H_
#define SRC_DRIVER_TEMPDRIVER_THERMOSTATDEVICEMAP_H_


#include <TempDriver/ThermostatDevice.h>

#define MEASUREMODULE_HEATER1           0
#define MEASUREMODULE_FAN1           1

#define TOTAL_THERMOSTATDEVICE     2

void ThermostatDeviceMap_Init(ThermostatDevice* device);
char* ThermostatDeviceMap_GetName(Uint8 index);


#endif /* SRC_DRIVER_TEMPDRIVER_THERMOSTATDEVICEMAP_H_ */
