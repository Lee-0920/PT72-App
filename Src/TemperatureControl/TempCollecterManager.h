/*
 * TempCollecterManager.h
 *
 *  Created on: 2019年8月17日
 *      Author: Administrator
 */

#ifndef SRC_TEMPERATURECONTROL_TEMPCOLLECTERMANAGER_H_
#define SRC_TEMPERATURECONTROL_TEMPCOLLECTERMANAGER_H_

#include <TempDriver/TempADCollect.h>
#include <TempDriver/TempCollecter.h>

void TempCollecterManager_Init();
char* TempCollecterManager_GetName(Uint8 index);
TempCalibrateParam TempCollecterManager_GetCalibrateFactor(Uint8 index);
Bool TempCollecterManager_SetCalibrateFactor(Uint8 index, TempCalibrateParam tempCalibrateParam);
float TempCollecterManager_GetTemp(Uint8 index);
float TempCollecterManager_GetEnvironmentTemp();

#endif /* SRC_TEMPERATURECONTROL_TEMPCOLLECTERMANAGER_H_ */
