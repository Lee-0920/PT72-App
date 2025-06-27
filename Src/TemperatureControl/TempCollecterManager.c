/*
 * TempCollecterManager.c
 *
 *  Created on: 2019年8月17日
 *      Author: Administrator
 */


#include <TempDriver/EnvTempCollect.h>
#include <TempDriver/TempCollecterMap.h>
#include "TempCollecterManager.h"
#include "Tracer/Trace.h"

TempCollecter s_tempCollecter[TOTAL_TEMP];

void TempCollecterManager_Init()
{
    TempADCollect_Init();
    EnvTempCollect_Init();
    TempCollecterMap_Init(s_tempCollecter);
}

char* TempCollecterManager_GetName(Uint8 index)
{
    return TempCollecterMap_GetName(index);
}

TempCalibrateParam TempCollecterManager_GetCalibrateFactor(Uint8 index)
{
    TempCalibrateParam calibratefactor;
    if (index < TOTAL_TEMP)
    {
        calibratefactor = TempCollecter_GetCalibrateFactor(&s_tempCollecter[index]);
    }
    else
    {
        TRACE_ERROR("\n GetCalibrateFactor  No. %d tempCollecter.", index);
    }
    return calibratefactor;
}

Bool TempCollecterManager_SetCalibrateFactor(Uint8 index, TempCalibrateParam tempCalibrateParam)
{
    Bool ret = FALSE;
    if (index < TOTAL_TEMP)
    {
        TempCollecter_SetCalibrateFactor(&s_tempCollecter[index], tempCalibrateParam);
        ret = TRUE;
    }
    else
    {
        TRACE_ERROR("\n SetCalibrateFactor  No. %d tempCollecter.", index);
    }
    return ret;
}

float TempCollecterManager_GetTemp(Uint8 index)
{
    float value = 0;
    if (index < TOTAL_TEMP)
    {
        value = TempCollecter_GetTemperature(&s_tempCollecter[index]);
    }
    else
    {
        TRACE_ERROR("\n GetTemp No. %d tempCollecter.", index);
    }
    return value;
}

float TempCollecterManager_GetEnvironmentTemp()
{
    return EnvironmentTemp_Get();
}

