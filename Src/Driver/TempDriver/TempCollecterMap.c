/*
 * TempCollecterMap.c
 *
 *  Created on: 2019年8月16日
 *      Author: Administrator
 */

#include "TempCollecterMap.h"
#include "Tracer/Trace.h"
#include "Driver/System.h"
#include <string.h>
#include "Driver/HardwareType.h"

#define TEMPADCOLLECT_AD_MAX  (4095)    // 24位AD
#define TEMPADCOLLECT_V_REF  (2.5)       // 单位(V)

//温度校准系统初始化参数
const static TempCalibrateParam s_kTempCalculateParam =
{ .negativeInput = 1.2475, .vref = 2.495, .vcal = 0, };

static double TempCollecterMap_GetResistanceValue(TempCalibrateParam *tempCalibrateParam, Uint16 ad);

void TempCollecterMap_Init(TempCollecter *tempCollecter)
{
    tempCollecter[MEASUREMODULE1_TEMP].getResistanceValueFunc = TempCollecterMap_GetResistanceValue;
    TempCollecter_SetNumber(&tempCollecter[MEASUREMODULE1_TEMP], MEASUREMODULE1_TEMP);
    TempCollecter_Init(&tempCollecter[MEASUREMODULE1_TEMP], s_kTempCalculateParam);

    tempCollecter[MEASUREMODULE2_TEMP].getResistanceValueFunc = TempCollecterMap_GetResistanceValue;
    TempCollecter_SetNumber(&tempCollecter[MEASUREMODULE2_TEMP], MEASUREMODULE2_TEMP);
    TempCollecter_Init(&tempCollecter[MEASUREMODULE2_TEMP], s_kTempCalculateParam);
}

static double TempCollecterMap_GetResistanceValue(TempCalibrateParam *tempCalibrateParam, Uint16 ad)
{
    double realV = ad * TEMPADCOLLECT_V_REF/ TEMPADCOLLECT_AD_MAX; //根据AD值计算得到电压值
    double rt;
//    TRACE_INFO("\n realV :%f ,ad: %d \n",realV,ad);
    if (realV >= TEMPADCOLLECT_V_REF)//超出PT1000计算范围
    {
        return 2000;
    }
    rt = ((tempCalibrateParam->vref - tempCalibrateParam->negativeInput)
            / (tempCalibrateParam->negativeInput - realV / 4)) * 1000;

//    TRACE_CODE("\n way 1 ad: %d ; realV: %f; Rt: %f,; Temp:", ad, realV, rt);
//    TRACE_INFO("\n way 1 ad: %d ; realV: %f; Rt: %f,; Temp:", ad, realV, rt);

    return rt;
}


char* TempCollecterMap_GetName(Uint8 index)
{
    static char name[20] = "";
    memset(name, 0, sizeof(name));
    switch(index)
    {
    case MEASUREMODULE1_TEMP:
        strcpy(name, "MeasureModule1");
        break;
    case MEASUREMODULE2_TEMP:
        strcpy(name, "MeasureModule2");
        break;
    default:
        strcpy(name, "NULL");
        break;
    }
    return name;
}

