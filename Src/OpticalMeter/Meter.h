/*
 * Meter.h
 *
 *  Created on: 2016年6月14日
 *      Author: Administrator
 */

#ifndef SRC_OPTICALMETER_METER_H_
#define SRC_OPTICALMETER_METER_H_

#include <LiquidDriver/MeterADC.h>
#include "Common/Types.h"

#define METER_PUMP_NUM         0        //用于定量泵号

#define METER_POINT_NUM        2        //本系统定量点支持的最大数目
#define SETVIDX                0        //定量点体积的设置值列下标
#define REALIDX                1        //定量点体积的真实值列下标
/**
 * @brief 定量点体积体系。
 */
typedef struct
{
    Uint8 num;                          //定量点数目
    float volume[METER_POINT_NUM][2];   //定量点体积，行表示某个定量点，列表示设置体积和真实体积
}MeterPoint;

/**
 * @brief 上报定量点AD值。
 */
typedef struct
{
    Uint8 num;
    Uint16 adValue[METER_POINT_NUM];
}OpticalMeterAD;

/**
 * @brief 定量模式。
 */
typedef enum
{
    METER_MODE_ACCURATE = 0,            //精准定量模式，精准定量到指定体积的定量点。
    METER_MODE_DIRECT = 1,              //直接定量模式，泵只向一个方向启动一次便可完成定量
    METER_MODE_SMART = 2,               //智能定量模式，结合定量点和泵计步综合定量出任意体积。
    METER_MODE_ROPINESS = 3,                // 粘稠液体定量模式
    METER_MODE_LAYERED = 4,             //分层液体定量模式
	METER_MODE_CALI = 5,//泵校准
    METER_MODE_MAX
} MeterMode;

/**
 * @brief 定量方向。
 */
typedef enum
{
    METER_DIR_EXTRACT = 0,
    METER_DIR_DRAIN,
    METER_EMPTY,
}MeterDir;
    


/**
 * @brief 定量状态。
 */
typedef enum
{
    METER_STATUS_IDLE,                  //定量空闲
    METER_STATUS_START,                 //定量开始
    METER_STATUS_FIRST_PUMPEXTRACT,     //定量泵第一次抽取
    METER_STATUS_SECOND_PUMPEXTRACT,    //定量泵第二次抽取
    METER_STATUS_DRAIN,                 //定量泵排空
    METER_STATUS_SMART                  //定量泵智能抽取
} MeterStatus;

void Meter_Init(void);
void Meter_Restore(void);
MeterStatus Meter_GetMeterStatus(void);
void Meter_SetMeterSpeed(float speed);
float Meter_GetMeterSpeed(void);
Bool Meter_SetMeterPoints(MeterPoint *dot);
MeterPoint Meter_GetMeterPoints(void);
Uint16 Meter_Start(MeterDir dir, MeterMode mode, float volume, float limitVolume);
Bool Meter_RequestStop(void);
OpticalMeterAD Meter_GetOpticalAD(void);
void Meter_SendMeterEventOpen(void);
void Meter_SetMeterADReportPeriod(float period);
Bool Meter_TurnOnLED(Uint8 num);
Bool Meter_TurnOffLED(Uint8 num);
Bool Meter_AutoCloseValve(Bool isAutoCloseValve);
Bool Meter_SetMeterFinishValveMap(Uint32 map);
Uint32 Meter_GetSingleOpticalAD(Uint8 num);
Bool Meter_SetRopinessOverValue(Uint16 value);
Uint16 Meter_GetRopinessOverValue(void);
void Meter_SetMeterThreshold(Uint16 value);
Uint16 Meter_GetMeterThreshold(void);
Uint16 Meter_GetMeterInterval(void);
void Meter_SetMeterInterval(Uint16 value);
Uint16 Meter_ReadAccurateModeOverValve();
Bool Meter_WriteAccurateModeOverValve(Uint16 valve);
void MeterAD_Threshold_Value_Set(int index, int meterAD);

#endif /* SRC_OPTICALMETER_METER_H_ */
