/*
 * OpticalMeter.c
 *
 *  Created on: 2016年6月27日
 *      Author: Administrator
 */

#include "Tracer/Trace.h"
#include "Dncp/App/DscpSysDefine.h"
#include <string.h>
#include "OpticalMeter/Meter.h"
#include "PeristalticPump/PumpManager.h"
#include "System.h"
#include "OpticalMeter.h"

void OpticalMeter_GetPumpFactor(DscpDevice* dscp, Byte* data, Uint16 len)
{
    float factor;
    factor = PumpManager_GetFactor(METER_PUMP_NUM);

    TRACE_MARK("Pump %d Factor", METER_PUMP_NUM);
    System_PrintfFloat(TRACE_LEVEL_MARK, factor, 8);
    TRACE_MARK(" ml/step");

    DscpDevice_SendResp(dscp, &factor, sizeof(float));
}

void OpticalMeter_SetPumpFactor(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 ret = DSCP_OK;
    float factor;
    Uint16 size = 0;

    //设置数据正确性判断
    size = sizeof(float);
    if ((len > size))
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("Parame Len Error\n");
        TRACE_ERROR("%d \n", size);
    }
    else
    {
        memcpy(&factor, data, sizeof(float));
        ret = PumpManager_SetFactor(METER_PUMP_NUM, factor);
    }
    DscpDevice_SendStatus(dscp, ret);
}

void OpticalMeter_GetMeterPoints(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint8 temp[METER_POINT_NUM * 8 + 1] =
    { 0 };
    MeterPoint dot;

    memset(&dot, 0, sizeof(MeterPoint));

    dot = Meter_GetMeterPoints();

    len = sizeof(Uint8) + sizeof(float) * 2 * dot.num;

    TRACE_CODE(" num :%d \n", dot.num);
    for (Uint8 i = 0; i < dot.num; i++)
    {
        TRACE_MARK("\n %d point ", i);
        TRACE_MARK("SetV:");
        System_PrintfFloat(TRACE_LEVEL_MARK, dot.volume[i][SETVIDX], 2);
        TRACE_MARK(" ml RealV:");
        System_PrintfFloat(TRACE_LEVEL_MARK, dot.volume[i][REALIDX], 2);
        TRACE_MARK(" ml");
    }

    temp[0] = dot.num;
    memcpy(temp + 1, dot.volume, sizeof(float) * 2 * dot.num);
    DscpDevice_SendResp(dscp, temp, len);
}

void OpticalMeter_SerMeterPoints(DscpDevice* dscp, Byte* data, Uint16 len)
{
    MeterPoint dot;
    Uint16 ret = DSCP_OK;

    memset(&dot, 0, sizeof(MeterPoint));
    dot.num = data[0];
    if (dot.num <= METER_POINT_NUM && dot.num > 0)
    {
        Uint8 i;
        for (i = 0; i < dot.num; i++)
        {
            dot.volume[i][SETVIDX] = *(float *) (data + 1 + 8 * i);
            dot.volume[i][REALIDX] = *(float *) (data + 5 + 8 * i);
        }
        if (FALSE == Meter_SetMeterPoints(&dot))
        {
            ret = DSCP_ERROR;
        }
    }
    else
    {
        TRACE_ERROR(
                "\n Set the volume of the meter point to fail because The point num must be 1 to %d.",
                METER_POINT_NUM);
        ret = (Uint16) DSCP_ERROR_PARAM;
    }
    DscpDevice_SendStatus(dscp, ret);
}

void OpticalMeter_GetMeterStatus(DscpDevice* dscp, Byte* data, Uint16 len)
{
    if (METER_STATUS_IDLE == Meter_GetMeterStatus())
    {
        TRACE_MARK("Meter:  IDLE\n");
        DscpDevice_SendStatus(dscp, DSCP_IDLE);
    }
    else
    {
        TRACE_MARK("Meter:  BUSY\n");
        DscpDevice_SendStatus(dscp, DSCP_BUSY);
    }
}

void OpticalMeter_StartMeter(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 ret = DSCP_OK;
    Uint16 size = 0;
    MeterDir dir;
    MeterMode mode;
    float volume;
    float limitVolume;
    //设置数据正确性判断
    size = sizeof(Uint8) + sizeof(MeterMode) + sizeof(float) * 2;
    if ((len > size))
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("Parame Len Error\n");
        TRACE_ERROR("%d \n", size);
    }
    else
    {
        memcpy(&dir, data, sizeof(Uint8));
        size = sizeof(Uint8);
        memcpy(&mode, data + size, sizeof(MeterMode));
        size += sizeof(MeterMode);
        memcpy(&volume, data + size, sizeof(float));
        size += sizeof(float);
        memcpy(&limitVolume, data + size, sizeof(float));
        Meter_SendMeterEventOpen();
        ret = Meter_Start(dir, mode, volume, limitVolume);
    }
    DscpDevice_SendStatus(dscp, ret);
}
void OpticalMeter_StopMeter(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 ret = DSCP_OK;
    Meter_SendMeterEventOpen();
    if (FALSE == Meter_RequestStop())
    {
        ret = DSCP_ERROR;
    }
    DscpDevice_SendStatus(dscp, ret);
}

void OpticalMeter_SetOpticalADNotifyPeriod(DscpDevice* dscp, Byte* data, Uint16 len)
{
    float period;
    int size = 0;
    Uint16 ret = DSCP_OK;
    //设置数据正确性判断
    size = sizeof(period);
    if ((len > size))
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("Parame Len Error\n");
        TRACE_ERROR("%d \n", size);
    }
    else
    {
        //修改并保存
        memcpy(&period, data, sizeof(float));
        Meter_SetMeterADReportPeriod(period);
    }
    DscpDevice_SendStatus(dscp, ret);
}

void OpticalMeter_TurnOnLED(DscpDevice* dscp, Byte* data, Uint16 len)
{
    DscpDevice_SendStatus(dscp, DSCP_OK);
}

void OpticalMeter_TurnOffLED(DscpDevice* dscp, Byte* data, Uint16 len)
{
    DscpDevice_SendStatus(dscp, DSCP_OK);
}

void OpticalMeter_AutoCloseValve(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Bool isAutoCloseValve;
    int size = 0;
    Uint16 ret = DSCP_OK;
    //设置数据正确性判断
    size = sizeof(Bool);
    if ((len > size))
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("Parame Len Error\n");
        TRACE_ERROR("%d \n", size);
    }
    else
    {
        //修改并保存
        memcpy(&isAutoCloseValve, data, sizeof(Bool));
        Meter_AutoCloseValve(isAutoCloseValve);
    }
    DscpDevice_SendStatus(dscp, ret);
}

void OpticalMeter_SetMeterSpeed(DscpDevice* dscp, Byte* data, Uint16 len)
{
    float speed;
    int size = 0;
    Uint16 ret = DSCP_OK;
    //设置数据正确性判断
    size = sizeof(float);
    if ((len > size))
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("Parame Len Error\n");
        TRACE_ERROR("%d \n", size);
    }
    else
    {
        //修改并保存
        memcpy(&speed, data, sizeof(float));
        Meter_SetMeterSpeed(speed);
    }
    DscpDevice_SendStatus(dscp, ret);
}

void OpticalMeter_GetMeterSpeed(DscpDevice* dscp, Byte* data, Uint16 len)
{
    float speed = Meter_GetMeterSpeed();

    TRACE_INFO("\n Meter speed:");
    System_PrintfFloat(TRACE_LEVEL_INFO, speed, 6);
    TRACE_INFO(" ml/s");
    DscpDevice_SendResp(dscp, &speed, sizeof(float));
}

void OpticalMeter_SetMeterFinishValveMap(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint32 map =  0;
    Uint16 ret = DSCP_OK;

    int size = 0;
    //设置数据正确性判断
    size = sizeof(Uint32);
    if ((len > size))
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("Parame Len Error\n");
        TRACE_ERROR("%d \n", size);
    }
    else
    {
        memcpy(&map, data, sizeof(Uint32));
        if (FALSE == Meter_SetMeterFinishValveMap(map))
        {
            ret = DSCP_ERROR;
        }
    }
    DscpDevice_SendStatus(dscp, ret);
}

void OpticalMeter_GetSingleOpticalAD(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint8 num =  0;
    Uint32 value =  0;
    memcpy(&num, data, sizeof(Uint8));

    value = Meter_GetSingleOpticalAD(num);

    TRACE_INFO("\nSingleOpticalAD: %d", value);
    DscpDevice_SendResp(dscp, &value, sizeof(Uint32));
}

void OpticalMeter_SetRopinessMeterOverValue(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 value = 0;
    int size = 0;
    Uint16 ret = DSCP_OK;
    //设置数据正确性判断
    size = sizeof(Uint16);
    if ((len > size))
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("Parame Len Error\n");
        TRACE_ERROR("%d \n", size);
    }
    else
    {
        //修改并保存
        memcpy(&value, data, sizeof(Uint16));
        if(!Meter_SetRopinessOverValue(value))
        {
            ret = DSCP_ERROR_PARAM;
        }
    }
    DscpDevice_SendStatus(dscp, ret);
}

void OpticalMeter_GetRopinessMeterOverValue(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 value = Meter_GetRopinessOverValue();

    DscpDevice_SendResp(dscp, &value, sizeof(Uint16));
}

void OpticalMeter_SetAccurateModeOverValve(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 cnt = 0;
    int size = 0;
    Uint16 ret = DSCP_OK;
    //设置数据正确性判断
    size = sizeof(Uint16);
    if ((len > size))
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("Parame Len Error\n");
        TRACE_ERROR("%d \n", size);
    }
    else
    {
        //修改并保存
        memcpy(&cnt, data, sizeof(Uint16));
        if(FALSE == Meter_WriteAccurateModeOverValve(cnt))
        {
            ret = DSCP_ERROR_PARAM;
        }
    }
    DscpDevice_SendStatus(dscp, ret);
}
void OpticalMeter_GetAccurateModeOverValve(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 cnt = Meter_ReadAccurateModeOverValve();
    DscpDevice_SendResp(dscp, &cnt, sizeof(Uint16));
}
