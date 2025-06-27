/*
 * PeristalticPump.c
 *
 *  Created on: 2016年6月4日
 *      Author: Administrator
 */
#include "Tracer/Trace.h"
#include "Dncp/App/DscpSysDefine.h"
#include <string.h>
#include "PeristalticPump/PumpManager.h"
#include "System.h"
#include "PeristalticPump.h"

/**
 * @brief 查询系统支持的总泵数目。
 * @param dscp
 * @param data
 * @param len
 */
void PeristalticPump_GetTotalPumps(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 totalPumps = PumpManager_GetTotalPumps();
    TRACE_MARK("totalPumps: %d", totalPumps);
    DscpDevice_SendResp(dscp, &totalPumps, sizeof(Uint16));
}

/**
 * @brief 查询指定泵的运动参数。
 * @param dscp
 * @param data
 * @param len
 */
void PeristalticPump_GetMotionParam(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint8 index;
    PumpParam param;
    memcpy(&index, data, sizeof(Uint8));
    param = PumpManager_GetMotionParam(index);

    TRACE_MARK("Pump %d maxSpeed", index);
    System_PrintfFloat(TRACE_LEVEL_MARK, param.maxSpeed, 4);
    TRACE_MARK(" step/s,acc:");
    System_PrintfFloat(TRACE_LEVEL_MARK, param.acceleration, 4);
    TRACE_MARK(" step/(s^2)\n");
    DscpDevice_SendResp(dscp, &param, sizeof(PumpParam));
}

/**
 * @brief 设置指定泵的运动参数。
 * @param dscp
 * @param data
 * @param len
 */
void PeristalticPump_SetMotionParam(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 ret = DSCP_OK;
    PumpParam param;
    Uint16 size = 0;
    Uint8 index;

    //设置数据正确性判断
    size = sizeof(PumpParam) + sizeof(Uint8);
    if ((len > size))
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("Parame Len Error\n");
        TRACE_ERROR("%d \n", size);
    }
    else
    {
        memcpy(&index, data, sizeof(Uint8));
        memcpy(&param, data + sizeof(Uint8), sizeof(PumpParam));
        ret = PumpManager_SetMotionParam(index, param.acceleration,
                param.maxSpeed, WriteFLASH);
    }
    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 查询指定泵的校准系数。
 * @param dscp
 * @param data
 * @param len
 */
void PeristalticPump_GetFactor(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint8 index;
    float factor;
    memcpy(&index, data, sizeof(Uint8));
    factor = PumpManager_GetFactor(index);

    TRACE_MARK("Pump %d Factor", index);
    System_PrintfFloat(TRACE_LEVEL_MARK, factor, 8);
    TRACE_MARK(" ml/step");

    DscpDevice_SendResp(dscp, &factor, sizeof(float));
}

/**
 * @brief 设置指定泵的校准系数。
 * @param dscp
 * @param data
 * @param len
 */
void PeristalticPump_SetFactor(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 ret = DSCP_OK;
    float factor;
    Uint16 size = 0;
    Uint8 index;

    //设置数据正确性判断
    size = sizeof(float) + sizeof(Uint8);
    if ((len > size))
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("Parame Len Error\n");
        TRACE_ERROR("%d \n", size);
    }
    else
    {
        memcpy(&index, data, sizeof(Uint8));
        memcpy(&factor, data + sizeof(Uint8), sizeof(float));
        ret = PumpManager_SetFactor(index, factor);
    }
    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 查询指定泵的工作状态。
 * @param dscp
 * @param data
 * @param len
 */
void PeristalticPump_GetStatus(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint8 index;

    memcpy(&index, data, sizeof(Uint8));

    if (PUMP_IDLE == PumpManager_GetStatus(index))
    {
        TRACE_MARK("\n Pump: %d idle", index);
        DscpDevice_SendStatus(dscp, DSCP_IDLE);
    }
    else
    {
        TRACE_MARK("\n Pump: %d busy", index);
        DscpDevice_SendStatus(dscp, DSCP_BUSY);
    }
}

/**
 * @brief 查询泵出的体积。
 * @param dscp
 * @param data
 * @param len
 */
void PeristalticPump_GetVolume(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint8 index;
    float volume;
    memcpy(&index, data, sizeof(Uint8));
    volume = PumpManager_GetVolume(index);
    TRACE_MARK("\n Pump: %d, Volume", index);
    System_PrintfFloat(TRACE_LEVEL_MARK, volume, 4);
    TRACE_MARK(" ml");
    DscpDevice_SendResp(dscp, &volume, sizeof(float));
}

/**
 * @brief 查询泵出的气体体积。
 * @param dscp
 * @param data
 * @param len
 */
void PeristalticPump_GetAirVolume(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint8 index;
    float volume;
    memcpy(&index, data, sizeof(Uint8));
    volume = PumpManager_GetAirVolume(index);
    TRACE_MARK("\n Pump: %d, AirVolume", index);
    System_PrintfFloat(TRACE_LEVEL_MARK, volume, 4);
    TRACE_MARK(" ml");
    DscpDevice_SendResp(dscp, &volume, sizeof(float));
}

/**
 * @brief 启动泵。
 * @param dscp
 * @param data
 * @param len
 */
void PeristalticPump_Start(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 ret = DSCP_OK;
    Direction dir;
    float volume;
    float speed;
    Uint16 size = 0;
    Uint8 index;
    ParamFLashOp FLashOp = ReadFLASH;

    //设置数据正确性判断
    size = sizeof(Uint8) + sizeof(Direction) + sizeof(float) + sizeof(float);
    if ((len > size))
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("Parame Len Error\n");
        TRACE_ERROR("%d \n", size);
    }
    else
    {
        memcpy(&index, data, sizeof(Uint8));
        size = sizeof(Uint8);
        memcpy(&dir, data + size, sizeof(Direction));
        size += sizeof(Direction);
        memcpy(&volume, data + size, sizeof(float));
        size += sizeof(float);
        memcpy(&speed, data + size, sizeof(float));

        PumpManager_SendEventOpen(index);
        if (speed > 0)
        {
            PumpParam param;
            param = PumpManager_GetMotionParam(index);
            param.maxSpeed = speed;
            PumpManager_SetMotionParam(index, param.acceleration,
                    param.maxSpeed, NoWriteFLASH);
            FLashOp = NoReadFLASH;
        }
        ret = PumpManager_Start(index, dir, volume, FLashOp);
    }
    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 停止泵。
 * @param dscp
 * @param data
 * @param len
 */
void PeristalticPump_Stop(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 ret = DSCP_OK;
    Uint8 index;

    memcpy(&index, data, sizeof(Uint8));
    PumpManager_SendEventOpen(index);
    ret = PumpManager_Stop(index);
    DscpDevice_SendStatus(dscp, ret);
}

