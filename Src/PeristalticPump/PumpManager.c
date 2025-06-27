/*
 * PumpManager.c
 *
 *  Created on: 2016年5月31日
 *      Author: Administrator
 */
#include "DNCP/App/DscpSysDefine.h"
#include "System.h"
#include "PumpManager.h"
#include "PumpEventScheduler.h"

Pump g_pumps[PUMPMANAGER_TOTAL_PUMP];

void PumpManager_Init(void)
{
    Uint8 i;

    memset(g_pumps, 0, sizeof(Pump) * PUMPMANAGER_TOTAL_PUMP);

    PumpMap_Init(g_pumps); //配置泵用到的引脚

    PumpTimer_Init(); //配置用于产生方波的定时器

    //先设定泵号再初始化泵
    for (i = 0; i < PUMPMANAGER_TOTAL_PUMP; i++)
    {
        Pump_SetNumber(&g_pumps[i], i);
        Pump_Init(&g_pumps[i]);
        Pump_SetSubdivision(&g_pumps[i], 16);
    }

    PumpEventScheduler_Init();
}

Pump* PumpManager_GetPump(Uint8 index)
{
	if(index < PUMPMANAGER_TOTAL_PUMP)
	{
		return &g_pumps[index];
	}
	else
	{
		TRACE_ERROR("\n index error 0-3");
		return 0;
	}
}

void PumpManager_Restore(void)
{
    for(Uint8 i = 0; i < PUMPMANAGER_TOTAL_PUMP; i++)
    {
        Pump_SendEventClose(&g_pumps[i]);
        PumpManager_Stop(i);
    }
}
/**
 * @brief 查询泵出的体积。
 * @note 启动泵到停止泵的过程中，泵的转动体积（步数）。
 * @param index 要查询的泵索引，0号泵为光学定量泵。
 * @return 泵出的体积，单位为 ml
 */
float PumpManager_GetVolume(Uint8 index)
{
    if (index < PUMPMANAGER_TOTAL_PUMP)
    {
        return (Pump_GetVolume(&g_pumps[index]));
    }
    else
    {
        TRACE_ERROR("\n No. %d pump.", index);
    }
    return 0;
}

/**
 * @brief 查询泵出的气体体积。
 * @note 启动泵到停止泵的过程中，泵的转动体积（步数）。
 * @param index 要查询的泵索引，0号泵为光学定量泵。
 * @return 泵出的体积，单位为 ml
 */
float PumpManager_GetAirVolume(Uint8 index)
{
    if (index < PUMPMANAGER_TOTAL_PUMP)
    {
        return (Pump_GetAirVolume(&g_pumps[index]));
    }
    else
    {
        TRACE_ERROR("\n No. %d pump.", index);
    }
    return 0;
}

Bool PumpManager_ImmediatelyStop(Uint8 index)
{
	return (Pump_ImmediatelyStop(&g_pumps[index]));
}


/**
 * @brief 查询指定泵的运动参数。
 * @note 系统将根据设定的运动参数进行运动控制和规划。
 * @param index 要查询的泵索引，0号泵为光学定量泵
 * @return 运动参数(参数错误为0)，数据格式为：
 *                  acceleration Float32，加速度，单位为 ml/平方秒。
 *                  speed Float32，最大速度，单位为 ml/秒。
 */
PumpParam PumpManager_GetMotionParam(Uint8 index)
{
    PumpParam param;

    if (index < PUMPMANAGER_TOTAL_PUMP)
    {
        param = Pump_GetMotionParam(&g_pumps[index]);

        TRACE_INFO("acc:");
        System_PrintfFloat(TRACE_LEVEL_INFO, param.acceleration, 4);
        TRACE_INFO(" step/(s^2),maxSpeed:");
        System_PrintfFloat(TRACE_LEVEL_INFO, param.maxSpeed, 4);
        TRACE_INFO(" step/s\n");

        param.acceleration = param.acceleration * g_pumps[index].factor;
        param.maxSpeed = param.maxSpeed * g_pumps[index].factor;
        return param;
    }
    else
    {
        TRACE_ERROR("\n No. %d pump.", index);
    }
    return param;
}

/**
 * @brief 查询指定泵的校准系数。
 * @param index 要查询的泵索引，0号泵为光学定量泵。
 * @return 校准系数， 每步泵出的体积，单位为 ml/步。 参数错误为0
 */
float PumpManager_GetFactor(Uint8 index)
{
    if (index < PUMPMANAGER_TOTAL_PUMP)
    {
        return (Pump_GetTempFactor(&g_pumps[index]));
    }
    else
    {
        TRACE_ERROR("\n No. %d pump.", index);
    }
    return 0;
}

/**
 * @brief 查询指定泵的实时校准系数。
 * @param index 要查询的泵索引，0号泵为光学定量泵。
 * @return 校准系数， 每步泵出的体积，单位为 ml/步。 参数错误为0
 */
float PumpManager_GetTempFactor(Uint8 index)
{
    if (index < PUMPMANAGER_TOTAL_PUMP)
    {
        return (Pump_GetTempFactor(&g_pumps[index]));
    }
    else
    {
        TRACE_ERROR("\n No. %d pump.", index);
    }
    return 0;
}



/**
 * @brief 查询系统支持的总泵数目。
 * @return 总泵数目， Uint16。
 */
Uint16 PumpManager_GetTotalPumps(void)
{
    return PUMPMANAGER_TOTAL_PUMP;
}

/**
 * @brief 设置指定泵的运动参数。
 * @note 系统将根据设定的运动参数进行运动控制和规划。运动参数将永久保存。
 * @param index 要设置的泵索引，0号泵为光学定量泵
 * @param acceleration 加速度，单位为 ml/平方秒
 * @param maxSpeed 最大速度，单位为 ml/秒
 * @return 状态回应 TRUE 操作成功; FALSE 操作失败(参数错误)
 */
Uint16 PumpManager_SetMotionParam(Uint8 index, float acceleration,
        float maxSpeed, ParamFLashOp flashOp)
{
    PumpParam param;

    if (index < PUMPMANAGER_TOTAL_PUMP && 0 != acceleration
            && (maxSpeed >= (PUMP_INIT_SPEED * g_pumps[index].factor))
            && (maxSpeed <= (PUMP_MAX_SPEED * g_pumps[index].factor))
            && 0 != g_pumps[index].factor)
    {

        TRACE_MARK("\n Pump %d set param maxSpeed:", index);
        System_PrintfFloat(TRACE_LEVEL_CODE, maxSpeed, 4);
        TRACE_CODE(" ml/s,acc:");
        System_PrintfFloat(TRACE_LEVEL_CODE, acceleration, 4);
        TRACE_CODE(" ml/(s^2)\n");

        param.acceleration = acceleration / g_pumps[index].factor;
        param.maxSpeed = maxSpeed / g_pumps[index].factor;
        if (TRUE == Pump_SetMotionParam(&g_pumps[index], param, flashOp))
        {
            return DSCP_OK;
        }
        else
        {
            return DSCP_ERROR;
        }
    }
    else
    {
        TRACE_ERROR(
                "\n The pump %d motion parameter setting failed because of the parameter error.", index);
        return ((Uint16) DSCP_ERROR_PARAM);
    }

}

/**
 * @brief 设置指定泵的校准系数。
 * @note 因为蠕动泵的生产工艺及工作时长，每个泵转一步对应体积都不同，出厂或使用时需要校准。 该参数将永久保存。
 * @param index 要设置的泵索引，0号泵为光学定量泵。
 * @param factor 要设置的校准系数， 每步泵出的体积，单位为 ml/步。
 * @return 状态回应 TRUE 操作成功; FALSE 操作失败(参数错误)
 */
Uint16 PumpManager_SetFactor(Uint8 index, float factor)
{
    if (index < PUMPMANAGER_TOTAL_PUMP && 0 != factor)
    {
        if (PUMP_IDLE == g_pumps[index].status)
        {
            TRACE_MARK("\n Pump %d factor:", index);
            System_PrintfFloat(TRACE_LEVEL_MARK, factor, 8);
            TRACE_MARK(" ml/step");

            Pump_SetFactor(&g_pumps[index], factor);
            return DSCP_OK;
        }
        else
        {
            TRACE_ERROR(
                    "\n The pump %d is running and can not change the calibration factor.", index);
            return DSCP_ERROR;
        }
    }
    else
    {
        TRACE_ERROR(
                "\n The pump %d calibration factor setting failed because of the parameter error.", index);
        return ((Uint16) DSCP_ERROR_PARAM);
    }
}

/**
 * @brief 设置指定泵的临时校准系数（不写入Flash）。
 * @note 因为蠕动泵会磨损，该参数可能会变化
 * @param index 要设置的泵索引，0号泵为光学定量泵。
 * @param factor 要设置的校准系数， 每步泵出的体积，单位为 ml/步。
 * @return 状态回应 TRUE 操作成功; FALSE 操作失败(参数错误)
 */
Uint16 PumpManager_SetTempFactor(Uint8 index, float factor)
{
    if (index < PUMPMANAGER_TOTAL_PUMP && 0 != factor)
    {
        Pump_SetTempFactor(&g_pumps[index], factor);
        return DSCP_OK;
    }
    else
    {
        return ((Uint16) DSCP_ERROR_PARAM);
    }
}

/**
 * @brief 启动泵。
 * @note 启动后，不管成功与否，操作结果都将以事件的形式上传给上位机。
 * @param index 要操作的泵索引，0号泵为光学定量泵
 * @param dir 泵转动方向，0为正向转动（抽取），1为反向转动（排空）。
 * @param volume Float32，泵取/排空体积，单位为 ml。
 * @return 状态回应 TRUE 操作成功; FALSE 操作失败如泵正在工作，无法启动泵，需要先停止
 */
Uint16 PumpManager_Start(Uint8 index, Direction dir, float volume, ParamFLashOp flashOp)
{
    if (PUMP_IDLE == g_pumps[index].status)
    {
        if (index < PUMPMANAGER_TOTAL_PUMP && volume > 0 && dir < MAX_DIRECTION)
        {
            if  (TRUE == Pump_Start(&g_pumps[index], dir, volume, flashOp))
            {
                return DSCP_OK;
            }
            else
            {
                TRACE_ERROR("\n Because no idle timer , the pump %d starts to fail.", index);
                return DSCP_ERROR;
            }
        }
        else
        {
            TRACE_ERROR(
                    "\n Because of the parameter error, the pump %d starts to fail. ", index);
            return ((Uint16) DSCP_ERROR_PARAM);
        }
    }
    else
    {
        TRACE_ERROR("\n Because the pump is running, the pump %d starts to fail.", index);
        return DSCP_ERROR;
    }
}

/**
 * @brief 停止泵。
 * @param index 要操作的泵索引，0号泵为光学定量泵。
 * @return 状态回应 TRUE 操作成功; FALSE 操作失败如泵已停止
 */
Uint16 PumpManager_Stop(Uint8 index)
{
    if (index < PUMPMANAGER_TOTAL_PUMP)
    {
        if(PUMP_BUSY == g_pumps[index].status && TRUE == Pump_RequestStop(&g_pumps[index]))
        {
            return DSCP_OK;
        }
        else
        {
            TRACE_ERROR("\n Because the pump %d does not run, stop failure.", index);
            return DSCP_ERROR;
        }
    }
    else
    {
        TRACE_ERROR(
                "\n Because of the parameter error, the pump %d stop to fail. ", index);
        return ((Uint16) DSCP_ERROR_PARAM);
    }
}

void PumpManager_ChangeVolume(Uint8 index, float volume)
{
    if (index < PUMPMANAGER_TOTAL_PUMP && volume > 0)
    {
        if (PUMP_BUSY == g_pumps[index].status)
        {
            Pump_ChangeVolume(&g_pumps[index], volume);
        }
        else
        {
            TRACE_ERROR("\n Because the pump %d does not run, change volume failure.", index);
        }
    }
    else
    {
        TRACE_ERROR(
                "\n Because of the parameter error, the pump %d change volume to fail. ", index);
    }
}

Bool PumpManager_SendEventOpen(Uint8 index)
{
    if (index < PUMPMANAGER_TOTAL_PUMP)
    {
        Pump_SendEventOpen(&g_pumps[index]);
        return TRUE;
    }
    else
    {
        TRACE_ERROR("\n No. %d pump.", index);
    }
    return FALSE;
}

Uint32 PumpManager_GetAlreadyStep(Uint8 index)
{
    if (index < PUMPMANAGER_TOTAL_PUMP)
    {
        return Pump_GetAlreadyStep(&g_pumps[index]);
    }
    else
    {
        TRACE_ERROR("\n No. %d pump.", index);
    }
    return 0;
}

Uint8 PumpManager_GetSubdivision(Uint8 index)
{
    if (index < PUMPMANAGER_TOTAL_PUMP)
    {
        return Pump_GetSubdivision(&g_pumps[index]);
    }
    else
    {
        TRACE_ERROR("\n No. %d pump.", index);
    }
    return 0;
}
/**
 * @brief  查询指定泵的工作状态。
 * @param index 要设置的泵索引，0号泵为光学定量泵。
 * @return  状态回应  空闲，忙碌，需要停止后才能做下一个动作
 */
PumpStatus PumpManager_GetStatus(Uint8 index)
{
    if (index < PUMPMANAGER_TOTAL_PUMP)
    {
        return (Pump_GetStatus(&g_pumps[index]));
    }
    else
    {
        TRACE_ERROR("\n No. %d pump.", index);
    }
    return PUMP_BUSY;
}

/**
 * @brief  泵复位。
 * @param index 要设置的泵索引，目前仅支持0号泵。
 * @return  状态回应  空闲，忙碌，需要停止后才能做下一个动作
 */
void PumpManager_Reset(void)
{
    for(Uint8 i = 0; i < PUMPMANAGER_TOTAL_PUMP; i++)
    {
        Pump_SendEventClose(&g_pumps[i]);
        Pump_Reset(&g_pumps[i]);
    }
}
