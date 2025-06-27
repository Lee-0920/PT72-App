/**
 #include <OpticalAcquirer.h>
 * @file
 * @brief 光学采集接口实现
 * @details
 * @version 1.0.0
 * @author lemon.xiaoxun
 * @date 2016-5-27
 */

//#include <OpticalDriver/OpticalLed.h>
#include <string.h>
#include "Common/Utils.h"
#include "DNCP/App/DscpSysDefine.h"
#include "Tracer/Trace.h"
#include "Driver/System.h"
#include "DncpStack/DncpStack.h"
#include "LuipApi/OpticalAcquireInterface.h"
#include "OpticalControl/OpticalControl.h"
//#include "OpticalControl/LEDController.h"
#include "OpticalControl/StaticADControl.h"
#include "OpticalAcquirer.h"
#include <OpticalDriver/MCP4651Driver.h>
#include "CheckLeaking/CheckLeakingControl.h"


void OpticalAcquirer_TurnOnLed(DscpDevice* dscp, Byte* data, Uint16 len)
{
    unsigned short ret = DSCP_OK;
//    LEDController_TurnOnLed(0);
    DscpDevice_SendStatus(dscp, ret);
}
/**
 * @brief 设置信号AD上报周期。
 * @param dscp
 * @param data
 * @param len
 */
void OpticalAcquirer_SetSignalADNotifyPeriod(DscpDevice* dscp, Byte* data,
        Uint16 len)
{
    float period;
    int size = 0;
    Uint16 ret = DSCP_OK;

    size = sizeof(float);
    if ((len > size))
    {
        ret = DSCP_ERROR;
    }
    else
    {
        memcpy(&period, data, sizeof(float));
        OpticalControl_SetSignalADNotifyPeriod(period);
    }
    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 启动采集过程
 * @param dscp
 * @param data
 * @param len
 */
void OpticalAcquirer_StartAcquirer(DscpDevice* dscp, Byte* data, Uint16 len)
{
    unsigned short ret = DSCP_OK;
    float adacquiretime;
    Uint8 adcType;
    Uint16 size = sizeof(float) + sizeof(Uint8);
    TRACE_INFO("OpticalAcquirer_StartAcquirer");
    if (len > size)
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("Parame Len Error\n");
        TRACE_ERROR("%d \n", size);
    }
    else if(len < size)  //兼容旧版本命令
    {
        memcpy(&adacquiretime, data, sizeof(adacquiretime));
        TRACE_DEBUG("\n Time %d ms", (Uint32 )(adacquiretime * 1000));
        OpticalControl_SendEventOpen();
        if (FALSE == OpticalControl_StartAcquirer(1, adacquiretime))
        {
            ret = DSCP_ERROR;
        }
    }
    else
    {
        memcpy(&adacquiretime, data, sizeof(adacquiretime));
        memcpy(&adcType, data + sizeof(adacquiretime), sizeof(adcType));
        TRACE_DEBUG("\n Time %d ms", (Uint32 )(adacquiretime * 1000));
        OpticalControl_SendEventOpen();
        if (FALSE == OpticalControl_StartAcquirer(adcType, adacquiretime))
        {
            ret = DSCP_ERROR;
        }
    }
    DscpDevice_SendStatus(dscp, ret);
}
/**
 * @brief 停止采集过程
 * @param dscp
 * @param data
 * @param len
 */
void OpticalAcquirer_StopAcquirer(DscpDevice* dscp, Byte* data, Uint16 len)
{
    unsigned short ret = DSCP_OK;
    Bool result;
    result = OpticalControl_StopAcquirer();
    if (result == FALSE)
    {
        ret = DSCP_ERROR;
    }
    OpticalControl_SendEventOpen();
    DscpDevice_SendStatus(dscp, ret);
}

void OpticalAcquirer_StartLEDController(DscpDevice* dscp, Byte* data,
        Uint16 len)
{
    unsigned short ret = DSCP_OK;
//    if (FALSE == LEDController_Start(0))
//    {
//        ret = DSCP_ERROR;
//    }
    DscpDevice_SendStatus(dscp, ret);
}

void OpticalAcquirer_StopLEDController(DscpDevice* dscp, Byte* data, Uint16 len)
{
    unsigned short ret = DSCP_OK;
//    LEDController_Stop(0);
    DscpDevice_SendStatus(dscp, ret);
}

void OpticalAcquirer_GetLEDControllerTarget(DscpDevice* dscp, Byte* data,
        Uint16 len)
{
    Uint32 target;

//    target = LEDController_GetTarget(0);
    TRACE_DEBUG("\n LED controller target: %d", target);
    DscpDevice_SendResp(dscp, &target, sizeof(Uint32));
}

void OpticalAcquirer_SetLEDControllerTarget(DscpDevice* dscp, Byte* data,
        Uint16 len)
{
    Uint16 ret = DSCP_OK;
    Uint32 target;
    Uint16 size = 0;

    size = sizeof(Uint32);
    if ((len > size))
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("Parame Len Error\n");
        TRACE_ERROR("%d \n", size);
    }
    else
    {
        memcpy(&target, data, sizeof(Uint32));
//        LEDController_SetTarget(0, target);
    }
    DscpDevice_SendStatus(dscp, ret);
}

void OpticalAcquirer_GetLEDControllerParam(DscpDevice* dscp, Byte* data,
        Uint16 len)
{
//    LEDControllerParam ledControllerParam;
//
//    ledControllerParam = LEDController_GetParam(0);
//    TRACE_DEBUG("\n p =");
//    System_PrintfFloat(TRACE_LEVEL_DEBUG, ledControllerParam.proportion, 5);
//    TRACE_DEBUG("\n i =");
//    System_PrintfFloat(TRACE_LEVEL_DEBUG, ledControllerParam.integration, 5);
//    TRACE_DEBUG("\n d =");
//    System_PrintfFloat(TRACE_LEVEL_DEBUG, ledControllerParam.differential, 5);
//
//    DscpDevice_SendResp(dscp, &ledControllerParam, sizeof(LEDControllerParam));
}

void OpticalAcquirer_SetLEDControllerParam(DscpDevice* dscp, Byte* data,
        Uint16 len)
{
//    LEDControllerParam ledControllerParam;
//    Uint16 ret = DSCP_OK;
//    Uint16 size = 0;
//
//    size = sizeof(LEDControllerParam);
//    if ((len > size))
//    {
//        ret = DSCP_ERROR;
//        TRACE_ERROR("Parame Len Error\n");
//        TRACE_ERROR("%d \n", size);
//    }
//    else
//    {
//        memcpy(&ledControllerParam, data, sizeof(LEDControllerParam));
//        LEDController_SetParam(0, ledControllerParam);
//    }
//    DscpDevice_SendStatus(dscp, ret);
}

void OpticalAcquirer_StartLEDAdjust(DscpDevice* dscp, Byte* data,
        Uint16 len)
{
//    unsigned short ret = DSCP_OK;
//    Uint32 targetAD;
//    Uint32 tolerance;
//    Uint32 timeout;
//
//    int size = 0;
//    //设置数据正确性判断
//    size =  sizeof(Uint32) * 3;
//    if ((len > size))
//    {
//        ret = DSCP_ERROR;
//        TRACE_ERROR("Parame Len Error\n");
//        TRACE_ERROR("%d \n", size);
//    }
//    else
//    {
//        memcpy(&targetAD, data, sizeof(Uint32));
//        memcpy(&tolerance, data+sizeof(Uint32), sizeof(Uint32));
//        memcpy(&timeout, data+2*sizeof(Uint32), sizeof(Uint32));
//
//        LEDController_SendEventOpen(0);
//        if (FALSE == LEDController_AdjustToValue(0, targetAD, tolerance, timeout))
//        {
//            LEDController_SendEventClose(0);
//            ret = DSCP_ERROR;
//        }
//    }
//
//    DscpDevice_SendStatus(dscp, ret);
}

void OpticalAcquirer_StopLEDAdjust(DscpDevice* dscp, Byte* data, Uint16 len)
{
//    unsigned short ret = DSCP_OK;
//    LEDController_SendEventOpen(0);
//    LEDController_AdjustStop(0);
//    DscpDevice_SendStatus(dscp, ret);
}

void OpticalAcquirer_StartStaticADControl(DscpDevice* dscp, Byte* data, Uint16 len)
{
    unsigned short ret = DSCP_OK;
    Uint8 index;
    Uint32 targetAD;

    int size = 0;
    //设置数据正确性判断
    size =  sizeof(Uint8)  + sizeof(Uint32);
    if ((len > size))
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("Parame Len Error\n");
        TRACE_ERROR("%d \n", size);
    }
    else
    {
        memcpy(&index, data, sizeof(Uint8));
        memcpy(&targetAD, data+sizeof(Uint8), sizeof(Uint32));
        //TRACE_DEBUG("StaticAD Control  index = %d, target = %d", index, targetAD);
        if (TRUE == StaticADControl_Start(index, targetAD, 0))
        {
            StaticADControl_SendEventOpen();
        }
        else
        {
            ret = DSCP_ERROR;
        }
    }

    DscpDevice_SendStatus(dscp, ret);
}

void OpticalAcquirer_StopStaticADControl(DscpDevice* dscp, Byte* data, Uint16 len)
{
    unsigned short ret = DSCP_OK;
    StaticADControl_SendEventOpen();
    StaticADControl_Stop();
    DscpDevice_SendStatus(dscp, ret);
}

void OpticalAcquirer_GetStaticADControlParam(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint8 index;
    Uint16 value = DSCP_ERROR;

    int size = 0;
    //设置数据正确性判断
    size =  sizeof(Uint8);
    if ((len > size))
    {
        TRACE_ERROR("param error len : %d > %d\n", len, size);
    }
    else
    {
        memcpy(&index, data, sizeof(Uint8));

        if(index < AD_CONTROLLER_NUM)
        {
            value = StaticADControl_GetRealValue(index);
        }
        else
        {
            TRACE_ERROR("param index error \n");
        }
    }

    DscpDevice_SendResp(dscp, &value, sizeof(Uint16));
}

void OpticalAcquirer_SetStaticADControlParam(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 ret = DSCP_OK;
    Uint16 size = 0;
    Uint8 index;
    Uint16 value;

    size = sizeof(Uint8) + sizeof(Uint16);
    if ((len > size))
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("param error len : %d > %d\n", len, size);
    }
    else
    {
        memcpy(&index, data, sizeof(Uint8));
        memcpy(&value, data+sizeof(Uint8), sizeof(Uint16));

        if(index < AD_CONTROLLER_NUM)
        {
        	if(AD5175_MAX_VALUM < value)
			{
				value = AD5175_MAX_VALUM;
			}
            StaticADControl_SetDefaultValue(index, value, 0);
            if(StaticADControl_SetRealValue(index, value) == FALSE)
            {
                ret = DSCP_ERROR;
            }
        }
        else
        {
            TRACE_ERROR("param index error \n");
            ret = DSCP_ERROR;
        }
    }
    DscpDevice_SendStatus(dscp, ret);
}

void OpticalAcquire_IsADControlValid(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 ret = DSCP_ERROR;
    if(TRUE == StaticADControl_IsValid())
    {
        ret = DSCP_OK;
    }
    DscpDevice_SendStatus(dscp, ret);
}

void OpticalAcquirer_GetLEDDefaultValue(DscpDevice* dscp, Byte* data, Uint16 len)
{
//    float value;
//
//    value = OpticalLed_GetDefaultValue();
//    TRACE_DEBUG("\n LED DAC default value: ");
//    System_PrintfFloat(TRACE_LEVEL_DEBUG, value, 3);
//    DscpDevice_SendResp(dscp, &value, sizeof(float));
}

void OpticalAcquirer_SetLEDDefaultValue(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 ret = DSCP_OK;
//    float value;
//    Uint16 size = 0;
//
//    size = sizeof(float);
//    if ((len > size))
//    {
//        ret = DSCP_ERROR;
//        TRACE_ERROR("Parame Len Error %d\n", size);
//    }
//    else
//    {
//        memcpy(&value, data, sizeof(float));
//
//        if(FALSE == OpticalLed_SetDefaultValue(value))
//        {
//            ret = DSCP_ERROR_PARAM;
//        }
//    }
    DscpDevice_SendStatus(dscp, ret);
}

void OpticalAcquirer_CtrlXonen(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 ret = DSCP_OK;
    Uint8 ctrlCode = 0;
    Uint16 size = 0;

    size = sizeof(Uint8);
    if ((len > size))
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("Parame Len Error %d\n", size);
    }
    else
    {
        memcpy(&ctrlCode, data, sizeof(Uint8));
        switch(ctrlCode)
        {
            case 0:
                OpticalControl_TurnOnXonen(FALSE);
                break;
            case 1:
                OpticalControl_TurnOnXonen(TRUE);
                break;
            case 2:
                OpticalControl_GlitterXonen(TRUE);
                break;
            default:
                ret = DSCP_ERROR;
                break;
        }
    }
    DscpDevice_SendStatus(dscp, ret);
}

void OpticalAcquirer_GetADCWorkMode(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint8 mode = 0;

    mode = (Uint8)OpticalControl_GetWorkMode();
    TRACE_DEBUG("\n ADC work mode : %d", mode);

    DscpDevice_SendResp(dscp, &mode, sizeof(Uint8));
}

void OpticalAcquirer_SetADCWorkMode(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 ret = DSCP_OK;
    Uint8 mode = 0;
    Uint16 size = 0;

//    size = sizeof(Uint8);
//    if ((len > size))
//    {
//        ret = DSCP_ERROR;
//        TRACE_ERROR("Parame Len Error %d\n", size);
//    }
//    else
//    {
//        memcpy(&mode, data, sizeof(Uint8));
//
//        if(mode >= ONLY_AD7791 && mode <= BOTH_AD7791_ADS1146)
//        {
//            OpticalControl_ResetWorkMode((OpticalWorkMode)mode);
//        }
//        else
//        {
//            ret = DSCP_ERROR_PARAM;
//        }
//    }
    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 设置漏液检测信号上报周期
 * @param Period Float 单位秒。
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_OK  操作成功；
 *  - @ref DSCP_ERROR 操作失败；
 */
void OpticalAcquirer_SetCheckLeakingReportPeriod(DscpDevice* dscp, Byte* data,Uint16 len)
{
    float period;
    int size = 0;
    Uint16 ret = DSCP_OK;

    size = sizeof(float);
    if ((len > size))
    {
        ret = DSCP_ERROR;
    }
    else
    {
        memcpy(&period, data, sizeof(float));
        CheckLeaking_SetCheckLeakingReportPeriod(period);
    }
    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 设置静态AD调节反算放大倍数
 * @param Period Float 单位秒。
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_OK  操作成功；
 *  - @ref DSCP_ERROR 操作失败；
 */
void OpticalAcquirer_SetStaticMultiple(DscpDevice* dscp, Byte* data, Uint16 len)
{
    float multiple;
    int size = 0;
    Uint16 ret = DSCP_OK;

    size = sizeof(float);
    if ((len > size))
    {
        ret = DSCP_ERROR;
    }
    else
    {
        memcpy(&multiple, data, sizeof(float));
        StaticADControl_SetMultiple(multiple);
    }
    DscpDevice_SendStatus(dscp, ret);
}

//void OpticalAcquirer_Gain_Set(DscpDevice* dscp, Byte* data, Uint16 len)
//{
//    int Gain;
//    int size = 0;
//    Uint16 ret = DSCP_OK;
//
//    size = sizeof(int);
//    if ((len > size))
//    {
//        ret = DSCP_ERROR;
//    }
//    else
//    {
//        memcpy(&Gain, data, sizeof(int));
//        ret = StaticADControl_Gain_Set(Gain);
//    }
//    DscpDevice_SendStatus(dscp, ret);
//}
