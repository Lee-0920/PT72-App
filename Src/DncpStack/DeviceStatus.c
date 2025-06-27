/**
 * @file
 * @brief 设备状态接口实现
 * @details
 * @version 1.0.0
 * @author nick.shushaohua
 * @date Dec 29, 2014
 */
#include "DeviceStatus.h"
#include <string.h>
#include "Common/Utils.h"
#include "Console/Console.h"
#include "DNCP/App/DscpSysDefine.h"
#include "Tracer/Trace.h"
#include "SolenoidValve/ValveManager.h"
#include "PeristalticPump/PumpManager.h"
#include "OpticalMeter/Meter.h"
//#include "OpticalControl/LEDController.h"
#include "OpticalControl/OpticalControl.h"
#include "TemperatureControl/ThermostatManager.h"
#include "TemperatureControl/ThermostatDeviceManager.h"
#include "Driver/TempDriver/BoxFanDriver.h"
#include "DeviceUpdate/UpdateHandle.h"
#include "DeviceIndicatorLED.h"
#include "DncpStack/DncpStack.h"

/**
 * @brief 闪烁设备指示灯。
 * @details 本命令只是抽象定义了设备的指示操作，而未指定是哪一盏灯，
 *  具体的指示灯（一盏或多盏）由设备实现程序决定。
 *  <p>闪烁方式由参数指定。通过调节参数，可设置LED灯为常亮或常灭：
 *  - onTime 为 0 表示灭灯
 *  - offTime 为 0 表示亮灯
 *  - onTime 和 offTime 都为0 时，不动作
 *  <p> 当持续时间结束之后，灯的状态将返回系统状态，受系统控制。
 * @param duration Uint32，持续时间，单位为毫秒。0 表示不起作用，-1 表示一直持续。
 * @param onTime Uint16，亮灯的时间，单位为毫秒。
 * @param offTime Uint16，灭灯的时间，单位为毫秒。
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_OK  操作成功；
 *  - @ref DSCP_ERROR 操作失败；
 *  - @ref DSCP_NOT_SUPPORTED 操作不被支持；
 *
 */
void DeviceStatus_BlinkDevice(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 ret = DSCP_OK;
    int size = 0;
    Uint16 onTime;
    Uint16 offTime;
    Uint32 duration;

    size = sizeof(Uint16) * 2 + sizeof(Uint32);
    if (len > size)
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("Parame Len Error\n");
        TRACE_ERROR("%d \n", size);
    }
    else
    {
        memcpy(&duration, data, sizeof(Uint32));
        size = sizeof(Uint32);
        memcpy(&onTime, data + size, sizeof(Uint16));
        size += sizeof(Uint16);
        memcpy(&offTime, data + size, sizeof(Uint16));
        DeviceIndicatorLED_SetBlink(duration, onTime, offTime);
    }
    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 查询程序当前的运行模式。
 * @details 本操作通常用于升级管理程序，以确保设备处于期望的运行模式。
 * @return 运行模式，Uint8。请参考 @ref DeviceRunMode 。
 * @note App 模式和 Updater 模式都支持本命令。见： @ref DSCP_CMD_DUI_GET_RUN_MODE ，
 *  注意这两条命令的码值是一致，只是名称不同而已。
 */
void DeviceStatus_GetRunMode(DscpDevice* dscp, Byte* data, Uint16 len)
{
    DeviceRunMode mode = UpdateHandle_GetRunMode();
    TRACE_MARK("\n mode:%d", mode);
    DscpDevice_SendResp(dscp, &mode, sizeof(DeviceRunMode));
}

/**
 * @brief 查询设备的电源供应类型。
 * @details 这里的电源供给类型是指设备实际的电源类型。
 * @return 电源供应类型，Uint8。请参考 @ref DevicePowerSupplyType 。
 */
void DeviceStatus_GetPowerSupplyType(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint8 powerSupply = DEVICE_POWER_TYPE_ONGOING_AC;
    DscpDevice_SendResp(dscp, &powerSupply, sizeof(powerSupply));
}

/**
 * @brief 设备初始化。
 * @details 恢复业务功能到初始状态。
 *
 */
void DeviceStatus_Initialize(DscpDevice* dscp, Byte* data, Uint16 len)
{
    short ret = DSCP_OK;
    //以下操作可能会因为没有启动而控制台出现错误提醒。
    Meter_Restore();//关闭定量，如果正在定量则最终会关闭阀，LED和AD，定量泵，
    PumpManager_Restore();//依次关闭所有的泵
    if (METER_STATUS_IDLE == Meter_GetMeterStatus())
    {
        ValveManager_SetValvesMap(0);//这只保证定量运行的时候不会关闭所有的阀。
    }
	OpticalControl_Restore();
//    LEDController_Restore();
    ThermostatManager_RestoreInit();
    ThermostatDeviceManager_RestoreInit();
    DscpDevice_SendStatus(dscp, ret);
}

void DeviceStatus_Reset(DscpDevice* dscp, Byte* data, Uint16 len)
{
    System_Delay(10);
    System_Reset();
}

/**
 * @brief 复位事件发生 向上位机报告
 * @param 无
 */
void DeviceStatus_ReportResetEvent(DeviceResetCause resetCause)
{
    DncpStack_SendEvent(DSCP_EVENT_DSI_RESET, &resetCause,
            sizeof(DeviceResetCause));
    TRACE_INFO(" \n DEVICE_RESET");
}

/**
 * @brief 设置漏液上报周期。
 * @details 系统将根据设定的周期，定时向上发出漏液上报事件。
 * @param period Float32，漏液上报周期，单位为秒。0表示不需要上报，默认为5。
 * @see DSCP_EVENT_DCI_CHECK_LEAKING_NOTICE
 * @note 所设置的上报周期将在下一次启动时丢失，默认为0，不上报。
 */
void DeviceStatus_SetCheckLeakingReportPeriod(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Float32 period;
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
    //修改并保存
    if (DSCP_OK == ret)
    {
        //修改并保存
        memcpy(&period, data, sizeof(period));
        CheckLeaking_SetCheckLeakingReportPeriod(period);
    }
    DscpDevice_SendStatus(dscp, ret);
}

