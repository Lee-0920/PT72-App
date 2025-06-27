
/**
 * @file
 * @brief 温度接口实现
 * @details
 * @version 1.0.0
 * @author lemon.xiaoxun
 * @date 2016-5-27
 */

#include <string.h>
#include <TempDriver/BoxFanDriver.h>
#include "TemperatureControl.h"
#include "Common/Utils.h"
#include "DNCP/App/DscpSysDefine.h"
#include "Tracer/Trace.h"
#include "Driver/System.h"
#include "TemperatureControl/TempCollecterManager.h"
#include "TemperatureControl/ThermostatManager.h"
#include "LuipApi/TemperatureControlInterface.h"
#include "FreeRTOS.h"

/**
 * @brief 查询温度传感器的校准系数。
 * @return 负输入分压，Float32。
 * @return 参考电压 ，Float32。
 * @return 校准电压 Float32。
 * @see DSCP_CMD_TCI_SET_CALIBRATE_FACTOR
 */
void TempControl_GetCalibrateFactor(DscpDevice* dscp, Byte* data, Uint16 len)
{
    TempCalibrateParam calibratefactor;
    calibratefactor = ThermostatManager_GetCalibrateFactor(0);
    // 发送回应
    DscpDevice_SendResp(dscp, &calibratefactor, sizeof(TempCalibrateParam));
}

/**
 * @brief 设置温度传感器的校准系数。
 * @details 因为个体温度传感器有差异，出厂或使用时需要校准。该参数将永久保存。
 *   @param negativeInput Float32，负输入分压 。
 *   @param vref Float32，参考电压。
 *   @param vcal Float32，校准电压。
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_OK  操作成功；
 *  - @ref DSCP_ERROR 操作失败；
 */
void TempControl_SetCalibrateFactor(DscpDevice* dscp, Byte* data, Uint16 len)
{
    TempCalibrateParam calibratefactor;
    int size = 0;
    Uint16 ret = DSCP_OK;
    //设置数据正确性判断
    size = sizeof(TempCalibrateParam);
    if ((len > size))
    {
        ret = DSCP_ERROR;
    }
    //修改并保存
    if (DSCP_OK == ret)
    {
        memcpy(&calibratefactor, data, sizeof(TempCalibrateParam));
        ThermostatManager_SetCalibrateFactor(0, calibratefactor);
    }
    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 查询当前温度。
 * @return 当前温度，包括控制室温度和环境温度，格式如下：
 *     - thermostatTemp Float32，恒温室温度，单位为摄氏度。
 *     - environmentTemp Float32，环境温度，单位为摄氏度。
 */
void TempControl_GetTemperature(DscpDevice* dscp, Byte* data, Uint16 len)
{
    OldTemperature temperature;
    temperature.thermostatTemp = ThermostatManager_GetCurrentTemp(0);
    temperature.environmentTemp = TempCollecterManager_GetEnvironmentTemp();
    //TRACE_INFO("\n thermostatTemp:%f environmentTemp:%f\n", temperature.thermostatTemp, temperature.environmentTemp);
    // 发送回应
    DscpDevice_SendResp(dscp, (void *) &temperature, sizeof(temperature));
}

/**
 * @brief 查询恒温控制参数。
 * @return 恒温控制参数，格式如下：
 *  - proportion Float32，PID的比例系数。
 *  - integration Float32，PID的积分系数。
 *  - differential Float32，PID的微分系数。
 * @see DSCP_CMD_TCI_SET_THERMOSTAT_PARAM
 */
void TempControl_GetThermostatParam(DscpDevice* dscp, Byte* data, Uint16 len)
{
    ThermostatParam thermostatparam;
    thermostatparam = ThermostatManager_GetPIDParam(0);
    // 发送回应
    DscpDevice_SendResp(dscp, (void *) &thermostatparam, sizeof(thermostatparam));
}

/**
 * @brief 设置恒温控制参数。
 * @details 该参数将永久保存。系统启动时同步到当前恒温控制参数上。
 * @param proportion Float32，PID的比例系数。
 * @param integration Float32，PID的积分系数。
 * @param differential Float32，PID的微分系数。
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_OK  操作成功；
 *  - @ref DSCP_ERROR 操作失败；
 */
void TempControl_SetThermostatParam(DscpDevice* dscp, Byte* data, Uint16 len)
{
    ThermostatParam thermostatparam;
    int size = 0;
    Uint16 ret = DSCP_OK;
    //设置数据正确性判断
    size = sizeof(thermostatparam);
    if (len > size)
    {
        ret = DSCP_ERROR;
    }
    if (DSCP_OK == ret)
    {
        //修改并保存
        memcpy(&thermostatparam, data, sizeof(ThermostatParam));
        ThermostatManager_SetPIDParam(0, thermostatparam);
    }
    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 查询恒温器的工作状态。
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_IDLE 空闲；
 *  - @ref DSCP_BUSY 忙碌，需要停止后才能做下一个动作；
 */
void TempControl_GetThermostatStatus(DscpDevice* dscp, Byte* data, Uint16 len)
{
    if (THERMOSTAT_IDLE == ThermostatManager_GetStatus(0))
    {
        DscpDevice_SendStatus(dscp, DSCP_IDLE);
    }
    else
    {
        DscpDevice_SendStatus(dscp, DSCP_BUSY);
    }
}

/**
 * @brief 开始恒温。
 * @details 恒温开始后，系统将根据设定的恒温控制参数进行自动温度控制，尝试达到指定温度。
 *  不管成功与否，操作结果都将以事件的形式上传给上位机，但恒温器将继续工作，
 *  直到接收到DSCP_CMD_TCI_STOP_THERMOSTAT才停止。关联的事件有：
 *   - @ref DSCP_EVENT_TCI_THERMOSTAT_RESULT
 * @param mode Uint8，恒温模式，支持的模式见： @ref ThermostatMode 。
 * @param targetTemp Float32，恒温目标温度。
 * @param toleranceTemp Float32，容差温度，与目标温度的差值在该参数范围内即认为到达目标温度。
 * @param timeout Float32，超时时间，单位为秒。超时仍未达到目标温度，也将返回结果事件。
 * @return 状态回应，Uint16，支持的状态有：
 *   - @ref DSCP_OK  操作成功；
 *   - @ref DSCP_BUSY 操作失败，如恒温已经工作，需要先停止；
 *   - @ref DSCP_ERROR_PARAM 恒温参数错误；
 *   - @ref DSCP_ERROR 温度传感器异常；
 * @note 该命令将立即返回，恒温完成将以事件的形式上报。
 */
void TempControl_StartThermostat(DscpDevice* dscp, Byte* data, Uint16 len)
{
    ThermostatMode mode;         //恒温
    float targetTemp;           // 目标温度
    float toleranceTemp;        // 容差
    float timeout;              // 超时时间

    int size = 0;
    unsigned short ret = DSCP_OK;
    //设置数据正确性判断
    size =  sizeof(ThermostatMode) + sizeof(float) * 3;
    if ((len > size))
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("Parame Len Error\n");
        TRACE_ERROR("%d \n", size);
    }
    else
    {
        //修改并保存
        memcpy(&mode, data, sizeof(ThermostatMode));
        size = sizeof(ThermostatMode);

        memcpy(&targetTemp, data+size, sizeof(float));
        size += sizeof(float);

        memcpy(&toleranceTemp, data+size, sizeof(float));
        size += sizeof(float);

        memcpy(&timeout, data+size, sizeof(float));

        ThermostatManager_SendEventOpen(0);
        ret = (Uint16)ThermostatManager_Start(0, mode, targetTemp, toleranceTemp, timeout);
    }
    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 停止恒温控制。
 * @details 停止后，加热器和冷却器将不工作。
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_OK  操作成功；
 *  - @ref DSCP_ERROR 操作失败；
 * @note 该命令将立即返回。
 */
void TempControl_StopThermostat(DscpDevice* dscp, Byte* data, Uint16 len)
{
    unsigned short ret = DSCP_OK;

    ThermostatManager_SendEventOpen(0);
    if(FALSE == ThermostatManager_RequestStop(0))
    {
        ret = DSCP_ERROR;
    }
    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 设置温度上报周期。
 * @details 系统将根据设定的周期，定时向上发出温度上报事件。
 * @param period Float32，温度上报周期，单位为秒。0表示不需要上报，默认为0。
 * @see DSCP_EVENT_TCI_TEMPERATURE_NOTICE
 * @note 所设置的上报周期将在下一次启动时丢失，默认为0，不上报。
 */
void TempControl_SetTemperatureNotifyPeriod(DscpDevice* dscp, Byte* data,
        Uint16 len)
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
        ThermostatManager_SetTempReportPeriod(period);
    }
    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 设置机箱风扇。
 * @details 根据设定的占空比，调节风扇速率
 * @param level float ,风扇速率，对应高电平占空比。默认为0，机箱风扇关闭。
 */
void TempControl_BoxFanSetOutput(DscpDevice* dscp, Byte* data,
        Uint16 len)
{
    float level= 0.0;
    int size = 0;
    unsigned short ret = DSCP_OK;   // 默认系统正常
    // 数据正确性判断
    size = sizeof(level);
    if ((len > size))
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("Parame Len Error\n");
        TRACE_ERROR("%d \n", size);
    }
    // 设置机箱风扇占空比 0为关
    if (DSCP_OK == ret)
    {
       memcpy(&level, data, sizeof(level));
       BoxFanDriver_SetOutput(level);
    }

    DscpDevice_SendStatus(dscp, ret);// 发送状态回应
}

/**
 * @brief 设置恒温器风扇。
 * @details 根据设定的占空比，调节风扇速率
 * @param level float ,风扇速率，对应高电平占空比。默认为0，恒温器风扇关闭。
 */
void TempControl_ThermostatFanSetOutput(DscpDevice* dscp, Byte* data,
        Uint16 len)
{
    float level= 0.0;
    int size = 0;
    unsigned short ret = DSCP_OK;   // 默认系统正常
    // 数据正确性判断
    size = sizeof(level);
    if ((len > size))
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("Parame Len Error\n");
        TRACE_ERROR("%d \n", size);
    }
    // 设置恒温器风扇占空比 0为关
    if (DSCP_OK == ret)
    {
       memcpy(&level, data, sizeof(level));
       ThermostatManager_SetSingleRefrigeratorOutput(0, 0, level);
    }

    DscpDevice_SendStatus(dscp, ret);// 发送状态回应
}

/**
 * @brief 获取加热丝输出的最大占空比。
 * @param maxDutyCycle float ,加热丝输出的最大占空比
 * @see DSCP_CMD_TCI_SET_HEATER_MAX_DUTY_CYCLE
 */
void TempControl_GetHeaterMaxDutyCycle(DscpDevice* dscp, Byte* data,Uint16 len)
{
    float value = ThermostatManager_GetHeaterMaxDutyCycle(0);
    TRACE_INFO("\n TempControl_GetHeaterMaxDutyCycle maxDutyCycle: %f", value);

    DscpDevice_SendResp(dscp, (void *) &value, sizeof(value));
}

/**
 * @brief 设置加热丝输出的最大占空比。
 * @param maxDutyCycle float，加热丝输出的最大占空比
 * @see DSCP_CMD_TCI_GET_HEATER_MAX_DUTY_CYCLE
 */
void TempControl_SetHeaterMaxDutyCycle(DscpDevice* dscp, Byte* data,Uint16 len)
{
    float value;
    int size = 0;
    unsigned short ret = DSCP_OK;

    size = sizeof(value);
    if ((len > size))
    {
        ret = DSCP_ERROR;
        TRACE_ERROR("Parame Len Error\n");
        TRACE_ERROR("%d \n", size);
    }
    if (DSCP_OK == ret)
    {
       memcpy(&value, data, sizeof(value));
       TRACE_INFO("\n TempControl_SetHeaterMaxDutyCycle maxDutyCycle: %f", value);

       if (FALSE == ThermostatManager_SetMaxDutyCycle(0, value))
       {
           ret = DSCP_ERROR;
       }
    }

    DscpDevice_SendStatus(dscp, ret);// 发送状态回应
}

/**
 * @brief 查询当前恒温控制参数。
 * @return 恒温控制参数，格式如下：
 *  - proportion Float32，PID的比例系数。
 *  - integration Float32，PID的积分系数。
 *  - differential Float32，PID的微分系数。
 * @see DSCP_CMD_TCI_SET_CURRENT_THERMOSTAT_PARAM
 */
void TempControl_GetCurrentThermostatParam(DscpDevice* dscp, Byte* data,Uint16 len)
{
    ThermostatParam thermostatparam;
    thermostatparam = ThermostatManager_GetCurrentPIDParam(0);
    // 发送回应
    DscpDevice_SendResp(dscp, (void *) &thermostatparam, sizeof(thermostatparam));
}

/**
 * @brief 设置当前恒温控制参数。
 * @details 恒温系统将根据设置的参数进行温度调节。此参数上电时获取FLASH的PID。
 * @param proportion Float32，PID的比例系数。
 * @param integration Float32，PID的积分系数。
 * @param differential Float32，PID的微分系数。
 * @return 状态回应，Uint16，支持的状态有：
 *  - @ref DSCP_OK  操作成功；
 *  - @ref DSCP_ERROR 操作失败；
 */
void TempControl_SetCurrentThermostatParam(DscpDevice* dscp, Byte* data,Uint16 len)
{
    ThermostatParam thermostatparam;
    int size = 0;
    Uint16 ret = DSCP_OK;
    //设置数据正确性判断
    size = sizeof(thermostatparam);
    if (len > size)
    {
        ret = DSCP_ERROR;
    }
    if (DSCP_OK == ret)
    {
        //修改并保存
        memcpy(&thermostatparam, data, sizeof(ThermostatParam));
        ThermostatManager_SetCurrentPIDParam(0, thermostatparam);
    }
    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 开启继电器
 * @note 该命令已不支持
 */
void TempControl_TurnOnRelay(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 ret = DSCP_NOT_SUPPORTED;

    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 关闭继电器
 * @note 该命令已不支持
 */
void TempControl_TurnOffRelay(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 ret = DSCP_NOT_SUPPORTED;

    DscpDevice_SendStatus(dscp, ret);
}

/**
 * @brief 查询系统支持的恒温器数目。
 * @return 总数目， Uint16。
 */
void TempControl_GetTotalThermostat(DscpDevice* dscp, Byte* data, Uint16 len)
{
    Uint16 number = TOTAL_THERMOSTAT;
    TRACE_INFO("\n GetTotalThermostat %d", number);
    DscpDevice_SendResp(dscp, (void *) &number, sizeof(number));
}

/**
 * @brief 查询当前环境温度。
 * @return 当前恒温室温度，Float32。
 */
void TempControl_GetEnvTemperature(DscpDevice* dscp, Byte* data, Uint16 len)
{
    float temp = TempCollecterManager_GetEnvironmentTemp();
    TRACE_INFO("\n EnvironmentTemp : ");
    System_PrintfFloat(TRACE_LEVEL_INFO, temp, 1);
    DscpDevice_SendResp(dscp, (void *) &temp, sizeof(temp));
}
