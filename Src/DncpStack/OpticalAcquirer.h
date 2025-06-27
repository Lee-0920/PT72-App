/**
 * @file
 * @brief 光学采集接口实现头文件
 * @details
 * @version 1.0.0
 * @author lemon.xiaoxun
 * @date 2016-5-27
 */

#ifndef SRC_DNCPSTACK_OPTICALACQUIRER_H_
#define SRC_DNCPSTACK_OPTICALACQUIRE_H_


#include "LuipApi/OpticalAcquireInterface.h"
#include "DNCP/App/DscpDevice.h"

#ifdef __cplusplus
extern "C" {
#endif

void OpticalAcquirer_Init(void);
void OpticalAcquirer_TurnOnLed(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalAcquirer_SetSignalADNotifyPeriod(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalAcquirer_StartAcquirer(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalAcquirer_StopAcquirer(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalAcquirer_StartLEDController(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalAcquirer_StopLEDController(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalAcquirer_GetLEDControllerTarget(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalAcquirer_SetLEDControllerTarget(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalAcquirer_GetLEDControllerParam(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalAcquirer_SetLEDControllerParam(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalAcquirer_StartLEDAdjust(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalAcquirer_StopLEDAdjust(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalAcquirer_StartStaticADControl(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalAcquirer_StopStaticADControl(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalAcquirer_GetStaticADControlParam(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalAcquirer_SetStaticADControlParam(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalAcquire_IsADControlValid(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalAcquirer_GetLEDDefaultValue(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalAcquirer_SetLEDDefaultValue(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalAcquirer_CtrlXonen(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalAcquirer_GetADCWorkMode(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalAcquirer_SetADCWorkMode(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalAcquirer_SetCheckLeakingReportPeriod(DscpDevice* dscp, Byte* data, Uint16 len);
void OpticalAcquirer_SetStaticMultiple(DscpDevice* dscp, Byte* data, Uint16 len);

//命令入口，全局宏定义：每一条命令对应一个命令码和处理函数
#define CMD_TABLE_OPTICALACQUIRE \
        DSCP_CMD_ENTRY(DSCP_CMD_OAI_TURN_ON_LED, OpticalAcquirer_TurnOnLed), \
        DSCP_CMD_ENTRY(DSCP_CMD_OAI_SET_SIGNAL_AD_NOTIFY_PERIOD, OpticalAcquirer_SetSignalADNotifyPeriod), \
        DSCP_CMD_ENTRY(DSCP_CMD_OAI_START_ACQUIRER, OpticalAcquirer_StartAcquirer), \
        DSCP_CMD_ENTRY(DSCP_CMD_OAI_STOP_ACQUIRER, OpticalAcquirer_StopAcquirer), \
        DSCP_CMD_ENTRY(DSCP_CMD_OAI_START_LEDCONTROLLER, OpticalAcquirer_StartLEDController), \
        DSCP_CMD_ENTRY(DSCP_CMD_OAI_STOP_LEDCONTROLLER, OpticalAcquirer_StopLEDController), \
        DSCP_CMD_ENTRY(DSCP_CMD_OAI_GET_LEDCONTROLLER_TARGET, OpticalAcquirer_GetLEDControllerTarget), \
        DSCP_CMD_ENTRY(DSCP_CMD_OAI_SET_LEDCONTROLLER_TARGET, OpticalAcquirer_SetLEDControllerTarget), \
        DSCP_CMD_ENTRY(DSCP_CMD_OAI_GET_LEDCONTROLLER_PARAM, OpticalAcquirer_GetLEDControllerParam), \
        DSCP_CMD_ENTRY(DSCP_CMD_OAI_SET_LEDCONTROLLER_PARAM, OpticalAcquirer_SetLEDControllerParam),\
        DSCP_CMD_ENTRY(DSCP_CMD_OAI_START_LEDADJUST, OpticalAcquirer_StartLEDAdjust), \
        DSCP_CMD_ENTRY(DSCP_CMD_OAI_STOP_LEDADJUST, OpticalAcquirer_StopLEDAdjust), \
        DSCP_CMD_ENTRY(DSCP_CMD_OAI_START_STATIC_AD_CONTROL, OpticalAcquirer_StartStaticADControl), \
        DSCP_CMD_ENTRY(DSCP_CMD_OAI_STOP_STATIC_AD_CONTROL, OpticalAcquirer_StopStaticADControl), \
        DSCP_CMD_ENTRY(DSCP_CMD_OAI_GET_STATIC_AD_CONTROL_PARAM, OpticalAcquirer_GetStaticADControlParam), \
        DSCP_CMD_ENTRY(DSCP_CMD_OAI_SET_STATIC_AD_CONTROL_PARAM, OpticalAcquirer_SetStaticADControlParam), \
        DSCP_CMD_ENTRY(DSCP_CMD_OAI_IS_STATIC_AD_CONTROL_VALID, OpticalAcquire_IsADControlValid), \
        DSCP_CMD_ENTRY(DSCP_CMD_OAI_GET_LED_DEFAULT_VALUE, OpticalAcquirer_GetLEDDefaultValue), \
        DSCP_CMD_ENTRY(DSCP_CMD_OAI_SET_LED_DEFAULT_VALUE, OpticalAcquirer_SetLEDDefaultValue), \
        DSCP_CMD_ENTRY(DSCP_CMD_OAI_CTRL_XONEN, OpticalAcquirer_CtrlXonen), \
		DSCP_CMD_ENTRY(DSCP_CMD_OAI_SET_CHECK_LEAKING_PERIOD, OpticalAcquirer_SetCheckLeakingReportPeriod), \
		DSCP_CMD_ENTRY(DSCP_CMD_OAI_SET_STATIC_MULTIPLE, OpticalAcquirer_SetStaticMultiple)

#ifdef __cplusplus
}
#endif

#endif /* SRC_DNCPSTACK_OPTICALACQUIRER_H_ */
